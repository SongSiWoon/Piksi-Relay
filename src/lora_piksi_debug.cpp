// Debug version of lora_piksi.cpp
// 기능은 원본과 동일하며, obs_callback에서 아래 3가지를 추가로 감시한다:
//   1. [OVERFLOW?]    : memcpy 전, len > 164 조건 감지 (사전 경고)
//   2. [CORRUPTION]   : memcpy 후, canary 패턴이 깨졌는지 확인 (실제 메모리 오염 확인)
//   3. [STATS]        : 10초마다 누적 통계 출력
//
// 빌드:
//   g++ lora_piksi_debug.cpp -I ../include/lora_mavlink/swarm -lsbp -o piksi_relay_debug -pthread
// 실행:
//   ./piksi_relay_debug -d /dev/ttyACM1 -l /dev/ttyUSB0

#include <stdio.h>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <poll.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <libsbp/piksi.h>
#include <libsbp/edc.h>
#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/navigation.h>
#include <libsbp/observation.h>

#include <chrono>
#include <array>

#include <mavlink.h>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#define MSG_OBS_HEADER_SEQ_SHIFT 4u
#define MSG_OBS_HEADER_SEQ_MASK ((1 << 4u) - 1)

#define LORA_BUF        (197)
#define LORA_PACKET     (LORA_BUF + 3)
#define MAXBUF          (1024)

#define BUFFER_LENGTH 2041
uint8_t mavbuf[BUFFER_LENGTH];

#define LORA_CHANNEL 80
uint8_t _header[3] = {0xFF, 0xFF, (uint8_t)LORA_CHANNEL};

int _g_serial_fd = 0;
int _lora_serial_fd = 0;

int _sock = 0;
struct sockaddr_in _sock_in;

#define DEVICE "/dev/ttyACM1"
#define LORA_DEVICE "/dev/ttyUSB0"
#define SLEEP_US 100
#define BAUDRATE B115200

#define OBS_DATA_SIZE 164   // sizeof(mavlink_piksi_obs_t::data)
#define CANARY_PATTERN 0xAB
#define CANARY_SIZE 64      // OBS 최대 overflow = 34 bytes → 64으로 충분히 커버

// ─────────────────────────────────────────────
// 전역 통계 카운터
// ─────────────────────────────────────────────
static uint64_t g_obs_total       = 0;   // 수신한 OBS 메시지 수
static uint64_t g_obs_overflow    = 0;   // len > 164 발생 횟수
static uint64_t g_obs_corruption  = 0;   // canary 오염 감지 횟수
static int      g_max_len_seen    = 0;   // 지금까지 본 가장 큰 len
static int      g_max_sats_seen   = 0;   // 지금까지 본 가장 많은 위성 수

struct options {
    char device[64];
    int sleep_us;
    uint32_t baudrate;
};
struct options _piksi_options = {DEVICE, SLEEP_US, BAUDRATE};
struct options _lora_options  = {LORA_DEVICE, SLEEP_US, BAUDRATE};

sbp_state_t _sbp_state;

msg_heartbeat_t     _heartbeat;
msg_obs_t           _obs;
msg_base_pos_ecef_t _basepos;
msg_glo_biases_t    _globiases;
msg_pos_llh_t       _pos_llh;
msg_pos_llh_t       _pos_llh_avg;
msg_base_pos_llh_t  _base_pos_llh;
int                 _pos_llh_count;

sbp_msg_callbacks_node_t _heartbeat_callback_node;
sbp_msg_callbacks_node_t _obs_callback_node;
sbp_msg_callbacks_node_t _basepos_callback_node;
sbp_msg_callbacks_node_t _globiases_callback_node;
sbp_msg_callbacks_node_t _pos_llh_callback_node;

std::vector<mavlink_piksi_obs_t> _obs_vec;
std::queue<mavlink_message_t> _mavlink_que;
std::mutex mtx;
uint8_t _sbp_seq = 0;

long long time_ms()
{
    struct timeval te;
    gettimeofday(&te, NULL);
    return te.tv_sec * 1000LL + te.tv_usec / 1000;
}

long long _prev_time  = 0L;
clock_t   _prev_clock = 0;

// ─────────────────────────────────────────────
// SIGSEGV / SIGABRT 핸들러 (크래시 감지)
// ─────────────────────────────────────────────
void crash_handler(int sig)
{
    printf("\n[CRASH] signal %d received!\n", sig);
    printf("[CRASH STATS] obs_total=%llu  overflow=%llu  corruption=%llu  max_len=%d  max_sats=%d\n",
           (unsigned long long)g_obs_total,
           (unsigned long long)g_obs_overflow,
           (unsigned long long)g_obs_corruption,
           g_max_len_seen,
           g_max_sats_seen);
    fflush(stdout);
    _exit(1);
}

void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
    _heartbeat = *(msg_heartbeat_t *)msg;
}

void basepos_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
    int status, buflen;
    uint8_t buffer[MAXBUF];

    printf("MSG_BASE_POS_ECEF  len:%d\n", len);
    _basepos = *(msg_base_pos_ecef_t *)msg;
    u16 crc = *(u16 *) context;

    mavlink_message_t message;
    mavlink_piksi_base_pos_ecef_t piksi_base_pos;
    piksi_base_pos.msg_type  = SBP_MSG_BASE_POS_ECEF;
    piksi_base_pos.sender_id = sender_id;
    piksi_base_pos.seq       = _sbp_seq++;
    piksi_base_pos.crc       = crc;
    piksi_base_pos.len       = len;
    memcpy(&piksi_base_pos.data, msg, len);
    mavlink_msg_piksi_base_pos_ecef_encode(255, 51, &message, &piksi_base_pos);

    buflen = mavlink_msg_to_send_buffer(buffer, &message);
    int sinlen = sizeof(struct sockaddr_in);
    status = sendto(_sock, buffer, buflen, 0, (struct sockaddr *)&_sock_in, sinlen);

    std::lock_guard<std::mutex> lock(mtx);
    _mavlink_que.push(message);
}

// ─────────────────────────────────────────────
// obs_callback : 디버그 핵심 부분
// ─────────────────────────────────────────────
void obs_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
    u8 obs_in_msg = (len - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);
    printf("MSG_OBS 0x%0X sats=%d len=%d\n", sender_id, obs_in_msg, len);

    u16 crc = *(u16 *) context;
    if (obs_in_msg == 0) return;

    g_obs_total++;
    if (len > g_max_len_seen)  g_max_len_seen  = len;
    if (obs_in_msg > g_max_sats_seen) g_max_sats_seen = obs_in_msg;

    // ──────────────────────────────────────────
    // [1] 사전 감지: memcpy 전에 len > 164 체크
    // ──────────────────────────────────────────
    if (len > OBS_DATA_SIZE) {
        g_obs_overflow++;
        printf("[OVERFLOW?] len=%d > data[%d]  sats=%d  overflow=%d bytes"
               "  (total=%llu  overflow_cnt=%llu)\n",
               len, OBS_DATA_SIZE, obs_in_msg,
               len - OBS_DATA_SIZE,
               (unsigned long long)g_obs_total,
               (unsigned long long)g_obs_overflow);
    }

    // ──────────────────────────────────────────
    // [2] canary 구조체: obs 바로 뒤에 canary[64] 배치
    //     obs.data[164] overflow → canary 영역으로 넘침
    // ──────────────────────────────────────────
    struct {
        mavlink_piksi_obs_t obs;
        uint8_t canary[CANARY_SIZE];
    } guarded;
    memset(&guarded, 0, sizeof(guarded));
    memset(guarded.canary, CANARY_PATTERN, CANARY_SIZE);

    guarded.obs.msg_type  = SBP_MSG_OBS;
    guarded.obs.sender_id = sender_id;
    guarded.obs.seq       = _sbp_seq++;
    guarded.obs.crc       = crc;
    guarded.obs.len       = len;

    // 원본 코드와 동일한 memcpy (overflow 그대로 재현)
    memcpy(guarded.obs.data, msg, len);

    // canary 패턴 깨졌는지 확인
    bool corrupted = false;
    for (int i = 0; i < CANARY_SIZE; i++) {
        if (guarded.canary[i] != CANARY_PATTERN) {
            if (!corrupted) {
                g_obs_corruption++;
                printf("[CORRUPTION] canary[%d]=0x%02X (0x%02X expected!) "
                       "sats=%d  len=%d  corruption_cnt=%llu\n",
                       i, guarded.canary[i], CANARY_PATTERN,
                       obs_in_msg, len,
                       (unsigned long long)g_obs_corruption);
            }
            corrupted = true;
        }
    }
    if (!corrupted && len > OBS_DATA_SIZE) {
        // 오버플로우 조건인데 canary가 안 깨졌다면 → 컴파일러가 구조체 사이에 패딩 삽입한 것
        printf("[OVERFLOW?] overflow 조건이지만 canary 안 깨짐 (컴파일러 패딩 의심)\n");
    }

    _obs_vec.insert(_obs_vec.begin(), guarded.obs);

    mavlink_message_t message;
    mavlink_msg_piksi_obs_encode(255, 51, &message, &guarded.obs);
    std::lock_guard<std::mutex> lock(mtx);
    _mavlink_que.push(message);
}

void globiases_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
    int status, buflen;
    uint8_t buffer[MAXBUF];

    printf("MSG_GLO_BIASES  len:%d\n", len);
    u16 crc = *(u16 *) context;
    _globiases = *(msg_glo_biases_t *)msg;

    mavlink_message_t message;
    mavlink_piksi_glo_biases_t piksi_glo_biases;
    piksi_glo_biases.msg_type  = SBP_MSG_GLO_BIASES;
    piksi_glo_biases.sender_id = sender_id;
    piksi_glo_biases.seq       = _sbp_seq++;
    piksi_glo_biases.crc       = crc;
    piksi_glo_biases.len       = len;
    memcpy(piksi_glo_biases.data, msg, len);
    mavlink_msg_piksi_glo_biases_encode(255, 51, &message, &piksi_glo_biases);

    buflen = mavlink_msg_to_send_buffer(buffer, &message);
    int sinlen = sizeof(struct sockaddr_in);
    status = sendto(_sock, buffer, buflen, 0, (struct sockaddr *)&_sock_in, sinlen);

    std::lock_guard<std::mutex> lock(mtx);
    _mavlink_que.push(message);
}

void sbp_setup(void)
{
    sbp_state_init(&_sbp_state);
    printf("test\n");
    sbp_register_callback(&_sbp_state, SBP_MSG_OBS,          &obs_callback,       &_sbp_state.crc, &_obs_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_HEARTBEAT,    &heartbeat_callback, NULL,             &_heartbeat_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_BASE_POS_ECEF,&basepos_callback,   &_sbp_state.crc, &_basepos_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_GLO_BIASES,   &globiases_callback, &_sbp_state.crc, &_globiases_callback_node);
}

int setupSerial(char* device, uint32_t baudrate)
{
    int uart = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart < 0) {
        printf("FAIL: Error opening port\n");
        return -1;
    }

    struct termios uart_config;
    tcgetattr(uart, &uart_config);

    uart_config.c_cflag |= CLOCAL | CREAD | CS8;
    uart_config.c_iflag  = IGNPAR;
    uart_config.c_oflag  = 0;
    uart_config.c_lflag  = 0;

    if (cfsetispeed(&uart_config, baudrate) < 0 || cfsetospeed(&uart_config, baudrate) < 0) {
        printf("FAIL: Error setting baudrate\n");
        return -1;
    }
    if (tcsetattr(uart, TCSANOW, &uart_config) < 0) {
        printf("FAIL: Error setting termios\n");
        return -1;
    }
    return uart;
}

static u32 piksi_port_read(u8 *buff, u32 n, void *context)
{
    u32 result = 0;
    if (_g_serial_fd > 0)
        result = read(_g_serial_fd, buff, n);
    return result;
}

void broadcast_setup(void)
{
    int yes = 1;
    int sinlen = sizeof(struct sockaddr_in);
    memset(&_sock_in, 0, sinlen);

    _sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    _sock_in.sin_addr.s_addr = htonl(INADDR_ANY);
    _sock_in.sin_port        = htons(0);
    _sock_in.sin_family      = PF_INET;

    int status = bind(_sock, (struct sockaddr *)&_sock_in, sinlen);
    printf("Bind Status = %d\n", status);
    status = setsockopt(_sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(int));
    printf("Setsockopt Status = %d\n", status);

    _sock_in.sin_addr.s_addr = inet_addr("10.42.0.255");
    _sock_in.sin_port        = htons(9750);
    _sock_in.sin_family      = PF_INET;
}

s32 fifo_read(u8 *buff, u32 n, void *context)
{
    int i = 0;
    for (i = 0; i < (int)n; i++) {
        u32 readLen = read(_g_serial_fd, buff + i, 1);
        if (readLen <= 0) break;
    }
    return i;
}

static int parse_options(int argc, char **argv)
{
    int ch;
    while ((ch = getopt(argc, argv, "d:l:w:b:")) != EOF) {
        switch (ch) {
            case 'd': if (nullptr != optarg) strcpy(_piksi_options.device, optarg); break;
            case 'l': if (nullptr != optarg) strcpy(_lora_options.device,  optarg); break;
            case 'w': _piksi_options.sleep_us = strtol(optarg,  nullptr, 10); break;
            case 'b': _piksi_options.baudrate = strtoul(optarg, nullptr, 10); break;
        }
    }
    return 0;
}

// ─────────────────────────────────────────────
// [3] 통계 출력: 10초마다 누적 현황 요약
// ─────────────────────────────────────────────
void print_stats()
{
    printf("\n========== [DEBUG STATS] ==========\n");
    printf("  OBS 수신 총계    : %llu\n",  (unsigned long long)g_obs_total);
    printf("  OVERFLOW 발생    : %llu  (len > 164)\n", (unsigned long long)g_obs_overflow);
    printf("  CORRUPTION 감지  : %llu  (canary 오염)\n", (unsigned long long)g_obs_corruption);
    printf("  최대 len         : %d bytes\n", g_max_len_seen);
    printf("  최대 위성 수     : %d개\n",     g_max_sats_seen);
    if (g_obs_total > 0)
        printf("  overflow 비율    : %.1f%%\n",
               100.0 * g_obs_overflow / g_obs_total);
    printf("====================================\n\n");
}

void write_to_lora()
{
    long long before_t = time_ms();
    long long stats_t  = time_ms();
    uint8_t lora_buffer[LORA_BUF];
    uint8_t packet[LORA_PACKET];

    while (1) {
        long long now_t = time_ms();

        // 10초마다 통계 출력
        if (now_t - stats_t > 10000) {
            print_stats();
            stats_t = now_t;
        }

        if (!_mavlink_que.empty() && now_t - before_t > 100) {
            std::lock_guard<std::mutex> lock(mtx);
            mavlink_message_t message = _mavlink_que.front();
            printf("lora queue size : %ld\n", _mavlink_que.size());
            mavlink_msg_to_send_buffer(lora_buffer, &message);
            memcpy(packet, _header, sizeof(_header));
            memcpy(packet + sizeof(_header), lora_buffer, sizeof(lora_buffer));
            ssize_t bytes_written = write(_lora_serial_fd, packet, sizeof(packet));
            if (bytes_written < 0) {
                perror("Failed to write to serial port\n");
                break;
            }
            _mavlink_que.pop();
            before_t = time_ms();
            if (tcdrain(_lora_serial_fd) != 0) {
                perror("Failed to flush serial port\n");
                break;
            }
        }
    }
}

int main(int argc, char **argv)
{
    // 크래시 핸들러 등록
    signal(SIGSEGV, crash_handler);
    signal(SIGABRT, crash_handler);

    printf("=== lora_piksi DEBUG version ===\n");
    printf("  OBS data[] 크기   : %d bytes\n", OBS_DATA_SIZE);
    printf("  Canary 크기       : %d bytes (패턴=0x%02X)\n", CANARY_SIZE, CANARY_PATTERN);
    printf("  overflow 발생 조건: 위성 10개 이상 (len > 164)\n\n");

    std::thread lora_thread(write_to_lora);
    long long before = time_ms();
    int status, buflen;
    uint8_t buffer[MAXBUF];

    broadcast_setup();

    mavlink_message_t message;

    if (-1 == parse_options(argc, argv)) {
        printf("EXITING...\n");
        return -1;
    }
    printf("piksi port : %s\n", _piksi_options.device);
    printf("lora port  : %s\n", _lora_options.device);

    _g_serial_fd = setupSerial(_piksi_options.device, _piksi_options.baudrate);
    if (_g_serial_fd < 0) {
        printf("ERROR: cannot use piksi serial fd (%d) %s\n", _g_serial_fd, strerror(errno));
        return -1;
    }

    _lora_serial_fd = setupSerial(_lora_options.device, _piksi_options.baudrate);
    if (_lora_serial_fd < 0) {
        printf("ERROR: cannot use lora serial fd (%d) %s\n", _lora_serial_fd, strerror(errno));
        return -1;
    }

    sbp_setup();

    struct pollfd fds[1];
    fds[0].fd     = _g_serial_fd;
    fds[0].events = POLLIN;

    int cnt_timeout = 0;
    int cnt_err     = 0;

    while (1) {
        long long now = time_ms();
        int status = poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);

        if (status > 0) {
            if (now - before > 80 && !_obs_vec.empty()) {
                mavlink_msg_piksi_obs_encode(255, 51, &message, &_obs_vec.back());
                buflen = mavlink_msg_to_send_buffer(buffer, &message);
                int sinlen = sizeof(struct sockaddr_in);
                status = sendto(_sock, buffer, buflen, 0, (struct sockaddr *)&_sock_in, sinlen);
                _obs_vec.pop_back();
                before = time_ms();
            }
            if (fds[0].revents & POLLIN) {
                s8 ret = sbp_process(&_sbp_state, &fifo_read);
                if (ret < 0)
                    printf("ERROR %d %d %s\n", ret, _sbp_state.msg_type, strerror(errno));
            }
        } else if (status == 0) {
            printf("poll:timeout (%d)\n", cnt_timeout);
            cnt_timeout++;
        } else {
            printf("poll:error (%d)\n", cnt_err);
            cnt_err++;
        }

        if (cnt_timeout > 5 || cnt_err > 5)
            return -1;
    }

    lora_thread.join();
    return 0;
}
