#include <stdio.h>   /* Standard input/output definitions  */
#include <cstring>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <poll.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>

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
#include <atomic>

#define MSG_OBS_HEADER_SEQ_SHIFT 4u
#define MSG_OBS_HEADER_SEQ_MASK ((1 << 4u) - 1)

#define LORA_BUF		    (197)
#define LORA_PACKET		(LORA_BUF + 3)
#define MAXBUF		    (1024)

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


struct options {
    char device[64];
    int sleep_us;
    uint32_t baudrate;
};
struct options _piksi_options = {DEVICE, SLEEP_US, BAUDRATE};
struct options _lora_options = {LORA_DEVICE, SLEEP_US, BAUDRATE};


/*
 * State of the SBP message parser.
 * Must be statically allocated.
 */
sbp_state_t _sbp_state;

/* SBP structs that messages from Piksi will feed. */
msg_heartbeat_t         _heartbeat;
msg_obs_t               _obs;
msg_base_pos_ecef_t     _basepos;
msg_glo_biases_t        _globiases;
msg_pos_llh_t           _pos_llh;
msg_pos_llh_t           _pos_llh_avg;
msg_base_pos_llh_t      _base_pos_llh;
int                     _pos_llh_count;


sbp_msg_callbacks_node_t _heartbeat_callback_node;
sbp_msg_callbacks_node_t _obs_callback_node;
sbp_msg_callbacks_node_t _basepos_callback_node;
sbp_msg_callbacks_node_t _globiases_callback_node;
sbp_msg_callbacks_node_t _pos_llh_callback_node;

std::vector<mavlink_piksi_obs_t> _obs_vec;
std::mutex mtx;
std::atomic<bool> _running{true};
uint8_t _sbp_seq = 0;

/* OBS: tow 기준 에폭 단위 큐 - 같은 에폭의 패킷은 모두 전송, 새 에폭 오면 이전 버림 */
std::queue<mavlink_message_t> _obs_epoch_que;
uint32_t _current_obs_tow = 0;

/* GLO/BASE: 최신 메시지 보관 후 5초마다 교대 전송 */
mavlink_message_t _latest_glo_msg;
bool _has_glo = false;
mavlink_message_t _latest_base_msg;
bool _has_base = false;

#define NON_OBS_INTERVAL_MS 5000
long long _last_non_obs_sent = 0;
bool _send_glo_next = true;

long long time_ms()
{
    struct timeval te;
    gettimeofday(&te, NULL);
    long long ms = te.tv_sec*1000LL + te.tv_usec/1000;
    return ms;
}


long long _prev_time = 0L;
clock_t _prev_clock = 0;


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
    piksi_base_pos.msg_type = SBP_MSG_BASE_POS_ECEF;
    piksi_base_pos.sender_id = sender_id;
    piksi_base_pos.seq = _sbp_seq++;
    piksi_base_pos.crc = crc;
    piksi_base_pos.len = len;
    memcpy(&piksi_base_pos.data, msg, len);
    mavlink_msg_piksi_base_pos_ecef_encode(255, 51, &message, &piksi_base_pos);
    buflen = mavlink_msg_to_send_buffer(buffer, &message);
    int sinlen = sizeof(struct sockaddr_in);
    status = sendto(_sock, buffer, buflen, 0, (struct sockaddr *)&_sock_in, sinlen);

    std::lock_guard<std::mutex> lock(mtx);
    _latest_base_msg = message;
    _has_base = true;
}

void obs_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
    u8 obs_in_msg = (len - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);
    printf("MSG_OBS 0x%0X %d (%d)\n", sender_id, obs_in_msg, len);
    u16 crc = *(u16 *) context;
    if ( obs_in_msg == 0 ) {
        return;
    }
    mavlink_message_t message;
    mavlink_piksi_obs_t piksi_obs = {0, };
    piksi_obs.msg_type = SBP_MSG_OBS;
    piksi_obs.sender_id = sender_id;
    piksi_obs.seq = _sbp_seq++;
    piksi_obs.crc = crc;
    piksi_obs.len = len;
    memcpy(piksi_obs.data, msg, len);
    _obs_vec.insert(_obs_vec.begin(), piksi_obs);

    mavlink_msg_piksi_obs_encode(255, 51, &message, &piksi_obs);

    uint32_t tow;
    memcpy(&tow, msg, sizeof(uint32_t));

    std::lock_guard<std::mutex> lock(mtx);
    if (tow != _current_obs_tow) {
        /* 새 에폭 도착 - 이전 에폭 잔여 패킷 버림 */
        while (!_obs_epoch_que.empty()) _obs_epoch_que.pop();
        _current_obs_tow = tow;
    }
    _obs_epoch_que.push(message);
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
    piksi_glo_biases.msg_type = SBP_MSG_GLO_BIASES;
    piksi_glo_biases.sender_id = sender_id;
    piksi_glo_biases.seq = _sbp_seq++;
    piksi_glo_biases.crc = crc;
    piksi_glo_biases.len = len;
    memcpy(piksi_glo_biases.data, msg, len);
    mavlink_msg_piksi_glo_biases_encode(255, 51, &message, &piksi_glo_biases);

    buflen = mavlink_msg_to_send_buffer(buffer, &message);
    int sinlen = sizeof(struct sockaddr_in);
    status = sendto(_sock, buffer, buflen, 0, (struct sockaddr *)&_sock_in, sinlen);

    std::lock_guard<std::mutex> lock(mtx);
    _latest_glo_msg = message;
    _has_glo = true;
}


void sbp_setup(void)
{
    /* SBP parser state must be initialized before sbp_process is called. */
    sbp_state_init(&_sbp_state);
    printf("test\n");
    /* register callback function */
    sbp_register_callback(&_sbp_state, SBP_MSG_OBS, &obs_callback, &_sbp_state.crc, &_obs_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL, &_heartbeat_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_BASE_POS_ECEF, &basepos_callback, &_sbp_state.crc, &_basepos_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_GLO_BIASES, &globiases_callback, &_sbp_state.crc, &_globiases_callback_node);
}

int setupSerial(char* device, uint32_t baudrate)
{
    int uart = open(device, O_RDWR | O_NOCTTY | O_NDELAY  /*O_NONBLOCK*/);
    if (uart < 0) {
        printf("FAIL: Error opening port\n");
        return -1;
    }

    struct termios uart_config;
    tcgetattr(uart, &uart_config);

    uart_config.c_cflag |= CLOCAL | CREAD | CS8;
    uart_config.c_iflag = IGNPAR;
    uart_config.c_oflag = 0;
    uart_config.c_lflag = 0;

    if (cfsetispeed(&uart_config, baudrate) < 0 || cfsetospeed(&uart_config, baudrate) < 0 ) {
        printf("FAIL: Error setting baudrate / termios config for cfsetispeed\n");
        return -1;
    }

    if (tcsetattr(uart, TCSANOW, &uart_config) < 0) {
        printf("FAIL: Error setting baudrate / termios config for tcsetattr\n");
        return -1;
    }

    return uart;

}

void broadcast_setup(void)
{
    int status, buflen, sinlen;
    char buffer[MAXBUF];
    int yes = 1;

    sinlen = sizeof(struct sockaddr_in);
    memset(&_sock_in, 0, sinlen);
    buflen = MAXBUF;

    _sock = socket (PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    _sock_in.sin_addr.s_addr = htonl(INADDR_ANY);
    _sock_in.sin_port = htons(0);
    _sock_in.sin_family = PF_INET;

    status = bind(_sock, (struct sockaddr *)&_sock_in, sinlen);
    printf("Bind Status = %d\n", status);

    status = setsockopt(_sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(int) );
    printf("Setsockopt Status = %d\n", status);

    _sock_in.sin_addr.s_addr= inet_addr("10.42.0.255");
    _sock_in.sin_port = htons(9750);
    _sock_in.sin_family = PF_INET;
}

s32 fifo_read(u8 *buff, u32 n, void *context)
{
    int i = 0;
    for ( i = 0 ; i < n  ; i++ ) {
        u32 readLen = read(_g_serial_fd, buff+i, 1);
        if ( readLen <= 0 ) break;
    }

    return i;
}

static int parse_options(int argc, char **argv)
{
    int ch;

    while ((ch = getopt(argc, argv, "d:l:w:b:")) != EOF)
    {
        switch (ch)
        {
            case 'd': if (nullptr != optarg) strcpy(_piksi_options.device, optarg); break;
            case 'l': if (nullptr != optarg) strcpy(_lora_options.device, optarg); break;
            case 'w': _piksi_options.sleep_us       = strtol(optarg, nullptr, 10);  break;
            case 'b': _piksi_options.baudrate       = strtoul(optarg, nullptr, 10); break;
                return -1;
        }
    }

    return 0;
}

void write_to_lora(){
    long long before_t, now_t;
    uint8_t lora_buffer[LORA_BUF];
    uint8_t packet[LORA_PACKET];
    before_t = time_ms();
    while (_running) {
        now_t = time_ms();
        if (now_t - before_t > 100) {
            mavlink_message_t message;
            bool has_message = false;

            {
                std::lock_guard<std::mutex> lock(mtx);
                if (!_obs_epoch_que.empty()) {
                    /* OBS 에폭 패킷 우선 전송 */
                    message = _obs_epoch_que.front();
                    _obs_epoch_que.pop();
                    has_message = true;
                } else if (now_t - _last_non_obs_sent >= NON_OBS_INTERVAL_MS) {
                    /* OBS 큐가 비었을 때 GLO/BASE 교대 전송 */
                    if (_send_glo_next && _has_glo) {
                        message = _latest_glo_msg;
                        _has_glo = false;
                        has_message = true;
                        _send_glo_next = false;
                        _last_non_obs_sent = now_t;
                    } else if (_has_base) {
                        /* GLO가 없어도 BASE는 전송 */
                        message = _latest_base_msg;
                        _has_base = false;
                        has_message = true;
                        _send_glo_next = true;
                        _last_non_obs_sent = now_t;
                    }
                }
            }

            if (has_message) {
                printf("lora send msgid:%d\n", message.msgid);
                mavlink_msg_to_send_buffer(lora_buffer, &message);
                memcpy(packet, _header, sizeof(_header));
                memcpy(packet + sizeof(_header), lora_buffer, sizeof(lora_buffer));
                ssize_t bytes_written = write(_lora_serial_fd, packet, sizeof(packet));
                if (bytes_written < 0) {
                    perror("Failed to write to serial port\n");
                    _running = false;
                    break;
                }
                before_t = time_ms();
                if (tcdrain(_lora_serial_fd) != 0) {
                    perror("Failed to flush serial port\n");
                    _running = false;
                    break;
                }
            }
        }
        usleep(1000);
    }
}

int main(int argc, char ** argv)
{
    long long before, now;
    int status, buflen;
    uint8_t buffer[MAXBUF];

    broadcast_setup();
    before = time_ms();

    mavlink_message_t message;

    if (-1 == parse_options(argc, argv))
    {
        printf("EXITING...\n");
        return -1;
    }
    printf("piksi port : %s\n", _piksi_options.device);
    printf("lora port : %s\n", _lora_options.device);
    /* serial setup */
    _g_serial_fd = setupSerial(_piksi_options.device, _piksi_options.baudrate);

    if ( _g_serial_fd < 0 ) {
        printf("ERROR: cannot use piksi serial fd (%d) %s \n", _g_serial_fd, strerror(errno));
        return -1;
    }

    _lora_serial_fd = setupSerial(_lora_options.device, _lora_options.baudrate);
    if ( _lora_serial_fd < 0 ) {
        printf("ERROR: cannot use lora serial fd (%d) %s \n", _lora_serial_fd, strerror(errno));
        return -1;
    }

    /* sbp setup */
    sbp_setup();

    std::thread lora_thread(write_to_lora);

    /* poll descriptor */
    struct pollfd fds[1];
    fds[0].fd = _g_serial_fd;
    fds[0].events = POLLIN;

    /* main thread loop */
    int cnt_timeout = 0;
    int cnt_err = 0;

    while (1) {
        now = time_ms();
        int status = poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);

        if ( status > 0 ) {
            cnt_timeout = 0;
            cnt_err = 0;
            if (now - before > 80 && !_obs_vec.empty()){
                mavlink_msg_piksi_obs_encode(255, 51, &message, &_obs_vec.back());
                buflen = mavlink_msg_to_send_buffer(buffer, &message);
                int sinlen = sizeof(struct sockaddr_in);
                status = sendto(_sock, buffer, buflen, 0, (struct sockaddr *)&_sock_in, sinlen);
                _obs_vec.pop_back();
                before = time_ms();
            }
            if (fds[0].revents & POLLIN) {
                s8 ret = sbp_process(&_sbp_state, &fifo_read);

                if ( ret == SBP_OK_CALLBACK_EXECUTED ) {
                    //printf("OK >>> %d %x \n", _sbp_state.sender_id, _sbp_state.msg_type);
                }
                else if (ret == SBP_OK_CALLBACK_UNDEFINED ) {
                    //printf("ERROR, %d\n", _sbp_state.msg_type);
                }
                else if (ret < 0) {
                    printf("ERROR %d %d %s \n", ret, _sbp_state.msg_type, strerror(errno));
                }
            }
        }
        else if ( status == 0 )  {
            printf("poll:timeout (%d)\n", cnt_timeout);
            cnt_timeout++;
        }
        else {
            printf("poll:error (%d)\n", cnt_err);
            cnt_err++;
        }

        if ( cnt_timeout > 5 || cnt_err > 5 ) {
            _running = false;
            lora_thread.join();
            return -1;
        }

    }
    lora_thread.join();
    return 0;
}
