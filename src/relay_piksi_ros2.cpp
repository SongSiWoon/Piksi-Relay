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

#include <algorithm>
#include <chrono>
#include <array>

#include <mavlink.h>
#include <vector>

#define MSG_OBS_HEADER_SEQ_SHIFT 4u
#define MSG_OBS_HEADER_SEQ_MASK ((1 << 4u) - 1)

#define MAXBUF		    (1024)
#define BUFFER_LENGTH 2041
uint8_t mavbuf[BUFFER_LENGTH];

int g_serial_fd = 0;

int _sock = 0;
struct sockaddr_in _sock_in;

#define DEVICE "/dev/ttyACM1"
#define SLEEP_US 100
#define BAUDRATE B115200


struct options {
    char device[64] = DEVICE;
    int sleep_us = SLEEP_US;
    uint32_t baudrate = BAUDRATE;
} _options;

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

std::vector<mavlink_piksi_obs_t> mavlink_vec;

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

    // generate mavlink message
    mavlink_message_t message;
    mavlink_piksi_base_pos_ecef_t piksi_base_pos;
    piksi_base_pos.msg_type = SBP_MSG_BASE_POS_ECEF;
    piksi_base_pos.sender_id = sender_id;
    piksi_base_pos.len = len;
    memcpy(&piksi_base_pos.data, msg, len);
    mavlink_msg_piksi_base_pos_ecef_encode(255, 51, &message, &piksi_base_pos);

    // send data
    buflen = mavlink_msg_to_send_buffer(buffer, &message);
    int sinlen = sizeof(struct sockaddr_in);
    status = sendto(_sock, buffer, buflen, 0, (struct sockaddr *)&_sock_in, sinlen);
}

void obs_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
    u8 obs_in_msg = (len - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);
    printf("MSG_OBS 0x%0X %d (%d)\n", sender_id, obs_in_msg, len);

    if ( obs_in_msg == 0 ) {
        return;
    }

    mavlink_piksi_obs_t piksi_obs = {0, };
    piksi_obs.msg_type = SBP_MSG_OBS;
    piksi_obs.sender_id = sender_id;
    piksi_obs.len = len;
    memcpy(piksi_obs.data, msg, len);
    mavlink_vec.insert(mavlink_vec.begin(), piksi_obs);
}

void globiases_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
    int status, buflen;
    uint8_t buffer[MAXBUF];

    printf("MSG_GLO_BIASES  len:%d\n", len);
    _globiases = *(msg_glo_biases_t *)msg;

    mavlink_message_t message;
    mavlink_piksi_glo_biases_t piksi_glo_biases;
    piksi_glo_biases.msg_type = SBP_MSG_GLO_BIASES;
    piksi_glo_biases.sender_id = sender_id;
    piksi_glo_biases.len = len;
    memcpy(piksi_glo_biases.data, msg, len);
    mavlink_msg_piksi_glo_biases_encode(255, 51, &message, &piksi_glo_biases);

    buflen = mavlink_msg_to_send_buffer(buffer, &message);
    int sinlen = sizeof(struct sockaddr_in);
    status = sendto(_sock, buffer, buflen, 0, (struct sockaddr *)&_sock_in, sinlen);
}


void sbp_setup(void)
{
    /* SBP parser state must be initialized before sbp_process is called. */
    sbp_state_init(&_sbp_state);
    printf("test\n");
    /* register callback function */
    sbp_register_callback(&_sbp_state, SBP_MSG_OBS, &obs_callback, NULL, &_obs_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL, &_heartbeat_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_BASE_POS_ECEF, &basepos_callback, NULL, &_basepos_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_GLO_BIASES, &globiases_callback, NULL, &_globiases_callback_node);
}

int setupSerial()
{
    int uart = open(_options.device, O_RDWR | O_NOCTTY | O_NDELAY  /*O_NONBLOCK*/);
    if (uart < 0) {
        printf("FAIL: Error opening port");
        return -1;
    }

    struct termios uart_config;
    tcgetattr(uart, &uart_config);

    uart_config.c_cflag |= CLOCAL | CREAD | CS8;
    uart_config.c_iflag = IGNPAR;
    uart_config.c_oflag = 0;
    uart_config.c_lflag = 0;

    if (cfsetispeed(&uart_config, _options.baudrate) < 0 || cfsetospeed(&uart_config, _options.baudrate) < 0 ) {
        printf("FAIL: Error setting baudrate / termios config for cfsetispeed ");
        return -1;
    }

    if (tcsetattr(uart, TCSANOW, &uart_config) < 0) {
        printf("FAIL: Error setting baudrate / termios config for tcsetattr");
        return -1;
    }

    return uart;

}

static u32 piksi_port_read(u8 *buff, u32 n, void *context)
{
    u32 result = 0;

    if ( g_serial_fd > 0 ) {
        result = read(g_serial_fd, buff, n);
    }

    return result;
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

    _sock_in.sin_addr.s_addr= inet_addr("10.42.0.255"); //htonl(-1); /* send message to 255.255.255.255 */
    _sock_in.sin_port = htons(9750); /* port number */
    _sock_in.sin_family = PF_INET;
}


s32 fifo_read(u8 *buff, u32 n, void *context)
{
    int i = 0;
    for ( i = 0 ; i < n  ; i++ ) {
        u32 readLen = read(g_serial_fd, buff+i, 1);
        if ( readLen <= 0 ) break;
    }

    return i;
}

static int parse_options(int argc, char **argv)
{
    int ch;

    while ((ch = getopt(argc, argv, "d:b:")) != EOF)
    {
        switch (ch)
        {
            case 'd': if (nullptr != optarg) strcpy(_options.device, optarg); break;
            case 'w': _options.sleep_us       = strtol(optarg, nullptr, 10);  break;
            case 'b': _options.baudrate       = strtoul(optarg, nullptr, 10); break;
                return -1;
        }
    }

    return 0;
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

    /* serial setup */
    g_serial_fd = setupSerial();
    if ( g_serial_fd < 0 ) {
        printf("ERROR: cannot use serial fd (%d) %s \n", g_serial_fd, strerror(errno));
        return -1;
    }

    /* sbp setup */
    sbp_setup();
// piksi_rtk start
    /* poll descriptor */
    struct pollfd fds[1];
    fds[0].fd = g_serial_fd;
    fds[0].events = POLLIN;

    /* main thread loop */
    int cnt_timeout = 0;
    int cnt_err = 0;
    while (1) {
        now = time_ms();
        int status = poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);

        if ( status > 0 ) {
            if (now - before > 100 && !mavlink_vec.empty()){
                mavlink_msg_piksi_obs_encode(255, 51, &message, &mavlink_vec.back());
                // printf("WTF : %ld\n", mavlink_vec.size());
                buflen = mavlink_msg_to_send_buffer(buffer, &message);
                int sinlen = sizeof(struct sockaddr_in);
                status = sendto(_sock, buffer, buflen, 0, (struct sockaddr *)&_sock_in, sinlen);
                mavlink_vec.pop_back();
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
            return -1;
        }

    }

    return 0;
}