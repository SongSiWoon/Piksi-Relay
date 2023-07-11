#include <stdio.h>   /* Standard input/output definitions  */
#include <string.h>  /* String function definitions */
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

//
//#include <rclcpp/rclcpp.hpp>
//#include <std_msgs/msg/string.hpp>
//#include <px4_msgs/msg/piksi_data.hpp>
//#include <px4_msgs/msg/piksi_rtk.hpp>
//#include <px4_msgs/msg/piksi_pos_llh.hpp>
#include <mavlink.h>

#define MSG_OBS_HEADER_SEQ_SHIFT 4u
#define MSG_OBS_HEADER_SEQ_MASK ((1 << 4u) - 1)

#define MAXBUF		    (1024)
#define BUFFER_LENGTH 2041

#define DEVICE "/dev/ttyACM1"
#define SLEEP_US 100
#define BAUDRATE B115200
#define POSLLH_AVG_NUM 5

#define BASESTATION_LAT (36.375476014384994)
#define BASESTATION_LON (127.35468387653582)
#define BASESTATION_HGT (128.03895203170944)

int g_serial_fd = 0;
mavlink_message_t mavmsg;

struct sockaddr_in gcAddr;
int bytes_sent;

int _sock = 0;
struct sockaddr_in _sock_in;

uint16_t mavlen;

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
msg_base_pos_llh_t      _base_pos_llh;
int                     _pos_llh_count;


sbp_msg_callbacks_node_t _heartbeat_callback_node;
sbp_msg_callbacks_node_t _obs_callback_node;
sbp_msg_callbacks_node_t _basepos_callback_node;
sbp_msg_callbacks_node_t _globiases_callback_node;
sbp_msg_callbacks_node_t _pos_llh_callback_node;

/* ROS */
//rclcpp::Node::SharedPtr         mRelayPiksiNode;
//rclcpp::Publisher<px4_msgs::msg::PiksiData>::SharedPtr mPiksiDataPub_;          // Obs -> Data
//rclcpp::Publisher<px4_msgs::msg::PiksiPosLLH>::SharedPtr mPiksiPosLLHPub_;
//
//std::vector<px4_msgs::msg::PiksiData> _ros_msgs;
//px4_msgs::msg::PiksiPosLLH posllh_avg_msg;
//
//rclcpp::Clock ros_clock(RCL_ROS_TIME);

long long time_ms()
{
    struct timeval te;
    gettimeofday(&te, NULL);
    long long ms = te.tv_sec*1000LL + te.tv_usec/1000;
    return ms;
}


long long _prev_time = 0L;
clock_t _prev_clock = 0;

void obs_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
    u8 obs_in_msg = (len - sizeof(observation_header_t)) / sizeof(packed_obs_content_t);
//    printf("%d %d %d\n", len, sizeof(observation_header_t), sizeof(packed_obs_content_t));
//    if ( obs_in_msg == 0 ) return;


    int status, buflen;
    uint8_t mavbuf[BUFFER_LENGTH];

//    px4_msgs::msg::PiksiData rosmsg;
//    rosmsg.timestamp = ros_clock.now().nanoseconds();
//    rosmsg.msg_type = SBP_MSG_OBS;
//    rosmsg.sender_id = sender_id;
//    rosmsg.len = len;
    uint8_t msgarr[215];
//    std::array<uint8_t, 215> msg_vec;
    for ( int i = 0 ; i < 215 ; i++ ) {
        msgarr[i] = msg[i];
        printf("%02x ", (unsigned char)msg[i]);
    }
    printf("\n");
//    printf("%d %d\n", sender_id, SBP_MSG_OBS);
//    printf("%x %x\n", sender_id, SBP_MSG_OBS);
    mavlink_msg_piksi_obs_pack(1, 200, &mavmsg, sender_id, SBP_MSG_OBS, msgarr);
    mavlen = mavlink_msg_to_send_buffer(mavbuf, &mavmsg);
    bytes_sent = sendto(_sock, mavbuf, mavlen, 0, (struct sockaddr*)&_sock_in, sizeof (struct sockaddr_in));

//    _ros_msgs.insert(_ros_msgs.begin(), rosmsg);


    // debug message
    int total = (msg[10] & 0xF0) >> 4;
    int idx = msg[10] & 0x0F;
    static int num_sat = 0;
//    if ( idx == 0 ) {
//        printf("[%f] 0x%0X : ", (double)rosmsg.timestamp/1e9, sender_id);
//    }
//    num_sat += obs_in_msg;
//    printf(" / %d", obs_in_msg);
//    if (total == idx + 1 ) {
//        printf(" (%d)\n",num_sat);
//        num_sat = 0;
//    }
}

//void globiases_callback(u16 sender_id, u8 len, u8 msg[], void *context)
//{
//    int status, buflen;
//    char buffer[MAXBUF];
//
//    _globiases = *(msg_glo_biases_t *)msg;
//
//    px4_msgs::msg::PiksiData rosmsg;
//    rosmsg.msg_type = SBP_MSG_GLO_BIASES;
//    rosmsg.sender_id = sender_id;
//    rosmsg.len = len;
//
//    std::array<uint8_t, 215> msg_vec;
//    for ( int i = 0 ; i < 215 ; i++ ) {
//        msg_vec[i] = msg[i];
//    }
//    rosmsg.data = msg_vec;
//
//    _ros_msgs.insert(_ros_msgs.begin(), rosmsg);
//}

void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
    _pos_llh = *(msg_pos_llh_t *)msg;
    uint8_t mavbuf[BUFFER_LENGTH];

    mavlink_msg_piksi_pos_llh_pack(1, 200, &mavmsg, _pos_llh.tow, _pos_llh.lat, _pos_llh.lon, _pos_llh.height,
                                   _pos_llh.h_accuracy, _pos_llh.v_accuracy, _pos_llh.n_sats, _pos_llh.flags);
    mavlen = mavlink_msg_to_send_buffer(mavbuf, &mavmsg);
    bytes_sent = sendto(_sock, mavbuf, mavlen, 0, (struct sockaddr*)&_sock_in, sizeof (struct sockaddr_in));

}

void sbp_setup(void)
{
    /* SBP parser state must be initialized before sbp_process is called. */
    sbp_state_init(&_sbp_state);

    /* register callback function */
    sbp_register_callback(&_sbp_state, SBP_MSG_OBS, &obs_callback, NULL, &_obs_callback_node);
//    sbp_register_callback(&_sbp_state, SBP_MSG_OBS, &obs_callback, NULL, &_obs_callback_node);
    sbp_register_callback(&_sbp_state, SBP_MSG_POS_LLH, &pos_llh_callback, NULL, &_pos_llh_callback_node);
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
    uart_config.c_iflag  = IGNPAR;
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

    if (-1 == parse_options(argc, argv))
    {
        printf("EXITING...\n");
        return -1;
    }

    /* ros init */
//    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
//    rclcpp::init(argc, argv);
//    mRelayPiksiNode = rclcpp::Node::make_shared("relay_piksi_ros2");

//    auto timer_callback =
//            []() -> void
//            {
//                // printf("timer_callback!\n");
//                if ( _ros_msgs.empty() != true ) {
//                    px4_msgs::msg::PiksiData rosmsg = _ros_msgs.back();
//                    mPiksiDataPub_->publish(rosmsg);
//                    _ros_msgs.pop_back();
//                }
//            };

//    std::chrono::milliseconds period(_options.sleep_us);
//    auto timer = mRelayPiksiNode->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), timer_callback);

    /* publish setup */
//    mPiksiDataPub_ = mRelayPiksiNode->create_publisher<px4_msgs::msg::PiksiData>("/piksi_data", 10);
//    mPiksiPosLLHPub_ = mRelayPiksiNode->create_publisher<px4_msgs::msg::PiksiPosLLH>("/piksi_posllh", 10);

    /* serial setup */
    g_serial_fd = setupSerial();
    if ( g_serial_fd < 0 ) {
        printf("ERROR: cannot use serial fd (%d) %s \n", g_serial_fd, strerror(errno));
        return -1;
    }

    /* sbp setup */
    sbp_setup();

    /* poll descriptor */
    struct pollfd fds[1];
    fds[0].fd = g_serial_fd;
    fds[0].events = POLLIN;

    /* main thread loop */
    int cnt_timeout = 0;
    int cnt_err = 0;
    while (1) {
        int status = poll(fds, sizeof(fds) / sizeof(fds[0]), 1000);
        if ( status > 0 ) {
            if (fds[0].revents & POLLIN) {
                s8 ret = sbp_process(&_sbp_state, &fifo_read);
                if ( ret == SBP_OK_CALLBACK_EXECUTED ) {
//                    printf("OK >>> %d %x \n", _sbp_state.sender_id, _sbp_state.msg_type);
                }
                else if (ret == SBP_OK_CALLBACK_UNDEFINED ) {
//                    printf("ERROR, %d\n", _sbp_state.msg_type);
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

        // ros::spinOnce();
//        rclcpp::spin_some(mRelayPiksiNode);
    }

    return 0;
}
