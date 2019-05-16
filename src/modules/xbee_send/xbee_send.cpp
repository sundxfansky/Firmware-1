#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <math.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <px4_time.h>
// xbee driver needed
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
//uorb message
#include <uORB/topics/sensor_combined.h>




/**
 * @file xbee_send.cpp
 * @author Tonser <sundxfansky@sjtu.edu.cn>
 * v1.0 2019.5.14
 *
 * xbee驱动 通过串口数据 发布uORB消息
 *
 * This driver xbee_send send uorb data.
 */
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;

//same to xbee_send.cpp setting
short send_size=14;
//data buffer,size same to send_size
short data_out[14];


extern "C" __EXPORT int xbee_send_main(int argc, char *argv[]);
int xbee_send_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);    //
static int set_uart_baudrate(const int fd, unsigned int baud); //static
static void usage(const char *reason);            //static
int serial_send(int file_descriptor, char *buffer, size_t data_len);
unsigned short receive_crc_update (unsigned short crc, unsigned char data);
unsigned short receive_crc16(void* data, unsigned short cnt);
short xbee_send_message(int fd, const short * control_data,short send_size);
orb_advert_t mavlink_log_pub_DEBUG2 = NULL;

int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            PX4_INFO("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;


    tcgetattr(fd, &uart_config);

    uart_config.c_oflag &= ~ONLCR;

    uart_config.c_cflag &= ~(CSTOPB | PARENB);

    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        PX4_INFO("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        PX4_INFO("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        PX4_INFO("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "WARN: lose para,use {start|stop|status} [param]\n\n");
    exit(1);
}

/**
 * send data
 *
 * @param file_descriptor file descriptor of serial port device file
 * @param buffer   buffer of data to send
 * @param data_len data length
 *
 * @return positive integer of bytes sent if success.
 *  return -1 if failed.
 */
int serial_send(int file_descriptor, char *buffer, size_t data_len)
{
    size_t len = 0;

    len = write(file_descriptor, buffer, data_len);
    if ( len == data_len ) {
        return len;
    } else {
        tcflush(file_descriptor, TCOFLUSH);
        return -1;
    }

}

unsigned short receive_crc_update (unsigned short crc, unsigned char data)
{
    data ^= (crc & 0xff);
    data ^= data << 4;

    return ((((unsigned short )data << 8) | ((crc>>8)&0xff)) ^ (unsigned char )(data >> 4)
            ^ ((unsigned short )data << 3));
}

unsigned short receive_crc16(void* data, unsigned short cnt)
{
    unsigned short crc=0xff;
    unsigned char * ptr=(unsigned char *) data;
    int i;

    for (i=0;i<cnt;i++)
    {
        crc=receive_crc_update(crc,*ptr);
        ptr++;
    }
    return crc;

}

short xbee_send_message(int fd, const short * control_data,short xbee_send_size)
{
    char send_data[xbee_send_size*2+6];
    unsigned short crc = 0;
    short send_switch=1;
    short BytesSent=0;
    if (send_switch)
    {
        send_data[0] = '>';
        send_data[1] = '*';
        send_data[2] = '>';
        send_data[3] = 'c';
        memcpy(&send_data[4], control_data, xbee_send_size*2);
        crc = receive_crc16(&send_data[4], send_size*2);
        memcpy(&send_data[xbee_send_size*2+4], &crc, 2);

        BytesSent=serial_send(fd, send_data, xbee_send_size*2+6);
        return BytesSent;
    }
    else
    {
        return 0;
    }
}

int xbee_send_main(int argc, char *argv[])
{

    mavlink_log_info(&mavlink_log_pub_DEBUG2,"[inav] xbee_send init");
    if (argc < 2)
    {
        usage("[YCM]missing command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            PX4_INFO("[YCM]already running\n");
            exit(0);
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("xbee_send",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2500,
                         xbee_send_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        thread_running = false;
        return 0;
    }
    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("[YCM]running");

        } else {
            warnx("[YCM]stopped");
        }

       return 0;
    }

    usage("unrecognized command");
    return 1;
}

int xbee_send_thread_main(int argc, char *argv[])
{
    mavlink_log_info(&mavlink_log_pub_DEBUG2,"xbee_send run");

    int uart_read = uart_init("/dev/ttyS6");//fmuv5 ttys3 fmuv2,v3 ttys6
    if(false == uart_read)
    {
        mavlink_log_critical(&mavlink_log_pub_DEBUG2,"[YCM]uart init is failed\n");
        return -1;
    }
    if(false == set_uart_baudrate(uart_read,57600)){
        // xbee baudrate 57600
        mavlink_log_critical(&mavlink_log_pub_DEBUG2,"[YCM]set_uart_baudrate is failed\n");
        return -1;
    }
    mavlink_log_info(&mavlink_log_pub_DEBUG2,"[YCM]xbee_send init is successful\n");
    thread_running = true;
    /* subscribe to sensor_combined topic */
    int sensor_sub_handle = orb_subscribe(ORB_ID(sensor_combined));
    /* limit the update rate to 20 Hz */
    orb_set_interval(sensor_sub_handle, 50);
    // data
    struct sensor_combined_s raw;
    // 初始化数据
    memset(&raw, 0 , sizeof(raw));
    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[1];
    fds[0].fd=sensor_sub_handle;
    fds[0].events = POLLIN;
    int error_counter = 0;
    while(thread_running) {
        int poll_ret = px4_poll(fds,1,1000);
        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within a second");

        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }

            error_counter++;

        } else {
            if (fds[0].revents & POLLIN){
                orb_copy(ORB_ID(sensor_combined), sensor_sub_handle, &raw);
                data_out[0] = (short int)raw.accelerometer_m_s2[0]*10000;
                data_out[1] = (short int)raw.accelerometer_m_s2[1]*10000;
                data_out[2] = (short int)raw.accelerometer_m_s2[2]*10000;
                data_out[3] = (short int)raw.timestamp;
                PX4_INFO("send data:\t%8d\t%8d\t%8d,  time:%d",
                         (int)data_out[0],
                         (int)data_out[1],
                         (int)data_out[2],
                         (int)data_out[3]);
                xbee_send_message(uart_read,data_out,send_size);
            }
        }

    }


    thread_running = false;
    close(uart_read);
    fflush(stdout);
    return 0;
}
