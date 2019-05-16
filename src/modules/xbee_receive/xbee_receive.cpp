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
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <px4_time.h>
// xbee driver needed
#include <sys/select.h>
#include "sys/time.h"



/**
 * @file xbee_receive.cpp
 * @author Tonser <sundxfansky@sjtu.edu.cn>
 * v1.0 2019.5.14
 *
 * xbee驱动 通过串口数据 发布uORB消息
 *
 * This driver publish xbee_receive uorb data.
 */
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;

//same to xbee_send.cpp setting
short xbee_data_size=14;
//data buffer,size same to xbee_data_size
short xbee_data_in[14];


extern "C" __EXPORT int xbee_receive_main(int argc, char *argv[]);
int xbee_receive_thread_main(int argc, char *argv[]);
unsigned short crc_update (unsigned short crc, unsigned char data);
unsigned short crc16(void* data, unsigned short cnt);
short getlength(short s,short e);
short incindex(short num,short inc);
static int uart_init(const char * uart_name);    //
static int set_uart_baudrate(const int fd, unsigned int baud); //static
static void usage(const char *reason);            //static
int serial_receive(int file_descriptor, char *buffer,size_t data_len);
int parse_xbee_data(int fd, short *data, short send_size);
orb_advert_t mavlink_log_pub_DEBUG = NULL;

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
 * receive data
 *
 * @param file_descriptor file descriptor of serial port device file
 * @param buffer   buffer to receive data
 * @param data_len max data length
 *
 * @return positive integer of bytes received if success.
 *  return -1 if failed.
 */
int serial_receive(int file_descriptor, char *buffer,size_t data_len)
{
    int len,fs_sel;
    fd_set fs_read;
    struct timeval time;
    int fd = file_descriptor;

    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);

    time.tv_sec = 10;
    time.tv_usec = 0;

    //use select() to achieve multiple channel communication
    fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
    if ( fs_sel ) {
        len = read(fd, buffer, data_len);
        return len;
    } else {
        return -1;
    }
}

unsigned short crc_update (unsigned short crc, unsigned char data)
{
    data ^= (crc & 0xff);
    data ^= data << 4;

    return ((((unsigned short )data << 8) | ((crc>>8)&0xff)) ^ (unsigned char )(data >> 4)
            ^ ((unsigned short )data << 3));
}

unsigned short crc16(void* data, unsigned short cnt)
{
    unsigned short crc=0xff;
    unsigned char * ptr=(unsigned char *) data;
    int i;

    for (i=0;i<cnt;i++)
    {
        crc=crc_update(crc,*ptr);
        ptr++;
    }
    return crc;

}

short getlength(short s,short e)
{
    short data_length=0;
    if(s<=e)
    {
        data_length= e - s;
    }
    else
    {
        data_length = e + 256 - s;
    }
    return data_length;
}

short incindex(short num,short inc)
{
    num=num+inc;
    if(num>256-1)
    {
        num=num-256;
    }
    return num;
}

int parse_xbee_data(int fd, short *data, short send_size){
    int size = 0;
    static char data_in[256];
    char data_buf[256];
    char recbuf[32];
    short sync = 0;
    short i=0, j=0, k=0;;
    unsigned short* crc = NULL;
    short* data_ptr = NULL;
    short BytesReceived;
    static short data_out_store[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    short data_new_flag = 1;
    short data_length=0;
    static short starti=0;
    static short endi=0;
    size = send_size*2+3+3;
    data_length=getlength(starti,endi);
    BytesReceived = serial_receive(fd,data_buf,size*2);
    if(BytesReceived<1)
    {
        return 0;
    }
    {
        if(data_length+BytesReceived>256-1)
        {
            endi=0;starti=0;
        }
        j=0;i=endi;
        for(k=0;k<BytesReceived;k++)
        {
            if(i>256-1)
            {
                i=i-256;
            }
            data_in[i]=data_buf[j];
            j++;
            i++;
        }
        endi=i;
        data_length=getlength(starti,endi);
        if(data_length<size)
        {
            data_new_flag = 0;
        }
        else
        {
            data_new_flag = 1;
        }
    }

    i=starti;

    while ((sync!=3) && (data_new_flag == 1))
    {
        if (sync==0)
        {
            if (data_in[i]=='>')
            {
                sync=1;
                starti=i;
            }
            else
            {
                sync=0;
            }
        }
        else if (sync==1)
        {
            if (data_in[i]=='*')
            {
                sync=2;
            }
            else
            {
                sync=0;
                incindex(starti,1);
            }
        }
        else if (sync==2)
        {
            if (data_in[i]=='>')
            {
                sync=3;
            }
            else
            {
                sync=0;
                incindex(starti,1);
            }
        }

        data_length=getlength(starti,endi);
        if(sync==3&&data_length<size)
        {
            data_new_flag = 0;
        }
        i=incindex(i,1);
        if(i==endi)
        {
            starti=i;
            data_new_flag=0;
        }
    }
    // char descriptor = 0;
    if (data_new_flag == 1)
    {
        j=starti;
        for(k=0;k<size;k++)
        {
            recbuf[k]=data_in[j];
            j++;
            if(j>256-1)
            {
                j=j-256;
            }
        }
        i=3;
        // descriptor = recbuf[i];
        i=i+1;
        data_ptr = (short*) &(recbuf[i]);
        i = i+send_size*2;
        crc = (unsigned short*)&(recbuf[i]);

        if (crc16(data_ptr, send_size*2) == *crc)
        {
            memcpy(xbee_data_in, data_ptr, sizeof(data_out_store));
            starti=incindex(starti,size);
            return 1;
        }

    }
    return 0;
}

int xbee_receive_main(int argc, char *argv[])
{

    mavlink_log_info(&mavlink_log_pub_DEBUG,"[inav] xbee_receive init");
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
        daemon_task = px4_task_spawn_cmd("xbee_receive",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2500,
                         xbee_receive_thread_main,
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

int xbee_receive_thread_main(int argc, char *argv[])
{
    mavlink_log_info(&mavlink_log_pub_DEBUG,"xbee_receive run");

    int uart_read = uart_init("/dev/ttyS6");//fmuv5 ttys3 fmuv2,v3 ttys6
    if(false == uart_read)
    {
        mavlink_log_critical(&mavlink_log_pub_DEBUG,"[YCM]uart init is failed\n");
        return -1;
    }
    if(false == set_uart_baudrate(uart_read,57600)){
        // xbee baudrate 57600
        mavlink_log_critical(&mavlink_log_pub_DEBUG,"[YCM]set_uart_baudrate is failed\n");
        return -1;
    }
    mavlink_log_info(&mavlink_log_pub_DEBUG,"[YCM]xbee_receive init is successful\n");
    thread_running = true;
    // 定义话题结构
    struct vehicle_attitude_setpoint_s attspt_data;
    // 初始化数据
    memset(&attspt_data, 0 , sizeof(attspt_data));

    //公告消息
    orb_advert_t xbee_data_handle = orb_advertise(ORB_ID(vehicle_attitude_setpoint)
            , &attspt_data);//公告这个主题

    while(thread_running) {
        uint64_t timestamp = hrt_absolute_time();
        int serial_status = parse_xbee_data(uart_read, xbee_data_in, xbee_data_size);
        if (serial_status > 0) {
            attspt_data.roll_body = (float) (xbee_data_in[0]) / 10000.0f;
            attspt_data.pitch_body = (float) (xbee_data_in[1]) / 10000.0f;
            attspt_data.yaw_body = (float) (xbee_data_in[2]) / 10000.0f;
            attspt_data.thrust = (float) (xbee_data_in[3]) / 10000.0f;
            attspt_data.timestamp = timestamp;

            // int secs[2];
            // memcpy(secs,&xbee_data_in[10],sizeof(secs));
            // // secs[0]=msg.header.stamp.sec;
            // // secs[1]=msg.header.stamp.nsec;
            // attspt_data.timestamp = (float)secs[0]+(float)[1]secs/1000000.0f

            orb_publish(ORB_ID(vehicle_attitude_setpoint), xbee_data_handle, &attspt_data);
            PX4_INFO("serial status:%d ,timestamp :%d ,data: %\t8.5f, %\t8.5f, %\t8.5f,thrust: %\t8.5f",
                     serial_status,
                     (int)attspt_data.timestamp,
                     (double)attspt_data.roll_body,
                     (double)attspt_data.pitch_body,
                     (double)attspt_data.yaw_body,
                     (double)attspt_data.thrust);
            // PX4_INFO("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
            //          xbee_data_in[0], xbee_data_in[1], xbee_data_in[2], xbee_data_in[3], xbee_data_in[4], xbee_data_in[5],
            //          xbee_data_in[6], xbee_data_in[7], xbee_data_in[8], xbee_data_in[9],
            //          xbee_data_in[12], xbee_data_in[13]);
        }
    }

    thread_running = false;
    close(uart_read);
    fflush(stdout);
    return 0;
}
