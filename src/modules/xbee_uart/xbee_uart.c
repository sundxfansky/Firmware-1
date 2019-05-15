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
#include <uORB/topics/optical_flow.h>
//#include <uORB/topics/vehicle_attitude.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <px4_time.h>

#include <sys/select.h>



/**
 * @file xbee_uart.c
 * @author Tonser <sundxfansky@sjtu.edu.cn>
 * v1.0 2019.5.14
 *
 * xbee驱动 通过串口传递数据 发布ORB_ID(optical_flow)
 *
 * This driver publish optical flow data.
 */
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;
#define RECBUFSIZE 256;
#define SEND_SIZE 14;

__EXPORT int xbee_uart_main(int argc, char *argv[]);
int xbee_uart_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);    //
static int set_uart_baudrate(const int fd, unsigned int baud); //static
static void usage(const char *reason);            //static
//extern orb_advert_t mavlink_log_pub_DEBUG;
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
        data_length=e-s;
    }
    else
    {
        data_length=e+RECBUFSIZE-s;
    }
    return data_length;
}

short incindex(short num,short inc)
{
    num=num+inc;
    if(num>RECBUFSIZE-1)
    {
        num=num-RECBUFSIZE;
    }
    return num;
}

int parse_xbee_data(int fd, short *data, short send_size = 14){
    int size = 0;
    static char data_in[RECBUFSIZE];
    char data_buf[RECBUFSIZE];
    char recbuf[32];
    short sync = 0;
    short i=0, j=0, k=0;
    int readcount = 0;
    short intstate=0;

    int read_tmp = 0;
    char descriptor = 0;
    unsigned short* crc = NULL;
    short* data_ptr = NULL;
    unsigned long RxBytes, BytesReceived;
    static short data_out_store[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    static short status_store[] = {0,0};
//    short send_size = 14;
    short data_new_flag = 1;
    short data_length=0;

    static char data_rec_store[256];
    static int data_rec_count=0;
    static short data_sync=0;
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
        if(data_length+BytesReceived>RECBUFSIZE-1)
        {
            endi=0;starti=0;
        }
        j=0;i=endi;
        for(k=0;k<BytesReceived;k++)
        {
            if(i>RECBUFSIZE-1)
            {
                i=i-RECBUFSIZE;
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

    if (data_new_flag == 1)
    {
        j=starti;
        for(k=0;k<size;k++)
        {
            recbuf[k]=data_in[j];
            j++;
            if(j>RECBUFSIZE-1)
            {
                j=j-RECBUFSIZE;
            }
        }
        i=3;
        descriptor = recbuf[i];
        i=i+1;
        data_ptr = (short*) &(recbuf[i]);
        i = i+send_size*2;
        crc = (unsigned short*)&(recbuf[i]);

        if (crc16(data_ptr, send_size*2) == *crc)
        {
            memcpy(data_out, data_ptr, sizeof(data_out_store));
            starti=incindex(starti,size);
            return 1;
        }

    }
    return 0;
}

int xbee_uart_main(int argc, char *argv[])
{

mavlink_log_info(&mavlink_log_pub_DEBUG,"[inav] upixels_flow_main on init");
// mavlink_log_critical(mavlink_log_pub_DEBUG, "test>>>>>>> %8.4f",9.9898);
			
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
        daemon_task = px4_task_spawn_cmd("xbee_uart",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_DEFAULT,
                         2500,
                         xbee_uart_thread_main,
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

int xbee_uart_thread_main(int argc, char *argv[])
{
    mavlink_log_info(&mavlink_log_pub_DEBUG,"upixels_flow run ");
    char data = '0';
    short data_out[SEND_SIZE];
    int sec[2];
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
    mavlink_log_info(&mavlink_log_pub_DEBUG,"[YCM]uart init is successful\n");
    thread_running = true;
    // 定义话题结构
    struct optical_flow_s flow_data;
    // 初始化数据
    memset(&flow_data, 0 , sizeof(flow_data));
   
    //公告消息
//    orb_advert_t flow_data_handle = orb_advertise(ORB_ID(optical_flow), &flow_data);//公告这个主题
    //  orb_advert_t flow_data_handle = orb_advertise(ORB_ID(upixels_flow), &flow_data);//公告这个主题

    // 测试订阅程序
    // int test_sub_handle = orb_subscribe(ORB_ID(upixels_flow));
    // struct upixels_flow_s test_sub;

    // int counter = 0;
//    uint64_t _previous_collect_timestamp = hrt_absolute_time();
//    uint64_t _flow_dt_sum_usec = 0;
//    float scale = 1.3f;
//    int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    while(thread_running)
   {
        parse_xbee_data(uart_read,data_out,SEND_SIZE);
        memcpy(secs,&data_out[10],sizeof(secs));
        PX4_INFO("%X,%X,%X,%X,%X,%X"
                "%X,%X,%X,%X,%X,%X"
                "%X,%X",
                data_out[0],data_out[1],data_out[2],data_out[3],data_out[4],data_out[5],
                data_out[6],data_out[7],data_out[8],data_out[9],data_out[10],data_out[11],
                data_out[12],data_out[13]);

//        PX4_INFO("uart :%d",uart_read);
//    	  read(uart_read,&data,1);
//        if((data == 0xFE))
//        {
//            for(int k = 0;k < 14;++k){
//            data = '0';
//            read(uart_read,&data,1);
//            fuck_buffer[k] = data;
//            }
//            PX4_INFO("%X,%X,%X,%X,%X,%X"
//                     "%X,%X,%X,%X,%X,%X"
//                     "%X,%X",
//                    fuck_buffer[0],fuck_buffer[1],fuck_buffer[2],fuck_buffer[3],fuck_buffer[4],fuck_buffer[5],
//                     fuck_buffer[6],fuck_buffer[7],fuck_buffer[8],fuck_buffer[9],fuck_buffer[10],fuck_buffer[11],
//                     fuck_buffer[12],fuck_buffer[13]);

//
//            if((data == 0x0A))
//            {
//                PX4_INFO("status: sucesss4");
//                for(int k = 0;k < 6;++k)
//                {
//                data = '0';
//                read(uart_read,&data,1);
//                buffer[k] = data;
//                }
//                for(int k = 0;k < 3;++k)
//                {
//                    data = '0';
//                    read(uart_read,&data,1);
//                }
//                PX4_INFO("status: sucesss5");
//                buffer[6] = data;
//                uint64_t timestamp = hrt_absolute_time();
//                uint64_t dt_flow = timestamp - _previous_collect_timestamp;
//                _previous_collect_timestamp = timestamp;
//                _flow_dt_sum_usec += dt_flow;
//                // debug 输出数据 十六进制
//                // PX4_INFO("%X,%X,%X,%X,%X,%X,%X,",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6]);
//                //X 像素点累计时间内的累加位移,(radians*10000) [除以 10000 乘以高度(m)后为实际位移(m)]
//                upixels_flow_x = (float)((int16_t)(buffer[1]<<8|buffer[0])/10000.0f);// 单位是m
//                upixels_flow_y = (float)((int16_t)(buffer[3]<<8|buffer[2])/10000.0f);// 单位是m
//                integration_timespan = (float)((uint16_t)(buffer[5]<<8|buffer[4]));// 单位是us
//                //flow_data.pixel_flow_x_integral = (float)(upixels_flow_x*1000000.0f/integration_timespan);// rad/s
//                //flow_data.pixel_flow_y_integral = (float)(upixels_flow_y*1000000.0f/integration_timespan);// rad/s
//                flow_data.pixel_flow_x_integral = -upixels_flow_y*scale;
//                flow_data.pixel_flow_y_integral = upixels_flow_x*scale;
//                //orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
//                flow_data.gyro_x_rate_integral = 0.0f*integration_timespan;
//                flow_data.gyro_y_rate_integral = 0.0f;
//                flow_data.gyro_z_rate_integral = 0.0f;
//                flow_data.min_ground_distance = 0.5;//与官方optical_flow不同，官方数据为0.7
//                flow_data.max_ground_distance = 3;//与官方optical_flow相同
//                flow_data.max_flow_rate = 2.5;//与官方optical_flow相同,最大限制角速度
//                flow_data.sensor_id = 0;
//                flow_data.timestamp = timestamp;
//                flow_data.frame_count_since_last_readout = 1; //4;
//                flow_data.integration_timespan = _flow_dt_sum_usec;
//                _flow_dt_sum_usec = 0;
//                if (buffer[6] == 0xF5) {
//                    //数据可用
//                    flow_data.quality = 255;
//                    orb_publish(ORB_ID(optical_flow),flow_data_handle,  &flow_data);
//                }
//                else{
//                    flow_data.quality = 0;
//                    orb_publish(ORB_ID(optical_flow),flow_data_handle,&flow_data);
//                }
//            }
//        }
   }
    thread_running = false;
    //取消订阅
    // orb_unsubscribe(test_sub_handle);
    close(uart_read);
    fflush(stdout);
    return 0;
}
