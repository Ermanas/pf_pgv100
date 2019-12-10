// This topic is operating the P+F PGV100
//
// Published message variable units
//      - msg.angle ------------| degree
//      - msg.x_pos ------------| millimeter
//      - msg.y_pos ------------| millimeter
//      - msg.direction --------| string;
//      - msg.color_lane_count -| integer;
//      - msg.no_color_lane ----| boolean;
//      - msg.no_pos -----------| boolean;
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pf_pgv100/pgv_scan_data.h>
// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWRs
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <bitset>
#include <string>
#include <stdlib.h>     /* strtoull */
#include <signal.h>
#include <sstream>
#include<cmath> // to use pow

using namespace std;

// Set the port which your device is connected.

//using namespace std;
// To handle CTRL + C Interrupt
void my_handler(int s);
unsigned long int string2decimal(string input);
int serial_port = open("/dev/ttyUSB0", O_RDWR);

int main(int argc, char **argv)
{
  
   // To handle CTRL + C Interrupt
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
      

// Check for errors
if (serial_port < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
}

// Create new termios struc, we call it 'tty' for convention
struct termios tty;
memset(&tty, 0, sizeof tty);

// Read in existing settings, and handle any error
if(tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
}

// Serial Communication Configuration
tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
tty.c_cflag |= CS7; // 8 bits per byte (most common)
tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
tty.c_lflag &= ~ICANON;
tty.c_lflag &= ~ECHO; // Disable echo
tty.c_lflag &= ~ECHOE; // Disable erasure
tty.c_lflag &= ~ECHONL; // Disable new-line echo
tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
tty.c_cc[VMIN] = 2;
// Set in/out baud rate to be 115200
cfsetispeed(&tty, B115200);
cfsetospeed(&tty, B115200);

// Save tty settings, also checking for error
if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
}   

double agv_x_pos_des = 0.0;
unsigned char dir_straight[ 2 ] = {0xEC, 0x13}; // Straight ahead 
unsigned char pos_req[ 2 ] = { 0xC8, 0x37}; // Position Request
string selected_dir = "Straight ahead";
write(serial_port,  dir_straight , sizeof( dir_straight));

ROS_INFO(" Direction set to <> Straight Ahead <>");
  ros::init(argc, argv, "pgv100_node");
  ros::NodeHandle n;
  ros::Publisher pgv100_pub = n.advertise<pf_pgv100::pgv_scan_data>("pgv100", 100); // second parameter is buffer
  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

  int count = 0;
  while (ros::ok())
  {

    write(serial_port, pos_req , 2);
    char read_buf [21];
    memset(&read_buf, '\0', sizeof(read_buf));
    int byte_count = read(serial_port, &read_buf, sizeof(read_buf));
    
        // Get Lane-Detection from the byte array [Bytes 1-2]
    bitset<7> lane_detect_byte1(read_buf[0]);
    bitset<7> lane_detect_byte0(read_buf[1]);
    string agv_lane_detect_str = lane_detect_byte1.to_string() + lane_detect_byte0.to_string();
    //cout << agv_lane_detect_str << endl;
    string agv_c_lane_count_str = agv_lane_detect_str.substr(8, 2);
    string agv_c_lane_detect_str = agv_lane_detect_str.substr(11, 1);
    string agv_no_pos_str = agv_lane_detect_str.substr(5, 1);
    string tag_detected = agv_lane_detect_str.substr(7,1);
    int agv_c_lane_count_des = string2decimal(agv_c_lane_count_str);
    int agv_no_color_lane_des = string2decimal(agv_c_lane_detect_str);
    int agv_no_pos_des = string2decimal(agv_no_pos_str);
    int tag_detected_des = string2decimal(tag_detected); 

    // Get the angle from the byte array [Byte 11-12]
    bitset<7> ang_1(read_buf[10]);
    bitset<7> ang_0(read_buf[11]);
    string agv_ang_str = ang_1.to_string() + ang_0.to_string();
    double agv_ang_des = string2decimal(agv_ang_str)/(10.0);

    // Get the X-Position from the byte array [Bytes 3-4-5-6]
    bitset<3> x_pos_3(read_buf[2]);
    bitset<7> x_pos_2(read_buf[3]);
    bitset<7> x_pos_1(read_buf[4]);
    bitset<7> x_pos_0(read_buf[5]);
    string agv_x_pos_str = x_pos_3.to_string() + x_pos_2.to_string() + x_pos_1.to_string() + x_pos_0.to_string();
    agv_x_pos_des = string2decimal(agv_x_pos_str);
    if(tag_detected_des != 0){
      if (agv_x_pos_des > 2000.0) // this makes x-pos zero centered
        agv_x_pos_des = agv_x_pos_des - pow(2,24) - 1;
    }
    // Get Y-Position from the byte array [Bytes 7-8]
    bitset<7> y_pos_1(read_buf[6]);
    bitset<7> y_pos_0(read_buf[7]);
    string agv_y_pos_str = y_pos_1.to_string() + y_pos_0.to_string();
    double agv_y_pos_des = string2decimal(agv_y_pos_str);
    if (agv_y_pos_des > 2000.0) // this makes y-pos zero centered
      agv_y_pos_des = agv_y_pos_des - 16383.0;
    // We get opposite values when we try the read the y-pos value from the colored and code strip.
    // So this is checking which strip that we're reading.
    if(agv_no_pos_des)
    agv_y_pos_des *= -1;

    pf_pgv100::pgv_scan_data msg;
    msg.angle = agv_ang_des; // degree
    msg.x_pos = agv_x_pos_des/10.0; // mm
    msg.y_pos = agv_y_pos_des/10.0; // mm
    msg.direction = selected_dir;
    msg.color_lane_count = agv_c_lane_count_des;
    msg.no_color_lane = agv_no_color_lane_des;
    msg.no_pos = agv_no_pos_des;
    msg.tag_detected = tag_detected_des;

    pgv100_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    sigaction(SIGINT, &sigIntHandler, NULL);
  }
  return 0;
}

void my_handler(int s){
           close(serial_port);
           printf("\n\nCaught signal %d\n Port closed.\n",s);
           exit(1); 
}

unsigned long int string2decimal(string input){
    int strlength = input.length();
    char input_char[strlength + 1];
    strcpy(input_char, input.c_str());
    char *pEnd;
    unsigned long int out_dec = strtoull(input_char, &pEnd, 2);
    return out_dec;
}