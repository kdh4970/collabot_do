#ifndef BOOKCASEREADER_H
#define BOOKCASEREADER_H
#include<ros.h>
#include<std_msgs/String.h>

class BookcaseReader
{
public:
  BookcaseReader();
  void init(HardwareSerial& serial, int baudrate, ros::NodeHandle& nh);
  void read();
  
private:
  HardwareSerial& _serial;
  ros::NodeHandle& _nh;
  ros::Publisher bluetooth_input_pub;
  int _count;
  String read_data;
  std_msgs::String pub_data;
  void clear_msg();

}

#endif