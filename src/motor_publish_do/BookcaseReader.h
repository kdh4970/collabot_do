#ifndef BOOKCASEREADER_H
#define BOOKCASEREADER_H
#include<ros.h>
#include<std_msgs/String.h>

class BookcaseReader
{
public:
  BookcaseReader(HardwareSerial& serial, ros::NodeHandle& nh);
  ~BookcaseReader();
  void init(int baudrate);
  void read();
  
private:
  HardwareSerial& _serial;
  int _count;
  String read_data;
  std_msgs::String pub_data;
  ros::NodeHandle* _nh;
  ros::Publisher bluetooth_input_pub;
  void clear_msg();

};

#endif