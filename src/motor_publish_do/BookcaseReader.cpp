#include<BookcaseReader.h>

BookcaseReader::BookcaseReader()
: _count(0) {};

BookcaseReader::init(HardwareSerial& serial, int baudrate, ros::NodeHandle& nh)
{
  _serial=serial;
  _nh=nh; 
  _serial.begin(baudrate);
  bluetooth_input_pub = _nh.advertise<std_msgs::String>("bluetooth_input", 1);
}

BookcaseReader::read()
{
  if (_serial.available())
  {
    read_data = Serial2.readStringUntil(' ');
    if(read_data =="reset"){
      pub_data.data = read_data.c_str();
      _count = 0;
      // total_count.data = count; 
      bluetooth_input_pub.publish(&pub_data);
      // count_pub.publish(&total_count);
    }
    else if(read_data.substring(0,4) == "book"){
      _count++;
      read_data.concat("#");
      read_data.concat(_count);
      pub_data.data = read_data.c_str();
      // total_count.data = count;
      bluetooth_input_pub.publish(&pub_data); // publish data : book1#1
      // count_pub.publish(&total_count);
    }
    clear_msg();
  }

}

BookcaseReader::clear_msg()
{
  read_data.remove(0);
  pub_data.data = "";
}