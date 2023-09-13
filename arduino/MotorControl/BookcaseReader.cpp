#include"BookcaseReader.h"

BookcaseReader::BookcaseReader(HardwareSerial& serial, ros::NodeHandle& nh)
: _serial(serial), _count(0), bluetooth_input_pub("bluetooth_input", &pub_data)
{
  this->_nh = &nh;
};

void BookcaseReader::init(int baudrate)
{
  _serial.begin(baudrate);
  this->_nh->advertise(bluetooth_input_pub);
}

void BookcaseReader::read()
{
  if(_serial.available())
  {
    read_data = _serial.readStringUntil(' ');
    if(read_data =="reset"){
      pub_data.data = read_data.c_str();
      _count = 0;
      bluetooth_input_pub.publish(&pub_data);
    }
    else if(read_data.substring(0,4) == "book"){
      _count++;
      read_data.concat("#");
      read_data.concat(_count);
      pub_data.data = read_data.c_str();
      bluetooth_input_pub.publish(&pub_data); // publish data : bookN#count
    }
    clear_msg();
  }
}

void BookcaseReader::clear_msg()
{
  read_data.remove(0);
  pub_data.data = "";
}