#include"BookcaseReader.h"

BookcaseReader::BookcaseReader(HardwareSerial& serial, ros::NodeHandle& nh)
: _serial(serial), bluetooth_input_pub("bluetooth_input", &pub_data)
{
  this->_nh = &nh;
};

void BookcaseReader::init(const int baudrate)
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
      bluetooth_input_pub.publish(&pub_data);
    }
    else if(read_data.substring(0,4) == "book"){
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
