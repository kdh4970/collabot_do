#define USE_USBCON  
// #include <std_msgs/Int32.h>
#include <DynamixelWorkbench.h>
#include "BookcaseReader.h"
#if defined(__OPENCM904__)
#define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
#define DEVICE_NAME ""
#endif

#define BAUDRATE  57600

#define MOTOR1  1
#define MOTOR2  2
#define MOTOR3  3
#define MOTOR4  4
#define MOTOR5  5
#define MOTOR6  6
#define MOTOR7  7
#define MOTOR8  8
#define MOTOR9  9

DynamixelWorkbench dxl_wb;

ros::NodeHandle nh;
BookcaseReader bookcaseReader(Serial2,nh);
// std_msgs::String state;
//make publisher
// ros::Publisher bookcase_num_pub("bookcase_num",  &moter_num);
// ros::Publisher count_pub("count",  &total_count);


// 바꿔야함.
/*
int isclose {};

void close_cb(const std_msgs::String& cmd_msg){
  //isclose = cmd_msg.data.c_str();
  if (state.data = "open"){
      String my_data = cmd_msg.data;
       //if (cmd_msg.data.c_str() == my_string){
        if (my_data == "diff"){
        isclose = 1;
        //delay(100); //if you want to turn on led aduino do7 
        digitalWrite(7,HIGH);
        }
  }
}

//make subscriber
ros::Subscriber<std_msgs::String> close_flag("change", close_cb); // same/diff
*/
uint16_t model_number = 0;
int32_t presentposition[13];
int initial_pos[13] = {0,};

bool motor_open[9] = {false,};
uint8_t motor[13] = {0, MOTOR1, MOTOR2, MOTOR3, MOTOR4, MOTOR5, MOTOR6, MOTOR7, MOTOR8, MOTOR9};
bool start_flag = false;


// Open the bookcase of given motor_num
void OpenBookcase(int motor_num){
  if(motor_num > 9 & motor_num < 1) return; // motor_num이 1~9가 아닌 경우 return
  if(!motor_open[motor_num-1]){
    dxl_wb.goalPosition(motor[motor_num], (int32_t)(initial_pos[motor_num] + 7900));
    motor_open[motor_num-1] = true;
  }
}

// Close the bookcase of given motor_num
void CloseBookcase(int motor_num){
  if(motor_num > 9 & motor_num < 1) return; // motor_num이 1~9가 아닌 경우 return
  if(motor_open[motor_num-1]){
    dxl_wb.goalPosition(motor[motor_num], (int32_t)(initial_pos[motor_num] + 100));
    motor_open[motor_num-1] = false;
  }
}

// Reset all bookcase, and reader's count
void Reset(){
  for(size_t i{1};i<10;i++) CloseBookcase(i);
  bookcaseReader.reset();
}

// The function that run the motor of given action and target.
void run(const String action,const String target){
  if(action == "open"){
    OpenBookcase(target.substring(4).toInt());
    // for(size_t i{1};i<10;i++){
      // OpenBookcase(i);
    // }
  }
  else if(action == "close"){
    CloseBookcase(target.substring(4).toInt());
    // for(size_t i{1};i<10;i++){
    //   CloseBookcase(i);
    // }
  }
  else if(action == "done"){
    // task_flag = false;
    int temp=0;
  }
  else if(action == "reset"){
    Reset();
  }
}



// The callback function of the topic set_bookcasae. seperate the target and action.
void readcmdCallback(const std_msgs::String &msg){
	String cmd = "";
  String cmd_action = "";
  String cmd_target = "";
  char cmd_seperator = ' ';
  cmd = msg.data;
  nh.loginfo("Command Received.");
	int separatorIndex = cmd.indexOf(cmd_seperator);
  if (separatorIndex != -1) { // cmd book1 open
      cmd_target = cmd.substring(0, separatorIndex);
      cmd_action = cmd.substring(separatorIndex + 1);
  } 
	else {
      cmd_action = cmd;
			cmd_target = "";
  }
  run(cmd_action, cmd_target);
}

// bookN open / bookN close / reset / done     (N = 1~9)
ros::Subscriber<std_msgs::String> command("set_bookcase", readcmdCallback); 


void setup() {
  nh.initNode();
  nh.subscribe(command);
  Serial.begin(9600);
  bookcaseReader.init(9600);

  const char *log;
  dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);

  for(size_t i{1};i<10;i++){
    dxl_wb.ping(motor[i], &model_number, &log);
    dxl_wb.setExtendedPositionControlMode(motor[i], &log);
    dxl_wb.torqueOn(motor[i], &log);
    dxl_wb.getPresentPositionData(motor[i], &presentposition[i], &log);
    initial_pos[i] = presentposition[i];
  }

  pinMode(7, OUTPUT);
}

void loop() {
  nh.spinOnce();
  
  while (nh.connected()) 
  {
    if(nh.getParam("kill")){
      nh.loginfo("Killing Process...");
      nh.shutdown();
      return;
    }
    if(start_flag == false){
      nh.loginfo("OpenCR Ready!");
      start_flag = true;
    }
    bookcaseReader.read();
    nh.spinOnce();
  }

}
