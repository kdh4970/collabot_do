#define USE_USBCON  
// #include <std_msgs/Int32.h>
#include <DynamixelWorkbench.h>
#include <BookcaseReader.h>
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
BookcaseReader bookcaseReader;
//ros node Handle
ros::NodeHandle nh;

// std_msgs::Int32 total_count;
std_msgs::String state;

//make publisher
// ros::Publisher bookcase_num_pub("bookcase_num",  &moter_num);
// ros::Publisher count_pub("count",  &total_count);

int isclose {};

//make callback function
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
int motor_open[9] = {0,};
// cmd parse
String cmd_action = "";
String cmd_target = "";
char cmd_seperator = ' ';
bool task_flag = false;

void readcmdCallback(const std_msgs::String &msg){
	cmd = msg.data;
	int separatorIndex = cmd.indexOf(cmd_seperator);
  if (separatorIndex != -1) { // cmd가 book1 open 같은 형태인경우
      cmd_target = cmd.substring(0, separatorIndex);
      cmd_action = cmd.substring(separatorIndex + 1);
  } 
	else { // cmd 가 reset 같은 형태인 경우
      cmd_action = cmd;
			cmd_target = "";
  }
}

ros::Subscriber<std_msgs::String> command("cmd_opencr", readcmdCallback); // main에서 퍼블리시할 cmd 토픽 sub



uint16_t model_number = 0;
int32_t presentposition[13];
int initial_pos[13] = {0,};
int initial_1 = 0;
int initial_2 = 0;
int initial_3 = 0;
int initial_4 = 0;
int initial_5 = 0;
int initial_6 = 0;
int initial_7 = 0;
int initial_8 = 0;
int initial_9 = 0;
int initial_10 = 0;
int count = 0;
uint8_t motor[13] = {0, MOTOR1, MOTOR2, MOTOR3, MOTOR4, MOTOR5, MOTOR6, MOTOR7, MOTOR8, MOTOR9};

void OpenBookcase(int motor_num){
  if(motor_num > 9 & motor_num < 1) return; // motor_num이 1~9가 아닌 경우 return
  if(motor_open[motor_num-1] == 0){
    dxl_wb.goalPosition(motor[motor_num], (int32_t)(initial_pos[motor_num] + 7900));
    motor_open[motor_num-1] = 1;
  }
}

void CloseBookcase(int montor_num){
  if(motor_num > 9 & motor_num < 1) return; // motor_num이 1~9가 아닌 경우 return
  if(motor_open[motor_num-1] == 1){
    dxl_wb.goalPosition(motor[motor_num], (int32_t)(initial_pos[motor_num] + 100));
    motor_open[motor_num-1] = 0;
  }
}

void Reset(){
  for(int i{1};i<10;i++) CloseBookcase(i);
}


void setup() {

  nh.initNode();
  // nh.advertise(bookcase_num_pub);
  // nh.advertise(count_pub);
  nh.subscribe(close_flag);
  nh.subscribe(command);
  // for(int i{1};i<10;i++) nh.setParam("bookcase_state" + String(i), "closed");
  
  
  Serial.begin(9600);
  // Serial2.begin(9600);
  bookcaseReader.init(Serial2, 9600, nh);

  pinMode(7, OUTPUT);
}

void loop() {
  const char *log;
  dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);

  dxl_wb.ping(motor[1], &model_number, &log); 
  dxl_wb.ping(motor[2], &model_number, &log);
  dxl_wb.ping(motor[3], &model_number, &log);
  dxl_wb.ping(motor[4], &model_number, &log);
  dxl_wb.ping(motor[5], &model_number, &log);
  dxl_wb.ping(motor[6], &model_number, &log);
  dxl_wb.ping(motor[7], &model_number, &log);
  dxl_wb.ping(motor[8], &model_number, &log);
  dxl_wb.ping(motor[9], &model_number, &log);
  //dxl_wb.ping(motor[10], &model_number, &log);

  dxl_wb.setExtendedPositionControlMode(motor[1], &log); 
  dxl_wb.setExtendedPositionControlMode(motor[2], &log);
  dxl_wb.setExtendedPositionControlMode(motor[3], &log);
  dxl_wb.setExtendedPositionControlMode(motor[4], &log);
  dxl_wb.setExtendedPositionControlMode(motor[5], &log);
  dxl_wb.setExtendedPositionControlMode(motor[6], &log);
  dxl_wb.setExtendedPositionControlMode(motor[7], &log);
  dxl_wb.setExtendedPositionControlMode(motor[8], &log);
  dxl_wb.setExtendedPositionControlMode(motor[9], &log);
  //dxl_wb.setExtendedPositionControlMode(motor[10], &log);

  dxl_wb.torqueOn(motor[1], &log);
  dxl_wb.torqueOn(motor[2], &log);
  dxl_wb.torqueOn(motor[3], &log);
  dxl_wb.torqueOn(motor[4], &log);
  dxl_wb.torqueOn(motor[5], &log);
  dxl_wb.torqueOn(motor[6], &log);
  dxl_wb.torqueOn(motor[7], &log);
  dxl_wb.torqueOn(motor[8], &log);
  dxl_wb.torqueOn(motor[9], &log);
  //dxl_wb.torqueOn(motor[10], &log);
 
  dxl_wb.getPresentPositionData(motor[1], &presentposition[1], &log); 
  dxl_wb.getPresentPositionData(motor[2], &presentposition[2], &log);
  dxl_wb.getPresentPositionData(motor[3], &presentposition[3], &log);
  dxl_wb.getPresentPositionData(motor[4], &presentposition[4], &log);
  dxl_wb.getPresentPositionData(motor[5], &presentposition[5], &log);
  dxl_wb.getPresentPositionData(motor[6], &presentposition[6], &log);
  dxl_wb.getPresentPositionData(motor[7], &presentposition[7], &log);
  dxl_wb.getPresentPositionData(motor[8], &presentposition[8], &log);
  dxl_wb.getPresentPositionData(motor[9], &presentposition[9], &log);
  //dxl_wb.getPresentPositionData(motor[10], &presentposition[10], &log);

  if (initial_1 == 0) {
    initial_pos[1] = presentposition[1];
    initial_1++;
  }
  
  if (initial_2 == 0) {
    initial_pos[2] = presentposition[2];
    initial_2++;
  }
  
  if (initial_3 == 0) {
    initial_pos[3] = presentposition[3];
    initial_3++;
  } 
  
  if (initial_4 == 0) {
    initial_pos[4] = presentposition[4];
    initial_4++;
  }

  if (initial_5 == 0) {
    initial_pos[5] = presentposition[5];
    initial_5++;
  }

  if (initial_6 == 0) {
    initial_pos[6] = presentposition[6];
    initial_6++;
  }

  if (initial_7 == 0) {
    initial_pos[7] = presentposition[7];
    initial_7++;
  }

  if (initial_8 == 0) {
    initial_pos[8] = presentposition[8];
    initial_8++;
  }

  if (initial_9 == 0) {
    initial_pos[9] = presentposition[9];
    initial_9++;
  }

  // do added start
  while (nh.connected()) {
    bookcaseReader.read();
    cmd_action = "";
    cmd_target = "";
    
    
    nh.spinOnce();
    if(cmd_action == "open"){
      OpenBookcase(cmd_target.substring(4).toInt());
    }
    else if(cmd_action == "close"){
      CloseBookcase(cmd_target.substring(4).toInt());
    }
    else if(cmd_action == "done"){
      // task_flag = false;
      cmd_action = "";
      cmd_target = "";
    }
    else if(cmd_action == "reset"){
      Reset();
    }
    // do added end
  }

}
