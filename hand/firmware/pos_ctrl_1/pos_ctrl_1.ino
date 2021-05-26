#include <Dynamixel2Arduino.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <hand/pos_ctrl_1.h>

ros::NodeHandle nh;

#define DXL_SERIAL_1   Serial1
#define DXL_SERIAL_2   Serial2
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN_1 = 2; // DYNAMIXEL Shield DIR PIN for the first motor
const uint8_t DXL_DIR_PIN_2 = 6; // DYNAMIXEL Shield DIR PIN for the second motor

const uint8_t DXL_ID_1 = 11;
const uint8_t DXL_ID_2 = 1;
const float DXL_PROTOCOL_VERSION_1 = 2.0;
const float DXL_PROTOCOL_VERSION_2 = 2.0;
const int32_t BAUD_RATE_MOTOR = 1000000;
Dynamixel2Arduino dxl_1(DXL_SERIAL_1, DXL_DIR_PIN_1);
Dynamixel2Arduino dxl_2(DXL_SERIAL_2, DXL_DIR_PIN_2);

//This namespace is required to use Control table item names
using namespace ControlTableItem;   
#define WINDOW_SIZE 5
float sum = 0;
float value = 0;
int my_index = 0;
float readings[WINDOW_SIZE];
float averaged = 0;

float temp_ave = 0.0;

float load_pre = 0.0;

const float pos_max_1 = 3883.0;
const float pos_min_1 = 1690.0;
const float pos_max_2 = 790.0;
const float pos_min_2 = 250.0;
const float pos_mid_2 = 580.0;

int motor_states[2] = {0, 0};
int open_cmd = 0;
int close_cmd = 0;
int spread_cmd = 0;

void messageCb(const sensor_msgs::Joy& data){
  close_cmd = data.buttons[2]; //X = close
  open_cmd = data.buttons[3];    //Y = open
  spread_cmd = data.axes[0];  //<-- --> spread / unspread
}

hand::pos_ctrl_1 motor_states_msg;
ros::Subscriber<sensor_msgs::Joy> sub("joy", &messageCb);
ros::Publisher pub_motor_states("motor_states", &motor_states_msg);

void grasping() {
 load_pre = dxl_1.readControlTableItem(PRESENT_LOAD, DXL_ID_1);
    
 float load = filter();
 if (abs(load - load_pre) < 25.0 && dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW) < pos_max_1  && dxl_1.readControlTableItem(PRESENT_LOAD, DXL_ID_1) < 300.0) {
     dxl_1.setGoalPosition(DXL_ID_1, dxl_1.getPresentPosition(DXL_ID_1, UNIT_RAW) + 100, UNIT_RAW);
     delay(10);
     motor_states[0] = 1;
     load_pre = load;
 }
 else {
    if (dxl_1.readControlTableItem(PRESENT_LOAD, DXL_ID_1) >= 400.0) {
     motor_states[0] = 2;
    }
    else {        
     motor_states[0] = 3;
    }
  delay(1);
  }
}

void opening() {
  dxl_1.setGoalPosition(DXL_ID_1, pos_min_1, UNIT_RAW);
  delay(500);
  motor_states[0] = 4;
}
  
float filter() {
  sum = sum - readings[my_index];
  value = dxl_1.readControlTableItem(PRESENT_LOAD, DXL_ID_1);
  readings[my_index] = value;
  sum = sum + value;
  my_index = (my_index + 1);
  if (my_index >= WINDOW_SIZE) {
    my_index = 0;
  }
  averaged = sum / WINDOW_SIZE;
  delay(1);
  return averaged;
}

void spreading() {
  dxl_2.setGoalPosition(DXL_ID_2, pos_min_2, UNIT_RAW);
  delay(500);
  motor_states[1] = 4;
}

void unspreading() {
  dxl_2.setGoalPosition(DXL_ID_2, pos_max_2, UNIT_RAW);
  delay(500);
  motor_states[1] = 6;
}

void initialization() {
  dxl_1.setGoalPosition(DXL_ID_1, pos_min_1, UNIT_RAW);
  delay(500);
  motor_states[0] = 4;
  dxl_2.setGoalPosition(DXL_ID_2, pos_mid_2, UNIT_RAW);
  delay(500);
  motor_states[1] = 2;
  for (int filter_run_time = 0; filter_run_time < 5; filter_run_time++) {
    temp_ave = filter();
    delay(10);
  }
}

void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // This has to match with DYNAMIXEL baudrate.
  dxl_1.begin(BAUD_RATE_MOTOR);
  dxl_2.begin(BAUD_RATE_MOTOR);
  
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl_1.setPortProtocolVersion(DXL_PROTOCOL_VERSION_1);
  dxl_2.setPortProtocolVersion(DXL_PROTOCOL_VERSION_2);
  
  // Get DYNAMIXEL information
  dxl_1.ping(DXL_ID_1);
  dxl_2.ping(DXL_ID_2);

  // Turn off torque when configuring items in EEPROM area
  dxl_1.torqueOff(DXL_ID_1);
  dxl_1.setOperatingMode(DXL_ID_1, OP_POSITION);
  dxl_1.torqueOn(DXL_ID_1);

  dxl_2.torqueOff(DXL_ID_2);
  dxl_2.setOperatingMode(DXL_ID_2, OP_POSITION);
  dxl_2.torqueOn(DXL_ID_2);

  nh.initNode();
  nh.advertise(pub_motor_states);
  nh.subscribe(sub);

  initialization();
}

void loop() {
  /*
  UNIT_DEGREE UNIT_RAW (4096/360*degree)
  0           0
  20          227
  40          455
  60          682
  80          910
  100         1138
  120         1365
  140         1593
  160         1820
  180         2048
  200         2275
  220         2503
  ...
  270         3072
  360         4095
  */
  /*
  pub motor_states look up table
  motor1_states = 1 --> grasping
  motor1_states = 2 --> grasped
  motor1_states = 3 --> fully close (nothing grasped)
  motor1_states = 4 --> fully open
  motor2_states = 1 --> fully un-spread
  motor2_states = 2 --> halfway
  motor2_states = 3 --> spreading
  motor2_states = 4 --> fully spread
  motor2_states = 5 --> unspreading
  motor2_states = 6 --> fully unspread
  */
  /*
  sub motor_command look up table
  motor1_states = 1 --> grasping
  motor1_states = 2 --> opening
  motor1_states = 3 --> stay
  motor2_states = 1 --> spreading
  motor2_states = 2 --> unspreading
  motor2_states = 3 --> stay
  */
  if (close_cmd == 1) { grasping(); }
  if (open_cmd == 1) { opening(); }
 
  if (spread_cmd == 1) { spreading(); }
  if (spread_cmd == -1) { unspreading(); }
    
  motor_states_msg.motor1_states = motor_states[0];
  motor_states_msg.motor2_states = motor_states[1];
  pub_motor_states.publish( &motor_states_msg);
  nh.spinOnce();
  delay(10);
}
