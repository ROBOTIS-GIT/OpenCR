#ifndef TURTLEBOT3_EXAMPLE05_1_OMNIWHEEL_INO
#define TURTLEBOT3_EXAMPLE05_1_OMNIWHEEL_INO

#include <DynamixelSDK.h>
#include <RC100.h>
#include <math.h>

#include "turtlebot3_example05_1_omniwheel_config.h"

int dxl_comm_result = COMM_TX_FAIL;             // Communication result
bool dxl_addparam_result = false;               // addParam result
bool dxl_getdata_result = false;                // GetParam resulttorque ON

uint8_t dxl_error = 0;                          // Dynamixel error
int RcvData = 0;
int32_t goal_velocity = 60;
uint8_t dynamixel_id[OMNIWHEEL_NUM] = {1, 2, 3};
float robot_linear_x_ = 0.0;
float robot_linear_y_ = 0.0;
float robot_angular_  = 0.0;

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

// Initialize GroupSyncWrite instance
dynamixel::GroupSyncWrite *groupSyncWrite;

RC100 Controller;
HardwareTimer Timer(TIMER_CH1);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
//  while(!Serial);

  Controller.begin(1);

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_XM_GOAL_VELOCITY, LEN_XM_GOAL_VELOCITY);

  setTorque();

  timerInit();
}

void loop()
{
  receiveRemoteControl();
}

void timerInit()
{
  Timer.pause();
  Timer.setPeriod(CONTROL_PEROID);           // in microseconds
  Timer.attachInterrupt(handler_control);
  Timer.refresh();
  Timer.resume();
}

void receiveRemoteControl(void)
{
  if (Controller.available())
  {
    RcvData = Controller.readData();
    if (RcvData & RC100_BTN_U)
    {
      robot_linear_x_ += VELOCITY_LINEAR_X;
    }
    else if (RcvData & RC100_BTN_D)
    {
      robot_linear_x_ -= VELOCITY_LINEAR_X;
    }
    else if (RcvData & RC100_BTN_L)
    {
      robot_linear_y_ -= VELOCITY_LINEAR_Y;
    }
    else if (RcvData & RC100_BTN_R)
    {
      robot_linear_y_ += VELOCITY_LINEAR_Y;
    }
    else if (RcvData & RC100_BTN_2)
    {
      robot_angular_ += VELOCITY_ANGULAR_Z;
    }
    else if (RcvData & RC100_BTN_4)
    {
      robot_angular_ -= VELOCITY_ANGULAR_Z;
    }
    else if (RcvData & RC100_BTN_5)
    {
      robot_linear_x_ = 0.0;
      robot_linear_y_ = 0.0;
      robot_angular_ = 0.0;
    }
  }
}

void handler_control(void)
{
  controlMotorSpeed();
}

void setTorque()
{
  uint8_t num  = 0;
  for (num = 0; num < OMNIWHEEL_NUM; num++)
  {
    // Enable Dynamixel#1 Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id[num], ADDR_XM_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->printRxPacketError(dxl_error);
    }
    else
    {
      Serial.print("Dynamixel has been successfully torque ON.\n");
    }
  }
}

void setDynamixelVelocity(int32_t goal_velocity[3])
{
  uint8_t num  = 0;
  for (num = 0; num < OMNIWHEEL_NUM; num++)
  {
    dxl_addparam_result = groupSyncWrite->addParam(dynamixel_id[num], (uint8_t*)&goal_velocity[num]);
    if (dxl_addparam_result)
      Serial.println("set velocity");
  }

  // Syncwrite goal position
  dxl_comm_result = groupSyncWrite->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

  // Clear syncwrite parameter storage
  groupSyncWrite->clearParam();
}

void controlMotorSpeed()
{
  int32_t dxl_goal_velocity[OMNIWHEEL_NUM] = {0.0, 0.0, 0.0};
  float wheel_angular_velocity[OMNIWHEEL_NUM] = {0.0, 0.0, 0.0};

  wheel_angular_velocity[0] = (robot_linear_x_ * 0) + (robot_linear_y_ * (1 / WHEEL_RADIUS)) + (robot_angular_ * (-DISTANCE_CENTER_TO_WHEEL/WHEEL_RADIUS));
  wheel_angular_velocity[1] = (robot_linear_x_ * (sqrt(3) / (2 * WHEEL_RADIUS))) + (robot_linear_y_ * (-1 / (2 * WHEEL_RADIUS))) + (robot_angular_ * (-DISTANCE_CENTER_TO_WHEEL/WHEEL_RADIUS));
  wheel_angular_velocity[2] = (robot_linear_x_ * (sqrt(3) / (-2 * WHEEL_RADIUS))) + (robot_linear_y_ * (-1 / (2 * WHEEL_RADIUS))) + (robot_angular_ * (-DISTANCE_CENTER_TO_WHEEL/WHEEL_RADIUS));

  for (int id = 0; id < OMNIWHEEL_NUM; id++)
  {
    dxl_goal_velocity[id] = wheel_angular_velocity[id] / VELOCITY_CONSTANT_VAULE;

    if (dxl_goal_velocity[id] > LIMIT_XM_MAX_VELOCITY)       dxl_goal_velocity[id] =  LIMIT_XM_MAX_VELOCITY;
    else if (dxl_goal_velocity[id] < -LIMIT_XM_MAX_VELOCITY) dxl_goal_velocity[id] = -LIMIT_XM_MAX_VELOCITY;
  }

  Serial.print("Vx : ");  Serial.print(robot_linear_x_);
  Serial.print(" Vy : "); Serial.print(robot_linear_y_);
  Serial.print(" W : "); Serial.println(robot_angular_);

  setDynamixelVelocity(dxl_goal_velocity);
}

#endif // TURTLEBOT3_EXAMPLE05_1_OMNIWHEEL_INO
