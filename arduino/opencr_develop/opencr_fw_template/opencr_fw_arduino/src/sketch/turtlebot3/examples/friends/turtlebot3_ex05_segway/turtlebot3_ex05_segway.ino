#ifndef TURTLEBOT3_EXAMPLE06_SEGWAY_INO
#define TURTLEBOT3_EXAMPLE06_SEGWAY_INO

#include "turtlebot3_example06_segway_config.h"

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  DEBUG_SERIAL.begin(57600);

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
    //Serial.print("Succeeded to open the port!\n");
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
    //Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_XM_GOAL_PWM, LEN_XM_GOAL_PWM);
  groupSyncRead = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_XM_PRESENT_POSITION, LEN_XM_PRESENT_POSITION);

  setTorque();
  imuInit();
}

void loop()
{
  if (IMU.update() > 0)
    getAngle(angle);

#ifdef DEBUG
  if (Serial.available())
  {
    keyboard = Serial.read();
    if (keyboard == 'u')
    {
      p_gain = p_gain + 1.0;
    }
    else if (keyboard == 'j')
    {
      p_gain = p_gain - 1.0;
    }
    else if (keyboard == 'o')
    {
      d_gain = d_gain + 0.1;
    }
    else if (keyboard == 'l')
    {
      d_gain = d_gain - 0.1;
    }
    else if (keyboard == 'i')
    {
      i_gain = i_gain + 0.1;
    }
    else if (keyboard == 'k')
    {
      i_gain = i_gain - 0.1;
    }
    else if (keyboard == 'a')
    {
      angle_offset = angle_offset + 0.01;
    }
    else if (keyboard == 's')
    {
      angle_offset = angle_offset - 0.01;
    }
  }

  Serial.print(" P : ");
  Serial.print(p_gain);
  Serial.print(" I : ");
  Serial.print(i_gain);
  Serial.print(" D : ");
  Serial.print(d_gain);
  Serial.print(" offset : ");
  Serial.print(angle_offset);
  Serial.print(" output : ");
  Serial.print(control_output);
  Serial.print(" angle : ");
  Serial.println(angle[0]);
#endif

#ifdef GRAPH
  // Serial.print(graph_max);
  // Serial.print(" ");
  // Serial.print(graph_min);
  // Serial.print(" ");
  // Serial.print(control_output);
  DEBUG_SERIAL.print(angle[0]);
  DEBUG_SERIAL.print(",0");
  DEBUG_SERIAL.print(",0");
  DEBUG_SERIAL.println(",0");

#endif

#ifdef REMOTE
  if (Controller.available())
  {
    RcvData = Controller.readData();
    if (RcvData & RC100_BTN_U)
    {
      control_input = -0.1;
    }
    else if (RcvData & RC100_BTN_D)
    {
      control_input = 0.1;
    }
    else
    {
      control_input = 0.0;
    }
  }
#endif
}

void getAngle(float angle[3])
{
  float roll, pitch, yaw;

  roll  = IMU.rpy[0];
  pitch = IMU.rpy[1];
  yaw   = IMU.rpy[2];

  angle[0] = lowpassFilter.input(pitch) + angle_offset;
  angle[1] = pitch + angle_offset;
}

void setDynamixelPWM(int64_t wheel_value)
{
  dxl_addparam_result = groupSyncWrite->addParam(DXL_LEFT_ID, (uint8_t*)&wheel_value);

  dxl_addparam_result = groupSyncWrite->addParam(DXL_RIGHT_ID, (uint8_t*)&wheel_value);

  // Syncwrite goal position
  dxl_comm_result = groupSyncWrite->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

  // Clear syncwrite parameter storage
  groupSyncWrite->clearParam();
}

void timerInit()
{
  Timer.pause();
  Timer.setPeriod(CONTOL_PERIOD);           // in microseconds
  Timer.attachInterrupt(handler_control);
  Timer.refresh();
  Timer.resume();
}

void imuInit()
{
  IMU.begin();

  //Serial.println("ACC Cali Start");

  IMU.SEN.acc_cali_start();
  while( IMU.SEN.acc_cali_get_done() == false )
  {
    IMU.update();
    //Serial.println( IMU.SEN.calibratingA );
  }

  //Serial.print("ACC Cali End ");

  //Serial.print("timer Start ");
  timerInit();
  //getEncoderValue(encoder_input);
}

void setTorque()
{
  // Enable Dynamixel#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_LEFT_ID, ADDR_XM_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

  // Enable Dynamixel#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_RIGHT_ID, ADDR_XM_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
}

void handler_control(void)
{
  cur_error  = control_input - angle[0];
  integral   = integral + (cur_error * diff_time);
  derivative = (cur_error - pre_error) / diff_time;

  if (cnt > 500)
  {
    integral = 0.0;
    cnt = 0;
  }
  else
  {
    cnt++;
  }

  control_output = p_gain * cur_error +
                   i_gain * integral  +
                   d_gain * derivative;

  if (control_output >= PWM_LIMIT)
  {
    control_output = PWM_LIMIT;
  }
  else if (control_output <= (-1) * PWM_LIMIT)
  {
    control_output = (-1) * PWM_LIMIT;
  }
  pre_error = cur_error;

  setDynamixelPWM(control_output);
}

#endif // TURTLEBOT3_EXAMPLE06_SEGWAY_INO
