#include <OMDynamixel.h>
#include <vector>

#define DXL_SIZE 3
#define BAUD_RATE 57600

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

std::vector<float> goal_position;
std::vector<float> present_position;

OM_DYNAMIXEL::Dynamixel dxl;

void setup()
{
  Serial.begin(57600);
  while(!Serial);

  dxl.init(BAUD_RATE);
  dxl.setMaxPositionLimit(1, 180*DEG2RAD);
  dxl.setMinPositionLimit(1, -180*DEG2RAD);

  dxl.enableAllDynamixel();

  goal_position.reserve(DXL_SIZE);
  goal_position.push_back(90.0f * DEG2RAD);
  goal_position.push_back(90.0f * DEG2RAD);
  goal_position.push_back(90.0f * DEG2RAD);

  present_position.reserve(DXL_SIZE);
}

void loop() 
{
  dxl.setAngle(goal_position);  

  do
  {
    present_position = dxl.getAngle();

    log();
  }while(abs(dxl.convertRadian2Value(1, goal_position.at(0)) - dxl.convertRadian2Value(1, present_position.at(0))) > 5);

  swap();
}

void log()
{
  Serial.print("[");
  for (int8_t index = 0; index < DXL_SIZE; index++)
  {
    Serial.print("ID : ");
    Serial.print(dxl.getDynamixelIDs().at(index));
    Serial.print(" GoalPos: ");
    Serial.print(dxl.convertRadian2Value(1, goal_position.at(index)));
    Serial.print(" PresPos: ");
    Serial.print(dxl.convertRadian2Value(1, present_position.at(index)));
    Serial.print(" | ");
  }
  Serial.println("]");
}

void swap()
{
  static bool flag = true;
  goal_position.clear();

  if (flag == true)
  {
    goal_position.push_back(-90.0f * DEG2RAD);
    goal_position.push_back(-90.0f * DEG2RAD);
    goal_position.push_back(-90.0f * DEG2RAD);
  }
  else
  {
    goal_position.push_back(90.0f * DEG2RAD);
    goal_position.push_back(90.0f * DEG2RAD);
    goal_position.push_back(90.0f * DEG2RAD);
  }

  flag = !flag;
}