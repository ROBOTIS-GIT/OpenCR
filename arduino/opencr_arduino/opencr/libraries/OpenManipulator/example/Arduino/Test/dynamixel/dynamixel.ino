#include <OMDynamixel.h>
#include <vector>

#define DXL_SIZE 5
#define BAUD_RATE 1000000

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define POSITION1 -20.0f
#define POSITION2 20.0f

std::vector<float> goal_position;
std::vector<float> present_position;

std::vector<uint8_t> id;

OM_DYNAMIXEL::Dynamixel dxl;

void setup()
{
  Serial.begin(57600);
  while (!Serial)
    ;

  dxl.init(BAUD_RATE);
  // dxl.setMaxPositionLimit(1, 180*DEG2RAD);
  // dxl.setMinPositionLimit(1, -180*DEG2RAD);

  dxl.enableAllDynamixel();

  goal_position.reserve(DXL_SIZE);
  for (uint8_t num = 0; num < DXL_SIZE; num++)
  {
    goal_position.push_back(POSITION1 * DEG2RAD);
  }

  present_position.reserve(DXL_SIZE);

  // id.push_back(2);
  // id.push_back(3);
}

void loop()
{
  dxl.setAngle(id, goal_position);

  do
  {
    present_position = dxl.getAngle();

    log();
  } while (abs(dxl.convertRadian2Value(dxl.getDynamixelIDs().at(0), goal_position.at(0)) -
               dxl.convertRadian2Value(dxl.getDynamixelIDs().at(0), present_position.at(0))) > 20);

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
    Serial.print(dxl.convertRadian2Value(dxl.getDynamixelIDs().at(index), goal_position.at(index)));
    Serial.print(" PresPos: ");
    Serial.print(dxl.convertRadian2Value(dxl.getDynamixelIDs().at(index), present_position.at(index)));
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
    for (uint8_t num = 0; num < DXL_SIZE; num++)
    {
      goal_position.push_back(POSITION2 * DEG2RAD);
    }
  }
  else
  {
    for (uint8_t num = 0; num < DXL_SIZE; num++)
    {
      goal_position.push_back(POSITION1 * DEG2RAD);
    }
  }

  flag = !flag;
}