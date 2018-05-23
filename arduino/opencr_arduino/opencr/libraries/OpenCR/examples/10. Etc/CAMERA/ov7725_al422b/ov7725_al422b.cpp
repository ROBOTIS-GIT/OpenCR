#include <DynamixelSDK.h>
#include "ov7725_al422b.h"
#include "settings.h"

object_info_t detected_object[MAX_OBJECT];
uint16_t img_width = LCD_WIDTH;
uint16_t img_height = LCD_HEIGHT;
uint8_t  detectCount = 0;
uint8_t  target = 0;
char detectCounter[3];
char coordinateX[4];
char coordinateY[4];
char fpsString[3];

#if _USE_FULLSCREEN == true
  uint16_t image_buf[LCD_WIDTH*LCD_HEIGHT];
  //use bit for each pixel info or else : RAM overflow
  uint16_t __attribute__((section(".NoneCacheableMem"))) masked_image_buf[(LCD_WIDTH*LCD_HEIGHT)/16];
#else
  uint16_t image_buf[(LCD_WIDTH/2)*(LCD_HEIGHT/2)];
  uint16_t masked_image_buf[(LCD_WIDTH/2)*(LCD_HEIGHT/2)];
#endif

void lcdInit()
{
    __SD_CS_DISABLE();
    SPI.beginFast();
    SPI.setDataMode(SPI_MODE3);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV4);

    TFTLCD.lcd_init();
}

void drawShape(shape_list shape, uint16_t x_pos, uint16_t y_pos, uint16_t radius, uint16_t color)
{
  switch(shape)
  {
    case CIRCLE:
     TFTLCD.lcd_draw_circle(x_pos, y_pos, radius, color);
     break;
     
    case SQUARE:
     TFTLCD.lcd_fill_rect(x_pos, y_pos, radius, radius, color);
     break;
  }
}

void drawScreen()
{
  TFTLCD.drawFrame();
}

void clearScreen(uint16_t color)
{
  TFTLCD.lcd_clear_screen(color);
}

void lcdRotation(uint8_t rotation)
{
  TFTLCD.LCDRotation(rotation);
}

void drawText(uint16_t x_pos, uint16_t y_pos, const uint8_t *string, uint8_t ch_size, uint16_t color)
{
  TFTLCD.lcd_display_string(x_pos, y_pos, string, ch_size, color);
}

void setGRamArea(uint8_t size)
{
  TFTLCD.setLcdMemoryArea(size);
}

/********************
 * Image Processing
 * Pixel : GGGBBBBB RRRRRGGG
 ********************/
void colorFilter(uint16_t *image, uint8_t size)
{
  uint32_t pixelCount = 0;
  img_width = LCD_WIDTH;
  img_height = LCD_HEIGHT;
  uint16_t u_quotient = 0;
  uint8_t  u_remainder = 0;
  
  if((size == QUARTERVIEW) || (size == QQVGA))
  {
    img_width = LCD_WIDTH / 2;
    img_height = LCD_HEIGHT / 2;
  }

  for(pixelCount = 0; pixelCount < img_width*img_height; pixelCount++)
  {
    if(colorFinder(image, pixelCount))
    {
      #if _USE_FULLSCREEN == true
        u_quotient = pixelCount / 16;
        u_remainder = pixelCount % 16;
        masked_image_buf[u_quotient] |= (0x8000 >> u_remainder);  // 1000 0000
      #else
        //change the detected color to BLUE
        masked_image_buf[pixelCount] = BLUE;
      #endif
    }
    else
    {
      #if _USE_FULLSCREEN == true
        masked_image_buf[pixelCount] = 0;
        masked_image_buf[u_quotient] &= ~(0x8000 >> u_remainder);  // 0111 1111
      #else
        masked_image_buf[pixelCount] = BLACK;
      #endif
    }
  }
}

bool colorFinder(const uint16_t *image, uint32_t pixelLocation)
{
  const color_range_t* target_color = &selected_color[2];	//select color to detect
  uint16_t pixelColor;
  float propRed;
  float propGreen;
  float propBlue;
  uint8_t rgbSUM;

  pixelColor = image[pixelLocation] >> 8 | image[pixelLocation] << 8;
  rgbSUM = ((pixelColor >> 11) & 0x1F) + ((pixelColor >> 5) & 0x3F) + (pixelColor & 0x1F);
  propRed = (float)((pixelColor >> 11) & 0x1F) / rgbSUM;
  propGreen = (float)((pixelColor >> 5) & 0x3F) / rgbSUM;
  propBlue = (float)(pixelColor & 0x001F) / rgbSUM;

  //compare Red range
  if ((propRed > target_color->minRed) && (propRed < target_color->maxRed))
  {
	//compare Green range
	if ((propGreen > target_color->minGreen) && (propGreen < target_color->maxGreen))
	{
	  //compare Blue ranges
	  if ((propBlue > target_color->minBlue) && (propBlue < target_color->maxBlue))
	  {
	    //if RGB is within TARGET_COLOR range, leave the color or else erase pixel to BLACK.
		  return true;
	  }
	  return false;
	}
	return false;
  }
  return false;
}

//find the most attracting detection and mark around it
void objectFinder(const uint16_t *masked_image)
{
  detectCount = 0;
  uint8_t  xCount = 0;
  uint8_t  yCount = 0;
  uint16_t cellValue = 0;
  uint16_t cellColumnCount;
  uint32_t cellRawCount = 0;
  uint32_t pixeltoCheck = 0;
  uint16_t u_quotient = 0;
  uint8_t  u_remainder = 0;

  for(; cellRawCount < img_width*img_height; )
  {
    cellColumnCount = 0;
    for(; cellColumnCount < img_height; )
    {
      cellValue = 0;
      for(yCount = 0; yCount < MIN_OBJECT_PIXEL; yCount++)
      {
        pixeltoCheck = cellRawCount + cellColumnCount + img_height*yCount;
        for(xCount = 0; xCount < MIN_OBJECT_PIXEL; xCount++)
        {
          #if _USE_FULLSCREEN == true
            u_quotient = pixeltoCheck / 16;
            u_remainder = pixeltoCheck % 16;
            cellValue = cellValue + (uint16_t)((masked_image[u_quotient] >> (7 - u_remainder)) & 0x0001);
            pixeltoCheck++;
          #else
            cellValue = cellValue + (masked_image[pixeltoCheck] >> 8 | masked_image[pixeltoCheck] << 8);
            pixeltoCheck++;
          #endif
        }
      }
      #if _USE_FULLSCREEN == true
        if(cellValue > ((MIN_OBJECT_PIXEL * MIN_OBJECT_PIXEL) / 2))
      #else
        if(cellValue > (15 * MIN_OBJECT_PIXEL * MIN_OBJECT_PIXEL))
      #endif
      {
        detected_object[detectCount].coordX = cellColumnCount;					  //X coordinate
        detected_object[detectCount].coordY = cellRawCount / img_height;		//Y coordinate
        detected_object[detectCount].object_width = MIN_OBJECT_PIXEL;
        detected_object[detectCount++].object_height = MIN_OBJECT_PIXEL;

        if(detectCount > MAX_OBJECT) { detectCount = MAX_OBJECT; }
      }
      cellColumnCount = cellColumnCount + MIN_OBJECT_PIXEL;
    }
    cellRawCount = cellRawCount + img_height*MIN_OBJECT_PIXEL;
  }
}

//calculate weight for each cell
void cellWeight(void)
{
  uint8_t cellWeightCounter = 0;
  uint8_t i = 0;
  int16_t x = 0;
  int16_t y = 0;
  uint8_t distance = 0;

  for(cellWeightCounter = 0; cellWeightCounter < detectCount; cellWeightCounter++)
  {
    detected_object[cellWeightCounter].object_weight = 0;
    for(i = 0; i < detectCount; i++)
    {
      x = (detected_object[cellWeightCounter].coordX - detected_object[i].coordX) / MIN_OBJECT_PIXEL;
      y = (detected_object[cellWeightCounter].coordY - detected_object[i].coordY) / MIN_OBJECT_PIXEL;
      distance = x*x + y*y;
      switch(distance)
      {
        case 1:
          detected_object[cellWeightCounter].object_weight += 1.0;
          break;
        case 2:
          detected_object[cellWeightCounter].object_weight += 0.5;
          break;
        case 4:
          detected_object[cellWeightCounter].object_weight += 0.25;
          break;
        case 5:
          detected_object[cellWeightCounter].object_weight += 0.2;
          break;
        default:
          break;
      }
    }
  }
}

void findCenterCell(void)
{
  float fWeight = 0.0;
  for(uint8_t i = 0; i < detectCount; i++)
  {
    if(detected_object[i].object_weight > fWeight)
    {
      fWeight = detected_object[i].object_weight;
      target = i;
    }
    TFTLCD.lcd_draw_rect(detected_object[i].coordX, detected_object[i].coordY, detected_object[i].object_width, detected_object[i].object_height, WHITE);
  }
  if (detectCount < MAX_OBJECT)
  {
    TFTLCD.lcd_draw_rect(detected_object[target].coordX, detected_object[target].coordY, detected_object[target].object_width, detected_object[target].object_height, RED);
  }
}

void displayInfo(uint8_t fps)
{
//  itoa((int16_t)fWeight, cWeight, 10);
//  itoa(presentPos, panPos, 10);
  itoa(fps, fpsString, 10);
  itoa(detectCount, detectCounter, 10);
  itoa(detected_object[target].coordX, coordinateX, 10);
  itoa(detected_object[target].coordY, coordinateY, 10);

//  drawText(10, 100, (const uint8_t *)cWeight, 16, WHITE);
//  drawText(10, 80, (const uint8_t *)panPos, 16, WHITE);
  drawText(20, 5, (const uint8_t *)"fps", 12, RED);
  drawText(5, 5, (const uint8_t *)fpsString, 12, RED);
  drawText(5, 15, (const uint8_t *)detectCounter, 12, WHITE);
  drawText(5, 25, (const uint8_t *)coordinateX, 12, WHITE);
  drawText(5, 35, (const uint8_t *)coordinateY, 12, WHITE);
}

/********************************************
 * Dynamixel Initialization for Object tracking
********************************************/
void initDynamixel(dynamixel::PortHandler *hPort, dynamixel::PacketHandler *hPacket)
{
  if(hPort->openPort()) {Serial.print("Succeeded to open the port!\n");}
  else                  {Serial.print("Failed to open the port!\n");}

  if(hPort->setBaudRate(BAUDRATE))  {Serial.print("Succeeded to set the baudrate!\n");}
  else                              {Serial.print("Failed to set the baudrate!\n");}

  dxlCommResult = hPacket->write1ByteTxRx(hPort, PAN_ID, ADD_TORQUE_ENABLE, TORQUE_OFF, &dxlError);
  if (dxlCommResult != COMM_SUCCESS)  {Serial.print(hPacket->getTxRxResult(dxlCommResult));}
  else if (dxlError != 0)             {Serial.print(hPacket->getRxPacketError(dxlError));}
  else                                {Serial.print("Pan Dynamixel On \n");}

  dxlCommResult = hPacket->write1ByteTxRx(hPort, TILT_ID, ADD_TORQUE_ENABLE, TORQUE_OFF, &dxlError);
  if (dxlCommResult != COMM_SUCCESS)  {Serial.print(hPacket->getTxRxResult(dxlCommResult));}
  else if (dxlError != 0)             {Serial.print(hPacket->getRxPacketError(dxlError));}
  else                                {Serial.print("Tilt Dynamixel On \n");}

  delay_ms(10);

  dxlCommResult = hPacket->write4ByteTxRx(hPort, PAN_ID, ADD_PROF_ACCEL, PROF_ACCEL, &dxlError);
  if (dxlCommResult != COMM_SUCCESS)  {Serial.print(hPacket->getTxRxResult(dxlCommResult));}
  else if (dxlError != 0)             {Serial.print(hPacket->getRxPacketError(dxlError));}
  else                                {Serial.print("Pan Profile Accel Set \n");}
  
  dxlCommResult = hPacket->write4ByteTxRx(hPort, TILT_ID, ADD_PROF_ACCEL, PROF_ACCEL, &dxlError);
  if (dxlCommResult != COMM_SUCCESS)  {Serial.print(hPacket->getTxRxResult(dxlCommResult));}
  else if (dxlError != 0)             {Serial.print(hPacket->getRxPacketError(dxlError));}
  else                                {Serial.print("Tilt Profile Accel Set \n");}

  delay_ms(1000);

  dxlCommResult = hPacket->write4ByteTxRx(hPort, PAN_ID, ADD_PROF_VEL, PROF_VEL, &dxlError);
  if (dxlCommResult != COMM_SUCCESS)  {Serial.print(hPacket->getTxRxResult(dxlCommResult));}
  else if (dxlError != 0)             {Serial.print(hPacket->getRxPacketError(dxlError));}
  else                                {Serial.print("Pan Profile Velocity Set \n");}
  
  dxlCommResult = hPacket->write4ByteTxRx(hPort, TILT_ID, ADD_PROF_VEL, PROF_VEL, &dxlError);
  if (dxlCommResult != COMM_SUCCESS)  {Serial.print(hPacket->getTxRxResult(dxlCommResult));}
  else if (dxlError != 0)             {Serial.print(hPacket->getRxPacketError(dxlError));}
  else                                {Serial.print("Tilt Profile Velocity Set \n");}

  delay_ms(1000);

  dxlCommResult = hPacket->write1ByteTxRx(hPort, PAN_ID, ADD_TORQUE_ENABLE, TORQUE_ON, &dxlError);
  if (dxlCommResult != COMM_SUCCESS)  {Serial.print(hPacket->getTxRxResult(dxlCommResult));}
  else if (dxlError != 0)             {Serial.print(hPacket->getRxPacketError(dxlError));}
  else                                {Serial.print("Pan Dynamixel On \n");}

  dxlCommResult = hPacket->write1ByteTxRx(hPort, TILT_ID, ADD_TORQUE_ENABLE, TORQUE_ON, &dxlError);
  if (dxlCommResult != COMM_SUCCESS)  {Serial.print(hPacket->getTxRxResult(dxlCommResult));}
  else if (dxlError != 0)             {Serial.print(hPacket->getRxPacketError(dxlError));}
  else                                {Serial.print("Tilt Dynamixel On \n");}

  delay_ms(10);

  // Pan Initial Center Position
  dxlCommResult = hPacket->write4ByteTxRx(hPort, PAN_ID, ADD_GOAL_POSITION, 2048, &dxlError);
  if (dxlCommResult != COMM_SUCCESS)  {Serial.print(hPacket->getTxRxResult(dxlCommResult));}
  else if (dxlError != 0)             {Serial.print(hPacket->getRxPacketError(dxlError));}
  else                                {Serial.print("Pan Dynamixel Init Completed \n");}

  // Tilt Initial Center Position
  dxlCommResult = hPacket->write4ByteTxRx(hPort, TILT_ID, ADD_GOAL_POSITION, 2048, &dxlError);
  if (dxlCommResult != COMM_SUCCESS)  {Serial.print(hPacket->getTxRxResult(dxlCommResult));}
  else if (dxlError != 0)             {Serial.print(hPacket->getRxPacketError(dxlError));}
  else                                {Serial.print("Tilt Dynamixel Init Completed \n");}
  
  delay_ms(1000);

  // Read Pan present position
  dxlCommResult = hPacket->read4ByteTxRx(hPort, PAN_ID, ADD_PRESENT_POSITION, (uint32_t*)&panPresentPosition, &dxlError);
  // Read Tilt present position
  dxlCommResult = hPacket->read4ByteTxRx(hPort, TILT_ID, ADD_PRESENT_POSITION, (uint32_t*)&tiltPresentPosition, &dxlError);
  Serial.print("PanPresentPos : ");
  Serial.print(panPresentPosition);
  Serial.print("\nTiltPresentPos : ");
  Serial.print(tiltPresentPosition);
}


/********************************************
 * Dynamixel Control for Object tracking
********************************************/
void trackObject(dynamixel::PortHandler *hPort, dynamixel::PacketHandler *hPacket)
{
  // Read Pan present position
  dxlCommResult = hPacket->read4ByteTxRx(hPort, PAN_ID, ADD_PRESENT_POSITION, (uint32_t*)&panPresentPosition, &dxlError);
  // Read Tilt present position
  dxlCommResult = hPacket->read4ByteTxRx(hPort, TILT_ID, ADD_PRESENT_POSITION, (uint32_t*)&tiltPresentPosition, &dxlError);

  if((detectCount > 3) && (detectCount < MAX_OBJECT))
  {
    //Track the object
    //find the angle between target x, y and center(80,60)
    if (detected_object[target].coordX > 80 + MOVING_THRESHOLD)       {panGoalPosition = panPresentPosition - (detected_object[target].coordX - 80);}
    else if (detected_object[target].coordX < 80 - MOVING_THRESHOLD)  {panGoalPosition = panPresentPosition + (80 - detected_object[target].coordX);}
    else                                                              {panGoalPosition = panPresentPosition;}

    if(panGoalPosition > PAN_MAX_POSITION)        {panGoalPosition = PAN_MAX_POSITION;}
    else if(panGoalPosition < PAN_MIN_POSITION)   {panGoalPosition = PAN_MIN_POSITION;}
    else                                          {}

    dxlCommResult = hPacket->write4ByteTxRx(hPort, PAN_ID, ADD_GOAL_POSITION, panGoalPosition, &dxlError);
    if (dxlCommResult != COMM_SUCCESS)  {Serial.print(hPacket->getTxRxResult(dxlCommResult));}
    else if (dxlError != 0)             {Serial.print(hPacket->getRxPacketError(dxlError));}
    else {}

    if (detected_object[target].coordY > (60 + MOVING_THRESHOLD))       {tiltGoalPosition = tiltPresentPosition - (detected_object[target].coordY - 60);}
    else if (detected_object[target].coordY < (60 - MOVING_THRESHOLD))  {tiltGoalPosition = tiltPresentPosition + (60 - detected_object[target].coordY);}
    else                                                                {tiltGoalPosition = tiltPresentPosition;}

    if(tiltGoalPosition > TILT_MAX_POSITION)        {tiltGoalPosition = TILT_MAX_POSITION;}
    else if(tiltGoalPosition < TILT_MIN_POSITION)   {tiltGoalPosition = TILT_MIN_POSITION;}
    else                                            {}

    dxlCommResult = hPacket->write4ByteTxOnly(hPort, TILT_ID, ADD_GOAL_POSITION, tiltGoalPosition);
    if (dxlCommResult != COMM_SUCCESS)  {Serial.print(hPacket->getTxRxResult(dxlCommResult));}
    else if (dxlError != 0)             {Serial.print(hPacket->getRxPacketError(dxlError));}
  }
  else
  {
    // Serial.print("The object is too small or too large!\n");
  }
}

/********************************************
 * Dynamixel Control for Operating Platform
 * Left ID : 1(Positive Forward), Right ID : 2(Negative Forward)
 * Velocity Control Mode
********************************************/
void movePlatform(dynamixel::PortHandler *hPort, dynamixel::PacketHandler *hPacket)
{

}