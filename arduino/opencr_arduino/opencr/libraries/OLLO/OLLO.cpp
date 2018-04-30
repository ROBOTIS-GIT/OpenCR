/*
 * OLLO.cpp
 *
 *  Created on: 2013. 5. 30.
 *      Author: ROBOTIS CO,.LTD.
 */

#include "OLLO.h"


uint8_t before_color_num =0;
uint8_t before_color_cnt =0;
uint8_t after_color_value = 0;
uint8_t bColorResult =0;

OLLO::OLLO() {
	// TODO Auto-generated constructor stub

}

OLLO::~OLLO() {
	// TODO Auto-generated destructor stub
}
int average_cnt = 0;
word long gwTheRmistor[108] = {
  67747,64577,61571,58721,
  56017,53452,51018,48707,
  46513,44430,42450,40569,
  38781,37081,35465,33927,
  32464,31072,29747,28485,
  27283,26138,25048,24008, 23017,
  22072,21171,20311,19491,
  18708,17960,17246,16565,
  15913,15291,14696,14128,
  13584,13064,12567,12091,
  11636,11200,10782,10383,
  10000, 9633, 9282,  8945,
  8622, 8313, 8016, 7731,
  7458, 7195, 6944, 6702,
  6470, 6247, 6033, 5828,
  5630, 5440, 5258, 5082,
  4914, 4751, 4595, 4445,
  4300, 4161, 4027, 3898,
  3774, 3654, 3539, 3427,
  3320, 3217, 3118, 3022,
  2929, 2840, 2754, 2671,
  2590, 2513, 2438, 2366,
  2296, 2229, 2164, 2101,
};


void OLLO::begin(int devNum){
	if( devNum == 0 ){
			return;
	}
	mMot_plus = 0;
	mMot_minus = 0;


  pinMode(OLLO_SLEEP, OUTPUT);
  digitalWrite(OLLO_SLEEP, HIGH);

	switch(devNum){
	case 1:
		pinMode(PORT1_SIG1, OUTPUT); //RED  (right)
		pinMode(PORT1_SIG2, OUTPUT); //BLUE (left)
		pinMode(PORT1_ADC, INPUT_ANALOG); //ADC input
		break;
	case 2:
		pinMode(PORT2_SIG1, OUTPUT); //RED  (right)
		pinMode(PORT2_SIG2, OUTPUT); //BLUE (left)
		pinMode(PORT2_ADC, INPUT_ANALOG);//ADC input
		break;
	case 3:
		pinMode(PORT3_SIG1, OUTPUT); //RED  (right)
		pinMode(PORT3_SIG2, OUTPUT); //BLUE (left)
		pinMode(PORT3_ADC, INPUT_ANALOG);//ADC input
		break;
	case 4:
		pinMode(PORT4_SIG1, OUTPUT); //RED  (right)
		pinMode(PORT4_SIG2, OUTPUT); //BLUE (left)
		pinMode(PORT4_ADC, INPUT_ANALOG);//ADC input
		break;
	default:
		break;
	}
}
void OLLO::begin(int devNum, OlloDeviceIndex device_index){ //MAGNETIC SENSOR, Button, IR Sensor, and etc...
	if( devNum == 0 ){
			return;
	}
	mMot_plus = 0;
	mMot_minus = 0;

  pinMode(OLLO_SLEEP, OUTPUT);
  digitalWrite(OLLO_SLEEP, HIGH);
    
	switch(devNum){
	case 1:
		if(device_index == TOUCH_SENSOR || device_index == PIR_SENSOR || device_index == MAGNETIC_SENSOR){
			pinMode(PORT1_ADC, INPUT_PULLUP);
		}else if(device_index == ULTRASONIC_SENSOR || device_index == COLOR_SENSOR || device_index == TEMPERATURE_SENSOR ){
			pinMode(PORT1_ADC, INPUT_ANALOG);
		}
		else{
			pinMode(PORT1_ADC, INPUT_ANALOG); //ADC input
		}

		pinMode(PORT1_SIG1, OUTPUT); //SIG1
		pinMode(PORT1_SIG2, OUTPUT); //SIG2
		if(device_index == IR_SENSOR ){
			digitalWrite(PORT1_SIG1,LOW); //SIG1 set to LOW
			digitalWrite(PORT1_SIG2,LOW); //SIG2 set to LOW
		}
		break;
	case 2:
		if(device_index == TOUCH_SENSOR || device_index == PIR_SENSOR || device_index == MAGNETIC_SENSOR){
			pinMode(PORT2_ADC, INPUT_PULLUP);
		}else if(device_index == ULTRASONIC_SENSOR || device_index == COLOR_SENSOR || device_index == TEMPERATURE_SENSOR ){
			pinMode(PORT2_ADC, INPUT_ANALOG);//ADC input
		}else{
			pinMode(PORT2_ADC, INPUT_ANALOG);//ADC input
		}

		pinMode(PORT2_SIG1, OUTPUT); //SIG1
		pinMode(PORT2_SIG2, OUTPUT); //SIG2
		if(device_index == IR_SENSOR ){
			digitalWrite(PORT2_SIG1,LOW); //set to LOW
			digitalWrite(PORT2_SIG2,LOW); //set to LOW
		}
		break;
	case 3:
		if(device_index == TOUCH_SENSOR || device_index == PIR_SENSOR || device_index == MAGNETIC_SENSOR){
			pinMode(PORT3_ADC, INPUT_PULLUP);
		}else if(device_index == ULTRASONIC_SENSOR || device_index == COLOR_SENSOR || device_index == TEMPERATURE_SENSOR ){
			pinMode(PORT3_ADC, INPUT_ANALOG);//ADC input
		}else{
			pinMode(PORT3_ADC, INPUT_ANALOG);//ADC input
		}

		pinMode(PORT3_SIG1, OUTPUT); //SIG1
		pinMode(PORT3_SIG2, OUTPUT); //SIG2
		if(device_index == IR_SENSOR ){
			digitalWrite(PORT3_SIG1,LOW); //set SIG1 to LOW
			digitalWrite(PORT3_SIG2,LOW); //set SIG2 to LOW
		}
		break;
	case 4:
		if(device_index == TOUCH_SENSOR || device_index == PIR_SENSOR || device_index == MAGNETIC_SENSOR ){
			pinMode(PORT4_ADC, INPUT_PULLUP);
		}else if(device_index == ULTRASONIC_SENSOR || device_index == COLOR_SENSOR || device_index == TEMPERATURE_SENSOR ){
			pinMode(PORT4_ADC, INPUT_ANALOG);//ADC input
		}else{
			pinMode(PORT4_ADC, INPUT_ANALOG);//ADC input
		}

		pinMode(PORT4_SIG1, OUTPUT); //SIG1
		pinMode(PORT4_SIG2, OUTPUT); //SIG2
		if(device_index == IR_SENSOR ){
			digitalWrite(PORT4_SIG1,LOW); //set SIG1 to LOW
			digitalWrite(PORT4_SIG2,LOW); //set SIG2 to LOW
		}
		break;
	default:
		break;
	}
}

void OLLO::begin(int devNum, OlloDeviceIndex device_index, voidFuncPtr handler){ //Button with handler function.
	if( devNum == 0 ){
		return;
	}
	switch(devNum){
	case 1:
		if(device_index == TOUCH_SENSOR){
			pinMode(PORT1_ADC, INPUT_PULLUP);
			attachInterrupt(PORT1_ADC,handler, RISING);
		}
		break;
	case 2:
		if(device_index == TOUCH_SENSOR){
			pinMode(PORT2_ADC, INPUT_PULLUP);
			attachInterrupt(PORT2_ADC,handler, RISING);
		}
		break;
	case 3:
		if(device_index == TOUCH_SENSOR){
			pinMode(PORT3_ADC, INPUT_PULLUP);
			attachInterrupt(PORT3_ADC,handler, RISING);
		}
		break;
	case 4:
		if(device_index == TOUCH_SENSOR){
			pinMode(PORT4_ADC, INPUT_PULLUP);
			attachInterrupt(PORT4_ADC,handler, RISING);
		}
		break;
	default:
		break;
	}
}


int OLLO::read(int devNum){ // general sensor reading method
	if( devNum == 0 ){
			return 0;
	}
	switch(devNum){
	case 1:
		return (int)analogRead(PORT1_ADC);
	case 2:
		return (int)analogRead(PORT2_ADC);
	case 3:
		return (int)analogRead(PORT3_ADC);
	case 4:
		return (int)analogRead(PORT4_ADC);
	default:
		return 0;
	}

}
int OLLO::read(int devNum, OlloDeviceIndex device_index){ // IR SENSOR, Button, MAGNETIC SENSOR, and etc...
	int adcValue = 0;

	signed int scount;
	word long vvalue = 0;
	uint32_t analogValue;
	int distance_value = 0;
	int dis_value = 0;

	signed int average_value = 0;
	if( devNum == 0 ){
		return 0;
	}
	switch(devNum){
	case 1:
		if(device_index == IR_SENSOR){
			digitalWrite(PORT1_SIG2, HIGH);
			delayMicroseconds(15);
			adcValue = analogRead(PORT1_ADC);
			digitalWrite(PORT1_SIG2, LOW);
			return adcValue;
		}else if(device_index == MAGNETIC_SENSOR || device_index == TOUCH_SENSOR  || device_index == PIR_SENSOR){
			return digitalRead(PORT1_ADC);
		}else if(device_index == ULTRASONIC_SENSOR){
			distance_value = (int)analogRead(PORT1_ADC);
			dis_value = (((distance_value * 0.24)/4) - 3);
			average_cnt++;
			average_value+=dis_value;
			if(average_cnt >= 100){
				average_value/=100;
				average_cnt = 0;
			}
			  return average_value;
		}else if(device_index == TEMPERATURE_SENSOR){
			analogValue = analogRead(PORT1_ADC);
			vvalue = (4095 - analogValue) * 10000 /analogValue;
			for(scount = -20; scount < 140; scount++){
				if(vvalue > gwTheRmistor[scount +20]){
				      return scount;
				}
			}
		}
		else if(device_index == COLOR_SENSOR){
			return this->detectColor(1);
		}else{
			return (int)analogRead(PORT1_ADC);
		}
		break;
	case 2:
		if(device_index == IR_SENSOR){
			digitalWrite(PORT2_SIG2, HIGH);//digitalWrite(PORT1_SIG2, HIGH); -> digitalWrite(PORT2_SIG2, HIGH); 140324
			delayMicroseconds(15);
			adcValue = analogRead(PORT2_ADC);//adcValue = analogRead(PORT1_ADC); -> adcValue = analogRead(PORT2_ADC); 140324
			digitalWrite(PORT2_SIG2, LOW);//digitalWrite(PORT1_SIG2, LOW); -> digitalWrite(PORT2_SIG2, LOW);
			return adcValue;
		}else if(device_index == MAGNETIC_SENSOR || device_index == TOUCH_SENSOR || device_index == PIR_SENSOR){
			return digitalRead(PORT2_ADC);
		}else if(device_index == ULTRASONIC_SENSOR){
			distance_value = (int)analogRead(PORT2_ADC); //analogRead(PORT1_ADC); -> analogRead(PORT2_ADC); 140324
			dis_value = (((distance_value * 0.24)/4) - 3);
			average_cnt++;
			average_value+=dis_value;
			if(average_cnt >= 100){
				average_value/=100;
				average_cnt = 0;
			}
			  return average_value;
		}else if(device_index == COLOR_SENSOR){
			return this->detectColor(2);
		}
		else if(device_index == TEMPERATURE_SENSOR){
			analogValue = analogRead(PORT2_ADC);
			vvalue = (4095 - analogValue) * 10000 /analogValue;
			for(scount = -20; scount < 140; scount++){
				if(vvalue > gwTheRmistor[scount +20]){
				      return scount;
				}
			}
		}
		else{
			return (int)analogRead(PORT2_ADC);
		}
		break;
	case 3:
		if(device_index == IR_SENSOR){
			digitalWrite(PORT3_SIG2, HIGH);////digitalWrite(PORT1_SIG2, HIGH); -> digitalWrite(PORT3_SIG2, HIGH); 140324
			delayMicroseconds(15);
			adcValue = analogRead(PORT3_ADC);//adcValue = analogRead(PORT1_ADC); -> adcValue = analogRead(PORT3_ADC); 140324
			digitalWrite(PORT3_SIG2, LOW);//digitalWrite(PORT1_SIG2, LOW); -> digitalWrite(PORT3_SIG2, LOW);
			return adcValue;
		}else if(device_index == MAGNETIC_SENSOR || device_index == TOUCH_SENSOR || device_index == PIR_SENSOR){
			return digitalRead(PORT3_ADC);
		}else if(device_index == ULTRASONIC_SENSOR){
			distance_value = (int)analogRead(PORT3_ADC); //analogRead(PORT1_ADC); -> analogRead(PORT3_ADC); 140324
			dis_value = (((distance_value * 0.24)/4) - 3);
			average_cnt++;
			average_value+=dis_value;
			if(average_cnt >= 100){
				average_value/=100;
				average_cnt = 0;
			}
			  return average_value;
		}else if(device_index == TEMPERATURE_SENSOR){
			analogValue = analogRead(PORT3_ADC);
			vvalue = (4095 - analogValue) * 10000 /analogValue;
			for(scount = -20; scount < 140; scount++){
				if(vvalue > gwTheRmistor[scount +20]){
				      return scount;
				}
			}
		}else if(device_index == COLOR_SENSOR){
			return OLLO::detectColor(3);
		}else{
			return (int)analogRead(PORT3_ADC);
		}
		break;
	case 4:
		if(device_index == IR_SENSOR){
			digitalWrite(PORT4_SIG2, HIGH); //digitalWrite(PORT1_SIG2, HIGH); -> digitalWrite(PORT4_SIG2, HIGH); 140324
			delayMicroseconds(15);
			adcValue = analogRead(PORT4_ADC); //adcValue = analogRead(PORT1_ADC); -> adcValue = analogRead(PORT4_ADC); 140324
			digitalWrite(PORT4_SIG2, LOW);//digitalWrite(PORT1_SIG2, LOW); -> digitalWrite(PORT4_SIG2, LOW);
			return adcValue;
		}else if(device_index == MAGNETIC_SENSOR || device_index == TOUCH_SENSOR || device_index == PIR_SENSOR ){
			return digitalRead(PORT4_ADC);
		}else if(device_index == ULTRASONIC_SENSOR){
			distance_value = (int)analogRead(PORT4_ADC); //analogRead(PORT1_ADC); -> analogRead(PORT4_ADC); 140324
			dis_value = (((distance_value * 0.24)/4) - 3);
			average_cnt++;
			average_value+=dis_value;
			if(average_cnt >= 100){
				average_value/=100;
				average_cnt = 0;
			}
			  return average_value;
		}else if(device_index == TEMPERATURE_SENSOR){
			analogValue = analogRead(PORT4_ADC);// 2014-04-17 shin
			vvalue = (4095 - analogValue) * 10000 /analogValue;
			for(scount = -20; scount < 140; scount++){
				if(vvalue > gwTheRmistor[scount +20]){
				      return scount;
				}
			}
		}else if(device_index == COLOR_SENSOR){
			return OLLO::detectColor(4);
		}else{
			return (int)analogRead(PORT4_ADC);
		}
		break;
	default:
		return 0;
	}
	return 0;
}

int OLLO::read(int devNum, OlloDeviceIndex device_index, ColorIndex sub_index){ //COLOR SENSOR
	//int adcValue = 0;
	if( devNum == 0 ){
		return 0;
	}
	if(device_index == COLOR_SENSOR){
		setColor(sub_index);
	}else{
		return 0;
	}

	switch(devNum){
	case 1:
		 digitalWrite(PORT1_SIG1, mMot_minus);
		 digitalWrite(PORT1_SIG2, mMot_plus);
		 delay(5); // after 20ms, read analog
		 return (((int)analogRead(PORT1_ADC))/4);

	case 2:
		digitalWrite(PORT2_SIG1, mMot_minus);
		digitalWrite(PORT2_SIG2, mMot_plus);
		delay(5);
		return ((int)analogRead(PORT2_ADC));

	case 3:
		digitalWrite(PORT3_SIG1, mMot_minus);
		digitalWrite(PORT3_SIG2, mMot_plus);
		delay(5);
		return ((int)analogRead(PORT3_ADC)/4);

	case 4:
		digitalWrite(PORT4_SIG1, mMot_minus);
		digitalWrite(PORT4_SIG2, mMot_plus);
		delay(5);
		return ((int)analogRead(PORT4_ADC)/4);

	default:
		return 0;
	}
}
void OLLO::writeLED(int devNum, uint8_t leftVal, uint8_t rightVal ){
	if( leftVal >1 || rightVal >1 || devNum == 0 ){
		return;
	}

	switch(devNum){
		case 1:
			 digitalWrite(PORT1_SIG1,rightVal);
			 digitalWrite(PORT1_SIG2,leftVal);
			break;
		case 2:
			digitalWrite(PORT2_SIG1,rightVal);
			digitalWrite(PORT2_SIG2,leftVal);
			break;
		case 3:
			digitalWrite(PORT3_SIG1,rightVal);
			digitalWrite(PORT3_SIG2,leftVal);
			break;
		case 4:
			digitalWrite(PORT4_SIG1,rightVal);
			digitalWrite(PORT4_SIG2,leftVal);
			break;
		default:
			break;
		}

}
void OLLO::write(int devNum, uint8_t leftVal, uint8_t rightVal ){
	if( leftVal >1 || rightVal >1 || devNum == 0 ){
		return;
	}

	switch(devNum){
		case 1:
			 digitalWrite(6,rightVal);
			 digitalWrite(7,leftVal);
			break;
		case 2:
			digitalWrite(8,rightVal);
			digitalWrite(9,leftVal);
			break;
		case 3:
			digitalWrite(10,rightVal);
			digitalWrite(11,leftVal);
			break;
		case 4:
			digitalWrite(12,rightVal);
			digitalWrite(13,leftVal);
			break;
		default:
			break;
		}

}
void OLLO::write(int devNum, uint8_t leftVal, uint8_t centerVal, uint8_t rightVal){

	if( leftVal >1 || rightVal >1 || centerVal > 1 || devNum == 0){
		return;
	}

	switch(devNum){
		case 1:
			 digitalWrite(6,rightVal);
			 digitalWrite(2,centerVal);
			 digitalWrite(7,leftVal);
			break;
		case 2:
			digitalWrite(8,rightVal);
			digitalWrite(3,centerVal);
			digitalWrite(9,leftVal);
			break;
		case 3:
			digitalWrite(10,rightVal);
			digitalWrite(0,centerVal);
			digitalWrite(11,leftVal);
			break;
		case 4:
			digitalWrite(12,rightVal);
			digitalWrite(1,centerVal);
			digitalWrite(13,leftVal);
			break;
		default:
			break;
	}

}

void OLLO::setColor(ColorIndex colorIndex){
	switch(colorIndex){
			case RED: //Red
				mMot_minus = LOW;
				mMot_plus = LOW;
				break;
			case GREEN://Green
				mMot_minus = LOW;
				mMot_plus = HIGH;
				break;
			case BLUE://Blue
				mMot_minus = HIGH;
				mMot_plus = LOW;
				break;
			default:
				break;
		}

}


int OLLO::detectColor(uint8_t port){

	// int temp_red,temp_green,temp_blue;

	// temp_red = 0;
	// temp_green = 0;
	// temp_blue= 0;
	int lColor[3]= {0,0,0};
	int lRed,lGreen,lBlue;
	//int bMaxColor, bMinColor;
	//bMaxColor=0;
	//bMinColor=0;
	int bColorResult;
	bColorResult=0;

	lRed = this->read(port, COLOR_SENSOR, RED);
//for(i=0; i < 3; i++)

	lGreen = (this->read(port, COLOR_SENSOR, GREEN));
//for(i=0; i < 3; i++)

	lBlue = this->read(port, COLOR_SENSOR, BLUE);

	if(lRed >= lGreen && lRed >= lBlue)
	{
	       //bMaxColor = 1;
	       lColor[0] = lRed;
	}
	else if(lGreen >= lRed && lGreen >= lBlue)
	{
	       //bMaxColor = 2;
	       lColor[0] = lGreen;
	}

	else if(lBlue >= lRed && lBlue >= lGreen)
	{
	       //bMaxColor = 3;
	       lColor[0] = lBlue;
	}
	if(lRed <= lGreen && lRed <= lBlue)
	{
	       //bMinColor = 1;
	       lColor[2] = lRed;
	}
	else if(lGreen <= lRed && lGreen <= lBlue)
	{
	       //bMinColor = 2;
	       lColor[2] = lGreen;
	}

	else if(lBlue <= lRed && lBlue <= lGreen)
	{
	       //bMinColor = 3;
	       lColor[2] = lBlue;
	}

	lColor[1] = lRed + lGreen + lBlue - lColor[0] - lColor[2];

	uint32_t RtoB = lRed * 100 / lBlue;
	uint32_t GtoB = lGreen * 100 / lBlue;
	uint32_t GtoR = lGreen * 100 / lRed;

//2014-03-24 sm6787@robotis.com
	if(lColor[0] < 90 || ( lColor[0] < 180 				 &&
						   RtoB > 50 					 &&
						   (GtoB < 110 || GtoR < 130) 	 &&
						   (GtoB + GtoR < 230)  		 &&
						   ((lColor[2] * 100 / lColor[0]) > 75)
						  )
	   ){//end of if()
	       bColorResult = 2; // blackz
	}
	else if((lColor[2] > 550) || ((lColor[2] > 200) && (lColor[0] > 300) && (lColor[2] * 100 / lColor[0] > 75) && (GtoB < 105)))
	       bColorResult = 1; // white
	else if(RtoB > 170 && GtoB > 130)
	       bColorResult = 6; // yellow
	else if(RtoB > 170 && GtoB <= 130)
	       bColorResult = 3; // red
	else if(GtoB > 80 && GtoR >= 100)//90 110
	       bColorResult = 4; // green
	else if(RtoB < 70 && GtoB <= 85)
	       bColorResult = 5; // blue
	else
	       bColorResult = 0; // unknown

	if(bColorResult == before_color_num){
		before_color_cnt++;
		if(before_color_cnt >= 10){
			//before_color_cnt = 0;
			return bColorResult;
		}
	}
	else{
		before_color_cnt = 0;
	}

	before_color_num = bColorResult;
	return 0;
}
