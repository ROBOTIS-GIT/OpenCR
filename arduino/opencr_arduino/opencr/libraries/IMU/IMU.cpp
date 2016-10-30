//----------------------------------------------------------------------------
//    프로그램명 	:
//
//    만든이     	: Made by Baram ( chcbaram@paran.com )
//
//    날  짜     :
//
//    최종 수정  	:
//
//    MPU_Type	:
//
//    파일명     	: IMU.ino
//----------------------------------------------------------------------------
/*
	Based on Multiwii : https://github.com/multiwii/multiwii-firmware
*/



#include <Arduino.h>
//#include <EEPROM.h>
#include "IMU.h"
//#include "I2C_CM.h"







//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC
   Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time
   Comment this if  you do not want filter at all.
   unit = n power of 2 */
// this one is also used for ALT HOLD calculation, should not be changed
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 4 // that means a LPF of 16
#endif

/* Set the Gyro Weight for Gyro/Acc complementary filter
   Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR 10 //  that means a CMP_FACTOR of 1024 (2^10)
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter
   Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#define GYR_CMPFM_FACTOR 8 // that means a CMP_FACTOR of 256 (2^8)




typedef struct
{
	int32_t X,Y,Z;
} t_int32_t_vector_def;

typedef struct
{
	uint16_t XL; int16_t X;
  	uint16_t YL; int16_t Y;
  	uint16_t ZL; int16_t Z;
} t_int16_t_vector_def;

// note: we use implicit first 16 MSB bits 32 -> 16 cast. ie V32.X>>16 = V16.X
typedef union
{
	int32_t A32[3];
	t_int32_t_vector_def V32;
	int16_t A16[6];
	t_int16_t_vector_def V16;
} t_int32_t_vector;





int32_t mul(int16_t a, int16_t b);
void rotateV32( t_int32_t_vector *v,int16_t* delta);
float InvSqrt (float x);
int16_t _atan2(int32_t y, int32_t x);






/*---------------------------------------------------------------------------
     TITLE   : BLE
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
cIMU::cIMU()
{
	accZ = 0;
	bConnected = false;
}




/*---------------------------------------------------------------------------
     TITLE   : begin
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
uint8_t cIMU::begin( void )
{
	uint8_t err_code = IMU_OK;

  bConnected = SEN.begin();


  if( bConnected == true )
  {
	  //SEN.gyro_init();
	  //SEN.acc_init();
  }

  /*
	if( I2C.i2c_errors_count > 0 )
	{
		err_code = IMU_ERR_I2C;
	}
	else
	{
		bConnected = true;
	}
  */
	return err_code;
}





/*---------------------------------------------------------------------------
     TITLE   : update
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
uint16_t cIMU::update( uint32_t priod_us )
{
	uint16_t ret_time = 0;

	static uint32_t tTime;


	if( (micros()-tTime) >= priod_us )
	{
		ret_time = micros()-tTime;
    tTime = micros();

		computeIMU();

		gyroData[0] = SEN.gyroData[0];
		gyroData[1] = SEN.gyroData[1];
		gyroData[2] = SEN.gyroData[2];

    //Serial.println(gyroData[0]);
	}

	return ret_time;
}





#if 1
/*---------------------------------------------------------------------------
     TITLE   : compute
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cIMU::computeIMU( void )
{
	uint8_t axis;
	static int16_t gyroADCprevious[3] = {0,0,0};
	static int16_t gyroADCinter[3] = {0,};

  	uint16_t timeInterleave = 0;

	SEN.acc_get_adc();

  //Serial.println(SEN.gyroADC[0]);

  getEstimatedAttitude();

	SEN.gyro_get_adc();
	for (axis = 0; axis < 3; axis++)
	{
		gyroADCinter[axis] =  SEN.gyroADC[axis];
	}

	timeInterleave=micros();

	//uint8_t t=0;
	//while((int16_t)((uint16_t)micros()-timeInterleave)<650) t=1; //empirical, interleaving delay between 2 consecutive reads

	SEN.gyro_get_adc();


	for (axis = 0; axis < 3; axis++)
	{
    	gyroADCinter[axis] =  SEN.gyroADC[axis]+gyroADCinter[axis];
		// empirical, we take a weighted value of the current and the previous values
		SEN.gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis])/3;
		gyroADCprevious[axis] = gyroADCinter[axis]>>1;
	}
}




/*---------------------------------------------------------------------------
     TITLE   : getEstimatedAttitude
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
void cIMU::getEstimatedAttitude( void )
{
	uint8_t axis;
	int32_t accMag = 0;
	float scale;
	int16_t deltaGyroAngle16[3];
	static t_int32_t_vector EstG = {0,0,(int32_t)ACC_1G<<16};
    static t_int32_t_vector EstM = {0,(int32_t)1<<24,0};

	static uint32_t LPFAcc[3];
	float invG; // 1/|G|
	static int16_t accZoffset = 0;
	int32_t accZ_tmp=0;
	static uint32_t previousT;
	uint32_t currentT = micros();

	// unit: radian per bit, scaled by 2^16 for further multiplication
	// with a delta time of 3000 us, and GYRO scale of most gyros, scale = a little bit less than 1
	scale = (currentT - previousT) * (GYRO_SCALE * 65536);
	previousT = currentT;

	// Initialization
	for (axis = 0; axis < 3; axis++)
	{
		// valid as long as LPF_FACTOR is less than 15
		SEN.accSmooth[axis]  = LPFAcc[axis]>>ACC_LPF_FACTOR;
		LPFAcc[axis]    += SEN.accADC[axis] - SEN.accSmooth[axis];
		// used to calculate later the magnitude of acc vector
		accMag   += mul(SEN.accSmooth[axis] , SEN.accSmooth[axis]);
		// unit: radian scaled by 2^16
		// imu.gyroADC[axis] is 14 bit long, the scale factor ensure deltaGyroAngle16[axis] is still 14 bit long
		deltaGyroAngle16[axis] = SEN.gyroADC[axis]  * scale;
	}

	// we rotate the intermediate 32 bit vector with the radian vector (deltaGyroAngle16), scaled by 2^16
	// however, only the first 16 MSB of the 32 bit vector is used to compute the result
	// it is ok to use this approximation as the 16 LSB are used only for the complementary filter part
	rotateV32(&EstG,deltaGyroAngle16);
	rotateV32(&EstM,deltaGyroAngle16);

	// Apply complimentary filter (Gyro drift correction)
	// If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
	// To do that, we just skip filter, as EstV already rotated by Gyro
	for (axis = 0; axis < 3; axis++)
	{
	    if ( (int16_t)(0.85*ACC_1G*ACC_1G/256) < (int16_t)(accMag>>8) && (int16_t)(accMag>>8) < (int16_t)(1.15*ACC_1G*ACC_1G/256) )
	      EstG.A32[axis] += (int32_t)(SEN.accSmooth[axis] - EstG.A16[2*axis+1])<<(16-GYR_CMPF_FACTOR);
	    accZ_tmp += mul(SEN.accSmooth[axis] , EstG.A16[2*axis+1]);
	}

  	/*
	if (EstG.V16.Z > ACCZ_25deg)
    	f.SMALL_ANGLES_25 = 1;
	else
		f.SMALL_ANGLES_25 = 0;
	*/


	// Attitude of the estimated vector
	int32_t sqGX_sqGZ = mul(EstG.V16.X,EstG.V16.X) + mul(EstG.V16.Z,EstG.V16.Z);
	invG = InvSqrt(sqGX_sqGZ + mul(EstG.V16.Y,EstG.V16.Y));

	angle[ROLL]  = _atan2(EstG.V16.X , EstG.V16.Z);
	angle[PITCH] = _atan2(EstG.V16.Y , InvSqrt(sqGX_sqGZ)*sqGX_sqGZ);

	//note on the second term: mathematically there is a risk of overflow (16*16*16=48 bits). assumed to be null with real values
	angle[YAW]   = _atan2(
    	mul(EstM.V16.Z , EstG.V16.X) - mul(EstM.V16.X , EstG.V16.Z),
    	(EstM.V16.Y * sqGX_sqGZ  - (mul(EstM.V16.X , EstG.V16.X) + mul(EstM.V16.Z , EstG.V16.Z)) * EstG.V16.Y)*invG );
	angle[YAW] /= 10;


	// projection of ACC vector to global Z, with 1G subtructed
	// Math: accZ = A * G / |G| - 1G
	accZ = accZ_tmp *  invG;
	//if (!f.ARMED)
	{
    	accZoffset -= accZoffset>>3;
    	accZoffset += accZ;
	}
	accZ -= accZoffset>>3;
}

#endif






/*---------------------------------------------------------------------------
     TITLE   : _atan2
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
//return angle , unit: 1/10 degree
int16_t _atan2(int32_t y, int32_t x)
{
	float z = y;
	int16_t a;
	uint8_t c;


	c = abs(y) < abs(x);

	if ( c ) {z = z / x;} else {z = x / z;}

	a = 2046.43 * (z / (3.5714 +  z * z));

	if ( c )
	{
		if (x<0)
		{
			if (y<0) a -= 1800;
			else a += 1800;
		}
	}
	else
	{
		a = 900 - a;
		if (y<0) a -= 1800;
	}

	return a;
}




/*---------------------------------------------------------------------------
     TITLE   : InvSqrt
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
float InvSqrt (float x)
{
  union{
    int32_t i;
    float   f;
  } conv;


  conv.f = x;
  conv.i = 0x5f1ffff9 - (conv.i >> 1);

  return conv.f * (1.68191409f - 0.703952253f * x * conv.f * conv.f);
}







/*---------------------------------------------------------------------------
     TITLE   : mul
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
int32_t mul(int16_t a, int16_t b)
{
  int32_t r;

  r = (int32_t)a * (int32_t)b;
  return r;
}





/*---------------------------------------------------------------------------
     TITLE   : rotateV32
     WORK    :
     ARG     : void
     RET     : void
---------------------------------------------------------------------------*/
// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV32( t_int32_t_vector *v,int16_t* delta)
{
  int16_t X = v->V16.X;
  int16_t Y = v->V16.Y;
  int16_t Z = v->V16.Z;

  v->V32.Z -=  mul(delta[ROLL]  ,  X)  + mul(delta[PITCH] , Y);
  v->V32.X +=  mul(delta[ROLL]  ,  Z)  - mul(delta[YAW]   , Y);
  v->V32.Y +=  mul(delta[PITCH] ,  Z)  + mul(delta[YAW]   , X);
}
