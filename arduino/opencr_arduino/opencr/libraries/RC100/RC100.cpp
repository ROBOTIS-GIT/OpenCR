/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Taehoon Lim (Darby) */

#include "RC100.h"

RC100::RC100()
{
	// TODO Auto-generated constructor stub

}

RC100::~RC100()
{
	// TODO Auto-generated destructor stub

}

void RC100::begin(int num)
{
	if(num == 1) rc100Initialize(57600);
	else if(num == 2) rc100Initialize(1900);
	check_mode = num;
}

void RC100::end(void)
{
	rc100Terminate();
}

int RC100::writeData(int data)
{
	return rc100TxData(data);
}

void RC100::writeRaw(byte temp)
{
	TxDByteSerial2(temp);
}

byte RC100::readRaw(void)
{
	return RxDByteSerial2();
}

int RC100::available(void)
{
	return rc100RxCheck();
}

int RC100::readData(void)
{
	return rc100RxData();
}

void RC100::setChannel(byte IR_channel)
{
	rc100channel(IR_channel);
}

void RC100::TxDByteSerial2(byte bTxdData)
{
	/*
		OpenCM 9.04 library
	*/

	// /*USART_SendData(USART1,bTxdData);
	// while( USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET );
	// */
	// USART2->regs->DR = (bTxdData & (u16)0x01FF);
	// while( (USART2->regs->SR & ((u16)0x0040)) == RESET );
}

byte RC100::RxDByteSerial2(void)
{
	/*
		OpenCM 9.04 library
	*/

	// byte bTemp;
	//
	// bTemp = gbpPacketDataBuffer[gbPacketReadPointer++];
	//
	// gbPacketReadPointer = gbPacketReadPointer & 0x1F; //added 2012-11-23
	// return bTemp;
}

int RC100::rc100_hal_tx(unsigned char *pPacket, int numPacket)
{
	// Transmiting date
	// *pPacket: data array pointer
	// numPacket: number of data array
	// Return: number of data transmitted. -1 is error.

	// unsigned char i;
	// for(i=0 ; i<numPacket; i++  ){
	// 	TxDByteSerial2(pPacket[i]);
	// }
	//
	// return numPacket;
}

byte RC100::CheckNewArrive(void)
{
	/*
		OpenCM 9.04 library
	*/

	// if(gbPacketReadPointer != gbPacketWritePointer)
	// 	return 1;
	// else
	// 	return 0;
}

int RC100::rc100_hal_rx(unsigned char *pPacket, int numPacket)
{
	// Recieving date
	// *pPacket: data array pointer
	// numPacket: number of data array
	// Return: number of data recieved. -1 is error.

	unsigned char i = 0;
	gbPacketReadPointer = 0;

	while (gbPacketReadPointer < numPacket)
	{
		if (Serial2.available())
		{
			gbpPacketDataBuffer[gbPacketReadPointer] = Serial2.read();
			pPacket[gbPacketReadPointer] = gbpPacketDataBuffer[gbPacketReadPointer];
			gbPacketReadPointer++;
		}
		else
		{
			return gbPacketReadPointer;
		}
	}

	return numPacket;
}

int RC100::rc100Initialize(unsigned int baudrate)
{
/*
 *  Opening device
 *
 * */

  Serial2.begin(baudrate);

	gbRcvFlag = 0;
	gwRcvData = 0;
	gbRcvPacketNum = 0;
	/*Clear rx tx uart2 buffer */
	gbPacketWritePointer = 0;
	gbPacketReadPointer = 0;

	return 1;
}

void RC100::rc100Terminate(void)
{
	Serial2.end();
}

int RC100::rc100TxData(int data)
{
	unsigned char SndPacket[6];
	unsigned short word = (unsigned short)data;
	unsigned char lowbyte = (unsigned char)(word & 0xff);
	unsigned char highbyte = (unsigned char)((word >> 8) & 0xff);

	SndPacket[0] = 0xff;
	SndPacket[1] = 0x55;
	SndPacket[2] = lowbyte;
	SndPacket[3] = ~lowbyte;
	SndPacket[4] = highbyte;
	SndPacket[5] = ~highbyte;

	if( rc100_hal_tx( SndPacket, 6 ) != 6 )
		return 0;

	return 1;
}

int RC100::rc100RxCheck(void)
{
	byte bChannel = 0;
	int RcvNum;
	unsigned char checksum;
	int i, j;
	if(check_mode == 1)
	{
		if(gbRcvFlag == 1)
			return 1;

		// Fill packet buffer
		if(gbRcvPacketNum < 6)
		{
			RcvNum = rc100_hal_rx( &gbRcvPacket[gbRcvPacketNum], (6 - gbRcvPacketNum) );
			if( RcvNum != -1 )
				gbRcvPacketNum += RcvNum;
		}

		// Find header
		if(gbRcvPacketNum >= 2)
		{
			for (i=0; i<gbRcvPacketNum; i++)
			{
				if(gbRcvPacket[i] == 0xff)
				{
					if(i <= (gbRcvPacketNum - 2))
					{
						if(gbRcvPacket[i+1] == 0x55)
							break;
					}
				}
			}

			if(i > 0)
			{
				if(i == gbRcvPacketNum)
				{
					// Can not find header
					if(gbRcvPacket[i - 1] == 0xff)
						i--;
				}

				// Remove data before header
				for( j=i; j<gbRcvPacketNum; j++)
				{
					gbRcvPacket[j - i] = gbRcvPacket[j];
				}
				gbRcvPacketNum -= i;
			}
		}

		// Verify packet
		if(gbRcvPacketNum == 6)
		{
			if(gbRcvPacket[0] == 0xff && gbRcvPacket[1] == 0x55)
			{
				checksum = ~gbRcvPacket[3];
				if(gbRcvPacket[2] == checksum)
				{
					checksum = ~gbRcvPacket[5];
					if(gbRcvPacket[4] == checksum)
					{
						gwRcvData = (unsigned short)((gbRcvPacket[4] << 8) & 0xff00);
						gwRcvData += gbRcvPacket[2];
						gbRcvFlag = 1;
					}
				}
			}
			gbRcvPacket[0] = 0x00;
			gbRcvPacketNum = 0;
		}
	}
////////////////////////////////////////////////////////////////////////////////
	else if(check_mode == 2)
	{
		while(CheckNewArrive())
		{
			gwZigbeeRxData = RxDByteSerial2();
			if(gbRcvPacketNum >= 3) gbRcvPacketNum = 0;

			if(gwZigbeeRxData == 0xAA)
			{
				gbRcvPacketNum = 0;
			}

			gbpPacket[gbRcvPacketNum++] = gwZigbeeRxData;

			if(gbRcvPacketNum == 3)
			{
				if( (gbpPacket[0]==0xAA) && ((gbpPacket[1]&0x80)==0) && ((gbpPacket[2]&0x80)==0) )
				{
					if( ((gbpPacket[1]+(gbpPacket[2]&0x0F)+(gbpPacket[2]>>4))&0x07)==0x07 )
					{
						if( gbpPacket[2] & 0x08 ) bChannel = 1;
						if( gbpPacket[2] & 0x04 ) bChannel|= 0x02;
						if( gbpPacket[2] & 0x02 ) bChannel|= 0x04;

						bChannel++;
						if( (gbIRChannel==0) || (bChannel==gbIRChannel) )
						{
							if( gbpPacket[2] & 0x01 )	gbpPacket[1] |= 0x80;
							gwRcvData = gbpPacket[1];
							gbRcvFlag = 1;
						}
					}
				}
		  }
	  }
	}
	return gbRcvFlag;
}

int RC100::rc100RxData(void)
{
	gbRcvFlag = 0;
	return (int)gwRcvData;
}

int RC100::rc100channel(byte IR_channel)
{
	gbIRChannel= IR_channel;
}
