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

#ifndef RC100_H_
#define RC100_H_

#include "variant.h"

////////// define RC-100 button key value ////////////////
#define RC100_BTN_U		(1)
#define RC100_BTN_D		(2)
#define RC100_BTN_L		(4)
#define RC100_BTN_R		(8)
#define RC100_BTN_1		(16)
#define RC100_BTN_2		(32)
#define RC100_BTN_3		(64)
#define RC100_BTN_4		(128)
#define RC100_BTN_5		(256)
#define RC100_BTN_6		(512)

#define PACKET_DATA0    		  2
#define INVERSE_PACKET_DATA0 	3
#define PACKET_DATA1    		  4
#define INVERSE_PACKET_DATA1 	5
#define PACKET_LENGTH 			  6

class RC100 {
 public:
	RC100();
	virtual ~RC100();

	void begin(int num);
	void end(void);
	int writeData(int data);
	int available(void);
	int readData(void);
	void writeRaw(byte temp);
	byte readRaw(void);
	void setChannel(byte IR_channel);

 private:
  volatile byte gbPacketWritePointer;
	volatile byte gbPacketReadPointer;
	volatile byte gbpPacketDataBuffer[16+1+16];
	volatile byte gbpPacket[PACKET_LENGTH+2];
	volatile byte gbNewPacket;
	volatile word gwZigbeeRxData;
	byte gbIRChannel;

	volatile byte check_mode;

	unsigned char gbRcvPacket[6];
	unsigned char gbRcvPacketNum;
	unsigned short gwRcvData;
	unsigned char gbRcvFlag;

	int rc100_hal_tx(unsigned char *pPacket, int numPacket);
	int rc100_hal_rx(unsigned char *pPacket, int numPacket);

	///////////// device control methods ////////////////////////
	int rc100Initialize(unsigned int baudrate);
	void rc100Terminate(void);
	byte CheckNewArrive(void);
	////////// communication methods ///////////////////////
	int rc100TxData(int data);
	int rc100RxCheck(void);
	int rc100RxData(void);
	byte RxDByteSerial2(void);
	void TxDByteSerial2(byte bTxdData);
	int rc100channel(byte IR_channel);
};
#endif /* RC100_H_ */
