/*
 * dxl_constants.h
 *
 *  Created on: 2012. 12. 17.
 *      Author: ROBOTIS(jason@robotis.com)
 */

#ifndef DXL_CONSTANTS_H_
#define DXL_CONSTANTS_H_


#define BROADCAST_ID		(254) /* 0xFE */

/*
 * Instruction command
 * */

/*
#define INST_PING           0x01
#define INST_READ           0x02
#define INST_WRITE          0x03
#define INST_REG_WRITE      0x04
#define INST_ACTION         0x05
#define INST_RESET          0x06
#define INST_DIGITAL_RESET  0x07
#define INST_SYSTEM_READ    0x0C
#define INST_SYSTEM_WRITE   0x0D
#define INST_SYNC_WRITE     0x83
#define INST_SYNC_REG_WRITE 0x84
*/
enum DXL_INSTRUCTION{  //2014-04-02 ROBOTIS DXL Protocol 2.0
	INST_PING           = 1,
	INST_READ           = 2,
	INST_WRITE          = 3,
	INST_REG_WRITE      = 4,
	INST_ACTION         = 5,
	INST_FACTORY_RESET  = 6,
	INST_REBOOT         = 8,
	INST_SYSTEM_WRITE   = 13,   // 0x0D
	INST_STATUS         = 85,   // 0x55
	INST_SYNC_READ      = 130,  // 0x82
	INST_SYNC_WRITE     = 131,  // 0x83
	INST_BULK_READ      = 146,  // 0x92
	INST_BULK_WRITE     = 147   // 0x93
};


/*
 * defines error message for protocol 1.0
 * */
#define ERRBIT_VOLTAGE      (1)
#define ERRBIT_ANGLE        (2)
#define ERRBIT_OVERHEAT     (4)
#define ERRBIT_RANGE        (8)
#define ERRBIT_CHECKSUM     (16)
#define ERRBIT_OVERLOAD     (32)
#define ERRBIT_INSTRUCTION  (64)

/*
 * defines error message for protocol 2.0
 * */
#define ERRBIT_RESULT_FAIL  (1)
#define ERRBIT_INST_ERROR   (2)
#define ERRBIT_CRC          (4)
#define ERRBIT_DATA_RANGE	  (8)
#define ERRBIT_DATA_LENGTH	(16)
#define ERRBIT_DATA_LIMIT	  (32)
#define ERRBIT_ACCESS		    (64)

/*
 * defines message of communication
 * */
#define	COMM_TXSUCCESS		  (0)
#define COMM_RXSUCCESS		  (1)
#define COMM_TXFAIL			    (2)
#define COMM_RXFAIL			    (3)
#define COMM_TXERROR		    (4)
#define COMM_RXWAITING		  (5)
#define COMM_RXTIMEOUT		  (6)
#define COMM_RXCORRUPT		  (7)

/* timing defines */
#define RX_TIMEOUT_COUNT2		    (1600L) //(1000L) //porting
#define NANO_TIME_DELAY			    (12000) //ydh added 20111228 -> 20120210 edited ydh
//#define RX_TIMEOUT_COUNT1  	  (RX_TIMEOUT_COUNT2*90L)// -> 110msec  before Ver 1.11e
#define RX_TIMEOUT_COUNT1  		  (RX_TIMEOUT_COUNT2*128L)//  -> 156msec for ID 110 safe access after Ver 1.11f //porting ydh

#define DXL_RX_BUF_SIZE         0x3FF
#define DXL_PARAMETER_BUF_SIZE  128


///////////////// utility for value ///////////////////////////
#define DXL_MAKEWORD(a, b)      ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b)     ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)           ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l)           ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)           ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)           ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

#define DXL_PACKET_TYPE1	1 //2014-04-08 sm6787@robotis.com ->  DXL protocol 1.0 packet type
#define DXL_PACKET_TYPE2	2 //2014-04-08 sm6787@robotis.com ->  DXL protocol 1.0 packet type


#define DXL_SERIAL              Serial3


#endif /* DXL_CONSTANTS_H_ */
