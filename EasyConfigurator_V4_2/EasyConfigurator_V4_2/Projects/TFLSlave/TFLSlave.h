#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project TFLSlave.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	10
#define CUST_BYTE_NUM_IN	2
#define TOT_BYTE_NUM_ROUND_OUT	12
#define TOT_BYTE_NUM_ROUND_IN	4


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		int16_t     Control;
		int16_t     TFLData1;
		int16_t     TFLData2;
		int16_t     TFLData3;
		int16_t     TFLData4;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		int16_t     Status;
	}Cust;
} PROCBUFFER_IN;

#endif