#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project HUslave.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	26
#define CUST_BYTE_NUM_IN	28
#define TOT_BYTE_NUM_ROUND_OUT	28
#define TOT_BYTE_NUM_ROUND_IN	28


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		int16_t     PWM1;
		int16_t     PWM2;
		int16_t     PWM3;
		int16_t     PWM4;
		int16_t     PWM5;
		int16_t     PWM6;
		int16_t     PWM7;
		int16_t     PWM8;
		int16_t     PWM9;
		int16_t     PWM10;
		int16_t     PWM11;
		int16_t     PWM12;
		int16_t     FAN;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		int16_t     ADC1;
		int16_t     ADC2;
		int16_t     ADC3;
		int16_t     ADC4;
		int16_t     ADC5;
		int16_t     ADC6;
		int16_t     ADC7;
		int16_t     ADC8;
		int16_t     ADC9;
		int16_t     ADC10;
		int16_t     ADC11;
		int16_t     ADC12;
		int16_t     Distance;
		int16_t     Control;
	}Cust;
} PROCBUFFER_IN;

#endif