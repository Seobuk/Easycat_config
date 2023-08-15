//******************************************************************************************* 
//*                                                                                         * 
//*     This file has been created by the Easy Navigator on Thu Aug 03 16:47:07 2023
//*                                                                                         * 
//*                    AB&T Tecnologie Informatiche - Ivrea Italy                           * 
//*                                                                                         * 
//*                    www.bausano.net           info@bausano.net                           * 
//*                                                                                         * 
//******************************************************************************************* 


#ifndef  NETWORK_H 
#define NETWORK_H 

#define _PACK_  __attribute__((packed))


// 1) ******* network size *******************************************************************

#define NUM_SLAVES 1                         // number of slaves

#define WKC 3                                // expected working counter


// 2) ******* slaves names *******************************************************************

const PROGMEM uint8_t NAME[NUM_SLAVES][64] =
{
   {"HUSlave"}
};


// 3) ******* entries of the input PDOs - data from the slaves to the master *****************

struct  _PACK_
RxData
{
   struct _PACK_
   {
      int16_t  ADC1;
      int16_t  ADC2;
      int16_t  ADC3;
      int16_t  ADC4;
      int16_t  ADC5;
      int16_t  ADC6;
      int16_t  ADC7;
      int16_t  ADC8;
      int16_t  ADC9;
      int16_t  ADC10;
      int16_t  ADC11;
      int16_t  ADC12;
      int16_t  Distance;
      int16_t  Control;
   }
   _1_HUSlave;

};


// 4) ******* entries of the output PDOs - data from the master to the slaves ****************

struct  _PACK_
TxData
{
   struct _PACK_
   {
      int16_t  PWM1;
      int16_t  PWM2;
      int16_t  PWM3;
      int16_t  PWM4;
      int16_t  PWM5;
      int16_t  PWM6;
      int16_t  PWM7;
      int16_t  PWM8;
      int16_t  PWM9;
      int16_t  PWM10;
      int16_t  PWM11;
      int16_t  PWM12;
      int16_t  FAN;
   }
   _1_HUSlave;

};//


// 5) ******* SM configuration ***************************************************************

const PROGMEM uint8_t  SM[NUM_SLAVES][4][8] =
{
   {
      {0x00,0x01,0x00,0x64,0x00,0x1A,0x10,0x00},
      {0x00,0x01,0x00,0x20,0x00,0x1C,0x12,0x00},
      {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
      {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
   }
};


// 6) ******* FMMU configuration *************************************************************

const PROGMEM uint8_t  FMMU[NUM_SLAVES][2][16] =
{
   {
      {0x00,0x00,0x00,0x01,0x02,0x00,0x10,0x00,0x07,0x00,0x00,0x1A,0x00,0x00,0x00,0x00},
      {0x00,0x00,0x00,0x01,0x01,0x00,0x12,0x00,0x07,0x00,0x00,0x1C,0x00,0x00,0x00,0x00},
   }
};

#endif 
