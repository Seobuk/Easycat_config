//******************************************************************************************* 
//*                                                                                         * 
//*     This file has been created by the Easy Navigator on Mon May 02 16:33:49 2022
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

#define NUM_SLAVES 2                         // number of slaves

#define WKC 6                                // expected working counter


// 2) ******* slaves names *******************************************************************

const PROGMEM uint8_t NAME[NUM_SLAVES][64] =
{
   {"TestEasyCAT_Custom"},
   {"TestEasyCAT_Custom"}
};


// 3) ******* entries of the input PDOs - data from the slaves to the master *****************

struct  _PACK_
RxData
{
   struct _PACK_
   {
      uint16_t  Analog_0;
      uint16_t  Analog_1;
      uint16_t  Bit16_RisingTestRamp;
      uint8_t  DipSwitches;
      uint8_t  Bit8_FallingTestRamp;
   }
   _1_TestEasyCAT_Custom;

   struct _PACK_
   {
      uint16_t  Analog_0;
      uint16_t  Analog_1;
      uint16_t  Bit16_RisingTestRamp;
      uint8_t  DipSwitches;
      uint8_t  Bit8_FallingTestRamp;
   }
   _2_TestEasyCAT_Custom;

};


// 4) ******* entries of the output PDOs - data from the master to the slaves ****************

struct  _PACK_
TxData
{
   struct _PACK_
   {
      uint8_t  Leds;
   }
   _1_TestEasyCAT_Custom;

   struct _PACK_
   {
      uint8_t  Leds;
   }
   _2_TestEasyCAT_Custom;

};//


// 5) ******* SM configuration ***************************************************************

const PROGMEM uint8_t  SM[NUM_SLAVES][4][8] =
{
   {
      {0x00,0x01,0x00,0x64,0x00,0x01,0x10,0x00},
      {0x00,0x01,0x00,0x20,0x00,0x08,0x12,0x00},
      {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
      {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
   },
   {
      {0x00,0x01,0x00,0x64,0x00,0x01,0x10,0x00},
      {0x00,0x01,0x00,0x20,0x00,0x08,0x12,0x00},
      {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
      {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
   }
};


// 6) ******* FMMU configuration *************************************************************

const PROGMEM uint8_t  FMMU[NUM_SLAVES][2][16] =
{
   {
      {0x00,0x00,0x00,0x01,0x02,0x00,0x10,0x00,0x07,0x00,0x00,0x01,0x00,0x00,0x00,0x00},
      {0x00,0x00,0x00,0x01,0x01,0x00,0x12,0x00,0x07,0x00,0x00,0x08,0x00,0x00,0x00,0x00},
   },
   {
      {0x00,0x00,0x00,0x01,0x02,0x00,0x10,0x00,0x07,0x00,0x00,0x01,0x00,0x00,0x00,0x01},
      {0x00,0x00,0x00,0x01,0x01,0x00,0x12,0x00,0x07,0x00,0x00,0x08,0x00,0x00,0x00,0x08},
   }
};

#endif 
