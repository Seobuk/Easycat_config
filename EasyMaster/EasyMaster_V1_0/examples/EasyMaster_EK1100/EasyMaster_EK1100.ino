//*********************************************************************************************
//                                                                                            *
// AB&T Tecnologie Informatiche - Ivrea Italy                                                 *
// http://www.bausano.net                                                                     *
// https://www.ethercat.org/en/products/791FFAA126AD43859920EA64384AD4FD.htm                  *
//                                                                                            *
//*********************************************************************************************
                                                                                       
//*********************************************************************************************    
//                                                                                            *
// This software is distributed as an example, "AS IS", in the hope that it could             *
// be useful, WITHOUT ANY WARRANTY of any kind, express or implied, included, but             *
// not limited,  to the warranties of merchantability, fitness for a particular               *
// purpose, and non infringiment. In no event shall the authors be liable for any             *    
// claim, damages or other liability, arising from, or in connection with this software.      *
//                                                                                            *
//*********************************************************************************************

//*********************************************************************************************
//                                                                                            *
// This is a demonstration of the EasyMaster library                                          *
//                                                                                            *
// Requirements for the master:                                                               *
// Arduino Uno or Zero or Due with an Ethernet 2 shield                                       *
//                                                                                            *
// Requirements for the Network:                                                              *
//                                                                                            *
// One EtherCAT slave consisting of an Arduino board, an EasyCAT shield customized with       * 
// the Easy Configurator and an EasyCAT Test shield                                           *
//                                                                                            *
// One Beckhoff EK1100 coupler                                                                *
//                                                                                            *
// One Beckhoff EL2004 4 digital outputs terminal                                             *
//                                                                                            *
// One Beckhoff EL1014 4 digital inputs terminal                                              *
//                                                                                            *
//*********************************************************************************************


#include "EasyMaster.h"                           // easy master library      

#include "NetworkInfo.h"                          // this file contains the network information   
                                                  // it has been created by the Easy Navigator      

TxData TXDATA;                                    // data structures instantiation
RxData RXDATA;                                    //

                                                  // Easy Master class instantiation
EasyMaster EASYMASTER (sizeof(TxData), sizeof(RxData), (uint8_t*)&TXDATA, (uint8_t*)&RXDATA,\
                       NUM_SLAVES, WKC, (uint8_t*)NAME, (uint8_t*)FMMU, (uint8_t*)SM);

#define PRINT_TIME 500                            // we print out the data from the slaves each 0.5 S
uint32_t PrevMillis;                              //


//------------------------------------------------------------------------------------------------------------

void setup (void)
{
  Serial.begin(115200);                               // enable the debug messages on the serial line
  Serial.println(F("\nEasy Master EK1100 example"));  // print out the banner
  Serial.println(F("\nAB&T Tecnologie Informatiche"));
  Serial.println(F("Ivrea Italy - www.bausano.net"));  
 
  EASYMASTER.Init(5);                                 // init the Easy Master library with a sampling time of 5mS                  
}


//------------------------------------------------------------------------------------------------------------

void loop (void)
{ 
  if (EASYMASTER.StartApp())                                      // synchronize the application                                  
  {                                                               // with the sampling interrupt

    if((millis() - PrevMillis) > PRINT_TIME)                      // every 500 mS
    {                                                             //
      PrevMillis += PRINT_TIME;                                   // 

      Serial.println(F("EasyCAT Custom"));                        // print out the analog values
      Serial.print(F("Analog 0 = "));                             // from the EasyCAT custom
      Serial.println(RXDATA._1_TestEasyCAT_Custom.Analog_0 >> 6); //
      Serial.print(F("Analog 1 = "));                             //
      Serial.println(RXDATA._1_TestEasyCAT_Custom.Analog_1 >> 6); //
      Serial.println();                                           //
    }
                                                                  // copy the dip switches from the EasyCAT custom
                                                                  // to the outputs of the EL2004
    TXDATA._3_EL2004.Output_0 = bool(RXDATA._1_TestEasyCAT_Custom.DipSwitches & 0x01);
    TXDATA._3_EL2004.Output_1 = bool(RXDATA._1_TestEasyCAT_Custom.DipSwitches & 0x02);
    TXDATA._3_EL2004.Output_2 = bool(RXDATA._1_TestEasyCAT_Custom.DipSwitches & 0x04);
    TXDATA._3_EL2004.Output_3 = bool(RXDATA._1_TestEasyCAT_Custom.DipSwitches & 0x08);

    uint8_t Temp = 0x00;                                          // copy the inputs of the EL1014 to  
    if (RXDATA._4_EL1014.Input_0)                                 // the leds of the EasyCAT custom
      Temp |= 0x01;                                               //
    if (RXDATA._4_EL1014.Input_1)                                 //    
      Temp |= 0x02;                                               //
    if (RXDATA._4_EL1014.Input_2)                                 //
      Temp |= 0x04;                                               //
    if (RXDATA._4_EL1014.Input_3)                                 //  
      Temp |= 0x08;                                               //
                                                                  //                                            
    TXDATA._1_TestEasyCAT_Custom.Leds = Temp;                     //
            
    EASYMASTER.EndApp();                                          // synchronize the application 
  }                                                               // with the sampling interrupt
}
