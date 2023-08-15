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
// This project has been covered in the following video                                       *
// https://www.youtube.com/watch?v=_Jz18RJZSH4                                                *
//                                                                                            *
// Requirements for the master:                                                               *
// Arduino Uno or Zero or Due with an Ethernet 2 shield                                       *
//                                                                                            *
// Requirements for the Network:                                                              *
//                                                                                            *
// One EtherCAT slave consisting of an Arduino board, an EasyCAT shield customized with       * 
// the Easy Configurator and an EasyCAT Test shield                                           *
//                                                                                            *
// One EtherCAT slave consisting of an EpoCAT IO1616                                          *
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
  Serial.begin(115200);                             // enable the debug messages on the serial line
  Serial.println(F("\nEasy Master Video example")); // print out the banner
  Serial.println(F("\nAB&T Tecnologie Informatiche"));
  Serial.println(F("Ivrea Italy - www.bausano.net"));  
 
  EASYMASTER.Init(5);                               // init the Easy Master library with a sampling time of 5mS                  
}


//------------------------------------------------------------------------------------------------------------

void loop (void)
{
  uint8_t static Counter;                                         // a static variable
  
  if (EASYMASTER.StartApp())                                      // synchronize the application                                  
  {                                                               // with the sampling interrupt

    if((millis() - PrevMillis) > PRINT_TIME)                      // every 500 mS do the
    {                                                             // following things
      PrevMillis += PRINT_TIME;                                   // 

      Serial.println(F("EasyCAT Custom"));                        // print out the analog values
      Serial.print(F("Analog 0 = "));                             // from the EasyCAT custom 
      Serial.println(RXDATA._1_TestEasyCAT_Custom.Analog_0 >> 6); //
      Serial.print(F("Analog 1 = "));                             //
      Serial.println(RXDATA._1_TestEasyCAT_Custom.Analog_1 >> 6); //
      Serial.println();

      TXDATA._2_EpoCAT_IO1616.Output_0 = Counter++;               // increment "Counter" and output
                                                                  // it to the EpoCAT IO_1616
    }
                                                                  // copy some inputs from the EPOCAT IO_1616
                                                                  // to the leds of the EasyCAT custom
    TXDATA._1_TestEasyCAT_Custom.Leds = RXDATA._2_EpoCAT_IO1616.Input_1;

                                                                  // copy the dip switches from the EasyCAT custom
                                                                  // to somee leds of the EpoCAT IO_1616
    TXDATA._2_EpoCAT_IO1616.Output_1  = RXDATA._1_TestEasyCAT_Custom.DipSwitches;
        
    EASYMASTER.EndApp();                                          // synchronize the application 
  }                                                               // with the sampling interrupt
}
