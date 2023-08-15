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


#include "EasyMaster.h"

#define SCS 10                                                    // pin 10 is the ethercat controller chip select

                                                                  //--- optimization for ethernet spi chip select ---------------
                                                                  //  
#if defined(ARDUINO_ARCH_AVR)                                     // -- AVR architecture (Uno - Mega) ---------          
  uint8_t Mask_SCS;                                               //                              
  uint8_t Port_SCS;                                               //
  volatile uint8_t* pPort_SCS;                                    //
                                                                  // 
  static inline void ScsLow()   {*pPort_SCS &= ~(Mask_SCS);}      //
  static inline void ScsHigh()  {*pPort_SCS |=  (Mask_SCS);}      //

#elif defined(ARDUINO_ARCH_SAMD)                                  //--- SAMD architecture (Zero) --------------
  EPortType Port_SCS;                                             //
  uint32_t Bit_SCS;                                               //
                                                                  //
  static inline void ScsLow()   {PORT->Group[Port_SCS].OUTCLR.reg = (1<<Bit_SCS);}
  static inline void ScsHigh()  {PORT->Group[Port_SCS].OUTSET.reg = (1<<Bit_SCS);}
  
#elif defined(ARDUINO_ARCH_SAM)                                   //---- SAM architecture (Due) ---------------
  Pio* pPort_SCS;                                                 //
  uint32_t Bit_SCS;                                               //
                                                                  //  
  static inline void ScsLow()   {pPort_SCS->PIO_CODR = Bit_SCS;}  //
  static inline void ScsHigh()  {pPort_SCS->PIO_SODR = Bit_SCS;}  // 
  
#else                                                             //--- others architectures not optimized ----
  static inline void ScsLow()   {digitalWrite(SCS, LOW);}         //
  static inline void ScsHigh()  {digitalWrite(SCS, HIGH);}        //

#endif      
  
                                                                  //--- configure a timer to generate the sampling interrupt ----
              
#if defined(ARDUINO_ARCH_AVR)                                     // -- AVR architecture (Uno - Mega) ---------   
                                                                  //
  static inline void ConfigInterrupt(uint8_t SampleTime)          // configure Timer 1 - SampleTime = milliseconds
  {                                                               //                                         
    TCCR1A = 0B00000000;                                          // prescaler 256   
    TCCR1B = 0B00001011;                                          // clear the timer on compare match (CTC) 
    OCR1A = SampleTime * 250;                                     // compare register   
  }                                                               //  
                                                                  //
  static inline void EnableInterrup()   {TIMSK1 |= 0B00000010;}   // enable interrupt
  static inline void DisableInterrup()  {TIMSK1 &= 0B11111101;}   // disable interrupt 
 
#elif defined(ARDUINO_ARCH_SAMD)                                  //--- SAMD architecture (Zero) --------------
                                                                  //
  static inline void ConfigInterrupt(uint8_t SampleTime)          // configure TCC0 0 - SampleTime = milliseconds
  {                                                               // 
                                                                  //
    GCLK->GENDIV.reg = ( GCLK_GENDIV_DIV(6) |                     // Divide the 48MHz clock source by 6: 48MHz/6= 8MHz
                         GCLK_GENDIV_ID(4) );                     // Select Generic Clock  GCLK 4
    while (GCLK->STATUS.bit.SYNCBUSY);                            //                         
                                                                  //
    GCLK->GENCTRL.reg = ( GCLK_GENCTRL_GENEN |                    // Enable GCLK4
                          GCLK_GENCTRL_SRC_DFLL48M |              // Set the 48MHz clock source
                          GCLK_GENCTRL_ID(4) );                   // Select GCLK4
    while (GCLK->STATUS.bit.SYNCBUSY);                            //
                                                                  //
    GCLK->CLKCTRL.reg = ( GCLK_CLKCTRL_CLKEN |                    // Enable GCLK4 
                          GCLK_CLKCTRL_GEN_GCLK4 |                // Select GCLK4
                          GCLK_CLKCTRL_ID_TCC0_TCC1 );            // Feed GCLK4 to TCC0 and TCC1
    while (GCLK->STATUS.bit.SYNCBUSY);                            //                          
                                                                  //
    TCC0->CTRLA.bit.ENABLE = 0;                                   // disable TCC0
    while (TCC0->SYNCBUSY.bit.ENABLE);                            //
    TCC0->CTRLA.bit.PRESCALER = 0x3;                              // prescaler /8 - timer clock 1MHz
    TCC0->PER.reg = SampleTime * 1000;                            // set period register    
    while (TCC0->SYNCBUSY.bit.PER);                               //    
    TCC0->INTENSET.reg |= TC_INTENSET_OVF;                        // enable interrupt on overflow  
    TCC0->CTRLA.bit.ENABLE = 1;                                   // enable TCC0  
    while (TCC0->SYNCBUSY.bit.ENABLE);                            //    
  }     

  static inline void EnableInterrup()   {NVIC_EnableIRQ(TCC0_IRQn);}  // enable interrupt
  static inline void DisableInterrup()  {NVIC_DisableIRQ(TCC0_IRQn);} // disable interrupt   
  
#elif defined(ARDUINO_ARCH_SAM)                                   //--- SAM architecture (Due) ------------------  
                                                                  //
  static inline void ConfigInterrupt(uint8_t SampleTime)          // configure TC2 channel 2 (TC8) - SampleTime = milliseconds
  {                                                               //
    PMC->PMC_PCER1 |= PMC_PCER1_PID35;                            // TC8 power on
                                                                  //
    TC2->TC_CHANNEL[2].TC_CMR =   TC_CMR_TCCLKS_TIMER_CLOCK4      // timer clock = MCK/128 = 656 Hz
                                | TC_CMR_WAVE                     // waveform mode
                                | TC_CMR_WAVSEL_UP_RC;            // count up with trigger on RC Compare
                                                                  //
    TC2->TC_CHANNEL[2].TC_RC = SampleTime * 656;                  // period
    TC2->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;                      // enable interrupt from compare
    TC2->TC_CHANNEL[2].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;      // software trigger and timer enable   
  }
                                                          
  static inline void EnableInterrup()   {NVIC_EnableIRQ(TC8_IRQn);}   // enable interrupt
  static inline void DisableInterrup()  {NVIC_DisableIRQ(TC8_IRQn);}  // disable interrupt  
  
#else                                                             //--- define the following function for your architecture
  //static inline void ConfigInterrupt(uint8_t SampleTime)  {...........}   // configure interrupt
  //static inline void EnableInterrup()                     {...........}   // enable interrupt
  //static inline void DisableInterrup()                    {...........}   // disable interrupt  
  
  #error "define the interrupt functions for your architecture"   
#endif      


SPISettings WIZ_SPI_SETTING(33000000,MSBFIRST,SPI_MODE0);	      // Wiz SPI settings
                                                                // 33MHz is the max Wiz guaranteed speed
                                                                // it will be limited by the choosen architecture
                                                                // ARDUINO_ARCH_AVR    8MHz
                                                                // ARDUINO_ARCH_SAMD  12MHz
                                                                // ARDUINO_ARCH_SAM   28MHz


static inline int getNAME_index(uint16_t x, uint16_t y){return y + (x*64);}
static inline int getSM_index(uint16_t x, uint16_t y, uint16_t z){return z + (y*8) + (x*4*8);}
static inline int getFMMU_index(uint16_t x, uint16_t y, uint16_t z){return z + (y*16) + (x*2*16);}


extern EasyMaster EASYMASTER;


//---- sampling interrupt -------------------------------------------------------------------------------------

#if defined(ARDUINO_ARCH_AVR)
  ISR(TIMER1_COMPA_vect)
#elif defined(ARDUINO_ARCH_SAMD)
  void TCC0_Handler(void)
#elif defined(ARDUINO_ARCH_SAM) 
  void TC8_Handler(void)
#else
  #error "define the interrupt vector for your architecture" 
#endif

{
  uint8_t Data_8 ;
  uint16_t Data_16;
  uint8_t Temp[2]; 

  EASYMASTER.WriteOrRead(SN_RX_RSR, RD_S0_REG, &Data_16, sizeof(Data_16)); 
  if(Data_16 == 0x0000)                                             // check if data have been received
  {                                                                 // if not we assume that the cable is disconnected
    EASYMASTER.bNoAnswErr = true;                                   // or the slaves are not powered on
    EASYMASTER.bSync = true;                                        //
    return;                                                         // exit with the error flag set
  }        

  Data_8 = OPEN;                                                    // reopen the socket to reset the read and write
  EASYMASTER.WriteOrRead(SN_CR, WR_S0_REG, &Data_8, sizeof(Data_8));// and write buffers pointers
  
  do                                                                // be sure that the command has ben executed 
  {                                                                 //  
    EASYMASTER.WriteOrRead(SN_SR, RD_S0_REG, &Data_8, sizeof(Data_8));  
  }                                                                 //
  while(Data_8 != 0x42);                                            //
    
  if (!EASYMASTER.bSync)                                            // if the application has completed his task
  {                                                                 // update the RXDATA structures with the data in the Wiz rx buffer
                                                                    // these are the data from the slaves to the master
                                                                    // if the application has not completed his task
                                                                    // we don't read the rx buffer and don't update the tx buffer                                                                         
                                                                    // we just send the old data to the slaves to keep them alive
                                                                     
    EASYMASTER.WriteOrRead(START_FIRST_RX_DATA, RD_RX_BUFF, EASYMASTER.m_pRxData_, EASYMASTER.RxSize_);

                                                                    //---- check for network errors ------------------

    EASYMASTER.WriteOrRead(WKC_LRD, RD_RX_BUFF, &Temp[0], 1);       // read the working counter for the LRD command
    Data_16 = Temp[0];                                              //

    EASYMASTER.WriteOrRead(WKC_LWR, RD_RX_BUFF, &Temp[0], 1);       // read the working counter for the LWR command
    Data_16 += Temp[0];                                             //

    EASYMASTER.WriteOrRead(WKC_BRD-1, RD_RX_BUFF, &Temp[0], 2);     // read the working counter for the BRD command
    Data_16 += Temp[1];                                             // and the response from the BRD command

    if (Data_16 != EASYMASTER.Wkc_)                                 // check if the total working counter is correct
    {                                                               //
      EASYMASTER.WkcTotal = Data_16;                                //
      EASYMASTER.bWkcErr = true;                                    // if not, set the working counter error flag
    }                                                               //

    if (Temp[0] != OP)                                              // check if all the slaves are in operational
    {                                                               //
      EASYMASTER.bOpErr = true;                                     // if not, set the operational error flag
    }                                                               //
    
                                                                    // update the data in the Wiz tx buffer with
                                                                    // the data from the TXDATA structures
                                                                    // these are the data from the master to the slaves
                                                                    
    EASYMASTER.WriteOrRead(START_FIRST_TX_DATA, WR_TX_BUFF, EASYMASTER.m_pTxData_, EASYMASTER.TxSize_);  
  }

  Data_16 = ETH_SIZE;                                               // update the tx buffer wr pointer
  Data_16 = ((Data_16 << 8) & 0xff00) | ((Data_16 >> 8) & 0x00ff);  // to send the ethernet frame
  EASYMASTER.WriteOrRead(SN_TX_WR, WR_S0_REG, &Data_16, sizeof(Data_16));

  Data_8 = SEND_MAC;                                                // send the ethernet frame
  EASYMASTER.WriteOrRead(SN_CR, WR_S0_REG, &Data_8, sizeof(Data_8));//

  EASYMASTER.bSync = true;                                          // set the flag to synchronize the application activity
  

  #if defined(ARDUINO_ARCH_SAMD)                                    // clear the interrupt flag 
    TCC0->INTFLAG.reg |= ~(TC_INTFLAG_OVF);                         //       
  #elif defined(ARDUINO_ARCH_SAM)                                   //
    uint32_t Dummy = TC2->TC_CHANNEL[2].TC_SR;                      //
  #elif defined(ARDUINO_ARCH_AVR)                                   //    
    // no need to clear the interrupt flag
  #else                                                             //
    #error "clear the interrupt flag for your architecture"         //
  #endif                                                            //    
}


//---- constructor -----------------------------------------------------------------------------------

EasyMaster::EasyMaster (uint16_t TxSize, uint16_t RxSize, uint8_t* m_pTxData, uint8_t* m_pRxData, uint8_t NumSlaves, uint8_t Wkc, uint8_t* NAME, uint8_t* FMMU, uint8_t* SM)
{
  TxSize_ = TxSize;
  RxSize_ = RxSize;  
  m_pTxData_ = m_pTxData; 
  m_pRxData_ = m_pRxData;
  NumSlaves_ = NumSlaves;
  Wkc_ = Wkc;
  NAME_ = (uint8_t*)NAME;
  FMMU_ = (uint8_t*)FMMU;
  SM_ = (uint8_t*)SM;
}


//---- init ------------------------------------------------------------------------------------------

void EasyMaster::Init (uint8_t SampTime)
{
  uint8_t s, j, k;
  uint8_t Data_8;
  uint16_t Data_16;  
  bool bSlaveErr;
  bool bGlobalErr;   
  uint8_t NameLen;
  uint8_t TempBuff[16];  
  
  bSync = false;
  bNoAnswErr = false; 
  bWkcErr = false;
  bOpErr = false;                                                 
  bGlobalErr = false;                                          
  bSlaveErr = false; 
 
 
  if ( (SampTime > 50) || (SampTime == 0) )                       // check the sampling time value
  {                                                               //
    Serial.println(F("\nSampling time must be between 1 and 50!"));
    BlinkForEver();                                               // stay in loop for ever blinking the led       
  }
 
  SPI.begin();
 
  #if defined(ARDUINO_ARCH_AVR)                                   // the spi is used inside the interrupt
    SPI.usingInterrupt(TIMER1_COMPA_vect);                        //                       
  #elif defined(ARDUINO_ARCH_SAMD)                                //
    SPI.usingInterrupt((int)TCC0_Handler);                        //
  #elif defined(ARDUINO_ARCH_SAM)                                 //
    SPI.usingInterrupt((int)TC8_Handler);                         //
  #else                                                           //
    #error "declare the interrupt vector for your architecture"   //
  #endif                                                          //
  
  
  #if defined(ARDUINO_ARCH_AVR)                                   // configure the spi chip select  
    Mask_SCS = (digitalPinToBitMask(SCS));                        //           
    Port_SCS = digitalPinToPort(SCS);                             //          
    pPort_SCS = portOutputRegister(Port_SCS);                     //
  #elif defined(ARDUINO_ARCH_SAMD)                                //
    Bit_SCS = g_APinDescription[SCS].ulPin;                       //     
    Port_SCS = g_APinDescription[SCS].ulPort;                     // 
  #elif defined(ARDUINO_ARCH_SAM)                                 // 
    Bit_SCS = g_APinDescription[SCS].ulPin;                       //    
    pPort_SCS = g_APinDescription[SCS].pPort;                     //    
  #endif                                                          //
 
  pinMode(SCS, OUTPUT);											                      //
  ScsHigh();                                                      //

  ConfigInterrupt(SampTime);                                      // configure the sampling interrupt generation

  delay(2000);                                                    // delay for Wiz to be ready
  
  Data_8 = SW_RESET;                                              // Wiz software reset
  WriteOrRead(MR, WR_CM_REG, &Data_8, sizeof(Data_8));            //  
                                                                  //
  do                                                              // 
  {                                                               //  
    EASYMASTER.WriteOrRead(MR, RD_CM_REG, &Data_8, sizeof(Data_8));  
  }                                                               //
  while(Data_8 != 0x00);                                          //
  
  
  Data_8 = MAC_RAW;                                               // we use only socket 0 in mac raw mode 
  WriteOrRead(SN_MR, WR_S0_REG, &Data_8, sizeof(Data_8));         //
   
  Serial.println(F("\nScanning the network"));                    //
  
                                                                  //---- check the EtherCAT configuration ----------------
 
  InitFrame(12+1);                                                //---- check how many slaves are on the network --------
  AddEcatDatagram(0, EE_DATA_REG, BRD, NULL , 1, false);          //   
  SendFrame(6+6+2 +2 +12+1);                                      //
  
  EASYMASTER.WriteOrRead(SN_RX_RSR, RD_S0_REG, &Data_16, sizeof(Data_16)); 
  if(Data_16 == 0x0000)                                           // if there is no answer from the network
  {                                                               //
    Data_8 = 0;                                                   // set the number of slaves found to zero
  }                                                               //
    else                                                          // else check the working counter to see how many slaves are present
  {                                                               //
    WriteOrRead(START_FIRST_RX_DATA+1, RD_RX_BUFF, &Data_8, sizeof(Data_8));     
  }                                                               //
  if(Data_8 != NumSlaves_)                                        // if the number of slaves is not the expected one
  {                                                               //
    Serial.println(F("\nNumber of slaves error!"));               // print out the error 
    Serial.print(F("Expected "));                                 //
    Serial.print(NumSlaves_);                                     //
    Serial.print(F("\nActual "));                                 //   
    Serial.print(Data_8);                                         //      
    BlinkForEver();                                               // stay in loop for ever blinking the led
  }
  
  Serial.print(F("\nSlaves found "));                             // print out the number of slaves                                                              
  Serial.println(Data_8);                                         //                                         

                                                                  //---- check slaves name ----------------------------
                                                                  //
  for(s=0; s<NumSlaves_; s++)                                     // iterate through the slaves to check their
  {                                                               // presence and compare their name from the eeprom
    Serial.print(F("\nSlave "));                                  // with the one from networkinfo.h
    Serial.println(s+1);                                          //
    
    bSlaveErr = false;                                            // reset the error flag for a single slave
  
    EEPROM_Read_8(-s, 0x84);                                      // !don't remove this line even if it seems useless!                                                    
    NameLen = EEPROM_Read_8(-s, 0x85);                            // check the name length   
    
    if(NameLen == 0x00)                                           //
    {                                                             //
      Serial.println(F("not detected!"));                         // if the name length is 0 we assume that the slave is not present
      bSlaveErr = true;                                           // set the error flags
      bGlobalErr = true;                                          //
    } 
    else if(NameLen > 63 )                                        //
    {                                                             //
      Serial.println(F("name too long!"));                        // if the name length is larger then 63 bytes we cannot handle it
      bSlaveErr = true;                                           // set the error flags
      bGlobalErr = true;                                          //
    }      
    
    else                                                          // if the name length from the eeprom is not equal
    {                                                             // to the one from network.f the name is not valid 
      for(k=0; k<64; k++)                                         //
      {                                                           //
        if (pgm_read_byte(&NAME_[getNAME_index(s,k)]) == 0x00)    //
        {                                                         //                                                                             
          break;                                                  //
        }                                                         //
      }                                                           //     
      
      if (k != NameLen)                                           //
      {                                                           //
        bSlaveErr = true;                                         // set the error flags
        bGlobalErr = true;                                        //    
      }                                                           //
      
      else                                                        // compare the slave name from the eeprom
      {                                                           // with the one from networkinfo.h
        for(k=0; k<NameLen; k++)                                  //
        {                                                         //  
          if(EEPROM_Read_8(-s, 0x86 + k) != pgm_read_byte(&NAME_[getNAME_index(s,k)]))
          {                                                       // if a difference is found
            bSlaveErr = true;                                     // set the error flags
            bGlobalErr = true;                                    //
          }                                                       //
        }                                                         //
      }                                                           //
          
      if (bSlaveErr)                                              // if the slave error flag is set
      {                                                           //                       
        Serial.println(F("Name error!"));                         // 
        Serial.print(F("Expected "));                             // print out the expected name from the networkinfo.txt
        Serial.print((const __FlashStringHelper *) &NAME_[getNAME_index(s,0)]);    
        Serial.print(F("\nActual "));                             //
                                                                  // and the actual one from the eeprom
        EEPROM_Read_8(-s, 0x84);                                  // !don't remove this line even if it seems useless! 
        for(k=0; k<NameLen; k++)                                  //
        {                                                         //
          EEPROM_Read_8(-s, 0x86 + k);                            //
          Serial.write(EEPROM_Read_8(-s, 0x86 + k));              //     
        }                                                         //
        Serial.println();                                         //  
      } 
      
      else                                                        // the name from the eeprom matches
      {                                                           // the one from the networkinfo.h
        Serial.println((const __FlashStringHelper *) &NAME_[getNAME_index(s,0)]);    // print out the slave name
      }                                                           //
    }                                                             //
  }
  
  if(bGlobalErr)                                                  // if some error was detected during
  {                                                               // the name check, stay in loop for ever blinking the led
    BlinkForEver();                                               //
  }                                                               //

                                                                  //---- configure the SM_ ----------------------------------
                                                                  //
  for(s=0; s<NumSlaves_; s++)                                     // iterate through the slave to configure the SM_
  {                                                               // with the value from networkinfo.h
                                                                  //
    for(j=0; j<4; j++)                                            // iterate through SM0, SM1, SM2 and SM3
    {                                                             //
      for(k=0; k<8; k++)                                          //
      {                                                           //
        TempBuff[k] = pgm_read_byte(&SM_[getSM_index(s,j,7-k)]);  //	
      }                                                           //
                                                                  //
      InitFrame(12+8);                                            // send the configuration
      AddEcatDatagram(-s, SM_0+(j*8), APWR, &TempBuff[0], 8, false);     
      SendFrame(6+6+2 +2 +12+8);                                  //
    }                                                             // 
  }                                                               // 

                                                                  //---- configure the FMMU_ -------------------------------
                                                                  //
  for(s=0; s<NumSlaves_; s++)                                     // iterate through the slave to configure the FMMU_
  {                                                               // with the value from networkinfo.h
                                                                  //
    for(j=0; j<2; j++)                                            // iterate through FMMU0 and FMMU1
    {                                                             //
      for(k=0; k<16; k++)                                         //
      {                                                           //
        TempBuff[k] = pgm_read_byte(&FMMU_[getFMMU_index(s,j,15-k)]);
      }                                                           //
                                                                  //
      InitFrame(12+16);                                           // send the configuration
      AddEcatDatagram(-s, FMMU_0+(j*16), APWR, &TempBuff[0], 16, false);     
      SendFrame(6+6+2 +2 +12+16);                                 //
    }                                                             //
  }                                                               //

                                                                  //---- set the slaves to operational ----------------------
  for(j=0; j<4; j++)                                              //  
  {                                                               // 
    TempBuff[0] = INIT << j;                                      // INIT -> PREOP -> SAFEOP -> OP

    InitFrame(12+1);                                              // send the command
    AddEcatDatagram(0, AL_CONTROL, BWR, &TempBuff[0], 1, false);  //   
    SendFrame(6+6+2 +2 +12+1);                                    //   
                                                               
    delay(200);                                                   // wait a bit between one state and another
  }                                                             
  
                                                                  //---- check that all slaves are in operational ---------
                                                                  //    
  InitFrame(12+1);                                                // AL_STATUS register broadcast read      
  AddEcatDatagram(0, AL_STATUS, BRD, NULL, 1, false);             //   
  SendFrame(6+6+2 +2 +12+1);                                      //  
  WriteOrRead(START_FIRST_RX_DATA, RD_RX_BUFF, &TempBuff[0], 1);  //                   
                                                                                                  
  if (TempBuff[0] != OP)                                          // if not all slaves have gone to operational                                      
  {                                                               // stay in loop for ever blinking the led                     
    Serial.println(F("Not all slaves in operational!"));          //
    BlinkForEver();                                               //
  }                                                               //
                                        
  Serial.println(F("\nAll slaves in operational\n\nRunning\n"));// all slaves are in operational, we can procede    

                                                                                   
                                                                  // create the ethernet frame for the sampling interrupt
  InitFrame(12+RxSize_ +12+TxSize_ +12+1);                        //
  AddEcatDatagram(0, 0x0000, LRD, NULL, RxSize_, true);           // logical read - data from the slaves to the master
  AddEcatDatagram(0, 0x0000, LWR, m_pTxData_, TxSize_, true);     // logical write - data from the master to the slave
  AddEcatDatagram(0, AL_STATUS, BRD, NULL, 1, false);             // broadcast read - state of all the slaves  
  SendFrame(6+6+2 +2 +12+RxSize_ +12+TxSize_ +12+1);              //

  EnableInterrup();                                               // enable the sampling interrupt 
} 


//---- application synchronization -----------------------------------------------------------------------------------

bool EasyMaster::StartApp(void)
{
  if (bSync)                                                      // synchronize the application with the
  {                                                               // execution of the sampling interrut service

    if (bNoAnswErr)                                               // no slave answered
    {                                                             // cable is disconnected or the first slave is not powered on 
      Serial.print(F("\nNo answer from the slaves "));            //
      BlinkForEver();                                             //
    }                                                             // stay in loop for ever blinking the led  

    if (bWkcErr)                                                  // check if the working counter is correct
    {                                                             //  
      Serial.print(F("\nExpected WKC ")); Serial.print(Wkc_); Serial.print(F(" actual ")); Serial.print(WkcTotal); Serial.print(F("!"));
      BlinkForEver();                                             // if not stay in loop for ever blinking the led 
    }
    
    if (bOpErr)                                                   // check if all the slaves are in operational
    {                                                             //       
      Serial.println(F("Not all slaves in operational"));         //
      BlinkForEver();                                             // if not stay in loop for ever blinking the led
    }
    
    return true;                                                  // return reporting that the application can start
  }                                                               //
  
  else                                                            // return reporting that the application cannot start
  {                                                               //
    return false;                                                 //
  }
}


//----- end of the the application --------------------------------------------------------------------------

void EasyMaster::EndApp (void)
{
  bSync = false;                                                  // clear the sync flag to signal to the sampling
}                                                                 // interrupt service that the application is over


//---- unrecoverable error -------------------------------------------------------------------

void EasyMaster::BlinkForEver(void)                               // an arror has been detected
{                                                                 //
  Serial.println(F("\n\nStopped!")); 
  
  DisableInterrup();                                              // disable the sampling interrupt
  SPI.end();                                                      // disable the spi to enable the led
  pinMode(LED_BUILTIN, OUTPUT);                                   //
  
  while(1)                                                        // blink the led for ever
  {                                                               //
    digitalWrite (LED_BUILTIN, HIGH);                             //
    delay(300);                                                   //
    digitalWrite (LED_BUILTIN, LOW);                              //
    delay(300);                                                   //
  }                                                               //   
}


//---- EEPROM functions ----------------------------------------------------------

//---- read one byte from the EEPROM ---------------------------------------------

uint8_t EasyMaster::EEPROM_Read_8 (int16_t Slave, uint16_t ByteAddress)
{
  uint8_t WhichByte;
  
  WhichByte = ByteAddress & 0x0003;

  switch (WhichByte)
  {
    case 0x00:
      EE_Data_32 = EEPROM_Read_32 (Slave, ByteAddress / 2);
      return (uint8_t)EE_Data_32;
    break;

    case 0x01:
      return (uint8_t)(EE_Data_32 >> 8);
    break;

    case 0x02:
      return (uint8_t)(EE_Data_32 >> 16);
    break;

    case 0x03:
      return (uint8_t)(EE_Data_32 >> 24);
    break;        
  } 
}


//----- read 4 bytes from the EEPROM ---------------------------------------------

uint32_t EasyMaster::EEPROM_Read_32 (int16_t Slave, uint16_t WordAddress)
{
  uint32_t Data;
  uint16_t Comm;
  uint16_t Command;
  uint8_t TempBuff[4];

  TempBuff[0] = (uint8_t)WordAddress;                    
  TempBuff[1] = (uint8_t)(WordAddress >> 8);

  InitFrame(12+2);
  AddEcatDatagram(Slave, EE_ADDR_REG, APWR, &TempBuff[0], 2, false);          
  SendFrame(6+6+2 +2 +12+2);      

  TempBuff[0] = (uint8_t)EE_READ; 
  TempBuff[1] = (uint8_t)(EE_READ >> 8); 
  InitFrame(12+2);
  AddEcatDatagram(Slave, EE_CTR_STAT_REG, APWR, &TempBuff[0], 2, false);     
  SendFrame(6+6+2 +2 +12+2);      
  
  delay (20);

  InitFrame(12+4);
  AddEcatDatagram(Slave, EE_DATA_REG, APRD, &TempBuff[0], 4, false);     
  SendFrame(6+6+2 +2 +12+4);      

  WriteOrRead(START_FIRST_RX_DATA, RD_RX_BUFF, &TempBuff[0], 4);     // we get 4 bytes at a time
  memcpy(&Data, &TempBuff[0], 4);                                    //

  return Data;
}


//---- functions to create and send an ethernet frame --------------------------------------------------

//---- add the ethernet header  -------------------------------------------------------------------------

void EasyMaster::InitFrame(uint16_t EcatLen)
{                                                             
  uint8_t Command;
  uint8_t i;
  
  static const PROGMEM uint8_t  EthHead[14] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,  // broadcast mac
                                                0x90,0xA2,0xDA,0x0D,0x12,0x30,  // my MAC 
                                                0x88, 0xA4};                    // EtherCAT protocol   
                        
  Command = OPEN;                                                 // reopen the socket to reset the read
  WriteOrRead(SN_CR, WR_S0_REG, &Command, sizeof(Command));       // and write buffers pointers
                                                                  // we don't use the circular buffer
  delay(10);
            
  SPI.beginTransaction(WIZ_SPI_SETTING);                          // write into the Wiz tx buffer
  
  ScsLow();                                                       //
  SPI.transfer(0x00);                                             // we always start from address 0x0000
  SPI.transfer(0x00);                                             // as the circular buffer is not used
  SPI.transfer(WR_TX_BUFF);                                       //

  for (i=0; i<14; i++)                                            // write the ethernet header
  {                                                               //
     SPI.transfer(pgm_read_byte(&EthHead[i]));                    //      
  }                                                               //

  SPI.transfer((uint8_t)EcatLen);                                 // write the EtherCAT header          
  SPI.transfer((uint8_t)(EcatLen >> 8) | 0x10);                   // EtherCAT datagrams length
}


//---- add an EtherCAT datagram ----------------------------------------------------------------------

void EasyMaster::AddEcatDatagram(int16_t Slave, uint16_t Addr, uint8_t Comm, uint8_t *Data, uint16_t Len, bool bMore)
{ 
  uint16_t i;
  uint16_t LenH;
                                                                  // write an EtherCAT datagram to the Wiz tx buffer
                                                                  // must be called after InitFrame() or another AddEcatDatagram() 

  SPI.transfer(Comm);                                             // command
  SPI.transfer(0x00);                                             // index

  SPI.transfer(((uint8_t)Slave));                                 // slave  
  SPI.transfer((uint8_t)(Slave >> 8));                            // 

  SPI.transfer((uint8_t)Addr);                                    // address 
  SPI.transfer((uint8_t)(Addr >> 8));                             // 

  SPI.transfer((uint8_t)Len);                                     // datagrams length
  LenH = (uint8_t)(Len>>8);                                       //
  if (bMore)                                                      // if this is not the last datagram
  {                                                               // set the bMore datagram bit  
    LenH |= 0x80;                                                 //
  }                                                               //
  SPI.transfer(LenH);                                             //               

  SPI.transfer(0x00);                                             // interrupt 
  SPI.transfer(0x00);                                             // 

  if ((Comm == APRD) || (Comm == BRD) ||(Comm == LRD))            // if it is a read command
  {                                                               // clear the data
    for (i=0; i<Len; i++)                                         //
    {                                                             //
      SPI.transfer(0x00);                                         //
    }                                                             //
  }     
                                                  
  else                                                            // if it is a write command
  {                                                               // transfer the data
    for (i=0; i<Len; i++)                                         //
    {                                                             //
      SPI.transfer(Data[i]);                                      //  
    }                                                             //
  }
  
  SPI.transfer(0x00);                                             // clear the working counter
  SPI.transfer(0x00);                                             //  
}


//---- close the data transfer and send the frame -------------------------------------------------------------------------

void  EasyMaster::SendFrame(uint16_t EthLen)                      // must be call after AddEcatDatagram() to
{                                                                 // terminate the data transfer to the tx buffer
  uint8_t Command;                                                // and send the frame
  
  ScsHigh();                                                      // close the SPI transfer
  SPI.endTransaction();                                           //
       
  EthLen = ((EthLen << 8) & 0xFF00) | ((EthLen >> 8) & 0x00FF);   // update the tx buffer write pointer
  WriteOrRead(SN_TX_WR, WR_S0_REG, &EthLen, sizeof(EthLen));      // 

  Command = SEND_MAC;                                             // send the frame
  WriteOrRead(SN_CR, WR_S0_REG, &Command, sizeof(Command));       //
  delay(1);
}


//---- read or write data a Wiz register or internal buffer -----------------------------------------

void EasyMaster::WriteOrRead(uint16_t Addr, uint8_t BlockSel, void *Data, uint16_t Len)
{  
    uint16_t i;
    
    SPI.beginTransaction(WIZ_SPI_SETTING);
      
    ScsLow(); 
    SPI.transfer((uint8_t)(Addr >> 8));
    SPI.transfer((uint8_t)Addr);
    SPI.transfer(BlockSel);  
    
    if(BlockSel == WR_TX_BUFF)
    {
      for (i=0; i<Len; i++)
      {
         SPI.transfer(*(uint8_t*)(Data + i));                 
      }
    }
    else    
    {
      SPI.transfer(Data, Len);      
    }

    ScsHigh();
    SPI.endTransaction();
}
