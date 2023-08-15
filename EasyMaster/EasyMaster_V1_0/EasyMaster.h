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


#ifndef EASYMASTER_H
#define EASYMASTER_H


#include <SPI.h>

#define APRD 1                      // EtherCAT autoincrement read  
#define APWR 2                      // EtherCAT autoincrement write
#define BRD 7                       // EtherCAT broadcast read
#define BWR 8                       // EtherCAT broadcast write
#define LRD 10                      // EtherCAT logical read
#define LWR 11                      // EtherCAT logical write

#define AL_CONTROL        0x120     // AL control register
#define AL_STATUS         0x130     // AL status register
#define EE_ADDR_REG       0x504     // EEPROM address register
#define EE_DATA_REG       0x508     // EEPROM data register
#define EE_CTR_STAT_REG   0x502     // EEPROM control/status register 16 bits

#define EE_READ           0x0100    // EEPROM read command

#define SM_0    0x800               // Synchronization managers
#define SM_1    0x808               //
#define SM_2    0x810               //
#define SM_3    0x818               //

#define FMMU_0  0x600               // Field memory management units
#define FMMU_1  0x610               //

#define INIT    1                   // slave states
#define PREOP   2                   //
#define SAFEOP  4                   //
#define OP      8                   //

#define ETH_SIZE 6+6+2 +2 +12+EASYMASTER.RxSize_ +12+EASYMASTER.TxSize_ +12+1 //--------------- Ethernet frame size -----------------------

                                                                              //--- Ethernet header ----                                                                  
                                                                              // 6 MAC destination
                                                                              // 6 MAC source
                                                                              // 2 protocol (EtherCAT)
                                                                              //
                                                                              // 2 EtherCAT header
                                                                              //
                                                                              //--- LWR datagram (data in output) -------
                                                                              // 12 datagram header +
                                                                              // sizeof(TxData) data  
                                                                              //
                                                                              //--- LRD datagram (data in input) --------
                                                                              // 12 datagram header + 
                                                                              // sizeof(RxData) data 
                                                                              //                                                                       
                                                                              //--- BRD datagram (check slaves status) --
                                                                              // 12 datagram header + 
                                                                              // 1 data   

#define START_FIRST_RX_DATA  (2 +6+6+2 +12)                                   // first data byte of the first EtherCAT datagram (LRD)
#define START_FIRST_TX_DATA  (   6+6+2 +12+EASYMASTER.RxSize_ +12)            // first data byte of the second EtherCAT datagram (LWR)

#define WKC_LRD     (6+6+2  +2 +12+EASYMASTER.RxSize_)                              // working counter for the LRD
#define WKC_LWR     (6+6+2  +2 +12+EASYMASTER.RxSize_ +12+EASYMASTER.TxSize_)       // working counter for the LWR
#define WKC_BRD     (6+6+2  +2 +12+EASYMASTER.RxSize_ +12+EASYMASTER.TxSize_ +12+1) // working counter for the BRD

#define ECAT_STATUS (6+6+2  +2 +12+EASYMASTER.RxSize_ +12+EASYMASTER.TxSize_ +12)   // EtherCAT status of all slaves during sampling


//----------------------------------------------------------------------------------------------------------

#define MR                0x0000          // mode register - 8 bit
#define SN_MR             0x0000          // socket mode register - 8 bit
#define SN_CR             0x0001          // socket command register - 8 bit
#define SN_SR             0x0003          // socket status register - 8 bit

#define SN_TX_WR          0x0024          // Tx write pointer - 16 bit
#define SN_RX_RSR         0x0026          // Rx received size - 16 bit

#define OPEN              0x01            // open socket
#define MAC_RAW           0x04            // socket raw mode

#define SEND_MAC          0x21            // send packet
#define SW_RESET          0x80            // software reset

#define WR_TX_BUFF        0x14            // write tx buffer
#define RD_RX_BUFF        0x18            // read rx buffer
#define WR_S0_REG         0x0C            // write a socket 0 register
#define RD_S0_REG         0x08            // read a socket 0 register
#define WR_CM_REG         0x04            // write a common register
#define RD_CM_REG         0x00            // read a common register


//---------------------------------------------------------------------------------------------------------

class EasyMaster 
{
  public:
    EasyMaster(uint16_t TxSize, uint16_t RxSize, uint8_t* m_pTxData, uint8_t* m_pRxData, uint8_t NumSlaves, uint8_t Wkc, uint8_t* NAME, uint8_t* FMMU, uint8_t* SM);
  
    void Init(uint8_t SampTime);
    bool StartApp(void);
    void EndApp(void);
    void WriteOrRead(uint16_t Addr, uint8_t BlockSel,  void *Data, uint16_t Len);
    
    uint16_t TxSize_;
    uint16_t RxSize_;
    uint8_t * m_pTxData_;
    uint8_t * m_pRxData_;
    uint8_t NumSlaves_;
    uint8_t Wkc_;
    uint8_t * NAME_;
    uint8_t * FMMU_;
    uint8_t * SM_;
                                                                   
    uint16_t WkcTotal;  
    bool bWkcErr;
    bool bOpErr;  
    bool bNoAnswErr; 
    bool volatile bSync;
     
  private:

    uint8_t EEPROM_Read_8(int16_t Slave, uint16_t ByteAddress);
    uint32_t EEPROM_Read_32(int16_t Slave, uint16_t WordAddress);                
    void BlinkForEver(void); 
    void InitFrame(uint16_t EcatLen);
    void AddEcatDatagram (int16_t Slave, uint16_t Addr, uint8_t Comm, uint8_t *Data, uint16_t Len, bool bMore);
    void SendFrame(uint16_t EthLen);
    
    uint32_t EE_Data_32;  
};

#endif
