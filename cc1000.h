/* 
 * Module responsible for communication using cc1000 module
 *
 * Howto:
 * - set up correct IO interface to microcontroller
 * - fill InterruptTransmit() and InterruptReceive() depending on processor
 * - fill cc1000_Wait() to delay correct time (1us resolution)
 * - fill macros handling IO interface
 * - put cc1000_Interrupt() to IRQ handler from DCLK
 * - call cc1000_Init() and set up mode: cc1000_SetToTransmit() or cc1000_SetToReceive()
 * - enjoy :)
 *
 * Default settings:
 * [ ...010101010101... | 00001111 | header | header | header | data........ ]
 */

/*
 * TODO: move battery to separated module
 * TODO: add LED support in right places
 * TODO: frame based on struct
 * TODO: add posibility to select rf channel
 * TODO: fixed frame size
 */

#ifndef __H_CC1000_H__
#define __H_CC1000_H__



//working modes definitions
#define CC1000_MODE_RX      0
#define CC1000_MODE_TX      1
#define CC1000_MODE_SLEEP	2

//module settings
#define CC1000_PREAMBLE_BYTE	0b01010101
#define CC1000_PREAMBLE_SIZE	16

#define CC1000_START_BYTE		0b00001111
#define CC1000_START_SIZE       1
#define CC1000_HEADER_SIZE      3   //how many bytes reserved for header
#define CC1000_DATA_SIZE        10  //maximum data size

#define CC1000_MAX_FRAME_SIZE   (CC1000_START_SIZE + CC1000_HEADER_SIZE + CC1000_DATA_SIZE)



void cc1000_Init(void);

void cc1000_SetModeTx(void);
void cc1000_SetModeRx(void);
uint8_t cc1000_GetMode(void);

int8_t cc1000_SendData(int8_t *data, uint8_t size);
int8_t cc1000_GetData(int8_t *buffer, uint8_t buffSize);

uint8_t cc1000_IsDataReceived(void);
void cc1000_ClearRxFlag(void);

uint16_t cc1000_GetSignalLevel(void);
uint16_t cc1000_GetBattery(void);

void cc1000_PowerAmp(uint8_t state);
void cc1000_SetPower(uint8_t powerLevel);


#endif

