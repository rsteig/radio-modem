//==============================================================================================
//PAWEL POLAWSKI CC1000 LIBRARY V2.0 pawel.polawski@gmail.com
//----------------------------------------------------------------------------------------------
//Howto:
//1) Ustawic odpowiednie piny i porty w definicjach
//2) Ustawic odpowiednie rejestry do aktywacji przerwan w InterruptTransmit() i InterruptReceive()
//3) Edytowac cc1000_Wait() aby po wywolaniu odczekal przynajmniej 1us
//4) Wstawic cc1000_Interrupt() do odpowiedniego przerwania od zmiany stanu pinu DCLK
//5) Wywolac cc1000_Init() i wybrac tryb pracy - cc1000_SetToTransmit() lub cc1000_SetToReceive()
//6) Stworzyc lub odebrac ramke za pomoca cc1000_MakeFrame() lub cc1000_GetFrame()
//7) ENJOY:)
//----------------------------------------------------------------------------------------------
//Domyslna dlugosc preambuly:	13B
//Domyslny bajt startu:			11011011
//Domyslny rozmiar ramki:		10B
//----------------------------------------------------------------------------------------------
//Todo:
//- posprawdzac jeszcze raz rejestry cc1000
//- zmienna dlugosc ramki (lub podawana przy inicjalizacji)
//- mozna odczytac wartosci rejestrow i w ten sposob zobaczyc czy scalak jest obecny / dziala
//- dodac mozliwosc wyboru czestotliwosci kanalu
//==============================================================================================



//==============================================================================================
#ifndef __H_CC1000_H__
#define __H_CC1000_H__
//==============================================================================================



//==============================================================================================
//INCLUDES
//----------------------------------------------------------------------------------------------
//==============================================================================================



//==============================================================================================
//EXPORTED DEFINES
#define CC1000_TRUE		1
#define CC1000_FALSE	0
//----------------------------------------------------------------------------------------------
#define CC1000_MODE_RECEIVE		0
#define CC1000_MODE_TRANSMIT	1
#define CC1000_MODE_SLEEP		2
//----------------------------------------------------------------------------------------------
#define CC1000_PREAMBLE_SIZE	16				//ile bajtow preambuly
#define CC1000_PREAMBLE_BYTE	0b01010101		//jaka preambula
//----------------------------------------------------------------------------------------------
#define CC1000_START_BYTE		0b00001111		//start byte
//----------------------------------------------------------------------------------------------
#define CC1000_STARTBYTE_SIZE   1               //ile bajtow startu
#define CC1000_HEADER_SIZE      3               //SIZE + CRC + CRC
#define CC1000_DATA_SIZE        10              //maksymalny rozmiar danych
//----------------------------------------------------------------------------------------------
#define CC1000_FRAME_SIZE		(CC1000_STARTBYTE_SIZE + CC1000_HEADER_SIZE + CC1000_DATA_SIZE)  //maksymalny rozmiar calej ramki
//----------------------------------------------------------------------------------------------
#define CC1000_DIO_PIN     GPIO_Pin_1   //pin of DIO
#define CC1000_DIO_PORT    GPIOB  //port of DIO

#define CC1000_DCLK_PIN    GPIO_Pin_0   //pin of DCLK
#define CC1000_DCLK_PORT   GPIOB  //port of DCLK

#define CC1000_PDATA_PIN   GPIO_Pin_6   //pin of PDATA
#define CC1000_PDATA_PORT  GPIOB  //port of PDATA

#define CC1000_PALE_PIN    GPIO_Pin_7   //pin of PALE
#define CC1000_PALE_PORT   GPIOB  //port of PALE

#define CC1000_PCLK_PIN    GPIO_Pin_5   //pin of PCLK
#define CC1000_PCLK_PORT   GPIOB  //port of PCLK
//==============================================================================================



//==============================================================================================
//EXPORTED MACROS
//----------------------------------------------------------------------------------------------
//==============================================================================================



//==============================================================================================
//EXPORTED TYPES
//----------------------------------------------------------------------------------------------
//==============================================================================================



//==============================================================================================
//EXPORTED FUNCTIONS PROTOTYPES
void cc1000_SetToTransmit(void);
void cc1000_SetToReceive(void);
//----------------------------------------------------------------------------------------------
//po wlaczeniu za pierwszym razem do zmiany trybu uzywa sie tych funkcji
void cc1000_SwitchToTransmit(void);
void cc1000_SwitchToReceive(void);
//----------------------------------------------------------------------------------------------
char cc1000_SendData(char * message, int size);
//----------------------------------------------------------------------------------------------
char cc1000_IsDataReceived(void);
void cc1000_ClearRecvBuffer(void);

char cc1000_GetData(char * dest, int destsize);
//----------------------------------------------------------------------------------------------
void cc1000_PrintFrame(void);
//----------------------------------------------------------------------------------------------
void cc1000_Init(void);
void cc1000_CalibrationVcoPll(void);
void cc1000_Reset(void);
void cc1000_WakeUp(void);
void cc1000_Sleep(void);
//----------------------------------------------------------------------------------------------
char cc1000_GetMode(void);
//----------------------------------------------------------------------------------------------
void cc1000_MonitorRx(void);
//----------------------------------------------------------------------------------------------
void cc1000_CheckSignalLevel(void);
int cc1000_GetSignalLevel(void);

int cc1000_GetBattery(void);
//----------------------------------------------------------------------------------------------
void cc1000_SetPower(char power);
void cc1000_PowerAmp(char state);
//==============================================================================================



//==============================================================================================
#endif
//==============================================================================================

