#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "cc1000.h"
#include "delay.h"
#include "portio.h"



//=============================================================
//cc1000 register adresses
#define CC1000_MAIN         0x00
#define CC1000_FREQ_2A      0x01
#define CC1000_FREQ_1A      0x02
#define CC1000_FREQ_0A      0x03
#define CC1000_FREQ_2B      0x04
#define CC1000_FREQ_1B      0x05
#define CC1000_FREQ_0B      0x06
#define CC1000_FSEP1        0x07
#define CC1000_FSEP0        0x08
#define CC1000_CURRENT      0x09
#define CC1000_FRONT_END    0x0A
#define CC1000_PA_POW       0x0B
#define CC1000_PLL          0x0C
#define CC1000_LOCK         0x0D
#define CC1000_CAL          0x0E
#define CC1000_MODEM2       0x0F
#define CC1000_MODEM1       0x10
#define CC1000_MODEM0       0x11
#define CC1000_MATCH        0x12
#define CC1000_FSCTRL       0x13
#define CC1000_PRESCALER    0x1C
#define CC1000_TEST6        0x40
#define CC1000_TEST5        0x41
#define CC1000_TEST4        0x42
#define CC1000_TEST3        0x43
#define CC1000_TEST2        0x44
#define CC1000_TEST1        0x45
#define CC1000_TEST0        0x46


//=============================================================
//cc1000 register values
#define CC1000_MAIN_VAL_RX          0b00010001  //rx on A
#define CC1000_MAIN_VAL_TX          0b11100001  //tx on B
#define CC1000_MAIN_VAL_PD          0b00111111
//-------------------------------------------------------------
//settings for Manchester
#define CC1000_FREQ_2A_VAL_MANCH    0x49
#define CC1000_FREQ_1A_VAL_MANCH    0x60
#define CC1000_FREQ_0A_VAL_MANCH    0x00
#define CC1000_FREQ_2B_VAL_MANCH    0x49
#define CC1000_FREQ_1B_VAL_MANCH    0x58
#define CC1000_FREQ_0B_VAL_MANCH    0x19
#define CC1000_FSEP1_VAL_MANCH      0x02
#define CC1000_FSEP0_VAL_MANCH      0xC7
//settings for APRS
#define CC1000_FREQ_2A_VAL_APRS     0x49
#define CC1000_FREQ_1A_VAL_APRS     0xAB
#define CC1000_FREQ_0A_VAL_APRS     0xFE
#define CC1000_FREQ_2B_VAL_APRS     0x49
#define CC1000_FREQ_1B_VAL_APRS     0xAB
#define CC1000_FREQ_0B_VAL_APRS     0xFE
#define CC1000_FSEP1_VAL_APRS       0x00
#define CC1000_FSEP0_VAL_APRS       0x4E
//-------------------------------------------------------------
#define CC1000_CURRENT_VAL_TX       0b10000001
#define CC1000_CURRENT_VAL_RX       0b01000100
#define CC1000_CURRENT_VAL_PD       0b01000100
//-------------------------------------------------------------
#define CC1000_FRONT_END_VAL        0b00000010  //LNA settings

#define CC1000_PA_POW_VAL           0b11111111  //output power

#define CC1000_PLL_VAL              0b01010000  //REFDIV - base freq divider
#define CC1000_LOCK_VAL             0b10000000  //CHP_OUT pin output setting
#define CC1000_CAL_VAL              0b00100110  //calibration
#define CC1000_MODEM2_VAL           0b10010110  //here calculation based on formula from datasheet
#define CC1000_MODEM1_VAL           0b01101111  //avereg filter
//---------------------------------------------
#define CC1000_MODEM0_VAL_MANCH_1k2 0b00010111  //1.2k, Manchester
#define CC1000_MODEM0_VAL_MANCH_9k6 0b01000111  //9.6k, Manchester
#define CC1000_MODEM0_VAL_APRS      0b00110011  //4.8k, UART
//---------------------------------------------
#define CC1000_MATCH_VAL            0b01110000  //tuning capacity setting
#define CC1000_FSCTRL_VAL           0b00000001  //spectrum smoothing
#define CC1000_PRESCALER_VAL        0b00000000  //frequency and current prescaler
//status registers, read only
#define CC1000_TEST6_VAL            0b00000000
#define CC1000_TEST5_VAL            0b00000000
#define CC1000_TEST4_VAL            0b00100101
#define CC1000_TEST3_VAL            0b00000000
#define CC1000_TEST2_VAL            0b00000000
#define CC1000_TEST1_VAL            0b00000000
#define CC1000_TEST0_VAL            0b00000000


//=============================================================
//processor IO interface definitions
#define CC1000_DIO_PIN     GPIO_Pin_1   //pin of DIO
#define CC1000_DIO_PORT    GPIOB        //port of DIO

#define CC1000_DCLK_PIN    GPIO_Pin_0   //pin of DCLK
#define CC1000_DCLK_PORT   GPIOB        //port of DCLK

#define CC1000_PDATA_PIN   GPIO_Pin_6   //pin of PDATA
#define CC1000_PDATA_PORT  GPIOB        //port of PDATA

#define CC1000_PALE_PIN    GPIO_Pin_7   //pin of PALE
#define CC1000_PALE_PORT   GPIOB        //port of PALE

#define CC1000_PCLK_PIN    GPIO_Pin_5   //pin of PCLK
#define CC1000_PCLK_PORT   GPIOB        //port of PCLK


//=============================================================
//pin manipulation macros
#define CC1000_DIO_HI       GPIO_SetBits(CC1000_DIO_PORT, CC1000_DIO_PIN)
#define CC1000_DIO_LOW      GPIO_ResetBits(CC1000_DIO_PORT, CC1000_DIO_PIN)

#define CC1000_DCLK_HI      GPIO_SetBits(CC1000_DCLK_PORT, CC1000_DCLK_PIN)
#define CC1000_DCLK_LOW     GPIO_ResetBits(CC1000_DCLK_PORT, CC1000_DCLK_PIN)

#define CC1000_PDATA_HI     GPIO_SetBits(CC1000_PDATA_PORT, CC1000_PDATA_PIN)
#define CC1000_PDATA_LOW    GPIO_ResetBits(CC1000_PDATA_PORT, CC1000_PDATA_PIN)

#define CC1000_PALE_HI      GPIO_SetBits(CC1000_PALE_PORT, CC1000_PALE_PIN)
#define CC1000_PALE_LOW     GPIO_ResetBits(CC1000_PALE_PORT, CC1000_PALE_PIN)

#define CC1000_PCLK_HI      GPIO_SetBits(CC1000_PCLK_PORT, CC1000_PCLK_PIN)
#define CC1000_PCLK_LOW     GPIO_ResetBits(CC1000_PCLK_PORT, CC1000_PCLK_PIN)

#define CC1000_DIO_STATE    GPIO_ReadInputDataBit(CC1000_DIO_PORT,CC1000_DIO_PIN)
#define CC1000_DCLK_STATE   GPIO_ReadInputDataBit(CC1000_DCLK_PORT,CC1000_DCLK_PIN)


//modem working mode
uint8_t g_mode = CC1000_MODE_SLEEP;  //by default module is in sleep mode

//data buffers
volatile const uint8_t RX_BUFF_SIZE = 2 + CC1000_FRAME_SIZE;    //FRAME_SIZE + 2B of preamble
volatile const uint8_t TX_BUFF_SIZE = CC1000_PREAMBLE_SZIE + CC1000_FRAME_SIZE;

volatile int8_t g_rxBuffer[RX_BUFF_SIZE];
volatile int8_t g_txBuffer[TX_BUFF_SIZE];

//send / recv flags
volatile uint8_t g_frameReceived = 0;
volatile uint8_t g_frameSent = 0;

//transmit / receive index
volatile uint8_t g_txIndex = 0;
volatile uint8_t g_rxIndex = 0;
volatile uint8_t g_lastDioState = 0;



void MsWait(uint16_t delay)
{
    //this function is only used during init and mode switching
    delay_MsBlockWait(delay, DEALY_TIMER0);
}

void UsWait(uint16_t delay)
{
    //this function is only used during init and mode switching
    delay_MsBlockWait(delay, DEALY_TIMER0);
}

void SendBit(uint8_t bite)
{
    //used processor can't be faster than 1us so we not need delay here

    if (bite)
        CC1000_PDATA_HI;
    else
        CC1000_PDATA_LOW;

    //cc1000_Wait(1);   //1us
    CC1000_PCLK_LOW;
    //cc1000_Wait(1);   //1us
    CC1000_PCLK_HI;
    //cc1000_Wait(1);   //1us
}

uint8_t RecvBit(void)
{
    //used processor can't be faster than 1us so we not need delay here

    uint8_t bite;

    CC1000_PCLK_LOW;
    //cc1000_Wait(1);   //1us
    bite = GPIO_ReadInputDataBit(CC1000_PDATA_PORT,CC1000_PDATA_PIN);
    //cc1000_Wait(1);   //1us
    CC1000_PCLK_HI;
    //cc1000_Wait(1);   //1us

    return bite;
}

void WriteRegister(uint8_t adress, uint8_t val)
{
    //used processor can't be faster than 1us so we not need delay here

    uint8_t b;
    //----------------------------------------------------
    CC1000_PALE_LOW;
    //cc1000_Wait(1);   //1us
    //send adress
    for (b = 6; b >= 0; --b)
        cc1000_BitTo(adress & (1 << b));

    //----------------------------------------------------
    cc1000_BitTo(1);
    CC1000_PALE_HI;
    //cc1000_Wait(1);   //1us
    //send value
    for (b = 7; b >= 0; --b)
        cc1000_BitTo(val & (1 << b));
}

uint8_t ReadRegister(uint8_t adress)
{
    //used processor can't be faster than 1us so we not need delay here

    GPIO_InitTypeDef GPIO_InitStructure;

    uint8_t val = 0;
    uint8_t b;
    //----------------------------------------------------
    CC1000_PALE_LOW;
    //cc1000_Wait(1);   //1us
    //send adress
    for (b = 6; b >= 0; --b)
        cc1000_BitTo(adress & (1 << b));

    //----------------------------------------------------
    cc1000_BitTo(0);
    CC1000_PDATA_LOW;

    //set PDATA to input mode
    GPIO_InitStructure.GPIO_Pin = CC1000_PDATA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_Init(GPIOB,&GPIO_InitStructure);

    //cc1000_Wait(1);   //1us

    CC1000_PALE_HI;
    //cc1000_Wait(1);   //1us
    //read value
    for (b = 7; b >= 0; --b)
        if (cc1000_BitFrom())
            val |= (1 << b);

    //----------------------------------------------------
    //set PDATA as output
    GPIO_InitStructure.GPIO_Pin = CC1000_PDATA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_Init(GPIOB,&GPIO_InitStructure);

    //cc1000_Wait(1);   //1us

    return val;
}

void Init_CalibrationVcoPll(void)
{
    WriteRegister(CC1000_FREQ_2A,   CC1000_FREQ_2A_VAL_MANCH);
    WriteRegister(CC1000_FREQ_1A,   CC1000_FREQ_1A_VAL_MANCH);
    WriteRegister(CC1000_FREQ_0A,   CC1000_FREQ_0A_VAL_MANCH);
    WriteRegister(CC1000_FREQ_2B,   CC1000_FREQ_2B_VAL_MANCH);
    WriteRegister(CC1000_FREQ_1B,   CC1000_FREQ_1B_VAL_MANCH);
    WriteRegister(CC1000_FREQ_0B,   CC1000_FREQ_0B_VAL_MANCH);
    WriteRegister(CC1000_FSEP1,     CC1000_FSEP1_VAL_MANCH);
    WriteRegister(CC1000_FSEP0,     CC1000_FSEP0_VAL_MANCH);

    //-------------------------------------------
    WriteRegister(CC1000_MAIN, 0b00010001);                     //RX callibration for channel A

    WriteRegister(CC1000_CURRENT, CC1000_CURRENT_VAL_RX);
    WriteRegister(CC1000_CAL, (CC1000_CAL_VAL | 0b10000000));   //CAL_START = 1
    cc1000_Wait(34);                                            //maximum 34ms

    WriteRegister(CC1000_CAL, (CC1000_CAL_VAL & 0b01111111));   //CAL_START = 0

    //-------------------------------------------
    WriteRegister(CC1000_MAIN, 0b11100001);                     //TX callibration for channel B

    WriteRegister(CC1000_CURRENT, CC1000_CURRENT_VAL_TX);
    WriteRegister(CC1000_PA_POW, 0x00);
    WriteRegister(CC1000_CAL, (CC1000_CAL_VAL | 0b10000000));   //CAL_START = 1
    cc1000_Wait(34);                                            //maximum 34ms

    WriteRegister(CC1000_CAL, (CC1000_CAL_VAL & 0b01111111));   //CAL_START = 0
}

void Init_Reset(void)
{
    cc1000_mode = CC1000_MODE_SLEEP;

    WriteRegister(CC1000_MAIN, 0b00111010);
    WriteRegister(CC1000_MAIN, 0b00111011);

    cc1000_Wait(2);                     //minimum 2ms
}

void Init_WakeUp(void)
{
    char buf;
    buf = ReadRegister(CC1000_MAIN);

    buf &= 0b11111011;                      //CORE_PD = 0
    WriteRegister(CC1000_MAIN,buf);
    cc1000_Wait(2);                         //minimum 2ms

    buf &= 0b11111101;                      //BIAS_PD = 0
    WriteRegister(CC1000_MAIN,buf);
    cc1000_Wait(1);                         //minimum 200us

    //after wake up  tx or rx mode should be enabled
}

void Init_Sleep(void)
{
    cc1000_mode = CC1000_MODE_SLEEP;

    WriteRegister(CC1000_MAIN, CC1000_MAIN_VAL_PD);
    WriteRegister(CC1000_PA_POW, 0x00);
}

uint16_t Crc16(uint8_t *data, uint8_t size)
{
    const uint16_t CC1000_POLYNOMIAL16  = 0x8005;
    const uint16_t CC1000_CRC16TOPBIT = 1 << 15;
    uint16_t remainder = 0;
    uint8_t bitIndex, byteIndex;

    //for each byte
    for (byteIndex = 0; byteIndex < size; ++byteIndex) {
        remainder ^= (data[byteIndex] << 8);
        //for each bit in byte
        for (bitIndex = 8; bitIndex > 0; --bitIndex) {
            if (remainder & CC1000_CRC16TOPBIT)
                remainder = (remainder << 1) ^ CC1000_POLYNOMIAL16;
            else
                remainder = (remainder << 1);
        }
    }

    return remainder;
}

void ClearRxBuffer(void)
{
    uint8_t i;

    for (i = 0; i < RX_BUFF_SIZE; ++i)
        g_rxBuffer[i] = 0;
}

void ClearTxBuffer(void)
{
    uint8_t i;

    for (i = 0; i < TX_BUFF_SIZE; ++i)
        g_txBuffer[i] = 0;
}

void InterruptFunction(void)
{
    volatile int u;

    if(cc1000_mode == CC1000_MODE_TRANSMIT) //#################################################################
    {
        //[CC1000_PREAMBLE_SIZE + CC1000_FRAME_SIZE]

        if(cc1000_sendedData == CC1000_FALSE)   //nie wyslano wszystkich danych
        {
            //-------------------------------------
            portio_Led(PORTIO_LED_RX, PORTIO_ON);
            //-------------------------------------

            if(cc1000_txData[txByte] & (1 << (7 - txBit)))  //jedynka - zmien stan
            {
                if(CC1000_DIO_STATE)    //jesli byl wysoki
                    CC1000_DIO_LOW;     //ustaw niski
                else
                    CC1000_DIO_HI;      //jesli niski - ustaw wysoki
            }
            else                                            //zero - nie zmieniaj stanu
            {
                //CC1000_DIO_LOW;       //dla zera stan zostaje bez zmian
            }

            txBit++;

            if(txBit > 7)
            {
                txBit = 0;
                txByte++;

                if(txByte >= CC1000_PREAMBLE_SIZE + CC1000_FRAME_SIZE)
                {
                    txBit = 0;
                    txByte = 0;
                    cc1000_sendedData = CC1000_TRUE;

                    //-------------------------------------
                    portio_Led(PORTIO_LED_RX, PORTIO_OFF);
                    //-------------------------------------
                }
            }
        }
        else    //wyslano wszystkie
        {
            CC1000_DIO_LOW; //DIO w stanie LOW - i tak sie przelacza bo to Manchester

            //nie moze tego byc bo sie wiesza dlatego ze jest kolizja przerwan
            //cc1000_SwitchToReceive();
            //cc1000_Sleep();
        }
    }
    if(cc1000_mode == CC1000_MODE_RECEIVE)  //#################################################################
    {
        //[2 + CC1000_FRAME_SIZE]

        if(cc1000_receivedData == CC1000_FALSE)
        {
            for(u = 0; u < 2 + CC1000_FRAME_SIZE - 1; u++)  //przesuwa wszystkie poza ostatnim w lewo
            {
                cc1000_rxData[u] = (cc1000_rxData[u] << 1);     //z prawej wskakuje zero

                if(cc1000_rxData[u + 1] & 0b10000000)           //jesli z kolejnego wskoczy 1
                    cc1000_rxData[u] |= 0b00000001;             //to wstaw ja
            }

            cc1000_rxData[u] = (cc1000_rxData[u] << 1);     //z prawej wskakuje zero

            //!!! uwaga na bit LO ktory decyduje o inwersji danych w odbiorniku
            if(CC1000_DIO_STATE == cc1000_lastDio)  //jesli stan sie nie zmienil wstaw 0
                cc1000_rxData[u] &= ~(0b00000001);
            else                                    //jesli stan sie zmienil wstaw 1
                cc1000_rxData[u] |= 0b00000001;

            cc1000_lastDio = CC1000_DIO_STATE;      //zapamietanie stanu

            //tu warunek na zakonczenie odbierania danych - wykrycie preambuly i bajtu startu
            if((cc1000_rxData[1] == (char)(CC1000_PREAMBLE_BYTE)) && (cc1000_rxData[2] == (char)(CC1000_START_BYTE)))
            {
                cc1000_receivedData = CC1000_TRUE;

                //-------------------------------------
                portio_Led(PORTIO_LED_TX, PORTIO_ON);
                //-------------------------------------
            }
        }
    }
}
//==============================================================================================



//==============================================================================================
//EXPORTED FUNCTIONS
char cc1000_SendData(char * message, int size)
{
    int i;
    int e;
    short crc;

    if(cc1000_mode != CC1000_MODE_TRANSMIT)
        cc1000_SwitchToTransmit();

    if(cc1000_sendedData == CC1000_TRUE)    //czy poprzednie dane wyslane
    {
        if(size > CC1000_DATA_SIZE)     //czy nie za duzo danych do wyslania
            return -2;
        //-------------------------------------------------------------
        for(i = 0; i < CC1000_PREAMBLE_SIZE; i++)       //wstawianie preambuly
            cc1000_txData[i] = CC1000_PREAMBLE_BYTE;
        //-------------------------------------------------------------
        cc1000_txData[CC1000_PREAMBLE_SIZE] = CC1000_START_BYTE;    //wstawianie bajtu startu
        //-------------------------------------------------------------
        e = CC1000_PREAMBLE_SIZE + CC1000_STARTBYTE_SIZE;               //od ktorego miejsca wstawiac dane

        for(i = 0; i < CC1000_DATA_SIZE; i++)
        {
            if(i < size)
                cc1000_txData[e + i] = message[i];      //wstawianie danych
            else
                cc1000_txData[e + i] = 0;           //puste wypelniane zerami
        }
        //-------------------------------------------------------------
        //wstawianie rozmiaru
        cc1000_txData[CC1000_PREAMBLE_SIZE + CC1000_STARTBYTE_SIZE + CC1000_DATA_SIZE] = size;
        //-------------------------------------------------------------
        //liczenie CRC
        crc = cc1000_Crc16(&(cc1000_txData[CC1000_PREAMBLE_SIZE + CC1000_STARTBYTE_SIZE]),CC1000_DATA_SIZE + 1);

        //wstawianie i liczenie z (DATA_SIZE + 1) bo dochodzi bajt rozmiaru
        cc1000_txData[CC1000_PREAMBLE_SIZE + CC1000_STARTBYTE_SIZE + CC1000_DATA_SIZE + 1] = (char)((crc >> 8) & 0xFF);
        cc1000_txData[CC1000_PREAMBLE_SIZE + CC1000_STARTBYTE_SIZE + CC1000_DATA_SIZE + 2] = (char)(crc & 0xFF);

        //tu mozna wypisac utworzona ramke
        //-------------------------------------------------------------
        cc1000_sendedData = CC1000_FALSE;
        //-------------------------------------------------------------
        //przelaczenie w odbieranie gdy juz wysle
        //while(cc1000_sendedData == CC1000_FALSE);
        //cc1000_SwitchToReceive();

        return 0;
    }
    else
        return -1;
}

















void cc1000_SetModeTx(void)
{
    ;
}

void cc1000_SetModeRx(void)
{
    ;
}

uint8_t cc1000_GetMode(void)
{
    return g_mode;
}

int8_t cc1000_SendData(int8_t *data, uint8_t size)
{
    ;
}

int8_t cc1000_GetData(int8_t *buffer, uint8_t buffSize)
{
    ;
}

uint8_t cc1000_IsDataReceived(void)
{
    return g_frameReceived;
}

void cc1000_ClearRxFlag(void)
{
    ClearRxBuffer();
    g_frameReceived = 0;
}



















char cc1000_GetData(char * dest, int destsize)
{
    int i;
    short crc;

    char dataSize = 0;

    if(cc1000_mode != CC1000_MODE_RECEIVE)
    {
        printf("WM!\r\n");
        return -5;
    }

    if(cc1000_receivedData == CC1000_TRUE)  //czy cos zostalo odebrane
    {
        if(destsize < CC1000_DATA_SIZE)
            return -2;
        //-------------------------------------------------------------
        //CRC jest przesuniete o 1 i liczone z (DATA_SIZE + 1) bo dochodzi jeszcze bajt ilosci danych przed CRC
        crc = cc1000_Crc16(&(cc1000_rxData[2 + CC1000_STARTBYTE_SIZE]),CC1000_DATA_SIZE + 1);   //CRC liczone z odebranych danych

        if((char)cc1000_rxData[2 + CC1000_STARTBYTE_SIZE + CC1000_DATA_SIZE + 1] != (char)((crc >> 8) & 0xFF))
        {
            printf("CRC error\r\n");
            cc1000_ClearRecvBuffer();
            return -3;
        }
        if((char)cc1000_rxData[2 + CC1000_STARTBYTE_SIZE + CC1000_DATA_SIZE + 2] != (char)(crc & 0xFF))
        {
            printf("CRC error\r\n");
            cc1000_ClearRecvBuffer();
            return -4;
        }
        //-------------------------------------------------------------
        for(i = 0; i < CC1000_DATA_SIZE; i++)
            dest[i] = cc1000_rxData[2 + CC1000_STARTBYTE_SIZE + i];     //wczesniej 2 bajty preambuly i bajt startu
        //-------------------------------------------------------------
        //rozmiar danych w ramce (fragment protokolu)
        dataSize = cc1000_rxData[2 + CC1000_STARTBYTE_SIZE + CC1000_DATA_SIZE];
        //-------------------------------------------------------------
        cc1000_ClearRecvBuffer();

        return dataSize;
    }
    else
        return -1;
}
//----------------------------------------------------------------------------------------------
void cc1000_SetToTransmit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    CC1000_DIO_LOW;

    //DIO wyjsciowy
    GPIO_InitStructure.GPIO_Pin = CC1000_DIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_Init(GPIOB,&GPIO_InitStructure);
    //-------------------------------------
    cc1000_WakeUp();
    //-------------------------------------
    WriteRegister(CC1000_PA_POW, 0x00);
    WriteRegister(CC1000_MAIN, CC1000_MAIN_VAL_TX);
    WriteRegister(CC1000_CURRENT, CC1000_CURRENT_VAL_TX);

    cc1000_Wait(1);                         //minimum 250us
    WriteRegister(CC1000_PA_POW, CC1000_PA_POW_VAL);
    //-------------------------------------
    cc1000_mode = CC1000_MODE_TRANSMIT;
    cc1000_sendedData = CC1000_TRUE;

    txBit = 0;
    txByte = 0;

    portio_Led(PORTIO_LED_TX, PORTIO_ON);
    portio_Led(PORTIO_LED_RX, PORTIO_OFF);

    //gotowy do transmisji danych
}
//----------------------------------------------------------------------------------------------
void cc1000_SetToReceive(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    CC1000_DIO_LOW;

    //DIO wejsciowy
    GPIO_InitStructure.GPIO_Pin = CC1000_DIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_Init(GPIOB,&GPIO_InitStructure);
    //-------------------------------------
    cc1000_WakeUp();
    //-------------------------------------
    WriteRegister(CC1000_MAIN, CC1000_MAIN_VAL_RX);
    WriteRegister(CC1000_CURRENT, CC1000_CURRENT_VAL_RX);

    cc1000_Wait(1);                         //minimum 250us
    //-------------------------------------
    cc1000_mode = CC1000_MODE_RECEIVE;
    cc1000_receivedData = CC1000_FALSE;

    portio_Led(PORTIO_LED_TX, PORTIO_OFF);
    portio_Led(PORTIO_LED_RX, PORTIO_ON);

    //gotowy do odbioru danych
}
//----------------------------------------------------------------------------------------------
void cc1000_SwitchToTransmit(void)
{
    cc1000_Sleep();
    cc1000_SetToTransmit();
}
//----------------------------------------------------------------------------------------------
void cc1000_SwitchToReceive(void)
{
    cc1000_Sleep();
    cc1000_SetToReceive();
}
//----------------------------------------------------------------------------------------------
void cc1000_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;  //sila sygnalu
    //-------------------------------------
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);   //sila sygnalu
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);    //sila sygnalu
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);    //sila sygnalu
    //----------------------------------------------------------------
    //wejscie sily sygnalu i napiecia baterii
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  //bateria i RSSI

    GPIO_Init(GPIOA,&GPIO_InitStructure);
    //----------------------------------------------------------------
    //przetwornik
    ADC_DeInit(ADC1);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;      //niezalezny
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;            //wylacz skanowanie
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;      //tryb ciagly
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //nie ma wyzwalania zewnetrznego
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //wyrownanie danych do prawej
    ADC_InitStructure.ADC_NbrOfChannel = 2;

    ADC_Init(ADC1,&ADC_InitStructure);  //inicjalizacja

    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);             //reset rejestrow kalibracji
    while(ADC_GetResetCalibrationStatus(ADC1));

    ADC_StartCalibration(ADC1);             //start kalibracji
    while(ADC_GetCalibrationStatus(ADC1));
    //-------------------------------------
    //PDATA, PALE i PCLK jako wyjsciowe
    GPIO_InitStructure.GPIO_Pin = CC1000_PDATA_PIN | CC1000_PALE_PIN | CC1000_PCLK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_Init(GPIOB,&GPIO_InitStructure);

    CC1000_PDATA_HI;
    CC1000_PALE_HI;
    CC1000_PCLK_HI;
    //-------------------------------------
    //DCLK i DIO jako wejsciowe
    GPIO_InitStructure.GPIO_Pin = CC1000_DCLK_PIN | CC1000_DIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    CC1000_DCLK_LOW;
    CC1000_DIO_LOW;
    //----------------------------------------------------------------
    //NVIC
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    //----------------------------------------------------------------
    //EXTI
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;

    EXTI_Init(&EXTI_InitStructure);
    //====================================================================
    cc1000_Reset();
    //-------------------------------------
    WriteRegister(CC1000_FREQ_2A,  CC1000_FREQ_2A_VAL_MANCH);
    WriteRegister(CC1000_FREQ_1A,  CC1000_FREQ_1A_VAL_MANCH);
    WriteRegister(CC1000_FREQ_0A,  CC1000_FREQ_0A_VAL_MANCH);
    WriteRegister(CC1000_FREQ_2B,  CC1000_FREQ_2B_VAL_MANCH);
    WriteRegister(CC1000_FREQ_1B,  CC1000_FREQ_1B_VAL_MANCH);
    WriteRegister(CC1000_FREQ_0B,  CC1000_FREQ_0B_VAL_MANCH);
    WriteRegister(CC1000_FSEP1,    CC1000_FSEP1_VAL_MANCH);
    WriteRegister(CC1000_FSEP0,    CC1000_FSEP0_VAL_MANCH);

    WriteRegister(CC1000_CURRENT,  CC1000_CURRENT_VAL_PD);
    WriteRegister(CC1000_FRONT_END,CC1000_FRONT_END_VAL);
    WriteRegister(CC1000_PA_POW,   CC1000_PA_POW_VAL);
    WriteRegister(CC1000_PLL,      CC1000_PLL_VAL);
    WriteRegister(CC1000_LOCK,     CC1000_LOCK_VAL);
    WriteRegister(CC1000_CAL,      CC1000_CAL_VAL);
    WriteRegister(CC1000_MODEM2,   CC1000_MODEM2_VAL);
    WriteRegister(CC1000_MODEM1,   CC1000_MODEM1_VAL);

    WriteRegister(CC1000_MODEM0,   CC1000_MODEM0_VAL_MANCH);

    WriteRegister(CC1000_MATCH,    CC1000_MATCH_VAL);
    WriteRegister(CC1000_FSCTRL,   CC1000_FSCTRL_VAL);
    WriteRegister(CC1000_PRESCALER,CC1000_PRESCALER_VAL);

    //status registers, read only
    // WriteRegister(CC1000_TEST6,    CC1000_TEST6_VAL);
    // WriteRegister(CC1000_TEST5,    CC1000_TEST5_VAL);
    // WriteRegister(CC1000_TEST4,    CC1000_TEST4_VAL);
    // WriteRegister(CC1000_TEST3,    CC1000_TEST3_VAL);
    // WriteRegister(CC1000_TEST2,    CC1000_TEST2_VAL);
    // WriteRegister(CC1000_TEST1,    CC1000_TEST1_VAL);
    // WriteRegister(CC1000_TEST0,    CC1000_TEST0_VAL);
    //-------------------------------------
    cc1000_CalibrationVcoPll();
    cc1000_Sleep();
}












char cc1000_GetMode(void)
{
    return cc1000_mode;
}

uint16_t cc1000_GetSignalLevel(void)
{
    uint16_t rssi;

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);  //rssi input
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    rssi = ADC_GetConversionValue(ADC1);

    //g_rssi = ((float)g_rssiAdc * 3300.0 / 4095.0);
    return 1;
}

uint16_t cc1000_GetBattery(void)
{
    uint16_t battery;
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_1Cycles5);  //battery input
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    battery = ADC_GetConversionValue(ADC1);

    //g_battery = ((float)g_battAdc * 3300.0 / 4095.0 * 6.49);
    return 1;
}

void cc1000_PowerAmp(uint8_t state)
{
    uint8_t mainReg = 0;

    if (cc1000_mode == CC1000_MODE_TX) {
        mainReg = ReadRegister(CC1000_MAIN); //odczytanie rejestru MAIN

        if(state)   //enable
            mainReg &= ~(1 << 4);    //setting to L, enabling
        else        //disable
            mainReg |= (1 << 4);    //setting to H, disabling

        WriteRegister(CC1000_MAIN, mainReg);    //zapisanie rejestu main
    }
}

void cc1000_SetPower(uint8_t powerLevel)
{
    char reg = 0;

    if (cc1000_mode == CC1000_MODE_TX) {
        if (power > 10)
            power = 10;

        switch (power) {
        case 0:
            reg = 0x0F;
            break;
        case 1:
            reg = 0x40;
            break;
        case 2:
            reg = 0x50;
            break;
        case 3:
            reg = 0x50;
            break;
        case 4:
            reg = 0x60;
            break;
        case 5:
            reg = 0x70;
            break;
        case 6:
            reg = 0x80;
            break;
        case 7:
            reg = 0x90;
            break;
        case 8:
            reg = 0xC0;
            break;
        case 9:
            reg = 0xE0;
            break;
        case 10:
            reg = 0xFF;
            break;
        default:
            break;
        }

        WriteRegister(CC1000_PA_POW, reg);
    }
}



void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) == SET) {
        cc1000_Interrupt();

        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

