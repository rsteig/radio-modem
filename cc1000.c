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


//data buffers
#define RX_BUFF_SIZE (CC1000_FRAME_SIZE)
#define TX_BUFF_SIZE (CC1000_PREAMBLE_SIZE + CC1000_FRAME_SIZE)


//modem working mode
volatile uint8_t g_mode = CC1000_MODE_SLEEP;  //by default module is in sleep mode

volatile int8_t g_rxBuffer[RX_BUFF_SIZE];
volatile int8_t g_txBuffer[TX_BUFF_SIZE];

//send / recv flags
volatile uint8_t g_frameReceived = 0;
volatile uint8_t g_frameSent = 0;

//transmit / receive index
volatile uint8_t g_txBit = 0;
volatile uint8_t g_txByte = 0;
volatile uint8_t g_txIndex = 0;

volatile uint8_t g_rxIndex = 0;
volatile uint8_t g_lastDioState = 0;



void MsWait(uint16_t delay)
{
    //this function is only used during init and mode switching
    delay_MsBlockWait(delay, DELAY_TIMER_CC1000);
}

void UsWait(uint16_t delay)
{
    //this function is only used during init and mode switching
    if (delay > 0)
        delay_MsBlockWait(1, DELAY_TIMER_CC1000);
}

void SendBit(uint8_t bite)
{
    //used processor can't be faster than 1us so we not need delay here

    if (bite)
        CC1000_PDATA_HI;
    else
        CC1000_PDATA_LOW;

    UsWait(1);    //1us
    CC1000_PCLK_LOW;
    UsWait(1);    //1us
    CC1000_PCLK_HI;
    UsWait(1);    //1us
}

uint8_t RecvBit(void)
{
    //used processor can't be faster than 1us so we not need delay here

    uint8_t bite;

    CC1000_PCLK_LOW;
    UsWait(1);    //1us
    bite = GPIO_ReadInputDataBit(CC1000_PDATA_PORT,CC1000_PDATA_PIN);
    UsWait(1);    //1us
    CC1000_PCLK_HI;
    UsWait(1);    //1us

    return bite;
}

void WriteRegister(uint8_t adress, uint8_t val)
{
    //used processor can't be faster than 1us so we not need delay here

    int8_t b;
    //----------------------------------------------------
    CC1000_PALE_LOW;
    UsWait(1);    //1us
    //send adress
    for (b = 6; b >= 0; --b)
        SendBit(adress & (1 << b));

    //----------------------------------------------------
    SendBit(1);
    CC1000_PALE_HI;
    UsWait(1);    //1us
    //send value
    for (b = 7; b >= 0; --b)
        SendBit(val & (1 << b));
}

uint8_t ReadRegister(uint8_t adress)
{
    //used processor can't be faster than 1us so we not need delay here

    GPIO_InitTypeDef GPIO_InitStructure;

    uint8_t val = 0;
    int8_t b;
    //----------------------------------------------------
    CC1000_PALE_LOW;
    UsWait(1);    //1us
    //send adress
    for (b = 6; b >= 0; --b)
        SendBit(adress & (1 << b));

    //----------------------------------------------------
    SendBit(0);
    CC1000_PDATA_LOW;

    //set PDATA to input mode
    GPIO_InitStructure.GPIO_Pin = CC1000_PDATA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_Init(GPIOB,&GPIO_InitStructure);

    UsWait(1);    //1us

    CC1000_PALE_HI;
    UsWait(1);    //1us
    //read value
    for (b = 7; b >= 0; --b)
        if (RecvBit())
            val |= (1 << b);

    //----------------------------------------------------
    //set PDATA as output
    GPIO_InitStructure.GPIO_Pin = CC1000_PDATA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_Init(GPIOB,&GPIO_InitStructure);

    UsWait(1);    //1us

    return val;
}

uint16_t Crc16(const int8_t *data, uint8_t size)
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

void cc1000_InterruptFunction(void)
{
    volatile uint8_t u;

    if (g_mode == CC1000_MODE_TX) {
        //check if there are still data to sent
        if (g_frameSent != 1) {
            portio_Led(PORTIO_LED_RX, PORTIO_ON);

            //if 1 then change state
            if (g_txBuffer[g_txByte] & (1 << (7 - g_txBit))) {
                if (CC1000_DIO_STATE)
                    CC1000_DIO_LOW;
                else
                    CC1000_DIO_HI;
            } else {
                //if 0 we do not change pin state
                //CC1000_DIO_LOW;
            }

            ++g_txBit;

            if (g_txBit > 7) {
                g_txBit = 0;
                ++g_txByte;

                if (g_txByte > TX_BUFF_SIZE - 1) {
                    g_txBit = 0;
                    g_txByte = 0;
                    g_frameSent = 1;

                    portio_Led(PORTIO_LED_RX, PORTIO_OFF);
                }
            }
        } else {    //all data sent
            //DIO set to LOW (in fact it is changing because Manchester coding is set)
            CC1000_DIO_LOW;
        }
    }

    if (g_mode == CC1000_MODE_RX) {
        //we assume that after shift there is 0 in lsb

        if (g_frameReceived != 1) {
            //shift left all data in rxBuffer without the last one
            for (u = 0; u < RX_BUFF_SIZE - 1; ++u) {
                //shift buffer left
                g_rxBuffer[u] = (g_rxBuffer[u] << 1);
                //check for 1 on the msb of next byte
                if (g_rxBuffer[u + 1] & 0b10000000)
                    g_rxBuffer[u] |= 0b00000001;
            }
            //shift last byte
            g_rxBuffer[u] = (g_rxBuffer[u] << 1);

            //WARNING: if LO bit is set in CC1000 data in receiver will be inverted
            if(CC1000_DIO_STATE == g_lastDioState)       //if state not chenged decode as 0
                g_rxBuffer[u] &= ~(0b00000001);
            else                                    //if state changed decode as 1
                g_rxBuffer[u] |= 0b00000001;

            g_lastDioState = CC1000_DIO_STATE;      //remember actual state of DIO

            //condition when stop receiving data
            if (g_rxBuffer[0] == g_rxBuffer[1] == CC1000_START_BYTE) {
                g_frameReceived = 1;

                portio_Led(PORTIO_LED_TX, PORTIO_ON);
            }
        }
    }
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
    WriteRegister(CC1000_MAIN, 0b00010001);                     //RX callibration, channel A

    WriteRegister(CC1000_CURRENT, CC1000_CURRENT_VAL_RX);
    WriteRegister(CC1000_CAL, (CC1000_CAL_VAL | 0b10000000));   //CAL_START = 1
    MsWait(34);                                                 //minimum 34ms

    WriteRegister(CC1000_CAL, (CC1000_CAL_VAL & 0b01111111));   //CAL_START = 0

    //-------------------------------------------
    WriteRegister(CC1000_MAIN, 0b11100001);                     //TX callibration, channel B

    WriteRegister(CC1000_CURRENT, CC1000_CURRENT_VAL_TX);
    WriteRegister(CC1000_PA_POW, 0x00);
    WriteRegister(CC1000_CAL, (CC1000_CAL_VAL | 0b10000000));   //CAL_START = 1
    MsWait(34);                                                 //minimum 34ms

    WriteRegister(CC1000_CAL, (CC1000_CAL_VAL & 0b01111111));   //CAL_START = 0
}

void Init_Reset(void)
{
    g_mode = CC1000_MODE_SLEEP;

    WriteRegister(CC1000_MAIN, 0b00111010);
    WriteRegister(CC1000_MAIN, 0b00111011);

    MsWait(2);  //minimum 2ms
}

void Init_WakeUp(void)
{
    uint8_t reg;
    reg = ReadRegister(CC1000_MAIN);

    reg &= 0b11111011;                      //CORE_PD = 0
    WriteRegister(CC1000_MAIN, reg);
    MsWait(2);                              //minimum 2ms

    reg &= 0b11111101;                      //BIAS_PD = 0
    WriteRegister(CC1000_MAIN, reg);
    UsWait(1);                              //minimum 200us

    //after wake up tx or rx mode should be chosen
}

void Init_Sleep(void)
{
    g_mode = CC1000_MODE_SLEEP;

    WriteRegister(CC1000_MAIN, CC1000_MAIN_VAL_PD);
    WriteRegister(CC1000_PA_POW, 0x00);
}

void cc1000_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;      //rssi measure


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);


    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;  //battery and rssi

    GPIO_Init(GPIOA,&GPIO_InitStructure);


    ADC_DeInit(ADC1);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;      //independent
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;           //disable scanning
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;     //continous mode
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //no extern triggering
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //data align to right
    ADC_InitStructure.ADC_NbrOfChannel = 2;

    ADC_Init(ADC1,&ADC_InitStructure);

    ADC_Cmd(ADC1,ENABLE);

    ADC_ResetCalibration(ADC1);             //reset callibration registers
    while(ADC_GetResetCalibrationStatus(ADC1));

    ADC_StartCalibration(ADC1);             //start callibration
    while(ADC_GetCalibrationStatus(ADC1));


    //PDATA, PALE and PCLK as output
    GPIO_InitStructure.GPIO_Pin = CC1000_PDATA_PIN | CC1000_PALE_PIN | CC1000_PCLK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_Init(GPIOB,&GPIO_InitStructure);

    CC1000_PDATA_HI;
    CC1000_PALE_HI;
    CC1000_PCLK_HI;

    //DCLK and DIO as input
    GPIO_InitStructure.GPIO_Pin = CC1000_DCLK_PIN | CC1000_DIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    CC1000_DCLK_LOW;
    CC1000_DIO_LOW;


    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);


    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;

    EXTI_Init(&EXTI_InitStructure);


    Init_Reset();

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

    WriteRegister(CC1000_MODEM0,   CC1000_MODEM0_VAL_MANCH_1k2);

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

    Init_CalibrationVcoPll();
    Init_Sleep();
}

void cc1000_SetModeTx(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    if (g_mode == CC1000_MODE_TX)
        return;

    //put module into sleep mode
    Init_Sleep();

    //DIO as output
    GPIO_InitStructure.GPIO_Pin = CC1000_DIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    CC1000_DIO_LOW;

    Init_WakeUp();

    WriteRegister(CC1000_PA_POW, 0x00);
    WriteRegister(CC1000_MAIN, CC1000_MAIN_VAL_TX);
    WriteRegister(CC1000_CURRENT, CC1000_CURRENT_VAL_TX);
    UsWait(250);    //minimum 250us
    WriteRegister(CC1000_PA_POW, CC1000_PA_POW_VAL);

    portio_Led(PORTIO_LED_TX, PORTIO_ON);
    portio_Led(PORTIO_LED_RX, PORTIO_OFF);

    g_frameSent = 1;
    g_txBit = 0;
    g_txByte = 0;
    g_mode = CC1000_MODE_TX;

    //ready to send data
}

void cc1000_SetModeRx(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    if (g_mode == CC1000_MODE_RX)
        return;

    //put module into sleep mode
    Init_Sleep();

    //DIO as input
    GPIO_InitStructure.GPIO_Pin = CC1000_DIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    CC1000_DIO_LOW;

    Init_WakeUp();

    WriteRegister(CC1000_MAIN, CC1000_MAIN_VAL_RX);
    WriteRegister(CC1000_CURRENT, CC1000_CURRENT_VAL_RX);
    UsWait(250);    //minimum 250us

    portio_Led(PORTIO_LED_TX, PORTIO_OFF);
    portio_Led(PORTIO_LED_RX, PORTIO_ON);

    g_frameReceived = 0;
    g_mode = CC1000_MODE_RX;

    //ready to receive data
}

uint8_t cc1000_GetMode(void)
{
    return g_mode;
}

int8_t cc1000_SendData(const int8_t *data, uint8_t size)
{
    //if size > CC1000_MAX_DATA_SIZE then data will be cuted
    uint8_t i;
    uint16_t crc16;

    //check if in right mode
    if (g_mode != CC1000_MODE_TX)
        return -1;
    //check if last data is already sent
    if (g_frameSent != 1)
        return -2;

    //-------------------------------------------------[PREAMBLE]
    //copy preamble
    for (i = 0 ; i < CC1000_PREAMBLE_SIZE; ++i)
        g_txBuffer[i] = CC1000_PREAMBLE_BYTE;

    //-------------------------------------------------[HEADER]
    //copy start bytes
    for (i = 0; i < CC1000_START_SIZE; ++i)
        g_txBuffer[CC1000_PREAMBLE_SIZE + i] = CC1000_START_BYTE;

    //calculate crc
    crc16 = Crc16(data, size);

    //copy crc
    g_txBuffer[CC1000_PREAMBLE_SIZE + CC1000_START_SIZE] = (crc16 >> 8);
    g_txBuffer[CC1000_PREAMBLE_SIZE + CC1000_START_SIZE + 1] = 0x00FF & crc16;

    //-------------------------------------------------[DATA]
    //copy data
    //if dataLen < maxData fill it with 0
    for (i = 0; i < CC1000_DATA_SIZE; ++i) {
        if (i < size)
            g_txBuffer[CC1000_PREAMBLE_SIZE + CC1000_HEADER_SIZE + i] = data[i];
        else
            g_txBuffer[CC1000_PREAMBLE_SIZE + CC1000_HEADER_SIZE + i] = 0;
    }

    //prepare env to send
    g_txByte = 0;
    g_txBit = 0;

    //let know transmitter that it can start sending data
    g_frameSent = 0;

    return 0;
}

int8_t cc1000_GetData(int8_t *buffer, uint8_t buffSize)
{
    //if buffer is smaller than received data then data excess will be lost
    uint8_t i;

    //check if in right mode
    if (g_mode != CC1000_MODE_RX)
        return -1;
    //check if any data already received
    if (g_frameReceived != 1)
        return -2;

    for (i = 0; i < buffSize; ++i)
        buffer[i] = g_rxBuffer[CC1000_HEADER_SIZE + i];

    g_rxIndex = 0;
    //g_frameReceived = 0;
    //user decide when enable receiving by calling cc1000_ClearRxFlag()

    return 0;
}

uint8_t cc1000_IsDataReceived(void)
{
    return g_frameReceived;
}

void cc1000_ClearRxFlag(void)
{
    uint8_t i;
    if (g_mode != CC1000_MODE_RX)
        return;

    //clearing rx buffer
    for (i = 0; i < RX_BUFF_SIZE; ++i)
        g_rxBuffer[i] = 0;

    portio_Led(PORTIO_LED_TX, PORTIO_OFF);
    g_frameReceived = 0;
}

uint16_t cc1000_GetSignalLevel(void)
{
    uint16_t rssi;

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);  //rssi input
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    rssi = ADC_GetConversionValue(ADC1);

    //g_rssi = ((float)g_rssiAdc * 3300.0 / 4095.0);
    return rssi;
}

uint16_t cc1000_GetBattlLevel(void)
{
    uint16_t batt;

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_1Cycles5);  //batt input
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    batt = ADC_GetConversionValue(ADC1);

    //g_batt = ((float)g_battAdc * 3300.0 / 4095.0);
    return batt;
}

void cc1000_PowerAmp(uint8_t state)
{
    uint8_t mainReg = 0;

    if (g_mode != CC1000_MODE_TX)
        return;

    mainReg = ReadRegister(CC1000_MAIN);    //read MAIN register

    if(state)   //enable
        mainReg &= ~(1 << 4);               //setting to L, enabling
    else        //disable
        mainReg |= (1 << 4);                //setting to H, disabling

    WriteRegister(CC1000_MAIN, mainReg);    //write modified MAIN register
}

void cc1000_SetPower(uint8_t powerLevel)
{
    char reg = 0;

    if (g_mode != CC1000_MODE_TX)
        return;

    if (powerLevel > 10)
        powerLevel = 10;

    switch (powerLevel) {
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



void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) == SET) {
        cc1000_InterruptFunction();

        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

