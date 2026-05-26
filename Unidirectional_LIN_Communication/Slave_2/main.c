#include "debug.h"
#include <stdio.h>

#define LIN_BAUDRATE         19200u
#define LIN_SYNC             0x55u
#define SLAVE_ID             0x23u

#define BTN_PORT             GPIOC
#define BTN_PIN              GPIO_Pin_1

#define LED_PORT             GPIOD
#define LED_PIN              GPIO_Pin_4
#define LED_ON()             GPIO_ResetBits(LED_PORT, LED_PIN)
#define LED_OFF()            GPIO_SetBits(LED_PORT, LED_PIN)

volatile uint8_t got_break = 0;
volatile uint8_t rx_state = 0;

void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

static uint8_t LIN_GetPid(uint8_t id)
{
    static const uint8_t tbl[64] = {
        0x80,0xC1,0x42,0x03,0xC4,0x85,0x06,0x47,
        0x08,0x49,0xCA,0x8B,0x4C,0x0D,0x8E,0xCF,
        0x50,0x11,0x92,0xD3,0x14,0x55,0xD6,0x97,
        0xD8,0x99,0x1A,0x5B,0x9C,0xDD,0x5E,0x1F,
        0x20,0x61,0xE2,0xA3,0x64,0x25,0xA6,0xE7,
        0xA8,0xE9,0x6A,0x2B,0xEC,0xAD,0x2E,0x6F,
        0xF0,0xB1,0x32,0x73,0xB4,0xF5,0x76,0x37,
        0x78,0x39,0xBA,0xFB,0x3C,0x7D,0xFE,0xBF,
    };
    return tbl[id & 0x3Fu];
}

static uint8_t LIN_Checksum_Enhanced(uint8_t pid, const uint8_t *data, uint8_t len)
{
    uint16_t sum = pid;
    uint8_t i;

    for (i = 0; i < len; i++) {
        sum += data[i];
        if (sum >= 256u) sum -= 255u;
    }

    return (uint8_t)(~sum);
}

static void GPIO_Init_Slave(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    gpio.GPIO_Pin   = BTN_PIN;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(BTN_PORT, &gpio);

    gpio.GPIO_Pin   = LED_PIN;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(LED_PORT, &gpio);
    LED_OFF();
}

static void USART1_Init_Slave(void)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef usart = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOD, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_5;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin  = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &gpio);

    usart.USART_BaudRate            = LIN_BAUDRATE;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);

    USART_LINBreakDetectLengthConfig(USART1, USART_LINBreakDetectLength_11b);
    USART_LINCmd(USART1, ENABLE);

    USART_ITConfig(USART1, USART_IT_LBD, ENABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel                   = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART1, ENABLE);
}

static void LIN_SlaveSendResponse(void)
{
    uint8_t pid = LIN_GetPid(SLAVE_ID);
    uint8_t data = (GPIO_ReadInputDataBit(BTN_PORT, BTN_PIN) == RESET) ? 1u : 0u;
    uint8_t cs = LIN_Checksum_Enhanced(pid, &data, 1u);

    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, data);

    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, cs);

    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    Delay_Init();

    GPIO_Init_Slave();
    USART1_Init_Slave();

    while (1)
    {
        if (GPIO_ReadInputDataBit(BTN_PORT, BTN_PIN) == RESET) {
            LED_ON();
        } else {
            LED_OFF();
        }

        Delay_Ms(1);
    }
}

void USART1_IRQHandler(void)
{
    uint8_t d;
    uint8_t pid = LIN_GetPid(SLAVE_ID);

    if (USART_GetITStatus(USART1, USART_IT_LBD) != RESET) {
        USART_ClearITPendingBit(USART1, USART_IT_LBD);
        got_break = 1;
        rx_state = 1;
        return;
    }

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        d = (uint8_t)USART_ReceiveData(USART1);

        if (!got_break) return;

        if (rx_state == 1) {
            if (d == LIN_SYNC) {
                rx_state = 2;
            } else {
                got_break = 0;
                rx_state = 0;
            }
        } else if (rx_state == 2) {
            if (d == pid) {
                LIN_SlaveSendResponse();
            }
            got_break = 0;
            rx_state = 0;
        }
    }
}