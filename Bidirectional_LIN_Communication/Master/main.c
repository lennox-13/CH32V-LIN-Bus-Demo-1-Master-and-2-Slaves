#include "debug.h"

/* =========================================================
 * LIN master with 2 channels
 *
 * Channel 1:
 *   master button PA4
 *   master LED    PA1
 *   slave 1 button status
 *   slave 1 LED command
 *
 * Channel 2:
 *   master button PA5
 *   master LED    PA0
 *   slave 2 button status
 *   slave 2 LED command
 *
 * Final LED logic on each channel:
 *   final_led = slave_button OR master_button
 * ========================================================= */

#define LIN_BAUDRATE            19200u
#define BREAK_BAUDRATE          13000u
#define RX_TIMEOUT_MS               8u
#define STEP_DELAY_MS               5u

#define SLAVE1_STATUS_ID         0x22u
#define SLAVE2_STATUS_ID         0x23u
#define SLAVE1_CMD_ID            0x24u
#define SLAVE2_CMD_ID            0x25u

#define LIN_SYNC                 0x55u
#define RAW_RX_LEN               3u   /* PID echo + DATA + CS */

/* ---------------- LEDs on master ----------------
 * Both are active LOW
 */
#define LED1_PORT                GPIOA
#define LED1_PIN                 GPIO_Pin_1
#define LED1_ON()                GPIO_ResetBits(LED1_PORT, LED1_PIN)
#define LED1_OFF()               GPIO_SetBits(LED1_PORT, LED1_PIN)

#define LED2_PORT                GPIOA
#define LED2_PIN                 GPIO_Pin_0
#define LED2_ON()                GPIO_ResetBits(LED2_PORT, LED2_PIN)
#define LED2_OFF()               GPIO_SetBits(LED2_PORT, LED2_PIN)

/* ---------------- Buttons on master ----------------
 * Buttons are active LOW with pull-up
 */
#define BTN1_PORT                GPIOA
#define BTN1_PIN                 GPIO_Pin_4
#define BTN2_PORT                GPIOA
#define BTN2_PIN                 GPIO_Pin_5

#define BTN1_PRESSED()           (GPIO_ReadInputDataBit(BTN1_PORT, BTN1_PIN) == RESET)
#define BTN2_PRESSED()           (GPIO_ReadInputDataBit(BTN2_PORT, BTN2_PIN) == RESET)

/* ---------------- Response state ---------------- */
volatile uint8_t response_expected = 0;
volatile uint8_t response_done     = 0;
volatile uint8_t response_timeout  = 0;
volatile uint8_t response_ms       = 0;

volatile uint8_t rx_buf[RAW_RX_LEN];
volatile uint8_t rx_cnt = 0;

static uint8_t current_slave_id = SLAVE1_STATUS_ID;

/* last valid slave button states */
static uint8_t slave1_button_state = 0;
static uint8_t slave2_button_state = 0;

void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM3_IRQHandler(void)   __attribute__((interrupt("WCH-Interrupt-fast")));

/* =========================================================
 * Protected LIN identifier
 * ========================================================= */
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

/* =========================================================
 * LIN enhanced checksum
 * ========================================================= */
static uint8_t LIN_Checksum_Enhanced(uint8_t pid, const uint8_t *data, uint8_t len)
{
    uint16_t sum = pid;
    uint8_t i;

    for (i = 0; i < len; i++) {
        sum += data[i];
        if (sum >= 256u) {
            sum -= 255u;
        }
    }

    return (uint8_t)(~sum);
}

/* ========================================================= */
static void GPIO_Init_Master(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* LEDs */
    gpio.GPIO_Pin   = LED1_PIN | LED2_PIN;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);
    LED1_OFF();
    LED2_OFF();

    /* Buttons */
    gpio.GPIO_Pin   = BTN1_PIN | BTN2_PIN;
    gpio.GPIO_Speed = GPIO_Speed_10MHz;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);
}

static void USART2_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_2;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin   = GPIO_Pin_3;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);
}

static void USART2_SetBaud(uint32_t baud)
{
    USART_InitTypeDef usart = {0};

    USART_Cmd(USART2, DISABLE);

    usart.USART_BaudRate            = baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &usart);

    USART_Cmd(USART2, ENABLE);
}

static void USART2_Init_Master(void)
{
    NVIC_InitTypeDef nvic = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    USART2_GPIO_Init();
    USART2_SetBaud(LIN_BAUDRATE);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel                   = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

static void TIM3_Init_Master(void)
{
    TIM_TimeBaseInitTypeDef tim = {0};
    NVIC_InitTypeDef nvic = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_Cmd(TIM3, DISABLE);

    tim.TIM_Period        = 1000u - 1u;
    tim.TIM_Prescaler     = (uint16_t)(SystemCoreClock / 1000000u) - 1u;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &tim);

    TIM_SetCounter(TIM3, 0);
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel                   = TIM3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

static void USART2_SendByte(uint8_t b)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, b);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

static void response_reset(void)
{
    response_expected = 0;
    response_done     = 0;
    response_timeout  = 0;
    response_ms       = 0;
    rx_cnt            = 0;
}

static void LIN_SendBreak(void)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);

    USART2_SetBaud(BREAK_BAUDRATE);
    Delay_Us(5);
    USART2_SendByte(0x00);

    USART2_SetBaud(LIN_BAUDRATE);
    Delay_Us(100);
}

static void LIN_MasterRequestFrame(uint8_t id)
{
    uint8_t pid = LIN_GetPid(id);

    response_reset();
    current_slave_id = id;

    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET) {
        (void)USART_ReceiveData(USART2);
    }

    LIN_SendBreak();
    USART2_SendByte(LIN_SYNC);
    USART2_SendByte(pid);

    response_expected = 1;
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

static void LIN_MasterCommandFrame(uint8_t id, uint8_t data)
{
    uint8_t pid = LIN_GetPid(id);
    uint8_t cs  = LIN_Checksum_Enhanced(pid, &data, 1u);

    while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET) {
        (void)USART_ReceiveData(USART2);
    }

    LIN_SendBreak();
    USART2_SendByte(LIN_SYNC);
    USART2_SendByte(pid);
    USART2_SendByte(data);
    USART2_SendByte(cs);
}

/* =========================================================
 * Process slave status response and store button state
 * ========================================================= */
static void process_response(void)
{
    uint8_t pid;
    uint8_t data;
    uint8_t cs;
    uint8_t exp_cs;

    if (response_timeout) {
        response_timeout = 0;
        return;
    }

    if (!response_done) {
        return;
    }

    response_done = 0;
    TIM_Cmd(TIM3, DISABLE);

    if (rx_cnt != RAW_RX_LEN) {
        return;
    }

    pid = LIN_GetPid(current_slave_id);

    if (rx_buf[0] != pid) {
        return;
    }

    data = rx_buf[1];
    cs   = rx_buf[2];
    exp_cs = LIN_Checksum_Enhanced(pid, &data, 1u);

    if (cs != exp_cs) {
        return;
    }

    if (current_slave_id == SLAVE1_STATUS_ID) {
        slave1_button_state = data ? 1u : 0u;
    } else if (current_slave_id == SLAVE2_STATUS_ID) {
        slave2_button_state = data ? 1u : 0u;
    }
}

/* =========================================================
 * Update master LEDs from:
 *   final = slave_button OR master_button
 * ========================================================= */
static void update_master_leds_and_send_commands(void)
{
    uint8_t ch1_final;
    uint8_t ch2_final;

    ch1_final = (slave1_button_state || BTN1_PRESSED()) ? 1u : 0u;
    ch2_final = (slave2_button_state || BTN2_PRESSED()) ? 1u : 0u;

    if (ch1_final) LED1_ON();
    else           LED1_OFF();

    if (ch2_final) LED2_ON();
    else           LED2_OFF();

    LIN_MasterCommandFrame(SLAVE1_CMD_ID, ch1_final);
    Delay_Ms(STEP_DELAY_MS);

    LIN_MasterCommandFrame(SLAVE2_CMD_ID, ch2_final);
    Delay_Ms(STEP_DELAY_MS);
}

/* =========================================================
 * Scheduler:
 *   0 -> request slave1 status
 *   1 -> request slave2 status
 *   2 -> update local LEDs and send both LED commands
 * ========================================================= */
int main(void)
{
    uint8_t step = 0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    Delay_Init();

    GPIO_Init_Master();
    USART2_Init_Master();
    TIM3_Init_Master();

    while (1)
    {
        process_response();

        if (!response_expected) {
            switch (step) {
            case 0:
                LIN_MasterRequestFrame(SLAVE1_STATUS_ID);
                step = 1;
                break;

            case 1:
                LIN_MasterRequestFrame(SLAVE2_STATUS_ID);
                step = 2;
                break;

            default:
                update_master_leds_and_send_commands();
                step = 0;
                break;
            }

            Delay_Ms(STEP_DELAY_MS);
        }
    }
}

void USART2_IRQHandler(void)
{
    uint8_t d;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        d = (uint8_t)USART_ReceiveData(USART2);

        if (response_expected) {
            if (rx_cnt < RAW_RX_LEN) {
                rx_buf[rx_cnt++] = d;
            }

            if (rx_cnt >= RAW_RX_LEN) {
                response_expected = 0;
                response_done = 1;
                TIM_Cmd(TIM3, DISABLE);
            }
        }
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        if (response_expected) {
            response_ms++;

            if (response_ms >= RX_TIMEOUT_MS) {
                response_expected = 0;
                response_timeout  = 1;
                response_ms       = 0;
                TIM_Cmd(TIM3, DISABLE);
            }
        } else {
            response_ms = 0;
        }
    }
}