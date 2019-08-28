#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stm32f0xx.h>
#include <string.h>
typedef enum {
    // Reserved for system functions only
    HW_NVIC_IRQ_PRIORITY_SYSTEM = 0,
    HW_NVIC_IRQ_PRIORITY_HIGH,
    HW_NVIC_IRQ_PRIORITY_NORMAL,
    HW_NVIC_IRQ_PRIORITY_LOW,
} HwIrqPriority;
#define LOG_W
#define LOG_I
#define LOG_E
#define X800 1
#include "imu.h"
#define BAUDRATE 115200
#define USARTx_TDR_ADDRESS 0x40013828
#define USARTx_RDR_ADDRESS 0x40013824
#define USARTx USART3
#define USARTx_CLK RCC_APB1Periph_USART3
#define USARTx_APBPERIPHCLOCK RCC_APB1PeriphClockCmd
#define USARTx_IRQn USART3_IRQn
#define USARTx_IRQHandler USART3_IRQHandler

#define USARTx_TX_PIN GPIO_Pin_8
#define USARTx_TX_GPIO_PORT GPIOD
#define USARTx_TX_GPIO_CLK RCC_AHBPeriph_GPIOD
#define USARTx_TX_SOURCE GPIO_PinSource8
#define USARTx_TX_AF GPIO_AF_0

#define USARTx_RX_PIN GPIO_Pin_9
#define USARTx_RX_GPIO_PORT GPIOD
#define USARTx_RX_GPIO_CLK RCC_AHBPeriph_GPIOD
#define USARTx_RX_SOURCE GPIO_PinSource9
#define USARTx_RX_AF GPIO_AF_0

#define DMAx_CLK RCC_AHBPeriph_DMA1
#define USARTx_TX_DMA_CHANNEL DMA1_Channel7
#define USARTx_TX_DMA_FLAG_TC DMA1_FLAG_TC7
#define USARTx_TX_DMA_FLAG_GL DMA1_FLAG_GL7
#define USARTx_RX_DMA_CHANNEL DMA1_Channel6
#define USARTx_RX_DMA_FLAG_TC DMA1_FLAG_TC6
#define USARTx_RX_DMA_FLAG_GL DMA1_FLAG_GL6

void hw_nvic_safe_disable_irq(const IRQn_Type a_irq)
{
    NVIC_DisableIRQ(a_irq);
    __DSB();
    __ISB();
}

/**
* @brief  Configures the USART Peripheral.
* @param  None
* @retval None
*/
static void USART_Config(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Enable GPIO clock */
    RCC_AHBPeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);

    /* Enable USART clock */
    USARTx_APBPERIPHCLOCK(USARTx_CLK, ENABLE);

    /* Enable the DMA periph */
    RCC_AHBPeriphClockCmd(DMAx_CLK, ENABLE);

    /* Connect PXx to USARTx_Tx */
    GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);

    /* Connect PXx to USARTx_Rx */
    GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);

    /* Configure USART Tx and Rx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
    GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
    GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);

    /* USARTx configuration ----------------------------------------------------*/
    /* USARTx configured as follow:
  - BaudRate = 230400 baud  
  - Word Length = 8 Bits
  - one Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
    USART_InitStructure.USART_BaudRate = BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    /* When using Parity the word length must be configured to 9 bits */
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTx, &USART_InitStructure);

    /* DMA Configuration -------------------------------------------------------*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->TDR);
    DMA_InitStructure.DMA_BufferSize = (uint16_t)0;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    ;

    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(USARTx_TX_DMA_CHANNEL, &DMA_InitStructure);

    USARTx->CR3 |= USART_CR3_OVRDIS;
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
    /* Enable USART */
    USART_Cmd(USARTx, ENABLE);

    /* Enable and set EXTI0 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /* Enable the USART Rx DMA request */
    USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);

    /* Enable the DMA channel */
    DMA_Cmd(USARTx_TX_DMA_CHANNEL, ENABLE);
}

typedef struct
{
    uint8_t index;
    int16_t angle;
    int16_t rate;
    struct
    {
        int16_t x, y, z;
    } accel;
    uint8_t calibration;
} Msg;

typedef struct
{
    /// @privatesection
    const uint8_t* p_data_;
    size_t size_;

    struct
    {
        bool is_invalid_;
        bool is_available_;
    } status_;
} Parser;

static void parser_construct(Parser* const this, const uint8_t* p_data,
    const size_t size);

static void parser_parse(Parser* const this);

static inline bool parser_is_invalid(Parser* const this)
{
    assert(this);
    return this->status_.is_invalid_;
}

static inline bool parser_is_available(Parser* const this)
{
    assert(this);
    return this->status_.is_available_;
}

typedef enum {
    MSG_TYPE_DATA = 0,
    MSG_TYPE_ON_RESPONSE,
    MSG_TYPE_STANDBY_RESPONSE,
    MSG_TYPE_SLEEP_RESPONSE,
    MSG_TYPE_END,
} MsgType;

static void pin_init(HwImu* const this);
static void uart_dma_init(HwImu* const this);
static void uart_uninit(HwImu* const this);
static void send_data(HwImu* const this, const uint8_t* data, const uint16_t len);
static void on_receive_data(HwImu* const this, const uint8_t d);
static void reset_rx_state(HwImu* const this);
/**
 * Parse the RX buffer and return the resulting msg
 *
 * @param this
 * @param p_s_product Msg output, valid only when MSG_TYPE_DATA is returned
 * @return Type of msg received
 */
static MsgType parse_rx_msg(HwImu* const this, Msg* p_s_product);
static bool is_cotrol_msg(const uint8_t* buf);

static void send_run_mode(HwImu* const this, const HwImuRunMode a_mode);
static void send_on_cmd(HwImu* const this);
static void send_standby_cmd(HwImu* const this);
static void send_sleep_cmd(HwImu* const this);
//static void send_debug_cmd(HwImu *const this);
static void send_calib_beg_cmd(HwImu* const this);
static void send_calib_end_cmd(HwImu* const this);

static inline void ensure_tx_idle(void);

static HwImu* p_active_imu;
static uint32_t time_t_2s;

static void parser_construct(Parser* const this, const uint8_t* a_p_data,
    const size_t a_size)
{
    assert(this);
    assert(a_p_data);
    memset(this, 0, sizeof(*this));
    this->p_data_ = a_p_data;
    this->size_ = a_size;
}

static void parser_parse(Parser* const this)
{
    assert(this);
    memset(&this->status_, 0, sizeof(this->status_));

    if (this->size_ >= 1) {
        // 1st byte, wait for header (0xAA)
        if (this->p_data_[0] != 0xAA) {
            // Wrong header, drop
            this->status_.is_invalid_ = true;
            return;
        }
    }
    if (this->size_ >= 2) {
        // 2nd byte, header
        if (this->p_data_[1] != 0x00) {
            // Wrong header, drop
            this->status_.is_invalid_ = true;
            return;
        }
    }
    if (this->size_ > 15) {
        // Shouldn't happen
        this->status_.is_invalid_ = true;
        return;
    }
    if (this->size_ == 15) {
        this->status_.is_available_ = true;
    }
}

void hw_imu_construct(HwImu* const this)
{
    assert(this);

    memset(this, 0, sizeof(*this));
    this->tx_buf_[0] = 0xAA;
    this->rx_it_ = this->rx_buf_;

    //	if (!time_t_2s)
    //	{
    //		time_t_2s = system_ms_to_time_t(2000);
    //	}

    pin_init(this);
    uart_dma_init(this);
    p_active_imu = this;
    this->mode_ = HW_IMU_RUN_MODE_NORMAL;
    hw_imu_set_run_mode(this, HW_IMU_RUN_MODE_STANDBY);
    this->remote_mode_ = HW_IMU_RUN_MODE_STANDBY;
}

void hw_imu_destruct(HwImu* const this)
{
    assert(this);
    hw_imu_set_run_mode(this, HW_IMU_RUN_MODE_STANDBY);
    // Wait until the cmd is sent
    ensure_tx_idle();

    hw_nvic_safe_disable_irq(USART3_8_IRQn);
    uart_uninit(this);
    GPIOD->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    p_active_imu = NULL;

    memset(this, 0, sizeof(*this));
}

void hw_imu_set_run_mode(HwImu* const this, const HwImuRunMode a_mode)
{
    assert(this);
    assert(a_mode < HW_IMU_RUN_MODE_END);
    if (this->mode_ == a_mode) {
        return;
    }
    this->mode_ = a_mode;
    //this->set_mode_at_ = system_get_time();
    send_run_mode(this, a_mode);
}

void hw_imu_set_enable_calib(HwImu* const this, const bool flag)
{
    assert(this);
    if (flag != this->is_enable_cal_) {
        if (flag) {
            send_calib_beg_cmd(this);
        } else {
            send_calib_end_cmd(this);
        }
    }
    this->is_enable_cal_ = flag;
}

void hw_imu_update(HwImu* const this)
{
    assert(this);
    //	const uint32_t now = system_get_time();
    if (this->remote_mode_ != this->mode_) {
        //			&& now - this->set_mode_at_ >= time_t_2s)
        //	{
        //		// out sync?
        //		LOG_W("HwImu::update", "IMU out of sync");
        send_run_mode(this, this->mode_);
        //		this->set_mode_at_ = now;
    }
}

static void pin_init(HwImu* const this)
{
    //	assert(this);
    //	RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    //	GPIOD->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    //	GPIOD->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;
    //	GPIOD->AFR[1] &= ~(GPIO_AFRH_AFR8 | GPIO_AFRH_AFR9);
}

static void uart_dma_init(HwImu* const this)
{
    USART_Config();
}

static void uart_uninit(HwImu* const this)
{
    assert(this);
    assert(USART3->CR1 & USART_CR1_UE);
    USART3->CR1 = 0;
    USART3->CR3 = 0;
    USART3->CR2 = 0;
    USART3->BRR = 0;
    RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
}

static void send_data(HwImu* const this, const uint8_t* a_p_data,
    const uint16_t a_len)
{
    assert(this);
    assert(a_p_data);
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;
    DMA1_Channel7->CPAR = (uint32_t) & (USART3->TDR);
    DMA1_Channel7->CMAR = (uint32_t)a_p_data;
    DMA1_Channel7->CNDTR = a_len;
    DMA1_Channel7->CCR |= DMA_CCR_MINC | DMA_CCR_DIR;
    DMA1_Channel7->CCR |= DMA_CCR_EN;
}

static void send_run_mode(HwImu* const this, const HwImuRunMode a_mode)
{
    assert(this);
    assert(a_mode < HW_IMU_RUN_MODE_END);

    switch (a_mode) {
    case HW_IMU_RUN_MODE_NORMAL:
        send_on_cmd(this);
        LOG_I("HwImu::send_run_mode", "normal");
        break;

    case HW_IMU_RUN_MODE_STANDBY:
        send_standby_cmd(this);
        LOG_I("HwImu::send_run_mode", "standby");
        break;

    case HW_IMU_RUN_MODE_SLEEP:
        send_sleep_cmd(this);
        LOG_I("HwImu::send_run_mode", "sleep");
        break;

    default:
        assert(false);
        LOG_E("HwImu::send_run_mode", "Unknown mode: %d", a_mode);
        break;
    }
}

static void send_on_cmd(HwImu* const this)
{
    assert(this);
    ensure_tx_idle();
    this->tx_buf_[2] = 0xEE;
    this->tx_buf_[3] = 0xEE;
    this->tx_buf_[4] = 0xEE;
    this->tx_buf_[14] = 0xCA;
    send_data(this, this->tx_buf_, sizeof(this->tx_buf_));
}

static void send_standby_cmd(HwImu* const this)
{
    assert(this);
    ensure_tx_idle();
    this->tx_buf_[2] = 0xFF;
    this->tx_buf_[3] = 0xFF;
    this->tx_buf_[4] = 0xFF;
    this->tx_buf_[14] = 0xFD;
    send_data(this, this->tx_buf_, sizeof(this->tx_buf_));
}

static void send_sleep_cmd(HwImu* const this)
{
    assert(this);
    ensure_tx_idle();
    this->tx_buf_[2] = 0xDD;
    this->tx_buf_[3] = 0xDD;
    this->tx_buf_[4] = 0xDD;
    this->tx_buf_[14] = 0x97;
    send_data(this, this->tx_buf_, sizeof(this->tx_buf_));
}

static void send_calib_beg_cmd(HwImu* const this)
{
    assert(this);
    ensure_tx_idle();
    this->tx_buf_[2] = 0x55;
    this->tx_buf_[3] = 0x55;
    this->tx_buf_[4] = 0x55;
    this->tx_buf_[14] = 0xFF;
    send_data(this, this->tx_buf_, sizeof(this->tx_buf_));
}

static void send_calib_end_cmd(HwImu* const this)
{
    assert(this);
    ensure_tx_idle();
    this->tx_buf_[2] = 0x44;
    this->tx_buf_[3] = 0x44;
    this->tx_buf_[4] = 0x44;
    this->tx_buf_[14] = 0xCC;
    send_data(this, this->tx_buf_, sizeof(this->tx_buf_));
}

/*
static void send_debug_cmd(HwImu *const this)
{
	assert(this);
	ensure_tx_idle();
	this->tx_buf_[2] = 0x66;
	this->tx_buf_[3] = 0x66;
	this->tx_buf_[4] = 0x66;
	this->tx_buf_[14] = 0x32;
	send_data(this, this->tx_buf_, sizeof(this->tx_buf_));
}
*/

static void on_receive_data(HwImu* const this, const uint8_t a_d)
{
    assert(this);
    *this->rx_it_++ = a_d;
    Parser parser;
    const size_t count = this->rx_it_ - this->rx_buf_;
    parser_construct(&parser, (const uint8_t*)this->rx_buf_, count);
    parser_parse(&parser);
    if (parser_is_invalid(&parser)) {
        reset_rx_state(this);
    } else if (parser_is_available(&parser)) {
        // TODO Move out of ISR
        Msg msg;
        const MsgType msg_type = parse_rx_msg(this, &msg);
        switch (msg_type) {
        case MSG_TYPE_DATA:
            if (this->mode_ == HW_IMU_RUN_MODE_NORMAL) {
                this->is_good_ = true;
            }
            this->angle_ = (-msg.angle + 36000) % 36000;
            this->gz_ = msg.rate;
            this->x_ = msg.accel.x;
            this->y_ = msg.accel.y;
            this->z_ = msg.accel.z;
            this->data_valible = true;
            break;

        case MSG_TYPE_ON_RESPONSE:
            LOG_I("HwImu::on_receive_data", "On ACK");
            this->remote_mode_ = HW_IMU_RUN_MODE_NORMAL;
            break;

        case MSG_TYPE_STANDBY_RESPONSE:
            LOG_I("HwImu::on_receive_data", "Standby ACK");
            this->is_good_ = false;
            this->remote_mode_ = HW_IMU_RUN_MODE_STANDBY;
            break;

        case MSG_TYPE_SLEEP_RESPONSE:
            LOG_I("HwImu::on_receive_data", "Sleep ACK");
            this->is_good_ = false;
            this->remote_mode_ = HW_IMU_RUN_MODE_SLEEP;
            break;

        default:
            LOG_E("HwImu::on_receive_data", "Unknown msg type: %d", msg_type);
            break;
        }

        // End
        reset_rx_state(this);
    }
}

static void reset_rx_state(HwImu* const this)
{
    assert(this);
    this->rx_it_ = this->rx_buf_;
}

static MsgType parse_rx_msg(HwImu* const this, Msg* a_p_s_product)
{
    // We can safely cast volatile away as this fn is only called inside the ISR
    const uint8_t* buf = (const uint8_t*)this->rx_buf_;
    uint8_t checksum = 0;
    for (int i = 2; i < 14; ++i) {
        checksum += buf[i];
    }
    if (buf[14] != checksum) {
        LOG_W("HwImu::parse_rx_msg", "Checksum failed");
        return MSG_TYPE_END;
    }
    if (is_cotrol_msg(buf)) {
        switch (buf[5]) {
        case 0xDD:
            return MSG_TYPE_SLEEP_RESPONSE;

        case 0xEE:
            return MSG_TYPE_ON_RESPONSE;

        case 0xFF:
            return MSG_TYPE_STANDBY_RESPONSE;

        default:
            assert(false);
            LOG_E("HwImu::parse_rx_msg", "Unknown control msg");
            return MSG_TYPE_END;
        }
    } else {
        a_p_s_product->index = buf[2];
        a_p_s_product->angle = buf[3] | (buf[4] << 8);
        a_p_s_product->rate = -(buf[5] | (buf[6] << 8));
        a_p_s_product->accel.y = buf[7] | (buf[8] << 8);
        a_p_s_product->accel.x = -(buf[9] | (buf[10] << 8));
        a_p_s_product->accel.z = -(buf[11] | (buf[12] << 8));
        a_p_s_product->calibration = buf[13];
        return MSG_TYPE_DATA;
    }
}

static bool is_cotrol_msg(const uint8_t* buf)
{
    for (int i = 2; i < 5; ++i) {
        if (buf[i]) {
            return false;
        }
    }
    if (buf[5] != 0xEE && buf[5] != 0xFF && buf[5] != 0xDD) {
        return false;
    }
    for (int i = 6; i < 8; ++i) {
        if (buf[i] != buf[5]) {
            return false;
        }
    }
    for (int i = 8; i < 14; ++i) {
        if (buf[i]) {
            return false;
        }
    }
    return true;
}

static inline void ensure_tx_idle(void)
{
    while (DMA1_Channel7->CNDTR) {
    }
}

#ifdef __cplusplus
extern "C" {
#endif

void USART3_4_IRQHandler(void)
{
    assert(p_active_imu);
    if (USART3->ISR & USART_ISR_RXNE) {
        USART3->ISR &= ~USART_ISR_RXNE;
        if (p_active_imu) {
            on_receive_data(p_active_imu, USART3->RDR);
        }
    } else {
        assert(false);
        LOG_W("HwImu::USART3_IRQHandler", "Unhandled IRQ %X", USART3->ISR);
    }
}
#ifdef __cplusplus
}
#endif
