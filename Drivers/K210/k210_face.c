#include "k210_face.h"

#include "clock.h"
#include "stepper_gimbal.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>
#include <string.h>

#define K210_FACE_IMAGE_W                  (320)
#define K210_FACE_IMAGE_H                  (240)
#define K210_FACE_CENTER_X                 (K210_FACE_IMAGE_W / 2)
#define K210_FACE_CENTER_Y                 (K210_FACE_IMAGE_H / 2)
#define K210_FACE_MIN_SCORE_PERMILLE       (600U)

#define K210_FACE_CONTROL_INTERVAL_MS      (10U)
#define K210_FACE_TARGET_FRESH_TIMEOUT_MS  (120U)
#define K210_FACE_TARGET_LOST_TIMEOUT_MS   (1000U)
#define K210_FACE_PREDICTION_MAX_MS        (250U)
#define K210_FACE_PREDICTION_TIME_S        (0.0f)

#define K210_FACE_SPEED_X_KP               (1.0f)
#define K210_FACE_SPEED_X_KI               (0.5f)
#define K210_FACE_SPEED_X_KD               (4.0f)
#define K210_FACE_SPEED_Y_KP               (-1.0f)
#define K210_FACE_SPEED_Y_KI               (0.5f)
#define K210_FACE_SPEED_Y_KD               (-3.0f)
#define K210_FACE_SPEED_PID_OUT_MAX        (2000.0f)
#define K210_FACE_SPEED_PID_I_MAX          (80.0f)

#define K210_FACE_A_MIN_STEP_HZ            (80U)
#define K210_FACE_A_MAX_STEP_HZ            (1800U)
#define K210_FACE_B_MIN_STEP_HZ            (60U)
#define K210_FACE_B_MAX_STEP_HZ            (800U)
#define K210_FACE_CENTER_DEADZONE_X        (8)
#define K210_FACE_CENTER_DEADZONE_Y        (8)
#define K210_FACE_A_ERR_SMALL              (12.0f)
#define K210_FACE_A_ERR_LARGE              (45.0f)
#define K210_FACE_B_ERR_SMALL              (10.0f)
#define K210_FACE_B_ERR_LARGE              (30.0f)
#define K210_FACE_A_SPEED_LIMIT_SMALL      (220.0f)
#define K210_FACE_A_SPEED_LIMIT_MID        (800.0f)
#define K210_FACE_A_SPEED_LIMIT_LARGE      (1600.0f)
#define K210_FACE_B_SPEED_LIMIT_SMALL      (90.0f)
#define K210_FACE_B_SPEED_LIMIT_MID        (260.0f)
#define K210_FACE_B_SPEED_LIMIT_LARGE      (560.0f)
#define K210_FACE_A_ACCEL_SMALL            (3000.0f)
#define K210_FACE_A_ACCEL_MID              (4500.0f)
#define K210_FACE_A_ACCEL_LARGE            (6000.0f)
#define K210_FACE_B_ACCEL_SMALL            (1000.0f)
#define K210_FACE_B_ACCEL_MID              (1800.0f)
#define K210_FACE_B_ACCEL_LARGE            (2600.0f)

/*
 * The WHEELTEC reference PID is kept, but this stepper gimbal's physical
 * positive directions are opposite to the reference servo axes.
 */
#define K210_FACE_AXIS_A_DIR               (-1.0f)
#define K210_FACE_AXIS_B_DIR               (-1.0f)

#define K210_FACE_USE_UART_DMA             (1)
#define K210_FACE_DMA_RX_CH                (0U)
#define K210_FACE_DMA_TX_CH                (1U)
#define K210_FACE_DMA_RX_BUFFER_LEN        (16U)
#define K210_FACE_DMA_TX_BUFFER_LEN        (96U)
#define K210_FACE_DMA_RX_INTERRUPT         (DL_DMA_INTERRUPT_CHANNEL0)
#define K210_FACE_DMA_TX_INTERRUPT         (DL_DMA_INTERRUPT_CHANNEL1)
#define K210_FACE_DMA_ERROR_INTERRUPTS \
    (DL_DMA_INTERRUPT_ADDR_ERROR | DL_DMA_INTERRUPT_DATA_ERROR)
#define K210_FACE_RX_QUEUE_LEN             (16U)
#define K210_FACE_PARSE_LINES_PER_TASK     (16U)

#define K210_FACE_RX_LINE_LEN              (128U)

/*
 * UART0 on PA10/PA11 is enabled by SysConfig, but the current board only sees
 * one stale/noise byte there. Use UART1 RX on PA9 for a clean hardware test.
 * Connect K210 P7(TX) -> PA9. Optional MCU TX uses PA17 -> K210 P6(RX).
 * PB9 and PA15 are not UART RX-capable pins on MSPM0G3507.
 */
#define K210_FACE_USE_UART1_PA9_RX         (1)

#if K210_FACE_USE_UART1_PA9_RX
#define K210_FACE_UART_INST                (UART1)
#define K210_FACE_UART_IRQN                (UART1_INT_IRQn)
#define K210_FACE_UART_RX_PORT             (GPIOA)
#define K210_FACE_UART_RX_PIN              (DL_GPIO_PIN_9)
#define K210_FACE_UART_RX_IOMUX            (IOMUX_PINCM20)
#define K210_FACE_UART_RX_IOMUX_FUNC       (IOMUX_PINCM20_PF_UART1_RX)
#define K210_FACE_UART_TX_PORT             (GPIOA)
#define K210_FACE_UART_TX_PIN              (DL_GPIO_PIN_17)
#define K210_FACE_UART_TX_IOMUX            (IOMUX_PINCM39)
#define K210_FACE_UART_TX_IOMUX_FUNC       (IOMUX_PINCM39_PF_UART1_TX)
#define K210_FACE_UART_RX_DMA_TRIGGER      (DMA_UART1_RX_TRIG)
#define K210_FACE_UART_TX_DMA_TRIGGER      (DMA_UART1_TX_TRIG)
#define K210_FACE_UART_MANUAL_INIT         (1)
#else
#define K210_FACE_UART_INST                (UART_0_INST)
#define K210_FACE_UART_IRQN                (UART_0_INST_INT_IRQN)
#define K210_FACE_UART_RX_PORT             (GPIO_UART_0_RX_PORT)
#define K210_FACE_UART_RX_PIN              (GPIO_UART_0_RX_PIN)
#define K210_FACE_UART_RX_IOMUX            (GPIO_UART_0_IOMUX_RX)
#define K210_FACE_UART_RX_IOMUX_FUNC       (GPIO_UART_0_IOMUX_RX_FUNC)
#define K210_FACE_UART_TX_PORT             (GPIO_UART_0_TX_PORT)
#define K210_FACE_UART_TX_PIN              (GPIO_UART_0_TX_PIN)
#define K210_FACE_UART_TX_IOMUX            (GPIO_UART_0_IOMUX_TX)
#define K210_FACE_UART_TX_IOMUX_FUNC       (GPIO_UART_0_IOMUX_TX_FUNC)
#define K210_FACE_UART_RX_DMA_TRIGGER      (DMA_UART0_RX_TRIG)
#define K210_FACE_UART_TX_DMA_TRIGGER      (DMA_UART0_TX_TRIG)
#define K210_FACE_UART_MANUAL_INIT         (0)
#endif

#define K210_FACE_UART_IBRD_40_MHZ_460800  (5U)
#define K210_FACE_UART_FBRD_40_MHZ_460800  (27U)

#if K210_FACE_USE_UART_DMA
#define K210_FACE_UART_RX_INTERRUPTS                                      \
    (DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |                            \
     DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |                               \
     DL_UART_MAIN_INTERRUPT_BREAK_ERROR |                                 \
     DL_UART_MAIN_INTERRUPT_PARITY_ERROR |                                \
     DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |                               \
     DL_UART_MAIN_INTERRUPT_NOISE_ERROR)
#else
#define K210_FACE_UART_RX_INTERRUPTS                                      \
    (DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR | \
     DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |                                \
     DL_UART_MAIN_INTERRUPT_BREAK_ERROR |                                  \
     DL_UART_MAIN_INTERRUPT_PARITY_ERROR |                                 \
     DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |                                \
     DL_UART_MAIN_INTERRUPT_NOISE_ERROR)
#endif

#define K210_FACE_UART_RX_EDGE_INTERRUPTS \
    (DL_UART_MAIN_INTERRUPT_RXD_POS_EDGE | DL_UART_MAIN_INTERRUPT_RXD_NEG_EDGE)

typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float last_error;
    float last_input;
    bool input_ready;
} K210FacePid;

static volatile char g_rx_build_line[K210_FACE_RX_LINE_LEN];
static volatile char g_rx_line_queue[K210_FACE_RX_QUEUE_LEN][K210_FACE_RX_LINE_LEN];
static volatile uint32_t g_rx_index;
static volatile uint8_t g_rx_queue_head;
static volatile uint8_t g_rx_queue_tail;
static volatile uint8_t g_rx_queue_count;
static volatile uint32_t g_rx_overflow_count;

static K210FaceDetection g_latest_detection;
static bool g_has_detection;
static bool g_tracking_enabled = true;
static bool g_motors_enabled;
static volatile uint32_t g_rx_byte_count;
static volatile uint32_t g_rx_line_count;
static volatile uint32_t g_rx_edge_count;
static volatile uint32_t g_rx_error_count;
static volatile uint32_t g_rx_dma_block_count;
static volatile uint32_t g_tx_dma_done_count;
static volatile bool g_tx_dma_busy;
static volatile uint32_t g_valid_detection_count;
static volatile uint32_t g_parse_error_count;
static volatile uint8_t g_last_rx_byte;
static volatile uint8_t g_rx_pin_level;
static volatile uint32_t g_track_command_count;
static volatile int32_t g_last_error_x;
static volatile int32_t g_last_error_y;
static volatile uint16_t g_object_center_x;
static volatile uint16_t g_object_center_y;
static volatile bool g_target_valid;
static volatile uint32_t g_target_update_ms;
static uint32_t g_control_elapsed_ms;
static K210FacePid g_speed_pid_a;
static K210FacePid g_speed_pid_b;
static float g_error_vx;
static float g_error_vy;
static volatile int32_t g_predicted_error_x;
static volatile int32_t g_predicted_error_y;
static float g_command_speed_a_hz;
static float g_command_speed_b_hz;

#if K210_FACE_USE_UART_DMA
static volatile uint8_t g_dma_rx_buffer[K210_FACE_DMA_RX_BUFFER_LEN];
static uint8_t g_dma_tx_buffer[K210_FACE_DMA_TX_BUFFER_LEN];
#endif

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float abs_f32(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint8_t next_rx_queue_index(uint8_t index)
{
    index++;
    if (index >= K210_FACE_RX_QUEUE_LEN) {
        index = 0U;
    }
    return index;
}

static void k210_rx_byte(uint8_t byte)
{
    uint32_t i;
    uint8_t write_index;

    g_rx_byte_count++;
    g_last_rx_byte = byte;

    if ((byte == '\r') || (byte == '\n')) {
        if (g_rx_index == 0U) {
            return;
        }

        g_rx_build_line[g_rx_index] = '\0';
        if (g_rx_queue_count >= K210_FACE_RX_QUEUE_LEN) {
            g_rx_queue_tail = next_rx_queue_index(g_rx_queue_tail);
            g_rx_queue_count--;
            g_rx_overflow_count++;
        }

        write_index = g_rx_queue_head;
        for (i = 0U; i <= g_rx_index; i++) {
            g_rx_line_queue[write_index][i] = g_rx_build_line[i];
        }
        g_rx_queue_head = next_rx_queue_index(g_rx_queue_head);
        g_rx_queue_count++;
        g_rx_line_count++;
        g_rx_index = 0U;
        return;
    }

    if (g_rx_index < (K210_FACE_RX_LINE_LEN - 1U)) {
        g_rx_build_line[g_rx_index++] = (char)byte;
    } else {
        g_rx_index = 0U;
        g_rx_overflow_count++;
    }
}

static void drain_uart_rx_fifo(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(K210_FACE_UART_INST)) {
        k210_rx_byte(DL_UART_Main_receiveData(K210_FACE_UART_INST));
    }
}

static void sample_uart_rx_pin(void)
{
    g_rx_pin_level =
        ((DL_GPIO_readPins(K210_FACE_UART_RX_PORT, K210_FACE_UART_RX_PIN) &
          K210_FACE_UART_RX_PIN) != 0U) ? 1U : 0U;
}

static void drain_uart_rx_fifo_from_task(void)
{
#if !K210_FACE_USE_UART_DMA
    NVIC_DisableIRQ(K210_FACE_UART_IRQN);
    drain_uart_rx_fifo();
    NVIC_EnableIRQ(K210_FACE_UART_IRQN);
#endif
}

#if K210_FACE_USE_UART_DMA
static void process_dma_rx_bytes(uint32_t count)
{
    uint32_t i;

    if (count > K210_FACE_DMA_RX_BUFFER_LEN) {
        count = K210_FACE_DMA_RX_BUFFER_LEN;
    }

    for (i = 0U; i < count; i++) {
        k210_rx_byte(g_dma_rx_buffer[i]);
    }
}

static void start_dma_rx(void)
{
    DL_DMA_disableChannel(DMA, K210_FACE_DMA_RX_CH);
    DL_DMA_setSrcAddr(DMA, K210_FACE_DMA_RX_CH,
                      (uint32_t)&K210_FACE_UART_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, K210_FACE_DMA_RX_CH,
                       (uint32_t)&g_dma_rx_buffer[0]);
    DL_DMA_setTransferSize(DMA, K210_FACE_DMA_RX_CH,
                           K210_FACE_DMA_RX_BUFFER_LEN);
    DL_DMA_enableChannel(DMA, K210_FACE_DMA_RX_CH);
}

static void process_dma_rx_partial(void)
{
    uint16_t remaining;
    uint32_t count;

    DL_DMA_disableChannel(DMA, K210_FACE_DMA_RX_CH);
    remaining = DL_DMA_getTransferSize(DMA, K210_FACE_DMA_RX_CH);
    count = K210_FACE_DMA_RX_BUFFER_LEN - remaining;
    process_dma_rx_bytes(count);
    drain_uart_rx_fifo();
    start_dma_rx();
}

static void init_uart_dma(void)
{
    static const DL_DMA_Config rx_config = {
        .transferMode  = DL_DMA_SINGLE_TRANSFER_MODE,
        .extendedMode  = DL_DMA_NORMAL_MODE,
        .destIncrement = DL_DMA_ADDR_INCREMENT,
        .srcIncrement  = DL_DMA_ADDR_UNCHANGED,
        .destWidth     = DL_DMA_WIDTH_BYTE,
        .srcWidth      = DL_DMA_WIDTH_BYTE,
        .trigger       = K210_FACE_UART_RX_DMA_TRIGGER,
        .triggerType   = DL_DMA_TRIGGER_TYPE_EXTERNAL,
    };
    static const DL_DMA_Config tx_config = {
        .transferMode  = DL_DMA_SINGLE_TRANSFER_MODE,
        .extendedMode  = DL_DMA_NORMAL_MODE,
        .destIncrement = DL_DMA_ADDR_UNCHANGED,
        .srcIncrement  = DL_DMA_ADDR_INCREMENT,
        .destWidth     = DL_DMA_WIDTH_BYTE,
        .srcWidth      = DL_DMA_WIDTH_BYTE,
        .trigger       = K210_FACE_UART_TX_DMA_TRIGGER,
        .triggerType   = DL_DMA_TRIGGER_TYPE_EXTERNAL,
    };

    DL_DMA_initChannel(DMA, K210_FACE_DMA_RX_CH, &rx_config);
    DL_DMA_initChannel(DMA, K210_FACE_DMA_TX_CH, &tx_config);
    DL_DMA_clearInterruptStatus(DMA,
        K210_FACE_DMA_RX_INTERRUPT | K210_FACE_DMA_TX_INTERRUPT |
        K210_FACE_DMA_ERROR_INTERRUPTS);
    DL_DMA_enableInterrupt(DMA,
        K210_FACE_DMA_RX_INTERRUPT | K210_FACE_DMA_TX_INTERRUPT |
        K210_FACE_DMA_ERROR_INTERRUPTS);
}
#endif

static void init_selected_uart(void)
{
#if K210_FACE_UART_MANUAL_INIT
    static const DL_UART_Main_ClockConfig clock_config = {
        .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1,
    };
    static const DL_UART_Main_Config uart_config = {
        .mode        = DL_UART_MAIN_MODE_NORMAL,
        .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity      = DL_UART_MAIN_PARITY_NONE,
        .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits    = DL_UART_MAIN_STOP_BITS_ONE,
    };

    DL_UART_Main_reset(K210_FACE_UART_INST);
    DL_UART_Main_enablePower(K210_FACE_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_UART_Main_setClockConfig(K210_FACE_UART_INST,
                                (DL_UART_Main_ClockConfig *)&clock_config);
    DL_UART_Main_init(K210_FACE_UART_INST,
                      (DL_UART_Main_Config *)&uart_config);
    DL_UART_Main_setOversampling(K210_FACE_UART_INST,
                                 DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(K210_FACE_UART_INST,
        K210_FACE_UART_IBRD_40_MHZ_460800,
        K210_FACE_UART_FBRD_40_MHZ_460800);
#endif
}

static void configure_uart_rx(void)
{
    NVIC_DisableIRQ(K210_FACE_UART_IRQN);
#if K210_FACE_USE_UART_DMA
    NVIC_DisableIRQ(DMA_INT_IRQn);
#endif
    DL_UART_Main_disableInterrupt(K210_FACE_UART_INST,
                                  K210_FACE_UART_RX_INTERRUPTS);
#if K210_FACE_USE_UART_DMA
    DL_UART_Main_disableDMAReceiveEvent(K210_FACE_UART_INST,
                                        DL_UART_DMA_INTERRUPT_RX);
    DL_UART_Main_disableDMATransmitEvent(K210_FACE_UART_INST);
#endif
    DL_GPIO_initPeripheralInputFunctionFeatures(K210_FACE_UART_RX_IOMUX,
        K210_FACE_UART_RX_IOMUX_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralOutputFunction(K210_FACE_UART_TX_IOMUX,
                                         K210_FACE_UART_TX_IOMUX_FUNC);
    init_selected_uart();
#if K210_FACE_USE_UART_DMA
    init_uart_dma();
    DL_UART_Main_enableFIFOs(K210_FACE_UART_INST);
#endif
    DL_UART_Main_setRXFIFOThreshold(K210_FACE_UART_INST,
                                    DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(K210_FACE_UART_INST,
                                    DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);
    DL_UART_Main_setRXInterruptTimeout(K210_FACE_UART_INST, 2U);
    drain_uart_rx_fifo();
    sample_uart_rx_pin();
    DL_UART_Main_clearInterruptStatus(K210_FACE_UART_INST,
                                      K210_FACE_UART_RX_INTERRUPTS);
    NVIC_ClearPendingIRQ(K210_FACE_UART_IRQN);
#if K210_FACE_USE_UART_DMA
    NVIC_ClearPendingIRQ(DMA_INT_IRQn);
    DL_UART_Main_enableDMAReceiveEvent(K210_FACE_UART_INST,
                                       DL_UART_DMA_INTERRUPT_RX);
    DL_UART_Main_enableDMATransmitEvent(K210_FACE_UART_INST);
#endif
    DL_UART_Main_enableInterrupt(K210_FACE_UART_INST,
                                 K210_FACE_UART_RX_INTERRUPTS);
    DL_UART_Main_enable(K210_FACE_UART_INST);
#if K210_FACE_USE_UART_DMA
    start_dma_rx();
    NVIC_EnableIRQ(DMA_INT_IRQn);
#endif
    NVIC_EnableIRQ(K210_FACE_UART_IRQN);
}

static bool pop_rx_line(char *line, uint32_t line_len)
{
    uint32_t i;
    uint8_t read_index;
    bool has_line;

    if ((line == NULL) || (line_len == 0U)) {
        return false;
    }

    NVIC_DisableIRQ(K210_FACE_UART_IRQN);
#if K210_FACE_USE_UART_DMA
    NVIC_DisableIRQ(DMA_INT_IRQn);
#endif
    has_line = (g_rx_queue_count > 0U);
    if (has_line) {
        read_index = g_rx_queue_tail;
        for (i = 0U; (i < (line_len - 1U)) &&
             (g_rx_line_queue[read_index][i] != '\0');
             i++) {
            line[i] = (char)g_rx_line_queue[read_index][i];
        }
        line[i] = '\0';
        g_rx_queue_tail = next_rx_queue_index(g_rx_queue_tail);
        g_rx_queue_count--;
    }
#if K210_FACE_USE_UART_DMA
    NVIC_EnableIRQ(DMA_INT_IRQn);
#endif
    NVIC_EnableIRQ(K210_FACE_UART_IRQN);

    return has_line;
}

static bool parse_next_int(const char **cursor, int32_t *value)
{
    const char *p = *cursor;
    int32_t sign = 1;
    int32_t result = 0;
    bool has_digit = false;

    while ((*p != '\0') && (((*p < '0') || (*p > '9')) && (*p != '-'))) {
        p++;
    }

    if (*p == '-') {
        sign = -1;
        p++;
    }

    while ((*p >= '0') && (*p <= '9')) {
        result = (result * 10) + (*p - '0');
        has_digit = true;
        p++;
    }

    if (!has_digit) {
        return false;
    }

    *value = result * sign;
    *cursor = p;
    return true;
}

static bool parse_next_score_permille(const char **cursor, uint16_t *score)
{
    const char *p = *cursor;
    uint32_t whole = 0U;
    uint32_t frac = 0U;
    uint32_t frac_digits = 0U;
    bool has_digit = false;

    while ((*p != '\0') && ((*p < '0') || (*p > '9'))) {
        p++;
    }

    while ((*p >= '0') && (*p <= '9')) {
        whole = (whole * 10U) + (uint32_t)(*p - '0');
        has_digit = true;
        p++;
    }

    if (*p == '.') {
        p++;
        while ((*p >= '0') && (*p <= '9')) {
            if (frac_digits < 3U) {
                frac = (frac * 10U) + (uint32_t)(*p - '0');
                frac_digits++;
            }
            has_digit = true;
            p++;
        }
    }

    if (!has_digit) {
        return false;
    }

    while (frac_digits < 3U) {
        frac *= 10U;
        frac_digits++;
    }

    if (whole >= 1U) {
        *score = 1000U;
    } else {
        *score = (uint16_t)clamp_i32((int32_t)frac, 0, 1000);
    }

    *cursor = p;
    return true;
}

static const char *find_detection_start(const char *line)
{
    const char *start = strstr(line, "[[");

    if (start == NULL) {
        return NULL;
    }
    return start + 2;
}

static bool detection_is_valid(const K210FaceDetection *detection)
{
    if (detection->score_permille < K210_FACE_MIN_SCORE_PERMILLE) {
        return false;
    }
    if ((detection->w == 0U) || (detection->h == 0U)) {
        return false;
    }
    if ((detection->x >= K210_FACE_IMAGE_W) ||
        (detection->y >= K210_FACE_IMAGE_H)) {
        return false;
    }
    if (((uint32_t)detection->x + detection->w) > K210_FACE_IMAGE_W) {
        return false;
    }
    if (((uint32_t)detection->y + detection->h) > K210_FACE_IMAGE_H) {
        return false;
    }
    return true;
}

static bool parse_detection_line(const char *line,
                                 K210FaceDetection *detection)
{
    const char *p = find_detection_start(line);
    int32_t values[5];
    uint16_t score;
    uint32_t i;
    unsigned long now;

    if ((p == NULL) || (detection == NULL)) {
        return false;
    }

    for (i = 0U; i < 5U; i++) {
        if (!parse_next_int(&p, &values[i])) {
            return false;
        }
    }

    if (!parse_next_score_permille(&p, &score)) {
        return false;
    }

    if ((values[0] < 0) || (values[1] < 0) ||
        (values[2] <= 0) || (values[3] <= 0) ||
        (values[4] < 0)) {
        return false;
    }

    detection->x = (uint16_t)values[0];
    detection->y = (uint16_t)values[1];
    detection->w = (uint16_t)values[2];
    detection->h = (uint16_t)values[3];
    detection->class_id = (uint8_t)values[4];
    detection->score_permille = score;
    detection->center_x = detection->x + (detection->w / 2U);
    detection->center_y = detection->y + (detection->h / 2U);
    (void)mspm0_get_clock_ms(&now);
    detection->last_update_ms = (uint32_t)now;
    detection->valid = detection_is_valid(detection);

    return detection->valid;
}

static bool parse_face_line(const char *line, K210FaceDetection *detection)
{
    const char *p;
    int32_t values[6];
    uint32_t i;
    unsigned long now;

    if ((line == NULL) || (detection == NULL)) {
        return false;
    }

    p = strstr(line, "FACE,");
    if (p == NULL) {
        return false;
    }
    p += 5;

    for (i = 0U; i < 6U; i++) {
        if (!parse_next_int(&p, &values[i])) {
            return false;
        }
    }

    if ((values[0] < 0) || (values[1] < 0) ||
        (values[2] <= 0) || (values[3] <= 0) ||
        (values[4] < 0)) {
        return false;
    }

    detection->x = (uint16_t)values[0];
    detection->y = (uint16_t)values[1];
    detection->w = (uint16_t)values[2];
    detection->h = (uint16_t)values[3];
    detection->class_id = (uint8_t)values[4];
    detection->score_permille =
        (uint16_t)clamp_i32(values[5], 0, 1000);
    detection->center_x = detection->x + (detection->w / 2U);
    detection->center_y = detection->y + (detection->h / 2U);
    (void)mspm0_get_clock_ms(&now);
    detection->last_update_ms = (uint32_t)now;
    detection->valid = detection_is_valid(detection);

    return detection->valid;
}

static void enable_tracking_motors(void)
{
    if (!g_motors_enabled) {
        StepperGimbal_Enable(STEPPER_GIMBAL_MOTOR_A, true);
        StepperGimbal_Enable(STEPPER_GIMBAL_MOTOR_B, true);
        g_motors_enabled = true;
    }
}

static void track_pid_init(K210FacePid *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_input = 0.0f;
    pid->input_ready = false;
}

static float speed_pid_update(K210FacePid *pid, float error)
{
    float derivative;
    float output;

    if (pid->input_ready) {
        derivative = error - pid->last_error;
    } else {
        derivative = 0.0f;
        pid->input_ready = true;
    }

    pid->integral += error;
    pid->integral = clamp_f32(pid->integral,
                              -K210_FACE_SPEED_PID_I_MAX,
                              K210_FACE_SPEED_PID_I_MAX);

    output = (pid->kp * error) +
             (pid->ki * pid->integral) +
             (pid->kd * derivative);
    output = clamp_f32(output,
                       -K210_FACE_SPEED_PID_OUT_MAX,
                       K210_FACE_SPEED_PID_OUT_MAX);

    pid->last_error = error;
    pid->last_input = error;
    return output;
}

static void speed_pid_reset(K210FacePid *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_input = 0.0f;
    pid->input_ready = false;
}

static float select_by_error(float abs_error, float small_error,
                             float large_error, float small_value,
                             float mid_value, float large_value)
{
    if (abs_error < small_error) {
        return small_value;
    }
    if (abs_error >= large_error) {
        return large_value;
    }
    return mid_value;
}

static float slew_speed(float current_speed, float target_speed,
                        float accel_hz_per_s)
{
    float max_delta =
        accel_hz_per_s * ((float)K210_FACE_CONTROL_INTERVAL_MS / 1000.0f);

    if (target_speed > (current_speed + max_delta)) {
        return current_speed + max_delta;
    }
    if (target_speed < (current_speed - max_delta)) {
        return current_speed - max_delta;
    }
    return target_speed;
}

static float age_to_prediction_time_s(uint32_t target_age_ms)
{
    if (target_age_ms <= K210_FACE_TARGET_FRESH_TIMEOUT_MS) {
        return K210_FACE_PREDICTION_TIME_S;
    }
    if (target_age_ms > K210_FACE_PREDICTION_MAX_MS) {
        target_age_ms = K210_FACE_PREDICTION_MAX_MS;
    }
    return (float)target_age_ms * 0.001f;
}

static int32_t speed_to_signed_step_hz(float speed_hz,
                                       uint32_t min_step_hz,
                                       uint32_t max_step_hz)
{
    float mag = abs_f32(speed_hz);
    int32_t signed_step_hz;

    if (mag < 1.0f) {
        return 0;
    }
    if (mag < (float)min_step_hz) {
        mag = (float)min_step_hz;
    }
    if (mag > (float)max_step_hz) {
        mag = (float)max_step_hz;
    }

    signed_step_hz = (int32_t)(mag + 0.5f);
    return (speed_hz >= 0.0f) ? signed_step_hz : -signed_step_hz;
}

static void apply_axis_speed(StepperGimbalMotor motor,
                             float *current_speed_hz,
                             float target_speed_hz,
                             float accel_hz_per_s,
                             uint32_t min_step_hz,
                             uint32_t max_step_hz)
{
    int32_t signed_step_hz;

    *current_speed_hz = slew_speed(*current_speed_hz,
                                   target_speed_hz,
                                   accel_hz_per_s);
    signed_step_hz = speed_to_signed_step_hz(*current_speed_hz,
                                             min_step_hz,
                                             max_step_hz);

    if (StepperGimbal_SetVelocity(motor, signed_step_hz)) {
        g_track_command_count++;
    }
}

static void stop_axis_tracking(StepperGimbalMotor motor,
                               float *current_speed_hz,
                               K210FacePid *pid)
{
    *current_speed_hz = 0.0f;
    speed_pid_reset(pid);
    (void)StepperGimbal_SetVelocity(motor, 0);
}

static void service_tracking_control(void)
{
    unsigned long now;
    int32_t predicted_error_x;
    int32_t predicted_error_y;
    float abs_error_a;
    float abs_error_b;
    float speed_a;
    float speed_b;
    float speed_limit_a;
    float speed_limit_b;
    float accel_a;
    float accel_b;
    uint32_t target_age_ms;
    float prediction_time_s;

    if (!g_tracking_enabled || !g_target_valid) {
        stop_axis_tracking(STEPPER_GIMBAL_MOTOR_A,
                           &g_command_speed_a_hz, &g_speed_pid_a);
        stop_axis_tracking(STEPPER_GIMBAL_MOTOR_B,
                           &g_command_speed_b_hz, &g_speed_pid_b);
        return;
    }

    enable_tracking_motors();

    (void)mspm0_get_clock_ms(&now);
    target_age_ms = (uint32_t)now - g_target_update_ms;
    if (target_age_ms > K210_FACE_TARGET_LOST_TIMEOUT_MS) {
        g_target_valid = false;
        stop_axis_tracking(STEPPER_GIMBAL_MOTOR_A,
                           &g_command_speed_a_hz, &g_speed_pid_a);
        stop_axis_tracking(STEPPER_GIMBAL_MOTOR_B,
                           &g_command_speed_b_hz, &g_speed_pid_b);
        return;
    }
    prediction_time_s = age_to_prediction_time_s(target_age_ms);

    predicted_error_x =
        (int32_t)((float)g_last_error_x +
                  (g_error_vx * prediction_time_s));
    predicted_error_y =
        (int32_t)((float)g_last_error_y +
                  (g_error_vy * prediction_time_s));
    g_predicted_error_x = predicted_error_x;
    g_predicted_error_y = predicted_error_y;

    abs_error_a = abs_f32((float)predicted_error_x);
    abs_error_b = abs_f32((float)predicted_error_y);

    if (abs_error_a <= (float)K210_FACE_CENTER_DEADZONE_X) {
        stop_axis_tracking(STEPPER_GIMBAL_MOTOR_A,
                           &g_command_speed_a_hz, &g_speed_pid_a);
    } else {
        speed_limit_a = select_by_error(abs_error_a,
            K210_FACE_A_ERR_SMALL, K210_FACE_A_ERR_LARGE,
            K210_FACE_A_SPEED_LIMIT_SMALL,
            K210_FACE_A_SPEED_LIMIT_MID,
            K210_FACE_A_SPEED_LIMIT_LARGE);
        accel_a = select_by_error(abs_error_a,
            K210_FACE_A_ERR_SMALL, K210_FACE_A_ERR_LARGE,
            K210_FACE_A_ACCEL_SMALL,
            K210_FACE_A_ACCEL_MID,
            K210_FACE_A_ACCEL_LARGE);
        speed_a = K210_FACE_AXIS_A_DIR *
                  speed_pid_update(&g_speed_pid_a,
                                   (float)predicted_error_x);
        speed_a = clamp_f32(speed_a, -speed_limit_a, speed_limit_a);
        apply_axis_speed(STEPPER_GIMBAL_MOTOR_A,
                         &g_command_speed_a_hz, speed_a, accel_a,
                         K210_FACE_A_MIN_STEP_HZ,
                         K210_FACE_A_MAX_STEP_HZ);
    }

    if (abs_error_b <= (float)K210_FACE_CENTER_DEADZONE_Y) {
        stop_axis_tracking(STEPPER_GIMBAL_MOTOR_B,
                           &g_command_speed_b_hz, &g_speed_pid_b);
    } else {
        speed_limit_b = select_by_error(abs_error_b,
            K210_FACE_B_ERR_SMALL, K210_FACE_B_ERR_LARGE,
            K210_FACE_B_SPEED_LIMIT_SMALL,
            K210_FACE_B_SPEED_LIMIT_MID,
            K210_FACE_B_SPEED_LIMIT_LARGE);
        accel_b = select_by_error(abs_error_b,
            K210_FACE_B_ERR_SMALL, K210_FACE_B_ERR_LARGE,
            K210_FACE_B_ACCEL_SMALL,
            K210_FACE_B_ACCEL_MID,
            K210_FACE_B_ACCEL_LARGE);
        speed_b = K210_FACE_AXIS_B_DIR *
                  speed_pid_update(&g_speed_pid_b,
                                   (float)predicted_error_y);
        speed_b = clamp_f32(speed_b, -speed_limit_b, speed_limit_b);
        apply_axis_speed(STEPPER_GIMBAL_MOTOR_B,
                         &g_command_speed_b_hz, speed_b, accel_b,
                         K210_FACE_B_MIN_STEP_HZ,
                         K210_FACE_B_MAX_STEP_HZ);
    }
}

static void update_tracking_target(const K210FaceDetection *detection)
{
    int32_t new_error_x;
    int32_t new_error_y;
    uint32_t frame_dt_ms;
    float frame_dt_s;

    if (!g_tracking_enabled || !detection->valid) {
        return;
    }

    g_object_center_x = detection->center_x;
    g_object_center_y = detection->center_y;
    new_error_x = K210_FACE_CENTER_X - (int32_t)detection->center_x;
    new_error_y = K210_FACE_CENTER_Y - (int32_t)detection->center_y;

    if (g_target_valid) {
        frame_dt_ms = detection->last_update_ms - g_target_update_ms;
        if (frame_dt_ms == 0U) {
            frame_dt_ms = 1U;
        }
        frame_dt_s = (float)frame_dt_ms * 0.001f;
        g_error_vx = ((float)new_error_x - (float)g_last_error_x) /
                     frame_dt_s;
        g_error_vy = ((float)new_error_y - (float)g_last_error_y) /
                     frame_dt_s;
    } else {
        g_error_vx = 0.0f;
        g_error_vy = 0.0f;
    }

    g_last_error_x = new_error_x;
    g_last_error_y = new_error_y;
    g_predicted_error_x = new_error_x;
    g_predicted_error_y = new_error_y;
    g_target_update_ms = detection->last_update_ms;
    g_target_valid = true;
}

void K210Face_Init(void)
{
    g_rx_index = 0U;
    g_rx_queue_head = 0U;
    g_rx_queue_tail = 0U;
    g_rx_queue_count = 0U;
    g_rx_overflow_count = 0U;
    g_has_detection = false;
    g_motors_enabled = false;
    g_rx_byte_count = 0U;
    g_rx_line_count = 0U;
    g_rx_edge_count = 0U;
    g_rx_error_count = 0U;
    g_rx_dma_block_count = 0U;
    g_tx_dma_done_count = 0U;
    g_tx_dma_busy = false;
    g_valid_detection_count = 0U;
    g_parse_error_count = 0U;
    g_last_rx_byte = 0U;
    g_rx_pin_level = 0U;
    g_track_command_count = 0U;
    g_last_error_x = 0;
    g_last_error_y = 0;
    g_object_center_x = K210_FACE_CENTER_X;
    g_object_center_y = K210_FACE_CENTER_Y;
    g_target_valid = false;
    g_target_update_ms = 0U;
    g_control_elapsed_ms = 0U;
    track_pid_init(&g_speed_pid_a, K210_FACE_SPEED_X_KP,
                   K210_FACE_SPEED_X_KI, K210_FACE_SPEED_X_KD);
    track_pid_init(&g_speed_pid_b, K210_FACE_SPEED_Y_KP,
                   K210_FACE_SPEED_Y_KI, K210_FACE_SPEED_Y_KD);
    g_error_vx = 0.0f;
    g_error_vy = 0.0f;
    g_predicted_error_x = 0;
    g_predicted_error_y = 0;
    g_command_speed_a_hz = 0.0f;
    g_command_speed_b_hz = 0.0f;
    StepperGimbal_SetHoldAfterMove(true);
    configure_uart_rx();
}

void K210Face_Task(void)
{
    char line[K210_FACE_RX_LINE_LEN];
    K210FaceDetection detection;
    uint32_t processed_lines = 0U;

    sample_uart_rx_pin();
#if !K210_FACE_USE_UART_DMA
    drain_uart_rx_fifo_from_task();
#endif

    while (processed_lines < K210_FACE_PARSE_LINES_PER_TASK) {
        if (!pop_rx_line(line, sizeof(line))) {
            break;
        }
        processed_lines++;

        if ((strstr(line, "PING") != NULL) || (strstr(line, "BOOT") != NULL)) {
            continue;
        }

        if (parse_face_line(line, &detection) ||
            parse_detection_line(line, &detection)) {
            g_latest_detection = detection;
            g_has_detection = true;
            g_valid_detection_count++;
            update_tracking_target(&detection);
        } else {
            g_parse_error_count++;
        }
    }
}

void K210Face_Tick1ms(void)
{
    g_control_elapsed_ms++;
    if (g_control_elapsed_ms >= K210_FACE_CONTROL_INTERVAL_MS) {
        g_control_elapsed_ms = 0U;
        service_tracking_control();
    }
}

bool K210Face_SendDMA(const uint8_t *data, uint16_t len)
{
#if K210_FACE_USE_UART_DMA
    uint16_t i;

    if ((data == NULL) || (len == 0U) ||
        (len > K210_FACE_DMA_TX_BUFFER_LEN)) {
        return false;
    }

    NVIC_DisableIRQ(DMA_INT_IRQn);
    if (g_tx_dma_busy) {
        NVIC_EnableIRQ(DMA_INT_IRQn);
        return false;
    }

    for (i = 0U; i < len; i++) {
        g_dma_tx_buffer[i] = data[i];
    }

    g_tx_dma_busy = true;
    DL_DMA_disableChannel(DMA, K210_FACE_DMA_TX_CH);
    DL_DMA_setSrcAddr(DMA, K210_FACE_DMA_TX_CH,
                      (uint32_t)&g_dma_tx_buffer[0]);
    DL_DMA_setDestAddr(DMA, K210_FACE_DMA_TX_CH,
                       (uint32_t)&K210_FACE_UART_INST->TXDATA);
    DL_DMA_setTransferSize(DMA, K210_FACE_DMA_TX_CH, len);
    DL_DMA_clearInterruptStatus(DMA, K210_FACE_DMA_TX_INTERRUPT);
    DL_DMA_enableChannel(DMA, K210_FACE_DMA_TX_CH);
    NVIC_EnableIRQ(DMA_INT_IRQn);
    return true;
#else
    (void)data;
    (void)len;
    return false;
#endif
}

bool K210Face_SendStringDMA(const char *text)
{
    uint16_t len = 0U;

    if (text == NULL) {
        return false;
    }

    while ((text[len] != '\0') &&
           (len < (K210_FACE_DMA_TX_BUFFER_LEN - 1U))) {
        len++;
    }

    if (text[len] != '\0') {
        return false;
    }

    return K210Face_SendDMA((const uint8_t *)text, len);
}

bool K210Face_IsTxBusy(void)
{
    return g_tx_dma_busy;
}

void K210Face_SetTrackingEnabled(bool enabled)
{
    g_tracking_enabled = enabled;
}

bool K210Face_GetLatest(K210FaceDetection *detection)
{
    if ((detection == NULL) || !g_has_detection) {
        return false;
    }

    *detection = g_latest_detection;
    return true;
}

uint32_t K210Face_GetValidCount(void)
{
    return g_valid_detection_count;
}

uint32_t K210Face_GetRxByteCount(void)
{
    return g_rx_byte_count;
}

uint32_t K210Face_GetRxLineCount(void)
{
    return g_rx_line_count;
}

uint32_t K210Face_GetRxEdgeCount(void)
{
    return g_rx_edge_count;
}

uint32_t K210Face_GetRxErrorCount(void)
{
    return g_rx_error_count;
}

uint32_t K210Face_GetRxDmaBlockCount(void)
{
    return g_rx_dma_block_count;
}

uint32_t K210Face_GetTxDmaDoneCount(void)
{
    return g_tx_dma_done_count;
}

uint32_t K210Face_GetParseErrorCount(void)
{
    return g_parse_error_count;
}

uint32_t K210Face_GetTrackCommandCount(void)
{
    return g_track_command_count;
}

int32_t K210Face_GetLastErrorX(void)
{
    return g_last_error_x;
}

int32_t K210Face_GetLastErrorY(void)
{
    return g_last_error_y;
}

uint8_t K210Face_GetLastByte(void)
{
    return g_last_rx_byte;
}

uint8_t K210Face_GetRxPinLevel(void)
{
    return g_rx_pin_level;
}

static void k210_uart_irq_handler(void)
{
    for (;;) {
        switch (DL_UART_Main_getPendingInterrupt(K210_FACE_UART_INST)) {
#if !K210_FACE_USE_UART_DMA
            case DL_UART_MAIN_IIDX_RX:
#endif
            case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
#if K210_FACE_USE_UART_DMA
                process_dma_rx_partial();
#else
                drain_uart_rx_fifo();
#endif
                break;

            case DL_UART_MAIN_IIDX_RXD_POS_EDGE:
            case DL_UART_MAIN_IIDX_RXD_NEG_EDGE:
                g_rx_edge_count++;
                sample_uart_rx_pin();
                DL_UART_Main_clearInterruptStatus(
                    K210_FACE_UART_INST, K210_FACE_UART_RX_EDGE_INTERRUPTS);
                break;

            case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            case DL_UART_MAIN_IIDX_BREAK_ERROR:
            case DL_UART_MAIN_IIDX_PARITY_ERROR:
            case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            case DL_UART_MAIN_IIDX_NOISE_ERROR:
                g_rx_error_count++;
#if K210_FACE_USE_UART_DMA
                process_dma_rx_partial();
#else
                drain_uart_rx_fifo();
#endif
                DL_UART_Main_clearInterruptStatus(K210_FACE_UART_INST,
                                                  K210_FACE_UART_RX_INTERRUPTS);
                break;

            case DL_UART_MAIN_IIDX_NO_INTERRUPT:
                return;

            default:
                DL_UART_Main_clearInterruptStatus(K210_FACE_UART_INST,
                                                  K210_FACE_UART_RX_INTERRUPTS);
                return;
        }
    }
}

#if K210_FACE_USE_UART1_PA9_RX
void UART1_IRQHandler(void)
{
    k210_uart_irq_handler();
}
#else
void UART0_IRQHandler(void)
{
    k210_uart_irq_handler();
}
#endif

#if K210_FACE_USE_UART_DMA
void DMA_IRQHandler(void)
{
    for (;;) {
        switch (DL_DMA_getPendingInterrupt(DMA)) {
            case DL_DMA_EVENT_IIDX_DMACH0:
                g_rx_dma_block_count++;
                process_dma_rx_bytes(K210_FACE_DMA_RX_BUFFER_LEN);
                start_dma_rx();
                break;

            case DL_DMA_EVENT_IIDX_DMACH1:
                g_tx_dma_done_count++;
                g_tx_dma_busy = false;
                DL_DMA_disableChannel(DMA, K210_FACE_DMA_TX_CH);
                break;

            case DL_DMA_EVENT_IIDX_ADDR_ERROR:
            case DL_DMA_EVENT_IIDX_DATA_ERROR:
                g_rx_error_count++;
                g_tx_dma_busy = false;
                DL_DMA_disableChannel(DMA, K210_FACE_DMA_RX_CH);
                DL_DMA_disableChannel(DMA, K210_FACE_DMA_TX_CH);
                start_dma_rx();
                break;

            case DL_DMA_EVENT_IIDX_NO_INTR:
                return;

            default:
                return;
        }
    }
}
#endif
