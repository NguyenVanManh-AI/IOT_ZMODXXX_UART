#include "hal_data.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

// Khai báo các hàm và biến
void R_BSP_WarmStart(bsp_warm_start_event_t event);
void user_uart_callback(uart_callback_args_t *p_args);
void console_write(const char *buffer);
void console_read(char *buffer, uint32_t length);
void split_float(float number, int *integer_part, int *decimal_part);

static volatile bool is_transfer_complete = false;


/* TODO: Enable if you want to open I2C bus */
void g_comms_i2c_bus0_quick_setup(void);

/* Quick setup for g_comms_i2c_bus0. */
void g_comms_i2c_bus0_quick_setup(void)
{
    fsp_err_t err;
    i2c_master_instance_t * p_driver_instance = (i2c_master_instance_t *) g_comms_i2c_bus0_extended_cfg.p_driver_instance;

    /* Open I2C driver, this must be done before calling any COMMS API */
    err = p_driver_instance->p_api->open(p_driver_instance->p_ctrl, p_driver_instance->p_cfg);
    assert(FSP_SUCCESS == err);

#if BSP_CFG_RTOS
    /* Create a semaphore for blocking if a semaphore is not NULL */
    if (NULL != g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore)
    {
#if BSP_CFG_RTOS == 1 // AzureOS
        tx_semaphore_create(g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore->p_semaphore_handle,
                            g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore->p_semaphore_name,
                            (ULONG) 0);
#elif BSP_CFG_RTOS == 2 // FreeRTOS
        *(g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore->p_semaphore_handle)
            = xSemaphoreCreateCountingStatic((UBaseType_t) 1, (UBaseType_t) 0, g_comms_i2c_bus0_extended_cfg.p_blocking_semaphore->p_semaphore_memory);
#endif
    }

    /* Create a recursive mutex for bus lock if a recursive mutex is not NULL */
    if (NULL != g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex)
    {
#if BSP_CFG_RTOS == 1 // AzureOS
        tx_mutex_create(g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex->p_mutex_handle,
                        g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex->p_mutex_name,
                        TX_INHERIT);
#elif BSP_CFG_RTOS == 2 // FreeRTOS
        *(g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex->p_mutex_handle)
            = xSemaphoreCreateRecursiveMutexStatic(g_comms_i2c_bus0_extended_cfg.p_bus_recursive_mutex->p_mutex_memory);
#endif
    }
#endif
}


/* TODO: Enable if you want to open ZMOD4XXX */
#define G_ZMOD4XXX_SENSOR0_NON_BLOCKING (1)
#define G_ZMOD4XXX_SENSOR0_IRQ_ENABLE   (1)

#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
volatile bool g_zmod4xxx_i2c_completed = false;
volatile rm_zmod4xxx_event_t g_zmod4xxx_i2c_callback_event;
#endif
#if G_ZMOD4XXX_SENSOR0_IRQ_ENABLE
volatile bool g_zmod4xxx_irq_completed = false;
#endif

/* TODO: Enable if you want to use a I2C callback */
#define G_ZMOD4XXX_SENSOR0_I2C_CALLBACK_ENABLE (1)
#if G_ZMOD4XXX_SENSOR0_I2C_CALLBACK_ENABLE
void zmod4xxx_comms_i2c_callback(rm_zmod4xxx_callback_args_t * p_args)
{
#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
    g_zmod4xxx_i2c_callback_event = p_args->event;

    if (RM_ZMOD4XXX_EVENT_ERROR != p_args->event)
    {
        g_zmod4xxx_i2c_completed = true;
    }
#else
    FSP_PARAMETER_NOT_USED(p_args);
#endif
}
#endif

/* TODO: Enable if you want to use a IRQ callback */
#define G_ZMOD4XXX_SENSOR0_IRQ_CALLBACK_ENABLE (1)
#if G_ZMOD4XXX_SENSOR0_IRQ_CALLBACK_ENABLE
void zmod4xxx_irq_callback(rm_zmod4xxx_callback_args_t * p_args)
{
#if G_ZMOD4XXX_SENSOR0_IRQ_ENABLE
    if (RM_ZMOD4XXX_EVENT_MEASUREMENT_COMPLETE == p_args->event)
    {
        g_zmod4xxx_irq_completed = true;
    }
#else
    FSP_PARAMETER_NOT_USED(p_args);
#endif
}
#endif

/* Delay */
#define G_ZMOD4XXX_SENSOR0_DELAY_50     (50)
#define G_ZMOD4XXX_SENSOR0_DELAY_5475   (5475)
#define G_ZMOD4XXX_SENSOR0_DELAY_1990   (1990)
#define G_ZMOD4XXX_SENSOR0_DELAY_1010   (1010)
#define G_ZMOD4XXX_SENSOR0_DELAY_90000  (90000)
#define G_ZMOD4XXX_SENSOR0_DELAY_1500   (1500)
#define G_ZMOD4XXX_SENSOR0_DELAY_2000   (2000)
#define G_ZMOD4XXX_SENSOR0_DELAY_3000   (3000)
#define G_ZMOD4XXX_SENSOR0_DELAY_5000   (5000)
#define G_ZMOD4XXX_SENSOR0_DELAY_MS     (1000)

/* Quick setup for g_zmod4xxx_sensor0.
 * - g_comms_i2c_bus0 must be setup before calling this function
 *     (See Developer Assistance -> g_zmod4xxx_sensor0 -> ZMOD4xxx ***** on rm_zmod4xxx -> g_comms_i2c_device0 -> g_comms_i2c_bus0 -> Quick Setup).
 */
void g_zmod4xxx_sensor0_quick_setup(void);

/* Quick setup for g_zmod4xxx_sensor0. */
void g_zmod4xxx_sensor0_quick_setup(void)
{
    fsp_err_t err;

    /* Open ZMOD4XXX sensor instance, this must be done before calling any ZMOD4XXX API */
    err = g_zmod4xxx_sensor0.p_api->open(g_zmod4xxx_sensor0.p_ctrl, g_zmod4xxx_sensor0.p_cfg);
    assert(FSP_SUCCESS == err);
}


/* Quick getting IAQ 1st Gen. Continuous mode values for g_zmod4xxx_sensor0.
 * - g_zmod4xxx_sensor0 must be setup before calling this function.
 */
bool g_zmod4xxx_sensor0_quick_getting_iaq_1st_gen_continuous_mode_data(rm_zmod4xxx_iaq_1st_data_t * p_gas_data);

/* Quick getting gas data for g_zmod4xxx_sensor0. */
bool g_zmod4xxx_sensor0_quick_getting_iaq_1st_gen_continuous_mode_data(rm_zmod4xxx_iaq_1st_data_t * p_gas_data)
{
    fsp_err_t            err;
    rm_zmod4xxx_raw_data_t zmod4xxx_raw_data;
    bool stabilization_complete = false;

    /* Clear callback flags */
#if G_ZMOD4XXX_SENSOR0_IRQ_ENABLE
    g_zmod4xxx_irq_completed = false;
#endif
#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
    g_zmod4xxx_i2c_completed = false;
#endif

    /* Start the measurement */
    /* If the MeasurementStart API is called once, a second call is not required. */
    err = g_zmod4xxx_sensor0.p_api->measurementStart(g_zmod4xxx_sensor0.p_ctrl);
    assert(FSP_SUCCESS == err);
#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
    while (!g_zmod4xxx_i2c_completed)
    {
        ;
    }
    g_zmod4xxx_i2c_completed = false;
#endif

    do
    {
        /* Wait for the measurement to complete */
#if G_ZMOD4XXX_SENSOR0_IRQ_ENABLE
        while (!g_zmod4xxx_irq_completed)
        {
            ;
        }
        g_zmod4xxx_irq_completed = false;
#else
        err = g_zmod4xxx_sensor0.p_api->statusCheck(g_zmod4xxx_sensor0.p_ctrl);
        assert(FSP_SUCCESS == err);
#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
        while (!g_zmod4xxx_i2c_completed)
        {
            ;
        }
        g_zmod4xxx_i2c_completed = false;
#endif
#endif
        /* Read ADC data from ZMOD4xxx sensor */
        err = g_zmod4xxx_sensor0.p_api->read(g_zmod4xxx_sensor0.p_ctrl, &zmod4xxx_raw_data);
        if (err == FSP_ERR_SENSOR_MEASUREMENT_NOT_FINISHED)
        {
#if BSP_CFG_RTOS == 1 // AzureOS
            tx_thread_sleep(G_ZMOD4XXX_SENSOR0_DELAY_50 * TX_TIMER_TICKS_PER_SECOND / G_ZMOD4XXX_SENSOR0_DELAY_MS);
#elif BSP_CFG_RTOS == 2 // FreeRTOS
            vTaskDelay(G_ZMOD4XXX_SENSOR0_DELAY_50 * configTICK_RATE_HZ / G_ZMOD4XXX_SENSOR0_DELAY_MS);
#else // Bare Metal
            R_BSP_SoftwareDelay(G_ZMOD4XXX_SENSOR0_DELAY_50, BSP_DELAY_UNITS_MILLISECONDS);
#endif
        }
    }
    while (err == FSP_ERR_SENSOR_MEASUREMENT_NOT_FINISHED);
    assert(FSP_SUCCESS == err);

#if G_ZMOD4XXX_SENSOR0_NON_BLOCKING
    while (!g_zmod4xxx_i2c_completed)
    {
        ;
    }
    g_zmod4xxx_i2c_completed = false;
#endif

    /* Calculate IAQ 1st Gen. values from ZMOD4xxx ADC data */
    err = g_zmod4xxx_sensor0.p_api->iaq1stGenDataCalculate(g_zmod4xxx_sensor0.p_ctrl, &zmod4xxx_raw_data, p_gas_data);
    if (err == FSP_SUCCESS)
    {
        stabilization_complete = true;
    }
    else if(err == FSP_ERR_SENSOR_IN_STABILIZATION)
    {
        stabilization_complete = false;
    }
    else
    {
        assert(false);
    }

    return stabilization_complete;
}

volatile rm_zmod4xxx_iaq_1st_data_t zmod_1st_data;

void hal_entry (void)
{
#if BSP_TZ_SECURE_BUILD
    R_BSP_NonSecureEnter();
#endif

    // Mở UART
    R_SCI_UART_Open(&g_uart0_ctrl, &g_uart0_cfg);

    char write_buffer[200] = {};
    int integer_part, decimal_part;
    char input_buffer[3] = {}; // Buffer để nhận số từ người dùng
    int input_number = 0;

    g_comms_i2c_bus0_quick_setup();
    g_zmod4xxx_sensor0_quick_setup();

    while (1) {
        g_zmod4xxx_sensor0_quick_getting_iaq_1st_gen_continuous_mode_data((rm_zmod4xxx_iaq_1st_data_t*)& zmod_1st_data);

        // Hiển thị tiêu đề
        sprintf(write_buffer, "zmod_1st_data value: \n\r");
        console_write(write_buffer);

        // In từng giá trị float
        #define PRINT_FLOAT_VALUE(label, value)                     \
            do {                                                    \
                sprintf(write_buffer, label);                       \
                console_write(write_buffer);                        \
                split_float(value, &integer_part, &decimal_part);   \
                sprintf(write_buffer, "%d.%02d\n\r", integer_part, decimal_part); \
                console_write(write_buffer);                        \
            } while (0)

        PRINT_FLOAT_VALUE("rmox: ", zmod_1st_data.rmox);
        PRINT_FLOAT_VALUE("rcda: ", zmod_1st_data.rcda);
        PRINT_FLOAT_VALUE("iaq: ", zmod_1st_data.iaq);
        PRINT_FLOAT_VALUE("tvoc: ", zmod_1st_data.tvoc);
        PRINT_FLOAT_VALUE("etoh: ", zmod_1st_data.etoh);
        PRINT_FLOAT_VALUE("eco2: ", zmod_1st_data.eco2);
    }

    while(1)
    {
        // Yêu cầu nhập số n
        sprintf(write_buffer, "Nhap vao chieu cao cua cay thong (n): ");
        console_write(write_buffer);

        // Đọc số n từ người dùng
        console_read(input_buffer, 2);
        input_buffer[2] = '\0';
        input_number = atoi(input_buffer);

        // Hiển thị số nhận được
        sprintf(write_buffer, "Chieu cao cua cay thong: %d \n\r", input_number);
        console_write(write_buffer);

        // Vẽ cây thông với chiều cao n
        for (int i = 1; i <= input_number; i++)
        {
            int spaces = input_number - i;
            int stars = 2 * i - 1;

            // In khoảng trống
            for (int j = 0; j < spaces; j++)
            {
                console_write(" ");
            }
            // In dấu *
            for (int j = 0; j < stars; j++)
            {
                console_write("*");
            }
            console_write("\r\n"); // Xuống dòng sau khi in xong một tầng của cây thông
        }

        R_BSP_SoftwareDelay(1000, BSP_DELAY_UNITS_MILLISECONDS); // Đợi một khoảng thời gian trước khi tiếp tục
    }
}

// Hàm khởi động
void R_BSP_WarmStart (bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_RESET == event)
    {
#if BSP_FEATURE_FLASH_LP_VERSION != 0
        R_FACI_LP->DFLCTL = 1U;
#endif
    }

    if (BSP_WARM_START_POST_C == event)
    {
        R_IOPORT_Open(&IOPORT_CFG_CTRL, &IOPORT_CFG_NAME);
    }
}

// Callback cho UART
void user_uart_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event){
        case UART_EVENT_TX_COMPLETE:
        case UART_EVENT_RX_COMPLETE:
        {
            is_transfer_complete = true;
            break;
        }
        default:
        {
        }
    }
}

// Hàm ghi dữ liệu ra console
void console_write(const char *buffer){
    is_transfer_complete = false;
    R_SCI_UART_Write(&g_uart0_ctrl, (uint8_t *) buffer, strlen(buffer));
    while (!is_transfer_complete){
        R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MICROSECONDS);
    }
}

// Hàm đọc dữ liệu từ console
void console_read(char *buffer, uint32_t length){
    is_transfer_complete = false;
    R_SCI_UART_Read(&g_uart0_ctrl, (uint8_t *) buffer, length);
    while (!is_transfer_complete){
        R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MICROSECONDS);
    }
}

// Hàm tách phần nguyên và phần thập phân của số float
void split_float(float number, int *integer_part, int *decimal_part) {
    *integer_part = (int)number; // Lấy phần nguyên
    // Tính phần thập phân với 2 chữ số, không có chuyển đổi kiểu float
    *decimal_part = (int)((number - (float)(*integer_part)) * 100);
}
