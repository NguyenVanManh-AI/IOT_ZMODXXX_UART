/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_sci_uart.h"
#include "r_uart_api.h"
#include "rm_comms_i2c.h"
#include "rm_comms_api.h"
#include "../ra/fsp/src/rm_zmod4xxx/zmod4xxx_types.h"
#include "../ra/fsp/src/rm_zmod4xxx/iaq_1st_gen/iaq_1st_gen.h"
#include "rm_zmod4xxx.h"
#include "rm_zmod4xxx_api.h"
FSP_HEADER
/** UART on SCI Instance. */
extern const uart_instance_t g_uart0;

/** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
extern sci_uart_instance_ctrl_t g_uart0_ctrl;
extern const uart_cfg_t g_uart0_cfg;
extern const sci_uart_extended_cfg_t g_uart0_cfg_extend;

#ifndef user_uart_callback
void user_uart_callback(uart_callback_args_t *p_args);
#endif
/* I2C Communication Device */
extern const rm_comms_instance_t g_comms_i2c_device0;
extern rm_comms_i2c_instance_ctrl_t g_comms_i2c_device0_ctrl;
extern const rm_comms_cfg_t g_comms_i2c_device0_cfg;
#ifndef rm_zmod4xxx_comms_i2c_callback
void rm_zmod4xxx_comms_i2c_callback(rm_comms_callback_args_t *p_args);
#endif
/* ZMOD4410 IAQ 1st Gen. */
extern rm_zmod4xxx_lib_extended_cfg_t g_zmod4xxx_sensor0_extended_cfg;
/* ZMOD4XXX Sensor */
extern const rm_zmod4xxx_instance_t g_zmod4xxx_sensor0;
extern rm_zmod4xxx_instance_ctrl_t g_zmod4xxx_sensor0_ctrl;
extern const rm_zmod4xxx_cfg_t g_zmod4xxx_sensor0_cfg;
#ifndef zmod4xxx_comms_i2c_callback
void zmod4xxx_comms_i2c_callback(rm_zmod4xxx_callback_args_t *p_args);
#endif
#ifndef rm_zmod4xxx_irq_callback
void rm_zmod4xxx_irq_callback(external_irq_callback_args_t *p_args);
#endif
#ifndef zmod4xxx_irq_callback
void zmod4xxx_irq_callback(rm_zmod4xxx_callback_args_t *p_args);
#endif
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
