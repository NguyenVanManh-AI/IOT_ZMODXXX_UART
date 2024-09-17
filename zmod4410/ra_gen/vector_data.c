/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = iic_master_rxi_isr, /* IIC0 RXI (Receive data full) */
            [1] = iic_master_txi_isr, /* IIC0 TXI (Transmit data empty) */
            [2] = iic_master_tei_isr, /* IIC0 TEI (Transmit end) */
            [3] = iic_master_eri_isr, /* IIC0 ERI (Transfer error) */
            [4] = r_icu_isr, /* ICU IRQ4 (External pin interrupt 4) */
            [5] = sci_uart_rxi_isr, /* SCI5 RXI (Receive data full) */
            [6] = sci_uart_txi_isr, /* SCI5 TXI (Transmit data empty) */
            [7] = sci_uart_tei_isr, /* SCI5 TEI (Transmit end) */
            [8] = sci_uart_eri_isr, /* SCI5 ERI (Receive error) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_IIC0_RXI,GROUP0), /* IIC0 RXI (Receive data full) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_IIC0_TXI,GROUP1), /* IIC0 TXI (Transmit data empty) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_IIC0_TEI,GROUP2), /* IIC0 TEI (Transmit end) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_IIC0_ERI,GROUP3), /* IIC0 ERI (Transfer error) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ4,GROUP4), /* ICU IRQ4 (External pin interrupt 4) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_SCI5_RXI,GROUP5), /* SCI5 RXI (Receive data full) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_SCI5_TXI,GROUP6), /* SCI5 TXI (Transmit data empty) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_SCI5_TEI,GROUP7), /* SCI5 TEI (Transmit end) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_SCI5_ERI,GROUP0), /* SCI5 ERI (Receive error) */
        };
        #endif
        #endif
