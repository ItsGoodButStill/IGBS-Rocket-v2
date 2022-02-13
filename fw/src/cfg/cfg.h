/*
 *  Created: Jun 25, 2021
 *  Author:  Jakub Parez
 *  Company: ATEsystem s.r.o.
 *  Project: Varitest
 */

#ifndef INC_CFG_H_
#define INC_CFG_H_

#include "stm32h7xx.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_utils.h"



// GENERAL FLAGS ---------------------------------------------------
#define ATESYSTEM
#define ATE_CORTEX_M7
#define ATE_DEBUG
//#define ATE_WATERMARK
#define ATE_SYSVIEW

// GENERAL SETTINGS ------------------------------------------------
#define ATE_DEV_AUTHOR      "ATEsystem s.r.o."
#define ATE_DEV_NAME        "Varitest"
#define ATE_DEV_SERIAL      "000001"
#define ATE_DEV_VERSION     "1.2.0"
#define ATE_DEFAULT_MODE    PC_VCP_ETH         // PC_VCP_ETH - MANUAL - TRIGGER

// CONST -----------------------------------------------------------
#define ATE_TRUE            1
#define ATE_FALSE           0
#define ATE_UWTICK_MAX      4294967295 // (2^32) - 1
#define ATE_LN2POW14        9.70406    // ln(2^14)
#define ATE_LN2POW10        6.93147    // ln(2^10)

// FLASH -----------------------------------------------------------
#define ATE_FLASHADDR_ISBLANK       0x80E0000
#define ATE_FLASHADDR_UP_GAIN       0x80E0020
#define ATE_FLASHADDR_UP_OFFSET     0x80E0040
#define ATE_FLASHADDR_I_GAIN        0x80E0060
#define ATE_FLASHADDR_I_OFFSET      0x80E0080
#define ATE_FLASHADDR_UM_GAIN       0x80E00A0
#define ATE_FLASHADDR_UM_OFFSET     0x80E00C0

// IRQ priorities --------------------------------------------------
#define ATE_IT_PRI_ADCS_DRDY    3
#define ATE_IT_PRI_DMA_I        4
#define ATE_IT_PRI_DMA_UM       4
#define ATE_IT_PRI_DMA_UP       4
//#define ATE_IT_PRI_TBASE      15
#define ATE_IT_PRI_UART_MEGA    6
#define ATE_IT_PRI_UART_PC      7
#define ATE_IT_PRI_ETH          5
#define ATE_IT_PRI_EXT_TRIG     7
#define ATE_IT_PRI_SYST         15

// IRQ map ---------------------------------------------------------
#define ATE_IRQN_UART_PC        USART1_IRQn
#define ATE_IRQN_UART_MEGA      USART6_IRQn
//#define ATE_IRQN_TBASE        TIM6_DAC_IRQn
#define ATE_IRQN_ADCS_DRDY      EXTI0_IRQn
#define ATE_IRQN_DMA_I          DMA1_Stream0_IRQn
#define ATE_IRQN_DMA_UM         DMA1_Stream1_IRQn
#define ATE_IRQN_DMA_UP         DMA1_Stream2_IRQn
#define ATE_IRQN_EXT_TRIG       EXTI9_5_IRQn

// IRQ handlers ----------------------------------------------------
#define ATE_UART_PC_RX_IRQHandler       USART1_IRQHandler
#define ATE_UART_MEGA_RX_IRQHandler     USART6_IRQHandler
//#define ATE_TBASE_IRQHandler          TIM6_DAC_IRQHandler
#define ATE_ADCS_DRDY_IRQHandler        EXTI0_IRQHandler
//#define ATE_DMA_I_IRQHandler          DMA1_Stream0_IRQHandler
//#define ATE_DMA_UM_IRQHandler         DMA1_Stream1_IRQHandler
//#define ATE_DMA_UP_IRQHandler         DMA1_Stream2_IRQHandler
#define ATE_DMA_UART_PC_IRQHandler      DMA1_Stream0_IRQHandler
#define ATE_DMA_UART_MEGA_IRQHandler    DMA2_Stream0_IRQHandler
#define ATE_EXT_TRIG_IRQHandler         EXTI9_5_IRQHandler

// STACK -----------------------------------------------------------
#define ATE_STACK_MIN           256
#define ATE_STACK_T1            1024
#define ATE_STACK_T2            1024
#define ATE_STACK_T3            1024
#define ATE_STACK_T4            1024

// TASK PRIORITIES -------------------------------------------------
#define ATE_PRI_T1              3
#define ATE_PRI_T2              1
#define ATE_PRI_T3              2
#define ATE_PRI_T4              4

// CLOCK  ----------------------------------------------------------
#define ATE_FREQ_LSI           40000      // LSI clock - wdg
#define ATE_FREQ_HCLK          400000000  // HCLK clock - main
#define ATE_FREQ_SPI           64000000   // APB1 clock
#define ATE_FREQ_TIM           200000000  // APB2 clock - TIM6
#define ATE_SYSTICK_FREQ       1000       // Systick clock

// SCPI  -------------------------------------------------------------
#define ATE_SCPI_IDN1           ATE_DEV_AUTHOR
#define ATE_SCPI_IDN2           ATE_DEV_NAME
#define ATE_SCPI_IDN3           ATE_DEV_VERSION
#define ATE_SCPI_IDN4           ATE_DEV_SERIAL
#define ATE_SCPI_OK             "OK"

// ETHERNET  ---------------------------------------------------------
#define ATE_IP_IPV4_1           192
#define ATE_IP_IPV4_2           168
#define ATE_IP_IPV4_3           0
#define ATE_IP_IPV4_4           1

#define ATE_MASK_IPV4_1         255
#define ATE_MASK_IPV4_2         255
#define ATE_MASK_IPV4_3         255
#define ATE_MASK_IPV4_4         0

#define ATE_GATE_IPV4_1         0
#define ATE_GATE_IPV4_2         0
#define ATE_GATE_IPV4_3         0
#define ATE_GATE_IPV4_4         0

// CALIB (13.11.2021)  -----------------------------------------------
//#define ATE_CALIB_ENABLED
//#define ATE_CALIB_OVERWRITE
#define ATE_I_GAIN              1 // 1.00572
#define ATE_I_OFFSET            0 // 0.00001
#define ATE_UP_GAIN             0.98790
#define ATE_UP_OFFSET           0.55758
#define ATE_UM_GAIN             1.07484
#define ATE_UM_OFFSET          -4.02729

// COMM  -------------------------------------------------------------
#define ATE_UART_PC                     USART1
#define ATE_UART_PC_DMA                 DMA1
#define ATE_UART_PC_DMA_CH              LL_DMA_STREAM_0
#define ATE_UART_MEGA                   USART6
#define ATE_UART_MEGA_DMA               DMA2
#define ATE_UART_MEGA_DMA_CH            LL_DMA_STREAM_0
#define ATE_UART_CLOCK                  100000000
#define ATE_UART_PRESCALER              LL_USART_PRESCALER_DIV1
#define ATE_UART_OVERSAMPLING           LL_USART_OVERSAMPLING_16
#define ATE_UART_XMEGA_CLEAR_FLAG(x)    LL_USART_ClearFlag_RTO(x);  // RTO flags needs clearing
//#define ATE_UART_XMEGA_POLLINIT                                   // if defined poll for init

//#define ATE_BINPROTO_ASCII
#define ATE_BINPROTO_ASCIIHEX_BIGENDIAN   // if defined, bin protocol headers (addr, cmd, chksm ..) are hex,
                                          // otherwise they are ascii

// SPI DAC -----------------------------------------------------------
#define ATE_DAC_I_SPI           SPI5
#define ATE_DAC_I_SPI_PORT      GPIOF
#define ATE_DAC_I_SPI_CS        GPIO_PIN_10
#define ATE_DAC_I_SPI_LATCH     GPIO_PIN_6

#define ATE_DAC_U_SPI           SPI4
#define ATE_DAC_U_SPI_PORT      GPIOE
#define ATE_DAC_U_SPI_CS        GPIO_PIN_4
#define ATE_DAC_U_SPI_LATCH     GPIO_PIN_3

#define ATE_DAC_VREF            4.0965

// SPI ADC ---------------------------------------------------------
#define ATE_ADC_I_SPI           SPI1
#define ATE_ADC_I_SPI_PORT      GPIOA
#define ATE_ADC_I_SPI_CS        GPIO_PIN_4

#define ATE_ADC_UM_SPI          SPI2
#define ATE_ADC_UM_SPI_PORT     GPIOC
#define ATE_ADC_UM_SPI_CS       GPIO_PIN_0

#define ATE_ADC_UP_SPI          SPI3
#define ATE_ADC_UP_SPI_PORT     GPIOC
#define ATE_ADC_UP_SPI_CS       GPIO_PIN_12

#define ATE_ADC_DRDY_PORT       GPIOE
#define ATE_ADC_DRDY_I_PIN      GPIO_PIN_13
#define ATE_ADC_DRDY_UP_PIN     GPIO_PIN_14
#define ATE_ADC_DRDY_UM_PIN     GPIO_PIN_15

//#define ATE_ADC_DMA
#define ATE_ADC_DMA_BUFF_SZ     100
#define ATE_ADC_VREF            2.5
#define ATE_ADC_BUFFER          1
#define ATE_ADC_I_PGA           ADS125X_PGA64
#define ATE_ADC_I_PGA_VAL       64
#define ATE_ADC_I_RESISTOR      4
#define ATE_ADC_UM_PGA          ADS125X_PGA1
#define ATE_ADC_UP_PGA          ADS125X_PGA1

#define ATE_ADC_UP_GAIN         45.0045
#define ATE_ADC_UM_GAIN         49.776

// GPIO G ---------------------------------------------------------
#define ATE_GPIO_G                        GPIOG
#define ATE_GPIO_G_OUT0                   6
#define ATE_GPIO_G_IN0                    7
#define ATE_GPIO_G_CPU_REL_STATUS         8
#define ATE_GPIO_G_RESERVE_MISO           9
#define ATE_GPIO_G_CPU_LOCK_OUTPUT        10
#define ATE_GPIO_G_BIN1                   11
#define ATE_GPIO_G_INDEX                  12
#define ATE_GPIO_G_EOM                    13

// GPIO D ---------------------------------------------------------
#define ATE_GPIO_D                        GPIOD
#define ATE_GPIO_D_ADCS_DRDY              0
#define ATE_GPIO_D_INT_SLAVE_NEW_DATA     1
#define ATE_GPIO_D_SLAVE_TRIGGER          2
#define ATE_GPIO_D_HEART_BEAT             3
#define ATE_GPIO_D_INT_MASTER_NEW_DATA    4
#define ATE_GPIO_D_EXT_TRIG               5
#define ATE_GPIO_D_KEY_LOCK               6
#define ATE_GPIO_D_RESET_ETH              13
#define ATE_GPIO_D_RESET_LCD              14
#define ATE_GPIO_D_RESET_REG              15

// EXTI -----------------------------------------------------------
#define ATE_EXTI_ADCS_DRDY                LL_EXTI_LINE_0
#define ATE_EXTI_EXT_TRIG                 LL_EXTI_LINE_5

// TIME BASE ------------------------------------------------------
//#define ATE_TBASE_FREQ          10000
//#define ATE_TBASE_TIM_MAX       65535
//#define ATE_TBASE_TIM_FREQ      200000000
//#define ATE_TBASE_TIM           TIM6
//#define ATE_TBASE_TIM_CH        LL_TIM_CHANNEL_CH1

// REG ------------------------------------------------------------
#define ATE_PID_AUTOSTART     ATE_TRUE  //ATE_FALSE // ATE_TRUE
#define ATE_REG_AVG_SIZE      100 // for calib use 1000, for testing use 100

#define ATE_PID_U_MAX_V       200
#define ATE_PID_U_MIN_V       0
#define ATE_PID_I_MAX_MA      20
#define ATE_PID_I_MIN_MA      0

#define ATE_PID_U_KP          0.0050
#define ATE_PID_U_KI          0.0001
#define ATE_PID_U_KD          0.0001
#define ATE_PID_U_INT_LIM     3 //4.096
#define ATE_PID_U_ACT_MAX     3 //4.096
#define ATE_PID_U_ACT_MIN     0

#define ATE_PID_I_KP          300
#define ATE_PID_I_KI          1.0
#define ATE_PID_I_KD          10.0   // pokud budou problemy tak snizit toto cislo
#define ATE_PID_I_INT_LIM     4.096
#define ATE_PID_I_ACT_MAX     4.096
#define ATE_PID_I_ACT_MIN     0

// TEST ------------------------------------------------------------
#define ATE_TEST_TIM_MAX            250
#define ATE_TEST_TIM_MIN            100

#define ATE_TEST_BD_SET_DEFAULT_V   200
#define ATE_TEST_BD_UB1_DEFAULT_V   40
#define ATE_TEST_BD_UB2_DEFAULT_V   60
#define ATE_TEST_BD_TIM_DEFAULT_MS  100

#define ATE_TEST_LK_SET_DEFAULT_MA  1
#define ATE_TEST_LK_IL1_DEFAULT_MA  1
#define ATE_TEST_LK_TIM_DEFAULT_MS  100

// HELPERS ---------------------------------------------------------
#define ATE_BIT_TOGGLE(x,y)    ((x)->ODR ^= (1 << y))
#define ATE_BIT_SET(x,y)       ((x)->ODR |= (1 << y))
#define ATE_BIT_RESET(x,y)     ((x)->ODR &= ~(1 << y))
#define ATE_BIT_READ(x,y)      ((x)->IDR & (1 << y))
#define ATE_MILIS(x)           (x / portTICK_PERIOD_MS) // FreeRTOS miliseconds

// GPIO  -----------------------------------------------------------
#define ATE_HEARTBEAT_TOGGLE   ATE_BIT_TOGGLE(ATE_GPIO_D, ATE_GPIO_D_HEART_BEAT)
//#define ATE_GPIO_TRG_SET     ATE_BIT_SET(ATE_GPIB, ATE_GPIB_TRG)                      // GPIB - test start
//#define ATE_GPIO_TRG_RESET   ATE_BIT_RESET(ATE_GPIB, ATE_GPIB_TRG)
#define ATE_GPIO_EOM_SET       ATE_BIT_SET(ATE_GPIO_G, ATE_GPIO_G_EOM)                  // GPIB - test end
#define ATE_GPIO_EOM_RESET     ATE_BIT_RESET(ATE_GPIO_G, ATE_GPIO_G_EOM)
#define ATE_GPIO_BIN1_SET      ATE_BIT_SET(ATE_GPIO_G, ATE_GPIO_G_BIN1)                 // GPIB - PASS
#define ATE_GPIO_BIN1_RESET    ATE_BIT_RESET(ATE_GPIO_G, ATE_GPIO_G_BIN1)
#define ATE_GPIO_NG_SET        ATE_BIT_SET(ATE_GPIO_G, ATE_GPIO_G_INDEX)                // GPIB - FAIL
#define ATE_GPIO_NG_RESET      ATE_BIT_RESET(ATE_GPIO_G, ATE_GPIO_G_INDEX)
#define ATE_LOCK_OUTPUT        ATE_BIT_SET(ATE_GPIO_G, ATE_GPIO_G_CPU_LOCK_OUTPUT)      // short output to ground
#define ATE_UNLOCK_OUTPUT      ATE_BIT_RESET(ATE_GPIO_G, ATE_GPIO_G_CPU_LOCK_OUTPUT)    // remove short from output
#define ATE_LOCK_READ          ATE_BIT_READ(ATE_GPIO_G, ATE_GPIO_G_CPU_LOCK_OUTPUT)

#define ATE_RESET_ETH          ATE_BIT_SET(ATE_GPIO_D, ATE_GPIO_D_RESET_ETH)
#define ATE_RESET_LCD          ATE_BIT_SET(ATE_GPIO_D, ATE_GPIO_D_RESET_LCD)
#define ATE_RESET_REG          ATE_BIT_SET(ATE_GPIO_D, ATE_GPIO_D_RESET_REG)
#define ATE_UNRESET_ETH        ATE_BIT_RESET(ATE_GPIO_D, ATE_GPIO_D_RESET_ETH)
#define ATE_UNRESET_LCD        ATE_BIT_RESET(ATE_GPIO_D, ATE_GPIO_D_RESET_LCD)
#define ATE_UNRESET_REG        ATE_BIT_RESET(ATE_GPIO_D, ATE_GPIO_D_RESET_REG)

#define ATE_TEST_START         ATE_BIT_SET(ATE_GPIO_D, ATE_GPIO_D_RESET_ETH)
#define ATE_TEST_STOP          ATE_BIT_RESET(ATE_GPIO_D, ATE_GPIO_D_RESET_ETH)

#define ATE_CASE_OPEN          ATE_BIT_READ(ATE_GPIO_G, ATE_GPIO_G_IN0)


#endif /* INC_CFG_H_ */
