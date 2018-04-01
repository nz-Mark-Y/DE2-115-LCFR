/*
 * system.h - SOPC Builder system and BSP software package information
 *
 * Machine generated for CPU 'nios2' in SOPC Builder design 'nios2'
 * SOPC Builder design path: C:/Users/mark_/Documents/COMPSYS_723/Assignment/DE2-115-LCFR/nios2.sopcinfo
 *
 * Generated: Mon Apr 02 09:49:46 NZST 2018
 */

/*
 * DO NOT MODIFY THIS FILE
 *
 * Changing this file will have subtle consequences
 * which will almost certainly lead to a nonfunctioning
 * system. If you do modify this file, be aware that your
 * changes will be overwritten and lost when this file
 * is generated again.
 *
 * DO NOT MODIFY THIS FILE
 */

/*
 * License Agreement
 *
 * Copyright (c) 2008
 * Altera Corporation, San Jose, California, USA.
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * This agreement shall be governed in all respects by the laws of the State
 * of California and by the laws of the United States of America.
 */

#ifndef __SYSTEM_H_
#define __SYSTEM_H_

/* Include definitions from linker script generator */
#include "linker.h"


/*
 * CPU configuration
 *
 */

#define ALT_CPU_ARCHITECTURE "altera_nios2_qsys"
#define ALT_CPU_BIG_ENDIAN 0
#define ALT_CPU_BREAK_ADDR 0x42820
#define ALT_CPU_CPU_FREQ 100000000u
#define ALT_CPU_CPU_ID_SIZE 1
#define ALT_CPU_CPU_ID_VALUE 0x00000000
#define ALT_CPU_CPU_IMPLEMENTATION "fast"
#define ALT_CPU_DATA_ADDR_WIDTH 0x1c
#define ALT_CPU_DCACHE_LINE_SIZE 32
#define ALT_CPU_DCACHE_LINE_SIZE_LOG2 5
#define ALT_CPU_DCACHE_SIZE 2048
#define ALT_CPU_EXCEPTION_ADDR 0x20
#define ALT_CPU_FLUSHDA_SUPPORTED
#define ALT_CPU_FREQ 100000000
#define ALT_CPU_HARDWARE_DIVIDE_PRESENT 1
#define ALT_CPU_HARDWARE_MULTIPLY_PRESENT 1
#define ALT_CPU_HARDWARE_MULX_PRESENT 0
#define ALT_CPU_HAS_DEBUG_CORE 1
#define ALT_CPU_HAS_DEBUG_STUB
#define ALT_CPU_HAS_JMPI_INSTRUCTION
#define ALT_CPU_ICACHE_LINE_SIZE 32
#define ALT_CPU_ICACHE_LINE_SIZE_LOG2 5
#define ALT_CPU_ICACHE_SIZE 4096
#define ALT_CPU_INITDA_SUPPORTED
#define ALT_CPU_INST_ADDR_WIDTH 0x19
#define ALT_CPU_NAME "nios2"
#define ALT_CPU_NUM_OF_SHADOW_REG_SETS 0
#define ALT_CPU_RESET_ADDR 0x1000000


/*
 * CPU configuration (with legacy prefix - don't use these anymore)
 *
 */

#define NIOS2_BIG_ENDIAN 0
#define NIOS2_BREAK_ADDR 0x42820
#define NIOS2_CPU_FREQ 100000000u
#define NIOS2_CPU_ID_SIZE 1
#define NIOS2_CPU_ID_VALUE 0x00000000
#define NIOS2_CPU_IMPLEMENTATION "fast"
#define NIOS2_DATA_ADDR_WIDTH 0x1c
#define NIOS2_DCACHE_LINE_SIZE 32
#define NIOS2_DCACHE_LINE_SIZE_LOG2 5
#define NIOS2_DCACHE_SIZE 2048
#define NIOS2_EXCEPTION_ADDR 0x20
#define NIOS2_FLUSHDA_SUPPORTED
#define NIOS2_HARDWARE_DIVIDE_PRESENT 1
#define NIOS2_HARDWARE_MULTIPLY_PRESENT 1
#define NIOS2_HARDWARE_MULX_PRESENT 0
#define NIOS2_HAS_DEBUG_CORE 1
#define NIOS2_HAS_DEBUG_STUB
#define NIOS2_HAS_JMPI_INSTRUCTION
#define NIOS2_ICACHE_LINE_SIZE 32
#define NIOS2_ICACHE_LINE_SIZE_LOG2 5
#define NIOS2_ICACHE_SIZE 4096
#define NIOS2_INITDA_SUPPORTED
#define NIOS2_INST_ADDR_WIDTH 0x19
#define NIOS2_NUM_OF_SHADOW_REG_SETS 0
#define NIOS2_RESET_ADDR 0x1000000


/*
 * Define for each module class mastered by the CPU
 *
 */

#define __ALTERA_AVALON_JTAG_UART
#define __ALTERA_AVALON_LCD_16207
#define __ALTERA_AVALON_NEW_SDRAM_CONTROLLER
#define __ALTERA_AVALON_ONCHIP_MEMORY2
#define __ALTERA_AVALON_PIO
#define __ALTERA_AVALON_SYSID_QSYS
#define __ALTERA_AVALON_TIMER
#define __ALTERA_AVALON_UART
#define __ALTERA_GENERIC_TRISTATE_CONTROLLER
#define __ALTERA_NIOS2_QSYS
#define __ALTERA_UP_AVALON_PS2
#define __ALTERA_UP_AVALON_SRAM
#define __ALTERA_UP_AVALON_VIDEO_CHARACTER_BUFFER_WITH_DMA
#define __ALTERA_UP_AVALON_VIDEO_PIXEL_BUFFER_DMA
#define __FREQUENCY_ANALYSER
#define __SEVEN_SEG


/*
 * System configuration
 *
 */

#define ALT_DEVICE_FAMILY "Cyclone IV E"
#define ALT_IRQ_BASE NULL
#define ALT_LEGACY_INTERRUPT_API_PRESENT
#define ALT_LOG_PORT "/dev/null"
#define ALT_LOG_PORT_BASE 0x0
#define ALT_LOG_PORT_DEV null
#define ALT_LOG_PORT_TYPE ""
#define ALT_NUM_EXTERNAL_INTERRUPT_CONTROLLERS 0
#define ALT_NUM_INTERNAL_INTERRUPT_CONTROLLERS 1
#define ALT_NUM_INTERRUPT_CONTROLLERS 1
#define ALT_STDERR "/dev/jtag_uart"
#define ALT_STDERR_BASE 0x430f8
#define ALT_STDERR_DEV jtag_uart
#define ALT_STDERR_IS_JTAG_UART
#define ALT_STDERR_PRESENT
#define ALT_STDERR_TYPE "altera_avalon_jtag_uart"
#define ALT_STDIN "/dev/jtag_uart"
#define ALT_STDIN_BASE 0x430f8
#define ALT_STDIN_DEV jtag_uart
#define ALT_STDIN_IS_JTAG_UART
#define ALT_STDIN_PRESENT
#define ALT_STDIN_TYPE "altera_avalon_jtag_uart"
#define ALT_STDOUT "/dev/jtag_uart"
#define ALT_STDOUT_BASE 0x430f8
#define ALT_STDOUT_DEV jtag_uart
#define ALT_STDOUT_IS_JTAG_UART
#define ALT_STDOUT_PRESENT
#define ALT_STDOUT_TYPE "altera_avalon_jtag_uart"
#define ALT_SYSTEM_NAME "nios2"


/*
 * character_lcd configuration
 *
 */

#define ALT_MODULE_CLASS_character_lcd altera_avalon_lcd_16207
#define CHARACTER_LCD_BASE 0x430a0
#define CHARACTER_LCD_IRQ -1
#define CHARACTER_LCD_IRQ_INTERRUPT_CONTROLLER_ID -1
#define CHARACTER_LCD_NAME "/dev/character_lcd"
#define CHARACTER_LCD_SPAN 16
#define CHARACTER_LCD_TYPE "altera_avalon_lcd_16207"


/*
 * flash_controller configuration
 *
 */

#define ALT_MODULE_CLASS_flash_controller altera_generic_tristate_controller
#define FLASH_CONTROLLER_BASE 0x1000000
#define FLASH_CONTROLLER_HOLD_VALUE 60
#define FLASH_CONTROLLER_IRQ -1
#define FLASH_CONTROLLER_IRQ_INTERRUPT_CONTROLLER_ID -1
#define FLASH_CONTROLLER_NAME "/dev/flash_controller"
#define FLASH_CONTROLLER_SETUP_VALUE 60
#define FLASH_CONTROLLER_SIZE 8388608u
#define FLASH_CONTROLLER_SPAN 8388608
#define FLASH_CONTROLLER_TIMING_UNITS "ns"
#define FLASH_CONTROLLER_TYPE "altera_generic_tristate_controller"
#define FLASH_CONTROLLER_WAIT_VALUE 160


/*
 * frequency_analyser configuration
 *
 */

#define ALT_MODULE_CLASS_frequency_analyser frequency_analyser
#define FREQUENCY_ANALYSER_BASE 0x43100
#define FREQUENCY_ANALYSER_IRQ 7
#define FREQUENCY_ANALYSER_IRQ_INTERRUPT_CONTROLLER_ID 0
#define FREQUENCY_ANALYSER_NAME "/dev/frequency_analyser"
#define FREQUENCY_ANALYSER_SPAN 4
#define FREQUENCY_ANALYSER_TYPE "frequency_analyser"


/*
 * green_leds configuration
 *
 */

#define ALT_MODULE_CLASS_green_leds altera_avalon_pio
#define GREEN_LEDS_BASE 0x43080
#define GREEN_LEDS_BIT_CLEARING_EDGE_REGISTER 0
#define GREEN_LEDS_BIT_MODIFYING_OUTPUT_REGISTER 1
#define GREEN_LEDS_CAPTURE 0
#define GREEN_LEDS_DATA_WIDTH 9
#define GREEN_LEDS_DO_TEST_BENCH_WIRING 0
#define GREEN_LEDS_DRIVEN_SIM_VALUE 0
#define GREEN_LEDS_EDGE_TYPE "NONE"
#define GREEN_LEDS_FREQ 100000000
#define GREEN_LEDS_HAS_IN 0
#define GREEN_LEDS_HAS_OUT 1
#define GREEN_LEDS_HAS_TRI 0
#define GREEN_LEDS_IRQ -1
#define GREEN_LEDS_IRQ_INTERRUPT_CONTROLLER_ID -1
#define GREEN_LEDS_IRQ_TYPE "NONE"
#define GREEN_LEDS_NAME "/dev/green_leds"
#define GREEN_LEDS_RESET_VALUE 0
#define GREEN_LEDS_SPAN 32
#define GREEN_LEDS_TYPE "altera_avalon_pio"


/*
 * hal configuration
 *
 */

#define ALT_MAX_FD 32
#define ALT_SYS_CLK TIMER1MS
#define ALT_TIMESTAMP_CLK none


/*
 * jtag_uart configuration
 *
 */

#define ALT_MODULE_CLASS_jtag_uart altera_avalon_jtag_uart
#define JTAG_UART_BASE 0x430f8
#define JTAG_UART_IRQ 5
#define JTAG_UART_IRQ_INTERRUPT_CONTROLLER_ID 0
#define JTAG_UART_NAME "/dev/jtag_uart"
#define JTAG_UART_READ_DEPTH 512
#define JTAG_UART_READ_THRESHOLD 8
#define JTAG_UART_SPAN 8
#define JTAG_UART_TYPE "altera_avalon_jtag_uart"
#define JTAG_UART_WRITE_DEPTH 512
#define JTAG_UART_WRITE_THRESHOLD 8


/*
 * onchip_memory configuration
 *
 */

#define ALT_MODULE_CLASS_onchip_memory altera_avalon_onchip_memory2
#define ONCHIP_MEMORY_ALLOW_IN_SYSTEM_MEMORY_CONTENT_EDITOR 0
#define ONCHIP_MEMORY_ALLOW_MRAM_SIM_CONTENTS_ONLY_FILE 0
#define ONCHIP_MEMORY_BASE 0x0
#define ONCHIP_MEMORY_CONTENTS_INFO ""
#define ONCHIP_MEMORY_DUAL_PORT 0
#define ONCHIP_MEMORY_GUI_RAM_BLOCK_TYPE "AUTO"
#define ONCHIP_MEMORY_INIT_CONTENTS_FILE "nios2_onchip_memory"
#define ONCHIP_MEMORY_INIT_MEM_CONTENT 1
#define ONCHIP_MEMORY_INSTANCE_ID "NONE"
#define ONCHIP_MEMORY_IRQ -1
#define ONCHIP_MEMORY_IRQ_INTERRUPT_CONTROLLER_ID -1
#define ONCHIP_MEMORY_NAME "/dev/onchip_memory"
#define ONCHIP_MEMORY_NON_DEFAULT_INIT_FILE_ENABLED 0
#define ONCHIP_MEMORY_RAM_BLOCK_TYPE "AUTO"
#define ONCHIP_MEMORY_READ_DURING_WRITE_MODE "DONT_CARE"
#define ONCHIP_MEMORY_SINGLE_CLOCK_OP 0
#define ONCHIP_MEMORY_SIZE_MULTIPLE 1
#define ONCHIP_MEMORY_SIZE_VALUE 204800
#define ONCHIP_MEMORY_SPAN 204800
#define ONCHIP_MEMORY_TYPE "altera_avalon_onchip_memory2"
#define ONCHIP_MEMORY_WRITABLE 1


/*
 * ps2 configuration
 *
 */

#define ALT_MODULE_CLASS_ps2 altera_up_avalon_ps2
#define PS2_BASE 0x430e0
#define PS2_IRQ 2
#define PS2_IRQ_INTERRUPT_CONTROLLER_ID 0
#define PS2_NAME "/dev/ps2"
#define PS2_SPAN 8
#define PS2_TYPE "altera_up_avalon_ps2"


/*
 * push_button configuration
 *
 */

#define ALT_MODULE_CLASS_push_button altera_avalon_pio
#define PUSH_BUTTON_BASE 0x430c0
#define PUSH_BUTTON_BIT_CLEARING_EDGE_REGISTER 1
#define PUSH_BUTTON_BIT_MODIFYING_OUTPUT_REGISTER 0
#define PUSH_BUTTON_CAPTURE 1
#define PUSH_BUTTON_DATA_WIDTH 3
#define PUSH_BUTTON_DO_TEST_BENCH_WIRING 0
#define PUSH_BUTTON_DRIVEN_SIM_VALUE 0
#define PUSH_BUTTON_EDGE_TYPE "FALLING"
#define PUSH_BUTTON_FREQ 100000000
#define PUSH_BUTTON_HAS_IN 1
#define PUSH_BUTTON_HAS_OUT 0
#define PUSH_BUTTON_HAS_TRI 0
#define PUSH_BUTTON_IRQ 1
#define PUSH_BUTTON_IRQ_INTERRUPT_CONTROLLER_ID 0
#define PUSH_BUTTON_IRQ_TYPE "EDGE"
#define PUSH_BUTTON_NAME "/dev/push_button"
#define PUSH_BUTTON_RESET_VALUE 0
#define PUSH_BUTTON_SPAN 16
#define PUSH_BUTTON_TYPE "altera_avalon_pio"


/*
 * red_leds configuration
 *
 */

#define ALT_MODULE_CLASS_red_leds altera_avalon_pio
#define RED_LEDS_BASE 0x43060
#define RED_LEDS_BIT_CLEARING_EDGE_REGISTER 0
#define RED_LEDS_BIT_MODIFYING_OUTPUT_REGISTER 1
#define RED_LEDS_CAPTURE 0
#define RED_LEDS_DATA_WIDTH 18
#define RED_LEDS_DO_TEST_BENCH_WIRING 0
#define RED_LEDS_DRIVEN_SIM_VALUE 0
#define RED_LEDS_EDGE_TYPE "NONE"
#define RED_LEDS_FREQ 100000000
#define RED_LEDS_HAS_IN 0
#define RED_LEDS_HAS_OUT 1
#define RED_LEDS_HAS_TRI 0
#define RED_LEDS_IRQ -1
#define RED_LEDS_IRQ_INTERRUPT_CONTROLLER_ID -1
#define RED_LEDS_IRQ_TYPE "NONE"
#define RED_LEDS_NAME "/dev/red_leds"
#define RED_LEDS_RESET_VALUE 0
#define RED_LEDS_SPAN 32
#define RED_LEDS_TYPE "altera_avalon_pio"


/*
 * sdram configuration
 *
 */

#define ALT_MODULE_CLASS_sdram altera_avalon_new_sdram_controller
#define SDRAM_BASE 0x8000000
#define SDRAM_CAS_LATENCY 3
#define SDRAM_CONTENTS_INFO
#define SDRAM_INIT_NOP_DELAY 0.0
#define SDRAM_INIT_REFRESH_COMMANDS 2
#define SDRAM_IRQ -1
#define SDRAM_IRQ_INTERRUPT_CONTROLLER_ID -1
#define SDRAM_IS_INITIALIZED 1
#define SDRAM_NAME "/dev/sdram"
#define SDRAM_POWERUP_DELAY 100.0
#define SDRAM_REFRESH_PERIOD 15.625
#define SDRAM_REGISTER_DATA_IN 1
#define SDRAM_SDRAM_ADDR_WIDTH 0x19
#define SDRAM_SDRAM_BANK_WIDTH 2
#define SDRAM_SDRAM_COL_WIDTH 10
#define SDRAM_SDRAM_DATA_WIDTH 32
#define SDRAM_SDRAM_NUM_BANKS 4
#define SDRAM_SDRAM_NUM_CHIPSELECTS 1
#define SDRAM_SDRAM_ROW_WIDTH 13
#define SDRAM_SHARED_DATA 0
#define SDRAM_SIM_MODEL_BASE 0
#define SDRAM_SPAN 134217728
#define SDRAM_STARVATION_INDICATOR 0
#define SDRAM_TRISTATE_BRIDGE_SLAVE ""
#define SDRAM_TYPE "altera_avalon_new_sdram_controller"
#define SDRAM_T_AC 5.5
#define SDRAM_T_MRD 3
#define SDRAM_T_RCD 20.0
#define SDRAM_T_RFC 70.0
#define SDRAM_T_RP 20.0
#define SDRAM_T_WR 14.0


/*
 * seven_seg configuration
 *
 */

#define ALT_MODULE_CLASS_seven_seg seven_seg
#define SEVEN_SEG_BASE 0x43104
#define SEVEN_SEG_IRQ -1
#define SEVEN_SEG_IRQ_INTERRUPT_CONTROLLER_ID -1
#define SEVEN_SEG_NAME "/dev/seven_seg"
#define SEVEN_SEG_SPAN 4
#define SEVEN_SEG_TYPE "seven_seg"


/*
 * slide_switch configuration
 *
 */

#define ALT_MODULE_CLASS_slide_switch altera_avalon_pio
#define SLIDE_SWITCH_BASE 0x430b0
#define SLIDE_SWITCH_BIT_CLEARING_EDGE_REGISTER 0
#define SLIDE_SWITCH_BIT_MODIFYING_OUTPUT_REGISTER 0
#define SLIDE_SWITCH_CAPTURE 0
#define SLIDE_SWITCH_DATA_WIDTH 18
#define SLIDE_SWITCH_DO_TEST_BENCH_WIRING 0
#define SLIDE_SWITCH_DRIVEN_SIM_VALUE 0
#define SLIDE_SWITCH_EDGE_TYPE "NONE"
#define SLIDE_SWITCH_FREQ 100000000
#define SLIDE_SWITCH_HAS_IN 1
#define SLIDE_SWITCH_HAS_OUT 0
#define SLIDE_SWITCH_HAS_TRI 0
#define SLIDE_SWITCH_IRQ -1
#define SLIDE_SWITCH_IRQ_INTERRUPT_CONTROLLER_ID -1
#define SLIDE_SWITCH_IRQ_TYPE "NONE"
#define SLIDE_SWITCH_NAME "/dev/slide_switch"
#define SLIDE_SWITCH_RESET_VALUE 0
#define SLIDE_SWITCH_SPAN 16
#define SLIDE_SWITCH_TYPE "altera_avalon_pio"


/*
 * sram configuration
 *
 */

#define ALT_MODULE_CLASS_sram altera_up_avalon_sram
#define SRAM_BASE 0x200000
#define SRAM_IRQ -1
#define SRAM_IRQ_INTERRUPT_CONTROLLER_ID -1
#define SRAM_NAME "/dev/sram"
#define SRAM_SPAN 2097152
#define SRAM_TYPE "altera_up_avalon_sram"


/*
 * system_id configuration
 *
 */

#define ALT_MODULE_CLASS_system_id altera_avalon_sysid_qsys
#define SYSTEM_ID_BASE 0x430f0
#define SYSTEM_ID_ID 7
#define SYSTEM_ID_IRQ -1
#define SYSTEM_ID_IRQ_INTERRUPT_CONTROLLER_ID -1
#define SYSTEM_ID_NAME "/dev/system_id"
#define SYSTEM_ID_SPAN 8
#define SYSTEM_ID_TIMESTAMP 1393900972
#define SYSTEM_ID_TYPE "altera_avalon_sysid_qsys"


/*
 * timer1ms configuration
 *
 */

#define ALT_MODULE_CLASS_timer1ms altera_avalon_timer
#define TIMER1MS_ALWAYS_RUN 0
#define TIMER1MS_BASE 0x43040
#define TIMER1MS_COUNTER_SIZE 32
#define TIMER1MS_FIXED_PERIOD 0
#define TIMER1MS_FREQ 100000000
#define TIMER1MS_IRQ 0
#define TIMER1MS_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TIMER1MS_LOAD_VALUE 99999
#define TIMER1MS_MULT 0.0010
#define TIMER1MS_NAME "/dev/timer1ms"
#define TIMER1MS_PERIOD 1
#define TIMER1MS_PERIOD_UNITS "ms"
#define TIMER1MS_RESET_OUTPUT 0
#define TIMER1MS_SNAPSHOT 1
#define TIMER1MS_SPAN 32
#define TIMER1MS_TICKS_PER_SEC 1000.0
#define TIMER1MS_TIMEOUT_PULSE_OUTPUT 0
#define TIMER1MS_TYPE "altera_avalon_timer"


/*
 * timer1us configuration
 *
 */

#define ALT_MODULE_CLASS_timer1us altera_avalon_timer
#define TIMER1US_ALWAYS_RUN 0
#define TIMER1US_BASE 0x43020
#define TIMER1US_COUNTER_SIZE 32
#define TIMER1US_FIXED_PERIOD 0
#define TIMER1US_FREQ 100000000
#define TIMER1US_IRQ 6
#define TIMER1US_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TIMER1US_LOAD_VALUE 99
#define TIMER1US_MULT 1.0E-6
#define TIMER1US_NAME "/dev/timer1us"
#define TIMER1US_PERIOD 1
#define TIMER1US_PERIOD_UNITS "us"
#define TIMER1US_RESET_OUTPUT 0
#define TIMER1US_SNAPSHOT 1
#define TIMER1US_SPAN 32
#define TIMER1US_TICKS_PER_SEC 1000000.0
#define TIMER1US_TIMEOUT_PULSE_OUTPUT 0
#define TIMER1US_TYPE "altera_avalon_timer"


/*
 * uart configuration
 *
 */

#define ALT_MODULE_CLASS_uart altera_avalon_uart
#define UART_BASE 0x43000
#define UART_BAUD 115200
#define UART_DATA_BITS 8
#define UART_FIXED_BAUD 1
#define UART_FREQ 100000000
#define UART_IRQ 3
#define UART_IRQ_INTERRUPT_CONTROLLER_ID 0
#define UART_NAME "/dev/uart"
#define UART_PARITY 'N'
#define UART_SIM_CHAR_STREAM ""
#define UART_SIM_TRUE_BAUD 0
#define UART_SPAN 32
#define UART_STOP_BITS 1
#define UART_SYNC_REG_DEPTH 2
#define UART_TYPE "altera_avalon_uart"
#define UART_USE_CTS_RTS 0
#define UART_USE_EOP_REGISTER 0


/*
 * video_character_buffer_with_dma_avalon_char_buffer_slave configuration
 *
 */

#define ALT_MODULE_CLASS_video_character_buffer_with_dma_avalon_char_buffer_slave altera_up_avalon_video_character_buffer_with_dma
#define VIDEO_CHARACTER_BUFFER_WITH_DMA_AVALON_CHAR_BUFFER_SLAVE_BASE 0x40000
#define VIDEO_CHARACTER_BUFFER_WITH_DMA_AVALON_CHAR_BUFFER_SLAVE_IRQ -1
#define VIDEO_CHARACTER_BUFFER_WITH_DMA_AVALON_CHAR_BUFFER_SLAVE_IRQ_INTERRUPT_CONTROLLER_ID -1
#define VIDEO_CHARACTER_BUFFER_WITH_DMA_AVALON_CHAR_BUFFER_SLAVE_NAME "/dev/video_character_buffer_with_dma_avalon_char_buffer_slave"
#define VIDEO_CHARACTER_BUFFER_WITH_DMA_AVALON_CHAR_BUFFER_SLAVE_SPAN 8192
#define VIDEO_CHARACTER_BUFFER_WITH_DMA_AVALON_CHAR_BUFFER_SLAVE_TYPE "altera_up_avalon_video_character_buffer_with_dma"


/*
 * video_character_buffer_with_dma_avalon_char_control_slave configuration
 *
 */

#define ALT_MODULE_CLASS_video_character_buffer_with_dma_avalon_char_control_slave altera_up_avalon_video_character_buffer_with_dma
#define VIDEO_CHARACTER_BUFFER_WITH_DMA_AVALON_CHAR_CONTROL_SLAVE_BASE 0x430e8
#define VIDEO_CHARACTER_BUFFER_WITH_DMA_AVALON_CHAR_CONTROL_SLAVE_IRQ -1
#define VIDEO_CHARACTER_BUFFER_WITH_DMA_AVALON_CHAR_CONTROL_SLAVE_IRQ_INTERRUPT_CONTROLLER_ID -1
#define VIDEO_CHARACTER_BUFFER_WITH_DMA_AVALON_CHAR_CONTROL_SLAVE_NAME "/dev/video_character_buffer_with_dma_avalon_char_control_slave"
#define VIDEO_CHARACTER_BUFFER_WITH_DMA_AVALON_CHAR_CONTROL_SLAVE_SPAN 8
#define VIDEO_CHARACTER_BUFFER_WITH_DMA_AVALON_CHAR_CONTROL_SLAVE_TYPE "altera_up_avalon_video_character_buffer_with_dma"


/*
 * video_pixel_buffer_dma configuration
 *
 */

#define ALT_MODULE_CLASS_video_pixel_buffer_dma altera_up_avalon_video_pixel_buffer_dma
#define VIDEO_PIXEL_BUFFER_DMA_BASE 0x430d0
#define VIDEO_PIXEL_BUFFER_DMA_IRQ -1
#define VIDEO_PIXEL_BUFFER_DMA_IRQ_INTERRUPT_CONTROLLER_ID -1
#define VIDEO_PIXEL_BUFFER_DMA_NAME "/dev/video_pixel_buffer_dma"
#define VIDEO_PIXEL_BUFFER_DMA_SPAN 16
#define VIDEO_PIXEL_BUFFER_DMA_TYPE "altera_up_avalon_video_pixel_buffer_dma"

#endif /* __SYSTEM_H_ */
