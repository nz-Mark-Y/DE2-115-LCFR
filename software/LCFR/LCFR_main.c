/*===========*/
/* Includes. */
/*===========*/
// Standard
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ISR
#include "system.h"
#include "altera_avalon_pio_regs.h"
#include "alt_types.h"
#include "sys/alt_irq.h"
#include "altera_up_avalon_ps2.h"
#include "io.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"

// RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

/*==============*/
/* Definitions. */
/*==============*/
// Tasks
#define mainREG_DECIDE_PARAMETER    ( ( void * ) 0x12345678 )
#define mainREG_LED_OUT_PARAMETER   ( ( void * ) 0x87654321 )
#define mainREG_VGA_OUT_PARAMETER 	( ( void * ) 0x12348765 )
#define mainREG_TEST_PRIORITY       ( tskIDLE_PRIORITY + 1)
#define SAMPLE_FREQ 				16000

// Keyboard
#define PS2_1 0x69
#define PS2_2 0x72
#define PS2_3 0x7A
#define PS2_4 0x6B
#define PS2_5 0x73
#define PS2_6 0x74
#define PS2_7 0x6C
#define PS2_8 0x75
#define PS2_9 0x7D
#define PS2_0 0x70
#define PS2_dp 0x71
#define PS2_ENTER 0x5A
#define PS2_DP 0x71
#define PS2_KEYRELEASE 0xF0

// Graphs
#define FREQPLT_ORI_X 101		// X axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 5	// Pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199.0		// Y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	// Number of pixels per Hz (y axis scale)

#define ROCPLT_ORI_X 101
#define ROCPLT_GRID_SIZE_X 5
#define ROCPLT_ORI_Y 259.0
#define ROCPLT_ROC_RES 0.5		// Number of pixels per Hz/s (y axis scale)
#define MIN_FREQ 45.0 			// Minimum frequency to draw

/*========================*/
/* Function Declarations. */
/*========================*/
static void prvDecideTask(void *pvParameters);
static void prvLEDOutTask(void *pvParameters);
static void prvVGAOutTask(void *pvParameters);
void translate_ps2(unsigned char byte, double *value);

/*===================*/
/* Global Variables. */
/*===================*/
// Flags
int first_load_shed = 0;
int shed_flag = 0;
int reconnect_load_timeout = 0;
int drop_load_timeout = 0;
int maintenance = 0;
int desired_flag = 0;

// Configurations
double desired_max_roc_freq = 8;
double desired_min_freq = 48.5;

// Data
double signal_freq = 0;
double roc_freq = 0;
int loads[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };
int switches[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };
double input_number = 0.0, input_decimal = 0.0, input_decimal_equiv = 0.0, input_final_number = 0.0;
int input_number_counter = 0, input_decimal_flag = 0, input_duplicate_flag = 0;

// System Status
int system_uptime = 0;
int drop_delay = 0;
double drop_average = 0.0;
int drop_delay_flag = 0;
int min_drop_delay = 0;
int max_drop_delay = 0;
double store_freq[5] = { 0, 0, 0, 0, 0 };
double store_dfreq[5] = { 0, 0, 0, 0, 0 };
char system_uptime_string[10];
char min_freq_string[12];
char max_roc_string[12];
char min_drop_string[8];
char max_drop_string[8];
char average_drop_string[12];
char m1[5], m2[5], m3[5], m4[5], m5[5];
char n1[5], n2[5], n3[5], n4[5], n5[5];

/*==========*/
/* Handles. */
/*==========*/
TimerHandle_t drop_timer;
TimerHandle_t recon_timer;
TimerHandle_t system_up_timer;
TimerHandle_t drop_delay_timer;
static QueueHandle_t Q_freq_data;
SemaphoreHandle_t shared_resource_mutex;

/*=============*/
/* Structures. */
/*=============*/
typedef struct{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
} Line;

/*=======*/
/* ISRs. */
/*=======*/
// Pushbutton
void button_interrupts_function(void* context, alt_u32 id) {
	int* temp = (int*) context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE); // Store which button was pressed

	if (maintenance == 1) { // Toggle Maintenance Mode
		xSemaphoreTakeFromISR(shared_resource_mutex, NULL);
		maintenance = 0; // Disable maintenance mode
		xSemaphoreGiveFromISR(shared_resource_mutex, NULL);
		printf("Maintenance Mode Disabled\n");

		alt_up_ps2_dev *ps2_device = alt_up_ps2_open_dev(PS2_NAME);
		alt_up_ps2_disable_read_interrupt(ps2_device); // Disable keyboard

		drop_delay = 0;
		drop_delay_flag = 0;
	} else {
		xSemaphoreTakeFromISR(shared_resource_mutex, NULL);
		maintenance = 1; // Enable maintenance mode
		xSemaphoreGiveFromISR(shared_resource_mutex, NULL);
		printf("Maintenance Mode Enabled\n");

		alt_up_ps2_dev *ps2_device = alt_up_ps2_open_dev(PS2_NAME);
		alt_up_ps2_clear_fifo(ps2_device); // Clear keyboard buffer
		alt_up_ps2_enable_read_interrupt(ps2_device); // Enable keyboard
	}
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7); // Clear edge capture register
}

// Frequency Analyser
void freq_relay() {
	unsigned int temp = IORD(FREQUENCY_ANALYSER_BASE, 0); // Get the sample count between the two most recent peaks

	// Important: do not swap the order of the two operations otherwise the roc will be 0 all the time
	if (temp > 0) {
		xSemaphoreTakeFromISR(shared_resource_mutex, NULL);
		roc_freq = ((SAMPLE_FREQ / (double) temp) - signal_freq) * (SAMPLE_FREQ / (double) temp); // Calculate and storeROC Frequency
		signal_freq = SAMPLE_FREQ / (double) temp; // Calculate abd store Instantaneous Frequency
		xSemaphoreGiveFromISR(shared_resource_mutex, NULL);
	}

	if ((first_load_shed == 0) && (drop_delay_flag == 0)) {
		if (fabs(roc_freq) > desired_max_roc_freq || desired_min_freq > signal_freq) {
			xSemaphoreTakeFromISR(shared_resource_mutex, NULL);
			drop_delay_flag = 1;
			drop_delay = 0;
			xSemaphoreGiveFromISR(shared_resource_mutex, NULL);
		}
	}

	xQueueSendToBackFromISR( Q_freq_data, &signal_freq, pdFALSE ); // Add data to xQueue

	return;
}

// Keyboard
void ps2_isr(void* ps2_device, alt_u32 id){
	unsigned char byte;
	alt_up_ps2_read_data_byte_timeout(ps2_device, &byte);

	if (byte == PS2_ENTER) { // Enter key pressed
		if (input_duplicate_flag == 1) { // Ignore key releases
			xSemaphoreTakeFromISR(shared_resource_mutex, NULL);
			input_duplicate_flag = 0;
			xSemaphoreGiveFromISR(shared_resource_mutex, NULL);
		} else {
			xSemaphoreTakeFromISR(shared_resource_mutex, NULL);
			if (input_decimal_flag == 1) {
				input_decimal *= 10;
			} else {
				input_number /= 10;
			}
			input_final_number = input_number + input_decimal;
			
			if (desired_flag == 0) {
				desired_min_freq = input_final_number; // Store entered value
				printf("The preferred minimum frequency was set to: %f\n", desired_min_freq);
				desired_flag = 1;
			} else {
				desired_max_roc_freq = input_final_number; // Store entered value
				printf("The preferred maximum rate of change of frequency was set to: %f\n", desired_max_roc_freq);
				desired_flag = 0;
			}

			// Clear numbers
			input_number = 0.0;
			input_decimal = 0.0;
			input_final_number = 0.0;
			input_decimal_equiv = 0.0;

			input_decimal_flag = 0;
			input_number_counter = 0;
			xSemaphoreGiveFromISR(shared_resource_mutex, NULL);
		}
	} else if (byte == PS2_KEYRELEASE) { // Ignore key releases
		xSemaphoreTakeFromISR(shared_resource_mutex, NULL);
		input_duplicate_flag = 1;
		xSemaphoreGiveFromISR(shared_resource_mutex, NULL);
	} else {
		if (input_duplicate_flag == 1) {
			xSemaphoreTakeFromISR(shared_resource_mutex, NULL);
			input_duplicate_flag = 0;
			xSemaphoreGiveFromISR(shared_resource_mutex, NULL);
		} else {
			xSemaphoreTakeFromISR(shared_resource_mutex, NULL);
			// Take care of decimal point
			if (byte == PS2_DP) { // Handle decimal point
				input_decimal_flag = 1;
			}

			if (input_decimal_flag == 0) {
				// Take care of upper part of number
				input_number *= 10;
				
				// Translate and add to upper part of number
				translate_ps2(byte, &input_number);
			} else {
				// Take care of lower part of number
				input_number_counter += 1;
				input_decimal_equiv *= 10;

				// Translate and add to lower part of number
				translate_ps2(byte, &input_decimal_equiv);

				// Convert to 'normal' decimal
				int i = 0;
				input_decimal = input_decimal_equiv;
				for(i = 0; i < input_number_counter; i++) {
					input_decimal /= 10.0;
				}
			}
			xSemaphoreGiveFromISR(shared_resource_mutex, NULL);
		}
	}
}

/*==================*/
/* Function Macros. */
/*==================*/
#define drop_load() { \
	if (loads[7] == 1) loads[7] = 0; \
	else if (loads[6] == 1) loads[6] = 0; \
	else if (loads[5] == 1) loads[5] = 0; \
	else if (loads[4] == 1) loads[4] = 0; \
	else if (loads[3] == 1) loads[3] = 0; \
	else if (loads[2] == 1) loads[2] = 0; \
	else if (loads[1] == 1) loads[1] = 0; \
	else loads[0] = 0; \
}

#define reconnect_load() { \
	if ((loads[0] == 0) && (switches[0] == 1)) loads[0] = 1; \
	else if ((loads[1] == 0) && (switches[1] == 1)) loads[1] = 1; \
	else if ((loads[2] == 0) && (switches[2] == 1)) loads[2] = 1; \
	else if ((loads[3] == 0) && (switches[3] == 1)) loads[3] = 1; \
	else if ((loads[4] == 0) && (switches[4] == 1)) loads[4] = 1; \
	else if ((loads[5] == 0) && (switches[5] == 1)) loads[5] = 1; \
	else if ((loads[6] == 0) && (switches[6] == 1)) loads[6] = 1; \
	else if (switches[7] == 1) loads[7] = 1; \
}
/*============*/
/* Functions. */
/*============*/
void translate_ps2(unsigned char byte, double *value) {
	switch(byte) {
		case PS2_0:
			*value += 0.0;
			break;
		case PS2_1:
			*value += 1.0;
			break;
		case PS2_2:
			*value += 2.0;
			break;
		case PS2_3:
			*value += 3.0;
			break;
		case PS2_4:
			*value += 4.0;
			break;
		case PS2_5:
			*value += 5.0;
			break;
		case PS2_6:
			*value += 6.0;
			break;
		case PS2_7:
			*value += 7.0;
			break;
		case PS2_8:
			*value += 8.0;
			break;
		case PS2_9:
			*value += 9.0;
			break;
		default:
			break;
	}
}

/*============*/
/* Callbacks. */
/*============*/
void vTimerDropCallback(xTimerHandle t_timer) {
	xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
	drop_load_timeout = 1;
	xSemaphoreGive(shared_resource_mutex);
}

void vTimerReconnectCallback(xTimerHandle t_timer) {
	xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
	reconnect_load_timeout = 1;
	xSemaphoreGive(shared_resource_mutex);
}

void vTimerSystemUptimeCallback(xTimerHandle t_timer){
	xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
	system_uptime += 1;
	xSemaphoreGive(shared_resource_mutex);
}

void vTimerDropDelayCallback(xTimerHandle t_timer){
	xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
	drop_delay += 1;
	xSemaphoreGive(shared_resource_mutex);
}

/*================*/
/* Main function. */
/*================*/
int main(void) {
	// Set up Interrupts
	int button_value = 0;
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x4);
	alt_irq_register(PUSH_BUTTON_IRQ,(void*)&button_value, button_interrupts_function);

	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);

	alt_up_ps2_dev *ps2_device = alt_up_ps2_open_dev(PS2_NAME);

	// Set up keyboard
	if(ps2_device == NULL){
		printf("Couldn't find a PS/2 device\n");
		return 1;
	}

	alt_up_ps2_disable_read_interrupt(ps2_device);
	alt_irq_register(PS2_IRQ, ps2_device, ps2_isr);

	// Create Timers
	drop_timer = xTimerCreate("Shedding Timer", 500, pdFALSE, NULL, vTimerDropCallback);
	recon_timer = xTimerCreate("Reconnect Timer", 500, pdFALSE, NULL, vTimerReconnectCallback);
	system_up_timer = xTimerCreate("System Uptime Timer", 1000, pdTRUE, NULL, vTimerSystemUptimeCallback);
	drop_delay_timer = xTimerCreate("Drop Delay Timer", 1, pdTRUE, NULL, vTimerDropDelayCallback);

	xTimerStart(system_up_timer, 0);
	xTimerStart(drop_delay_timer, 0);

	//Create queue
	Q_freq_data = xQueueCreate( 100, sizeof(double) );

	//Crete mutex
	shared_resource_mutex = xSemaphoreCreateMutex();

	// Set up Tasks
	xTaskCreate( prvDecideTask, "Rreg1", configMINIMAL_STACK_SIZE, mainREG_DECIDE_PARAMETER, mainREG_TEST_PRIORITY, NULL);
	xTaskCreate( prvLEDOutTask, "Rreg2", configMINIMAL_STACK_SIZE, mainREG_LED_OUT_PARAMETER, mainREG_TEST_PRIORITY, NULL);
	xTaskCreate( prvVGAOutTask, "Rreg3", configMINIMAL_STACK_SIZE, mainREG_VGA_OUT_PARAMETER, mainREG_TEST_PRIORITY, NULL);
	
	//Start task scheduler
	vTaskStartScheduler();

	// Only reaches here if not enough heap space to start tasks
	for(;;);
}

/*========*/
/* Tasks. */
/*========*/
// Decision Task
static void prvDecideTask(void *pvParameters) {
	while (1) {
		// Switch Load Management
		int switch_value = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
		int masked_switch_value = switch_value & 0x000ff;

		int i, k, no_loads_shed = 1;
		for (i = 7; i >= 0; i--) { // Iterate through switches array and set if the switch is on or off
			k = masked_switch_value >> i;
			if (k & 1) { // If the switch at this position is on
				xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
				switches[7-i] = 1;
				xSemaphoreGive(shared_resource_mutex);
				if (loads[7-i] == 0) {
					no_loads_shed = 0;
				}
				if (maintenance == 1) {
					xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
					loads[7-i] = 1;
					xSemaphoreGive(shared_resource_mutex);
				}
			} else { // If the switch at this position is off
				xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
				switches[7-i] = 0;
				loads[7-i] = 0;
				xSemaphoreGive(shared_resource_mutex);
			}
		}

		if (no_loads_shed == 1) { // If all available loads are connected, we are not managing loads.
			xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
			first_load_shed = 0;
			xSemaphoreGive(shared_resource_mutex);
		}

		// Frequency Load Management
		if (maintenance == 0) {
			if (fabs(roc_freq) > desired_max_roc_freq || desired_min_freq > signal_freq) { // If the current system is unstable
				if (first_load_shed == 0) { // Drop a load, if we have no dropped loads. First load drop.
					xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
					first_load_shed = 1;
					drop_load();
					xSemaphoreGive(shared_resource_mutex);

					// Timing Drop Delay
					if (drop_delay_flag == 1) {
						// Set min and max
						if (drop_delay > max_drop_delay) {
							xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
							max_drop_delay = drop_delay;
							xSemaphoreGive(shared_resource_mutex);
						}
						if (((drop_delay < min_drop_delay) && (drop_delay != 0)) || (min_drop_delay == 0)) {
							xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
							min_drop_delay = drop_delay;
							xSemaphoreGive(shared_resource_mutex);
						}

						xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
						// Calculate accumulated average
						if (drop_average == 0) {
							drop_average = (double) drop_delay;
						} else {
							drop_average = (drop_average + (double) drop_delay) / 2.0;
						}

						if (drop_delay != 0) {
							printf("Drop Time: %d ms\n", drop_delay);
						}

						drop_delay_flag = 0;
						xSemaphoreGive(shared_resource_mutex);
					}
				} else {
					xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
					reconnect_load_timeout = 0; // No longer a continuous run of stable data
					xSemaphoreGive(shared_resource_mutex);

					if(shed_flag == 0) { // Stop the timer if we are timing a 500ms for a load reconnection
						xTimerStop(recon_timer, 0);
					}

					if (drop_load_timeout == 0) { // If we haven't had a continuous run of unstable data
						if (xTimerIsTimerActive(drop_timer) == pdFALSE) {
							xTimerStart(drop_timer, 0);
						}
					} else {
						drop_load();
					}
					xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
					shed_flag = 1;
					xSemaphoreGive(shared_resource_mutex);
				}
			} else {
				xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
				drop_load_timeout = 0; // No longer a continuous run of unstable data
				xSemaphoreGive(shared_resource_mutex);

				if (shed_flag == 1) { // Stop the timer if we are timing a 500ms for a load drop
					xTimerStop(drop_timer, 0);
				}

				if (reconnect_load_timeout == 0) { // If we haven't had a continuous run of stable data
					if (xTimerIsTimerActive(recon_timer) == pdFALSE) {
						xTimerStart(recon_timer, 0);
					}
				} else {
					reconnect_load();
				}
				xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
				shed_flag = 0;
				xSemaphoreGive(shared_resource_mutex);

			}
		}
		vTaskDelay(20);
	}
}

// LED Output Task
static void prvLEDOutTask(void *pvParameters) {
	while (1) {
		int loads_num = 0;
		int loads_num_rev = 0;
		int i;
		
		// Inverse array for Red LEDS
		int rev_loads[8];
		for (i = 0; i < 8; i++) {
			if (loads[i] == 0) {
				rev_loads[i] = 1;
			} else {
				rev_loads[i] = 0;
			}
		}

		// Translate to binary
		for (i = 0; i < 8; i++) {
			loads_num = loads_num << 1;
			loads_num = loads_num + loads[i];
			loads_num_rev = loads_num_rev << 1;
			loads_num_rev = loads_num_rev + rev_loads[i];
		}

		// Write to LEDs base
		if (maintenance == 0) {
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, loads_num_rev);
		} else {
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0);
		}
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, loads_num);
		
		// Compute VGA data
		for (i = 4; i >= 1; i--) {
			xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
			store_freq[i] = store_freq[i-1];
			store_dfreq[i] = store_dfreq[i-1];
			xSemaphoreGive(shared_resource_mutex);
		}
		xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
		store_freq[0] = signal_freq;
		store_dfreq[0] = roc_freq;
		xSemaphoreGive(shared_resource_mutex);

		xSemaphoreTake(shared_resource_mutex, portMAX_DELAY);
		snprintf(m1, 5,"%f", store_freq[0]);
		snprintf(m2, 5,"%f", store_freq[1]);
		snprintf(m3, 5,"%f", store_freq[2]);
		snprintf(m4, 5,"%f", store_freq[3]);
		snprintf(m5, 5,"%f", store_freq[4]);

		snprintf(n1, 5,"%f", store_dfreq[0]);
		snprintf(n2, 5,"%f", store_dfreq[1]);
		snprintf(n3, 5,"%f", store_dfreq[2]);
		snprintf(n4, 5,"%f", store_dfreq[3]);
		snprintf(n5, 5,"%f", store_dfreq[4]);
		
		snprintf(system_uptime_string, 10,"%d s",system_uptime);

		snprintf(min_freq_string, 12, "%.1f Hz  ", desired_min_freq);
		snprintf(max_roc_string, 12, "%.1f Hz/s  ", desired_max_roc_freq);

		snprintf(min_drop_string, 8, "%d ms  ", min_drop_delay);
		snprintf(max_drop_string, 8, "%d ms  ", max_drop_delay);

		snprintf(average_drop_string, 12, "%.2f ms  ", drop_average);
		
		xSemaphoreGive(shared_resource_mutex);

		vTaskDelay(10);
	}
}

// VGA Output Task
static void prvVGAOutTask(void *pvParameters) {
	// Initialize VGA controllers
	alt_up_pixel_buffer_dma_dev *pixel_buf;
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
	if (pixel_buf == NULL) {
		printf("can't find pixel buffer device\n");
	}
	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

	alt_up_char_buffer_dev *char_buf;
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if (char_buf == NULL) {
		printf("can't find char buffer device\n");
	}
	alt_up_char_buffer_clear(char_buf);

	// Set up plot axes
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);

	alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
	alt_up_char_buffer_string(char_buf, "52", 10, 7);
	alt_up_char_buffer_string(char_buf, "50", 10, 12);
	alt_up_char_buffer_string(char_buf, "48", 10, 17);
	alt_up_char_buffer_string(char_buf, "46", 10, 22);

	alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
	alt_up_char_buffer_string(char_buf, "60", 10, 28);
	alt_up_char_buffer_string(char_buf, "30", 10, 30);
	alt_up_char_buffer_string(char_buf, "0", 10, 32);
	alt_up_char_buffer_string(char_buf, "-30", 9, 34);
	alt_up_char_buffer_string(char_buf, "-60", 9, 36);

	// Write static text
	alt_up_char_buffer_string(char_buf, "Frequency Relay System v1.4", 28, 2);
	alt_up_char_buffer_string(char_buf, "System uptime: ", 10, 40);
	alt_up_char_buffer_string(char_buf, "Current mode: ", 10, 42);
	alt_up_char_buffer_string(char_buf, "Latest 5 frequency measurements: ", 10, 44);
	alt_up_char_buffer_string(char_buf, "Latest 5 df/dt measurements: ", 10, 46);
	alt_up_char_buffer_string(char_buf, "Minimum Allowable Frequency: ", 10, 48);
	alt_up_char_buffer_string(char_buf, "Maximum Allowable Frequency ROC: ", 10, 50);
	alt_up_char_buffer_string(char_buf, "Minimum Time Taken: ", 10, 52);
	alt_up_char_buffer_string(char_buf, "Maximum Time Taken: ", 10, 54);
	alt_up_char_buffer_string(char_buf, "Average Time Taken: ", 10, 56);


	double freq[100], dfreq[100];
	int i = 99, j = 0;
	Line line_freq, line_roc;

	while(1){
		// Receive frequency data from queue
		while(uxQueueMessagesWaiting( Q_freq_data ) != 0){
			xQueueReceive( Q_freq_data, freq+i, 0 );

			// Calculate frequency RoC
			if(i==0){
				dfreq[0] = (freq[0]-freq[99]) * 2.0 * freq[0] * freq[99] / (freq[0]+freq[99]);
			}
			else{
				dfreq[i] = (freq[i]-freq[i-1]) * 2.0 * freq[i]* freq[i-1] / (freq[i]+freq[i-1]);
			}

			if (dfreq[i] > 100.0){
				dfreq[i] = 100.0;
			}

			i =	++i%100; // Point to the next data (oldest) to be overwritten

		}

		// Clear old graph to draw new graph
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 639, 199, 0, 0);
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 639, 299, 0, 0);

		for (j = 0; j < 99; ++j) { // i here points to the oldest data, j loops through all the data to be drawn on VGA
			if (((int)(freq[(i+j)%100]) > MIN_FREQ) && ((int)(freq[(i+j+1)%100]) > MIN_FREQ)){
				// Calculate coordinates of the two data points to draw a line in between
				// Frequency plot
				line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
				line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j)%100] - MIN_FREQ));

				line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
				line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j+1)%100] - MIN_FREQ));

				// Frequency RoC plot
				line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
				line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(i+j)%100]);

				line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
				line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(i+j+1)%100]);

				// Draw
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, 0x3ff << 0, 0);
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);

				// Write dynamic text
				alt_up_char_buffer_string(char_buf, system_uptime_string, 25, 40);

				if (maintenance == 0) {
					if (first_load_shed == 0) {
						alt_up_char_buffer_string(char_buf, "Monitoring     ", 24, 42);
					} else {
						alt_up_char_buffer_string(char_buf, "Load Management", 24, 42);
					}
				} else {
					alt_up_char_buffer_string(char_buf, "Maintenance    ", 24, 42);
				}

				alt_up_char_buffer_string(char_buf, m1, 43, 44);
				alt_up_char_buffer_string(char_buf, m2, 48, 44);
				alt_up_char_buffer_string(char_buf, m3, 53, 44);
				alt_up_char_buffer_string(char_buf, m4, 58, 44);
				alt_up_char_buffer_string(char_buf, m5, 63, 44);

				alt_up_char_buffer_string(char_buf, n1, 39, 46);
				alt_up_char_buffer_string(char_buf, n2, 44, 46);
				alt_up_char_buffer_string(char_buf, n3, 49, 46);
				alt_up_char_buffer_string(char_buf, n4, 54, 46);
				alt_up_char_buffer_string(char_buf, n5, 59, 46);

				alt_up_char_buffer_string(char_buf, min_freq_string, 39, 48);
				alt_up_char_buffer_string(char_buf, max_roc_string, 43, 50);

				alt_up_char_buffer_string(char_buf, min_drop_string, 30, 52);
				alt_up_char_buffer_string(char_buf, max_drop_string, 30, 54);

				alt_up_char_buffer_string(char_buf, average_drop_string, 30, 56);
			}
		}
		vTaskDelay(20);
	}
}
