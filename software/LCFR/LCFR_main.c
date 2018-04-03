/* Standard includes. */
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ISR Includes. */
#include "system.h"
#include "altera_avalon_pio_regs.h"
#include "alt_types.h"
#include "sys/alt_irq.h"

/* Scheduler includes. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

/* Definitions. */
#define mainREG_DECIDE_PARAMETER    ( ( void * ) 0x12345678 )
#define mainREG_LED_OUT_PARAMETER   ( ( void * ) 0x87654321 )
#define mainREG_VGA_OUT_PARAMETER 	( ( void * ) 0x12348765 )
#define mainREG_TEST_PRIORITY       ( tskIDLE_PRIORITY + 1)
#define SAMPLE_FREQ 				16000

/* Function Declarations. */
static void prvDecideTask(void *pvParameters);
static void prvLEDOutTask(void *pvParameters);
static void prvVGAOutTask(void *pvParameters);

/* Global Variables. */
int first_load_shed = 0;
int shed_flag = 0;
int reconnect_load_timeout = 0;
int drop_load_timeout = 0;

double max_roc_freq = 800838383;
double min_freq = 50;
double signal_freq = 0;
double roc_freq = 0;
int loads[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };

/* Handles. */
TimerHandle_t timer;

/* ISRs. */
void button_interrupts_function(void* context, alt_u32 id) {
	int* temp = (int*) context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE); // Store which button was pressed
	printf("Hello\n");
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7); // Clear edge capture register
}

void freq_relay() {
	unsigned int temp = IORD(FREQUENCY_ANALYSER_BASE, 0); // Get the sample count between the two most recent peaks
	
	//Important: do not swap the order of the two operations otherwise the roc will be 0 all the time
	if (temp > 0) {
		roc_freq = ((SAMPLE_FREQ / (double) temp) - signal_freq) * (SAMPLE_FREQ / (double) temp);
		signal_freq = SAMPLE_FREQ / (double) temp;
	}

	return;
}

/* Function Macros. */
#define drop_load() { \
	if (loads[0] == 1) loads[0] = 0; \
	else if (loads[1] == 1) loads[1] = 0; \
	else if (loads[2] == 1) loads[2] = 0; \
	else if (loads[3] == 1) loads[3] = 0; \
	else if (loads[4] == 1) loads[4] = 0; \
	else if (loads[5] == 1) loads[5] = 0; \
	else if (loads[6] == 1) loads[6] = 0; \
	else loads[7] = 0; \
}

#define reconnect_load() { \
	if (loads[7] == 0) loads[7] = 1; \
	else if (loads[6] == 0) loads[6] = 1; \
	else if (loads[5] == 0) loads[5] = 1; \
	else if (loads[4] == 0) loads[4] = 1; \
	else if (loads[3] == 0) loads[3] = 1; \
	else if (loads[2] == 0) loads[2] = 1; \
	else if (loads[1] == 0) loads[1] = 1; \
	else loads[0] = 1; \
}

/* Callbacks. */
void vTimerDropCallback(xTimerHandle t_timer) { //Timer flashes green LEDs
	drop_load();
	drop_load_timeout = 1;
}

void vTimerReconnectCallback(xTimerHandle t_timer) { //Timer flashes green LEDs
	reconnect_load();
	reconnect_load_timeout = 1;
}

/* Main function. */
int main(void) {
	// Set up Interrupts
	int button_value = 0;
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x4);
	alt_irq_register(PUSH_BUTTON_IRQ,(void*)&button_value, button_interrupts_function);

	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);

	// Set up Tasks
	xTaskCreate( prvDecideTask, "Rreg1", configMINIMAL_STACK_SIZE, mainREG_DECIDE_PARAMETER, mainREG_TEST_PRIORITY, NULL);
	xTaskCreate( prvLEDOutTask, "Rreg2", configMINIMAL_STACK_SIZE, mainREG_LED_OUT_PARAMETER, mainREG_TEST_PRIORITY, NULL);
	xTaskCreate( prvVGAOutTask, "Rreg3", configMINIMAL_STACK_SIZE, mainREG_VGA_OUT_PARAMETER, mainREG_TEST_PRIORITY, NULL);

	vTaskStartScheduler();

	// Only reaches here if not enough heap space to start tasks
	for(;;);
}

/* Tasks. */
static void prvDecideTask(void *pvParameters) {
	while (1) {
		int switch_value = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
		int masked_switch_value = switch_value & 0x000ff;
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, masked_switch_value);
		
		if(fabs(roc_freq) > max_roc_freq || min_freq > signal_freq) {
			if (first_load_shed == 0) {
				first_load_shed = 1;
				drop_load();
			} else {
				reconnect_load_timeout = 0;
				
				if(shed_flag == 0) {
					xTimerStop(timer, 0);
				}
				
				if (drop_load_timeout == 0) {
					timer = xTimerCreate("Shedding Timer", 500, pdTRUE, NULL, vTimerDropCallback);
					xTimerStart(timer, 0);
				} else {
					drop_load();
				}
				shed_flag = 1;
			}
		} else {
			drop_load_timeout = 0;
			
			if (shed_flag == 1) {
				xTimerStop(timer, 0);
			}

			if (reconnect_load_timeout == 0) {
				timer = xTimerCreate("Reconnect Timer", 500, pdTRUE, NULL, vTimerReconnectCallback);
				xTimerStart(timer, 0);
			} else {
				reconnect_load();
			}
			shed_flag = 0;
		}
		
		vTaskDelay(10);
	}
}

static void prvLEDOutTask(void *pvParameters)
{
	while (1)
	{
		int loads_num = 0;
		int i;
		for (i = 0; i < 8; i++)
		{
			loads_num = loads_num << 1;
			loads_num = loads_num + loads[i];
		}
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, loads_num);
		vTaskDelay(10);
	}
}

static void prvVGAOutTask(void *pvParameters)
{
	while (1)
	{
		printf("Signal frequency: %f Hz\n", signal_freq);
		printf("Rate of change: %f\n", roc_freq);
		vTaskDelay(100);
	}
}
