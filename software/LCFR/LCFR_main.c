/* Standard includes. */
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* ISR Includes. */
#include "system.h"
#include "altera_avalon_pio_regs.h"
#include "alt_types.h"
#include "sys/alt_irq.h"

/* Scheduler includes. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

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
double signalFreq = 0;
double rocFreq = 0;
int loads[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };

/* ISRs. */
void button_interrupts_function(void* context, alt_u32 id)
{
	int* temp = (int*) context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE); // Store which button was pressed
	printf("Hello\n");
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7); // Clear edge capture register
}

void freq_relay()
{
	unsigned int temp = IORD(FREQUENCY_ANALYSER_BASE, 0); // Get the sample count between the two most recent peaks
	
	//Important: do not swap the order of the two operations otherwise the roc will be 0 all the time
	if (temp > 0)
	{
		rocFreq = ((SAMPLE_FREQ / (double) temp) - signalFreq) * (SAMPLE_FREQ / (double) temp);
		signalFreq = SAMPLE_FREQ / (double) temp;
	}

	return;
}

/* Main function. */
int main(void)
{
	// Set up Interrupts
	int buttonValue = 0;
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x4);
	alt_irq_register(PUSH_BUTTON_IRQ,(void*)&buttonValue, button_interrupts_function);

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
static void prvDecideTask(void *pvParameters)
{
	while (1)
	{
		printf("Signal frequency: %f Hz\n", signalFreq);
		vTaskDelay(100);
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
		printf("%d \n", loads_num);
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, loads_num);
		vTaskDelay(100);
	}
}

static void prvVGAOutTask(void *pvParameters)
{
	while (1)
	{
		printf("Rate of change: %f\n", rocFreq);
		vTaskDelay(100);
	}
}
