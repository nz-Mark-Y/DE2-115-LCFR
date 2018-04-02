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
#define mainREG_TEST_1_PARAMETER    ( ( void * ) 0x12345678 )
#define mainREG_TEST_2_PARAMETER    ( ( void * ) 0x87654321 )
#define mainREG_TEST_PRIORITY       ( tskIDLE_PRIORITY + 1)
#define SAMPLE_FREQ 				16000

/* Function Declarations. */
static void prvFirstRegTestTask(void *pvParameters);
static void prvSecondRegTestTask(void *pvParameters);

/* Global Variables. */
double signalFreq = 0;
double rocFreq = 0;

/* ISRs. */
void button_interrupts_function(void* context, alt_u32 id)
{
	int* temp = (int*) context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);
	printf("Hello \n");
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

void freq_relay()
{
	unsigned int temp = IORD(FREQUENCY_ANALYSER_BASE, 0);
	
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
	int buttonValue = 0;
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);
	alt_irq_register(PUSH_BUTTON_IRQ,(void*)&buttonValue, button_interrupts_function);

	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);

	xTaskCreate( prvFirstRegTestTask, "Rreg1", configMINIMAL_STACK_SIZE, mainREG_TEST_1_PARAMETER, mainREG_TEST_PRIORITY, NULL);
	xTaskCreate( prvSecondRegTestTask, "Rreg2", configMINIMAL_STACK_SIZE, mainREG_TEST_2_PARAMETER, mainREG_TEST_PRIORITY, NULL);

	vTaskStartScheduler();

	for(;;);
}

/* Tasks. */
static void prvFirstRegTestTask(void *pvParameters)
{
	while (1)
	{
		printf("Signal frequency: %f Hz\n", signalFreq);
		vTaskDelay(100);
	}
}

static void prvSecondRegTestTask(void *pvParameters)
{
	while (1)
	{
		printf("Rate of change: %f\n", rocFreq);
		vTaskDelay(100);
	}
}
