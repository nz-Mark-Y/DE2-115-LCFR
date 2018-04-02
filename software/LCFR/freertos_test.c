/* Standard includes. */
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "system.h"                     // to use the symbolic names
#include "altera_avalon_pio_regs.h" 	// to use PIO functions
#include "alt_types.h"                 	// alt_u32 is a kind of alt_types
#include "sys/alt_irq.h"              	// to register interrupts

/* Scheduler includes. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define mainREG_TEST_1_PARAMETER    ( ( void * ) 0x12345678 )
#define mainREG_TEST_2_PARAMETER    ( ( void * ) 0x87654321 )
#define mainREG_TEST_PRIORITY       ( tskIDLE_PRIORITY + 1)
static void prvFirstRegTestTask(void *pvParameters);
static void prvSecondRegTestTask(void *pvParameters);

void button_interrupts_function(void* context, alt_u32 id)
{
	int* temp = (int*) context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);
	printf("Hello \n");
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

void freq_interrupts_function(void* context, alt_u32 id)
{
	int* temp = (int*) context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(FREQUENCY_ANALYSER_BASE);
	printf("Hello World \n");
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(FREQUENCY_ANALYSER_BASE, 1);
}

int main(void)
{
	int buttonValue = 0;
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);
	alt_irq_register(PUSH_BUTTON_IRQ,(void*)&buttonValue, button_interrupts_function);

	int freqValue = 0;
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(FREQUENCY_ANALYSER_BASE, 0x1);
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(FREQUENCY_ANALYSER_BASE, 0x1);
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, (void*)&freqValue, freq_interrupts_function);

	xTaskCreate( prvFirstRegTestTask, "Rreg1", configMINIMAL_STACK_SIZE, mainREG_TEST_1_PARAMETER, mainREG_TEST_PRIORITY, NULL);
	xTaskCreate( prvSecondRegTestTask, "Rreg2", configMINIMAL_STACK_SIZE, mainREG_TEST_2_PARAMETER, mainREG_TEST_PRIORITY, NULL);

	vTaskStartScheduler();

	while(1)
	{
	}
}
static void prvFirstRegTestTask(void *pvParameters)
{
	while (1)
	{
		printf("Task 1\n");
		vTaskDelay(1000);
	}
}

static void prvSecondRegTestTask(void *pvParameters)
{
	while (1)
	{
		printf("Task 2\n");
		vTaskDelay(1000);
	}
}
