/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_gpio.h"
#include "state_machine.h"

#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Buttons */
gpio_pin_config_t gpio_config_sw3 = {
		kGPIO_DigitalInput,
		0
};

gpio_pin_config_t gpio_config_sw2 = {
		kGPIO_DigitalInput,
		0
};

/* RGB LED */
gpio_pin_config_t gpio_config_red = {
		kGPIO_DigitalOutput,
		1
};
gpio_pin_config_t gpio_config_green = {
		kGPIO_DigitalOutput,
		1
};
gpio_pin_config_t gpio_config_blue = {
		kGPIO_DigitalOutput,
		1
};

static states_t ctrl_state_machine;

/* BUTTON SW3 Interrupt */
void PORTA_IRQHandler (void){

	PRINTF("Interrupt SW3!!!.\r\n");
	NVIC_DisableIRQ(PORTA_IRQn);
	PORTA->PCR[4] = ((PORTA->PCR[4] &
	          (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
	            | PORT_PCR_PS(0x1)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
	            | PORT_PCR_PE(0x1)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
				| PORT_PCR_ISF_MASK
				| PORT_PCR_IRQC(0xA)
	          );
	PORTA->ISFR = 0x0;
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);

	state_machine(BT_SW3);

}

/* BUTTON SW2 Interrupt */
void PORTC_IRQHandler (void){

	PRINTF("Interrupt SW2!!!.\r\n");
	NVIC_DisableIRQ(PORTC_IRQn);
	PORTC->PCR[6] = ((PORTC->PCR[6] &
	          (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK))) /* Mask bits to zero which are setting */
	            | PORT_PCR_PS(0x1)                               /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
	            | PORT_PCR_PE(0x1)                          /* Pull Enable: Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a digital input. */
				| PORT_PCR_ISF_MASK
				| PORT_PCR_IRQC(0xA)
	          );
	PORTC->ISFR = 0x0;
	NVIC_ClearPendingIRQ(PORTC_IRQn);
	NVIC_EnableIRQ(PORTC_IRQn);

	state_machine(BT_SW2);
}

/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void hello_task(void *pvParameters);
static void hello_task2(void *pvParameters);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
  //  xTaskCreate(hello_task, "Hello_task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
  //  xTaskCreate(hello_task2, "Hello_task2", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
  //  vTaskStartScheduler();

    /* Buttons */
    GPIO_PinInit (GPIOA, 4u, &gpio_config_sw3);
    GPIO_PinInit (GPIOC, 6u, &gpio_config_sw2);

    /* RGB Led */
    GPIO_PinInit (GPIOB, 22u, &gpio_config_red);
    GPIO_PinInit (GPIOE, 26u, &gpio_config_green);
    GPIO_PinInit (GPIOB, 21u, &gpio_config_blue);

    state_machine_init(&ctrl_state_machine);


    for (;;)
        ;
}



/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
static void hello_task(void *pvParameters)
{
    for (;;)
    {
        PRINTF("Hello world.\r\n");
        vTaskSuspend(NULL);
    }
}

static void hello_task2(void *pvParameters)
{
    for (;;)
    {
        PRINTF("Hello world PRIORITY.\r\n");
        vTaskSuspend(NULL);
    }
}

