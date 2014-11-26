/*
 * bluetooth.c
 *
 *  Created on: Oct 6, 2014
 *      Author: Owner
 */
#include "bluetooth.h"
#include "driverlib.h"


uint8_t init_bluetooth(void)
{
    uint8_t result = 0;

    /*init UART*/
    CYC_SYS_UART1_Initialize();


    /*init GPIOS*/
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN5);
    /*reset Port 1.5.  RESET ACTIVE LOW*/
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN5);
    __delay_cycles(0xffff);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN5);
    __delay_cycles(0xffffff);
    //uint8_t reset_cmd[4] = {0x01, 0x03, 0x0c, 0x00};
   // CYC_SYS_UART_TransmitData(1, reset_cmd, sizeof(reset_cmd));   /*    we expect response 0x04, 0x0e, 0x04, 0x04, 0x03, 0x0c 0x00   TCU_HCI_RESET_RESP  */






    return result;
}
