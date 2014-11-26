/**
 * \file
 *
 * \brief SAM D20/D21 I2C Master Quick Start Guide with Callbacks
 *
 * Copyright (C) 2012-2014 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include <asf.h>
#include <system_interrupt.h>
#include "tc_samd.h"
#include "uart_bt.h"

void i2c_write_complete_callback(struct i2c_master_module *const module);
void configure_i2c(void);
void configure_i2c_callbacks(void);

void tc_disable_2(void);
//void i2c_write_T(uint8_t port);


/* Data for EDBG communication */
//static uint8_t tx_data_I2C;
//static uint8_t rx_data;
//static uint8_t tx_data_I2C;
//static int tx_data_I2C;
static int TEMPx;

//! [packet_data]
#define MAX_DATA_LENGTH 80

// OFFSET_WRITE_LEN_CH1 must be < MAX_DATA_LENGTH - 1
#define OFFSET_WRITE_LEN_CH1 64
#define OFFSET_WRITE_LEN_CH2 2*OFFSET_WRITE_LEN_CH1

#define MAX_I2C_DATA_LENGTH OFFSET_WRITE_LEN_CH1-1

static uint8_t buffer_cdc[MAX_DATA_LENGTH];

static uint8_t buffer_i2c[MAX_DATA_LENGTH];
//! [packet_data]

//! [address]
#define SLAVE_ADDRESS 0x02
//! [address]

//! [packet_glob]
struct i2c_packet packet;
//! [packet_glob]

/* Init software module instance. */
//! [dev_inst]
struct i2c_master_module i2c_master_instance;
struct i2c_master_module i2c_master_instance2;
//! [dev_inst]


/* Structure for USART module connected to EDBG */
struct usart_module usart_module_bt;

static char bytBTBufferIndex=0;

void i2c_write(void)
{
	// define a variable for buffer length
	iram_size_t intBufferLen = 0;

	if (udi_cdc_is_rx_ready())
	{
		char bytA=1;
		//char bytB=0;
		// we have data available
		intBufferLen = udi_cdc_get_nb_received_data();
				
		if (intBufferLen>MAX_DATA_LENGTH)
			intBufferLen=MAX_DATA_LENGTH;
		// copy the data to buffer
		udi_cdc_read_buf(buffer_cdc, intBufferLen);	//(int*)
		// check the dongle pattern
		if (buffer_cdc[0]==255) {
			// respond with the same pattern
			buffer_cdc[0] = buffer_cdc[1];
			buffer_cdc[1] = buffer_cdc[2];
			buffer_cdc[2] = buffer_cdc[0];
			buffer_cdc[0] = 255;
			TEMPx = udi_cdc_write_buf(&buffer_cdc,intBufferLen);
		}
		else{
			// copy partially the array
			for(bytA=1;bytA<intBufferLen;bytA++)
				buffer_i2c[(int)bytA-1]=buffer_cdc[(int)bytA];

			/* Init i2c packet. */
			//! [packet]
			packet.address     = SLAVE_ADDRESS;
			packet.data_length = intBufferLen-1;
			packet.data        = &buffer_i2c;
			//! [packet]
		
			// check if is for Left of Right
			if (buffer_cdc[0]==1) {
				// write to i2c_master_instance
				i2c_master_write_packet_wait(&i2c_master_instance, &packet);
			}
			else{
				if (buffer_cdc[0]==2) {
					// write to i2c_master_instance2
					i2c_master_write_packet_wait(&i2c_master_instance2, &packet);
				}
				else{
					// first we have to check OFFSET_WRITE_LEN_CH2 because is > OFFSET_WRITE_LEN_CH1
					if (buffer_cdc[0]>=OFFSET_WRITE_LEN_CH2) {
						// read to i2c_master_instance
						i2c_master_write_packet_wait(&i2c_master_instance2, &packet);
						intBufferLen = buffer_cdc[0] - OFFSET_WRITE_LEN_CH2;
						// limit to 63
						if (intBufferLen > MAX_I2C_DATA_LENGTH) intBufferLen = MAX_I2C_DATA_LENGTH;
						packet.data_length = intBufferLen;
						i2c_master_read_packet_wait(&i2c_master_instance2, &packet);
			
						/* Transfer UART RX fifo to CDC TX */
						if (!udi_cdc_is_tx_ready()) {
							/* Fifo full */
							udi_cdc_signal_overrun();
							//ui_com_overflow();
						} 
						else {
							TEMPx = udi_cdc_write_buf(&buffer_i2c,intBufferLen);
						}
					}
					else{
						if (buffer_cdc[0]>=OFFSET_WRITE_LEN_CH1) {
							// read to i2c_master_instance
							i2c_master_write_packet_wait(&i2c_master_instance, &packet);
							intBufferLen = buffer_cdc[0] - OFFSET_WRITE_LEN_CH1;
							// limit to 63
							if (intBufferLen > MAX_I2C_DATA_LENGTH) intBufferLen = MAX_I2C_DATA_LENGTH;
							packet.data_length = intBufferLen;
							i2c_master_read_packet_wait(&i2c_master_instance, &packet);
					
							/* Transfer UART RX fifo to CDC TX */
							if (!udi_cdc_is_tx_ready()) {
								/* Fifo full */
								udi_cdc_signal_overrun();
								//ui_com_overflow();
								} 
							else {
								TEMPx = udi_cdc_write_buf(&buffer_i2c,intBufferLen);
							}
						}
					}
				}
			}
		}
	}
}

void load_buffer_cdc(uint8_t data_value)
{
	buffer_cdc[(int)bytBTBufferIndex] = data_value;
	bytBTBufferIndex++;
}
	
void i2c_writeBT(void)
{
	// define a variable for buffer length
	iram_size_t intBufferLen = 0;
	char bytA=1;

	intBufferLen = bytBTBufferIndex;
	bytBTBufferIndex=0;
		
	tc_disable_2();
		
	if (intBufferLen>MAX_DATA_LENGTH)
	intBufferLen=MAX_DATA_LENGTH;
	// copy the data to buffer
	udi_cdc_read_buf(buffer_cdc, intBufferLen);	//(int*)
		
	// check the dongle pattern
	if (buffer_cdc[0]==255) {
		// respond with the same pattern
		buffer_cdc[0] = buffer_cdc[1];
		buffer_cdc[1] = buffer_cdc[2];
		buffer_cdc[2] = buffer_cdc[0];
		buffer_cdc[0] = 255;
		for(bytA=0;bytA<intBufferLen;bytA++)
		{
			usart_write_wait(&usart_module_bt, buffer_cdc[bytA]);
			delay_ms(20);
		}
	}
	else {
		// copy partially the array
		for(bytA=1;bytA<intBufferLen;bytA++)
		buffer_i2c[(int)bytA-1]=buffer_cdc[(int)bytA];

		/* Init i2c packet. */
		//! [packet]
		packet.address     = SLAVE_ADDRESS;
		packet.data_length = intBufferLen-1;
		packet.data        = &buffer_i2c;
		//! [packet]
			
		// check if is for Left of Right
		if (buffer_cdc[0]==1) {
			// write to i2c_master_instance
			i2c_master_write_packet_wait(&i2c_master_instance, &packet);
		}
		else{
			if (buffer_cdc[0]==2) {
				// write to i2c_master_instance2
				i2c_master_write_packet_wait(&i2c_master_instance2, &packet);
			}
			else{
				// first we have to check OFFSET_WRITE_LEN_CH2 because is > OFFSET_WRITE_LEN_CH1
				if (buffer_cdc[0]>=OFFSET_WRITE_LEN_CH2) {
					// read to i2c_master_instance
					i2c_master_write_packet_wait(&i2c_master_instance2, &packet);
					intBufferLen = buffer_cdc[0] - OFFSET_WRITE_LEN_CH2;
					// limit to 63
					if (intBufferLen > MAX_I2C_DATA_LENGTH) intBufferLen = MAX_I2C_DATA_LENGTH;
					packet.data_length = intBufferLen;
					i2c_master_read_packet_wait(&i2c_master_instance2, &packet);
					/* Transfer buffer to UART RX BT*/
					intBufferLen = packet.data_length;
					//usart_write_buffer_wait(&usart_module_bt, &buffer_i2c, intBufferLen);

					for(bytA=0;bytA<intBufferLen;bytA++)
					{
						usart_write_wait(&usart_module_bt, buffer_i2c[bytA]);
						delay_ms(20);
					}
				}
				else{
					if (buffer_cdc[0]>=OFFSET_WRITE_LEN_CH1) {
						// read to i2c_master_instance
						i2c_master_write_packet_wait(&i2c_master_instance, &packet);
						intBufferLen = buffer_cdc[0] - OFFSET_WRITE_LEN_CH1;
						// limit to 63
						if (intBufferLen > MAX_I2C_DATA_LENGTH) intBufferLen = MAX_I2C_DATA_LENGTH;
						packet.data_length = intBufferLen;
						i2c_master_read_packet_wait(&i2c_master_instance, &packet);
						/* Transfer buffer to UART RX BT*/
						intBufferLen = packet.data_length;
						//TEMPx = usart_write_buffer_job(&usart_module_bt, &buffer_i2c, intBufferLen);
						
						for(bytA=0;bytA<intBufferLen;bytA++)
						{
							usart_write_wait(&usart_module_bt, buffer_i2c[bytA]);
							delay_ms(20);
						}
					}
				}
			}
		}
	}
	/* Transmit first data 
	ui_com_rx_start();
	usart_enable_callback(&usart_module_edbg, USART_CALLBACK_BUFFER_TRANSMITTED);
	tx_data = udi_cdc_getc();
	usart_write_buffer_job(&usart_module_edbg, &tx_data, 1); */
}

//! [callback_func]
void i2c_write_complete_callback(
		struct i2c_master_module *const module)
{
	/* Send response back over CDC */
	

	/* Send every other packet with reversed data 
	//! [revert_order]
	if (packet.data[0] == 0x00) {
		packet.data = &buffer_reversed[0];
	} else {
		packet.data = &buffer[0];
	}
	//! [revert_order]

	// Initiate new packet write //
	//! [write_next]
	i2c_master_write_packet_job(module, &packet);
	//! [write_next] */
}
//! [callback_func]

//! [initialize_i2c]
void configure_i2c(void)
{
	/* Initialize config structure and software module */
	//! [init_conf]
	struct i2c_master_config config_i2c_master;
	struct i2c_master_config config_i2c_master2;
	i2c_master_get_config_defaults(&config_i2c_master);
	i2c_master_get_config_defaults(&config_i2c_master2);
	//! [init_conf]

	/* Change buffer timeout to something longer */
	//! [conf_change]
	config_i2c_master.buffer_timeout = 65535;
	config_i2c_master2.buffer_timeout = 65535;
	//! [conf_change]

	/* Initialize and enable device with config */
	//! [init_module]
	while(i2c_master_init(&i2c_master_instance, SERCOM4, &config_i2c_master) != STATUS_OK);
	while(i2c_master_init(&i2c_master_instance2, SERCOM2, &config_i2c_master2) != STATUS_OK);
	//! [init_module]

	//! [enable_module]
	i2c_master_enable(&i2c_master_instance);
	i2c_master_enable(&i2c_master_instance2);
	//! [enable_module]
}
//! [initialize_i2c]

//! [setup_callback]
void configure_i2c_callbacks(void)
{
	/* Register callback function. */
	//! [callback_reg]
	i2c_master_register_callback(&i2c_master_instance, i2c_write_complete_callback,
			I2C_MASTER_CALLBACK_WRITE_COMPLETE);
	//! [callback_reg]
	//! [callback_en]
	i2c_master_enable_callback(&i2c_master_instance,
			I2C_MASTER_CALLBACK_WRITE_COMPLETE);
	//! [callback_en]
}
//! [setup_callback]
