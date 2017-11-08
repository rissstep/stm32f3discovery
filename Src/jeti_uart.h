/*
 * jeti_uart.h
 *
 *  Created on: Nov 2, 2017
 *      Author: stepan
 */

#ifndef JETI_UART_H_
#define JETI_UART_H_

#include "stm32f3xx_hal.h"

#define POLY 0x07
#define EX_SEPARATOR 0x7E
#define EX_ID 0x6F
#define EX_NULL 0x00
#define JETI_TEXT_START 0xFE
#define JETI_TEXT_STOP 0xFF
#define JETI_PROTOCOL_LENGHT 13 // 1 start + 9 data + 1 parity + 2 stop = 13



//!! STATICKY NA JEDEN BYTE DATA
typedef struct{
	uint16_t man_ID;
	uint16_t dev_ID;
	uint8_t identifier_1;
	uint8_t data_type_1; // 1 = 1B -> 1|01|00000 = sign(zero = positive)|position of decimal point|value, 2 = 2B dtto...
	uint16_t data_1;
	uint8_t identifier_2;
	uint8_t data_type_2;
	uint16_t data_2;
	//I know you wont listen, but don't ever touch this members.
	uint8_t _msg_lenght;
	uint16_t _seq[34];

}JETI_EX_DATA;

typedef struct{
	uint16_t man_ID;
	uint16_t dev_ID;
	uint8_t identifier;
	uint8_t * label_value;
	uint8_t * label_unit;
	//I know you wont listen, but don't ever touch this members.
	uint8_t _msg_lenght;
	uint16_t _seq[34];
}JETI_EX_TEXT;

unsigned char update_crc (unsigned char crc, unsigned char crc_seed);
unsigned char crc8 (unsigned char *crc, unsigned char crc_lenght);
int has_odd_parity(uint16_t x);
void print_bite(uint8_t b);
void print_binary(uint16_t msg);
void generate_seq(uint8_t * msg, uint16_t * seq, uint8_t full_length);
int send_jeti_data(JETI_EX_DATA *msg);
int send_jeti_text(JETI_EX_TEXT *msg);
void jeti_uart();

void esemble_seq_data(JETI_EX_DATA * msg);
void esemble_seq_text(JETI_EX_TEXT * msg);



#endif /* JETI_UART_H_ */
