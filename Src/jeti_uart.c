/*
 * jeti_uart.c
 *
 *  Created on: Nov 2, 2017
 *      Author: stepan
 */

#include "jeti_uart.h"
#include <string.h>

uint16_t jeti_uart_count = 0;
uint8_t jeti_uart_start = 0;

uint16_t  * stack_seq[2048];
uint8_t stack_lenght[2048];

uint16_t p_stack_low = 0;
uint16_t p_stack_high = 0;

extern uint32_t timetick_ms;

unsigned char update_crc (unsigned char crc, unsigned char crc_seed){
	unsigned char crc_u;
	unsigned char i;
	crc_u = crc;
	crc_u ^= crc_seed;
	for (i=0; i<8; i++)
	{
		crc_u = ( crc_u & 0x80 ) ? POLY ^ ( crc_u << 1 ) : ( crc_u << 1 );
	}
	return crc_u;
}

unsigned char crc8 (unsigned char *crc, unsigned char crc_lenght){
	unsigned char crc_up = 0;
	unsigned char c;
	for(c=0;c < crc_lenght; c++) {
		crc_up = update_crc (crc[c], crc_up);
	}
	return crc_up;
}

int has_odd_parity(uint16_t x){
    unsigned int count = 0, i, b = 1;

    for(i = 0; i < 16; i++){
        if( x & (b << i) ){count++;}
    }

    if( (count % 2) ){return 0;}

    return 1;
}

void generate_seq(uint8_t * msg, uint16_t * seq, uint8_t full_length){
	int i;
	uint8_t postfix;
	uint8_t tmp = 0;
	for(i =0;i<full_length;++i){

		tmp = 0;
		for(int j = 0; j<8;++j){
			if((msg[i] >> j) & 1){
				tmp |= 0x1;
			}
			if(j!=7)tmp<<=1;
		}

		seq[i] = tmp;
		seq[i] <<= (JETI_PROTOCOL_LENGHT == 12) ? 3 : 4;

		postfix =0b0011;
		if(JETI_PROTOCOL_LENGHT == 13){
			if(msg[i] != 0xff && msg[i] != 0x7e && msg[i] != 0xfe){
				seq[i] |= 0b1000;
			}
		}
		if(has_odd_parity(seq[i])){
			postfix |= 0b100;
		}

		seq[i] |= postfix;

	}

}

int send_jeti_data( JETI_EX_DATA * msg){

	if((p_stack_low-1) == p_stack_high){
		//pointer high dohonil low zespoda -> neni mozne pridat zpravu
		HAL_GPIO_TogglePin(LD7_GPIO_Port,LD7_Pin);
		return 1;

	}else{
		stack_seq[p_stack_high] = &msg->_seq[0];
		stack_lenght[p_stack_high] = msg->_msg_lenght;
		p_stack_high++;

		if(p_stack_high >= 2048) p_stack_high=0;
	}

	return 0;
}

int send_jeti_text( JETI_EX_TEXT * msg){

	if((p_stack_low-1) == p_stack_high){
		//pointer high dohonil low zespoda -> neni mozne pridat zpravu
		HAL_GPIO_TogglePin(LD7_GPIO_Port,LD7_Pin);
		return 1;

	}else{
		stack_seq[p_stack_high] = &msg->_seq[0];
		stack_lenght[p_stack_high] = msg->_msg_lenght;
		p_stack_high++;

		if(p_stack_high >= 2048) p_stack_high=0;
	}

	return 0;
}

void jeti_uart(){

	static uint8_t seq_p = 0;
	static uint8_t seq_shift = JETI_PROTOCOL_LENGHT;

	static int16_t i = 1;

	static uint8_t msg_on_sending = 0;

	static uint16_t * seq;
	static uint32_t time_between = 0;

	static int interval_gone = 1;

	if(p_stack_low != p_stack_high && msg_on_sending == 0){
		msg_on_sending = 1;
		jeti_uart_start = 1;
		seq = stack_seq[p_stack_low];
	}

	if(timetick_ms >= (time_between + 20) && !interval_gone){
		interval_gone = 1;
	}

	if(jeti_uart_count >= i && msg_on_sending){
		//HAL_GPIO_TogglePin(LD8_GPIO_Port,LD8_Pin);

		seq_shift--;

		HAL_GPIO_WritePin(SW_TX_GPIO_Port,SW_TX_Pin,(seq[seq_p] >> seq_shift) & 1);
		//print_bite((seq[seq_p] >> seq_shift) & 1);

		if(seq_shift == 0 && seq_p == stack_lenght[p_stack_low]-1){
			jeti_uart_start = 0;
			seq_p = 0;
			seq_shift = JETI_PROTOCOL_LENGHT;
			i = 1;
			jeti_uart_count = 0;
			msg_on_sending = 0;
			p_stack_low++;

			if(p_stack_low >= 2048) p_stack_low = 0;

			HAL_GPIO_WritePin(SW_TX_GPIO_Port,SW_TX_Pin,1);

			//time_between = timetick_ms;	// tady mel byt jeste zajisten 20ms interval
			//interval_gone =0;


			//HAL_UART_Transmit(&huart2, "\n",1,1);
			return;
		}

		if(seq_shift == 0){
			seq_p++;
			seq_shift = JETI_PROTOCOL_LENGHT;
			//HAL_UART_Transmit(&huart2, "\n",1,1);
		}

		i++;

	}

}

void esemble_seq_data(JETI_EX_DATA * msg){

	msg->_msg_lenght = 13; // viz datasheet s jednim datovym bytem, max 29

	uint8_t array[msg->_msg_lenght];

	array[0] = EX_SEPARATOR;
	array[1] = EX_ID;
	array[2] = (msg->_msg_lenght-3) | 0x40;
	array[3] = (uint8_t)(msg->man_ID & 0x00FF);
	array[4] = (uint8_t)(msg->man_ID >> 8);
	array[5] = (uint8_t)(msg->dev_ID & 0x00FF);
	array[6] = (uint8_t)(msg->dev_ID  >> 8);
	array[7] = EX_NULL;
	array[8] = (msg->identifier_1 << 4) | msg->data_type_1;
	array[9] = msg->data_1;
	array[10] = (msg->identifier_2 << 4) | msg->data_type_2;
	array[11] = msg->data_2;
	array[12] = crc8(&array[2],msg->_msg_lenght-3);

	generate_seq(array,msg->_seq,msg->_msg_lenght);

}

void esemble_seq_text(JETI_EX_TEXT * msg){

	int value_len = strlen(msg->label_value);
	int unit_len = strlen(msg->label_unit);
	msg->_msg_lenght = 8+3+unit_len+value_len; // viz datasheet s jednim datovym bytem, max 29

	uint8_t array[18];

	array[0] = EX_SEPARATOR;
	array[1] = EX_ID;
	array[2] = (msg->_msg_lenght-3);
	array[3] = (uint8_t)(msg->man_ID & 0x00FF);
	array[4] = (uint8_t)(msg->man_ID >> 8);
	array[5] = (uint8_t)(msg->dev_ID & 0x00FF);
	array[6] = (uint8_t)(msg->dev_ID  >> 8);
	array[7] = EX_NULL;
	array[8] = msg->identifier;

	array[9] = (value_len << 3) | unit_len;
	memcpy(&array[10],msg->label_value,value_len);
	memcpy(&array[10+value_len],msg->label_unit,unit_len);

	array[msg->_msg_lenght-1] = crc8(&array[2],msg->_msg_lenght-3);

	generate_seq(array,msg->_seq,msg->_msg_lenght);

}
