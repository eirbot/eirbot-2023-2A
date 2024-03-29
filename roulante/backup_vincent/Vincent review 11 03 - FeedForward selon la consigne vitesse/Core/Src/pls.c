/*
 * pls.c
 *
 *  Created on: May 12, 2022
 *      Author: lucas
 */

#include "pls.h"
//#include "usart.h"

UART_HandleTypeDef* huart_pls;

void send_byte(uint8_t ch) {
	HAL_UART_Transmit(huart_pls, &ch, 1, 0xFFFF);
}

// init pointer to UART handle
void pls_init(UART_HandleTypeDef *huart) {
	huart_pls = huart;
}

uint8_t wait_cmd(int16_t params[], uint8_t* param_size) {
  uint8_t cmd_buffer[MAX_CMD_BUF];
  uint8_t header, id, type, n_var;

  // receive header
  if( HAL_UART_Receive(huart_pls, cmd_buffer, 1, PLS_TIMEOUT) != HAL_OK ) {
	  return 0xFF;
  }

  // parse header
  header = cmd_buffer[0];
  id = (header & ID_MASK) >> 4;
  type = (header & TYPE_MASK) >> 3;
  n_var = (header & NCMD_MASK);

  // check header
  if( n_var <= MAX_PARAM && type <= 1 && id <= 6 ) {

	  // no parameters
	  if( n_var == 0 ) {
		  send_byte(KMS);
		  *param_size = 0;
		  return id;
	  }

	  // receive parameters
	  if( HAL_UART_Receive(huart_pls, cmd_buffer+1, (type+1)*2*n_var, PLS_TIMEOUT) != HAL_OK ) {
		  return 0xFF;
	  }

	  // parse parameters and update param_size variable
	  // !!! only int16 for the moment
	  uint8_t ind = 1; // temp buf index
	  for (uint8_t i = 0; i < n_var; i++) {
		  params[i] = (cmd_buffer[ind] << 8) + cmd_buffer[ind + 1];
		ind += 2;
	  }

	  *param_size = n_var;

	  // envoie confirmation réception
	  send_byte(KMS);

	  return id;
  }
  else {
	  //flush serial buffer
	  __HAL_UART_FLUSH_DRREGISTER(huart_pls);

	  send_byte(ERROR);
	  return 0xFF;
  }
}
