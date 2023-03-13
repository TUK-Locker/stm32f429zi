/*
 * ESP8266_HAL.h
 *
 *  Created on: Apr 14, 2020
 *      Author: Controllerstech
 */

#ifndef INC_ESP8266_HAL_H_
#define INC_ESP8266_HAL_H_


void ESP_Init (char *SSID, char *PASSWD);

void Server_Start (void);

extern unsigned char Elevator_Move_Flag; // 1일때만 사물함이 움직일 수 있음
extern unsigned char Elevator_Flag;
extern unsigned char Elevator_Floor; // 어떤 사물함이 움직일 것인지




#endif /* INC_ESP8266_HAL_H_ */
