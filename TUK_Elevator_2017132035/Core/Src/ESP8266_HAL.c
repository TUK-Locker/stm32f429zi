/*
 * ESP8266_HAL.c
 *
 *  Created on: Apr 14, 2020
 *      Author: Controllerstech
 */


#include "UartRingbuffer_multi.h"
#include "ESP8266_HAL.h"
#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define wifi_uart &huart2
#define pc_uart &huart3


char buffer[20];




void ESP_Init (char *SSID, char *PASSWD)
{
   char data[80];

   Ringbuf_init();

   Uart_sendstring("AT+RST\r\n", wifi_uart);
   Uart_sendstring("RESETTING.", pc_uart);
   for (int i=0; i<5; i++)
   {
      Uart_sendstring(".", pc_uart);
      HAL_Delay(1000);
   }

   /********* AT **********/
   Uart_sendstring("AT\r\n", wifi_uart);
   while(!(Wait_for("AT\r\r\n\r\nOK\r\n", wifi_uart)));
   Uart_sendstring("AT---->OK\n\n", pc_uart);


   /********* AT+CWMODE=1 **********/
   Uart_sendstring("AT+CWMODE=1\r\n", wifi_uart);
   while (!(Wait_for("AT+CWMODE=1\r\r\n\r\nOK\r\n", wifi_uart)));
   Uart_sendstring("CW MODE---->1\n\n", pc_uart);



   /********* static ip***********/
   Uart_sendstring("AT+CIPSTA_CUR=\"192.168.0.224\",\"255.255.255.0\",\"192.168.0.1\"\r\n",wifi_uart);
   while (!(Wait_for("AT+CIPSTA_CUR=\"192.168.0.224\",\"255.255.255.0\",\"192.168.0.1\"\r\r\n\r\nOK\r\n", wifi_uart)));
   Uart_sendstring("Static ip OK\r\n\n", pc_uart);

   /********* AT+CWJAP="SSID","PASSWD" **********/
   Uart_sendstring("connecting... to the provided AP\n", pc_uart);
   sprintf (data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
   Uart_sendstring(data, wifi_uart);
   while (!(Wait_for("WIFI GOT IP\r\n\r\nOK\r\n", wifi_uart)));
   sprintf (data, "Connected to,\"%s\"\n\n", SSID);
   Uart_sendstring(data,pc_uart);



   /********* AT+CIFSR **********/
   Uart_sendstring("AT+CIFSR\r\n", wifi_uart);
   while (!(Wait_for("CIFSR:STAIP,\"", wifi_uart)));
   while (!(Copy_upto("\"",buffer, wifi_uart)));
   while (!(Wait_for("OK\r\n", wifi_uart)));
   int len = strlen (buffer);
   buffer[len-1] = '\0';
   sprintf (data, "IP ADDR: %s\n\n", buffer);
   Uart_sendstring(data, pc_uart);


   Uart_sendstring("AT+CIPMUX=1\r\n", wifi_uart);
   while (!(Wait_for("AT+CIPMUX=1\r\r\n\r\nOK\r\n", wifi_uart)));
   Uart_sendstring("CIPMUX---->OK\n\n", pc_uart);

   Uart_sendstring("AT+CIPSERVER=1,80\r\n", wifi_uart);
   while (!(Wait_for("OK\r\n", wifi_uart)));
   Uart_sendstring("CIPSERVER---->OK\n\n", pc_uart);

   Uart_sendstring("Now Connect to the IP ADRESS\n\n", pc_uart);
}





