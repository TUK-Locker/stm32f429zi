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


char *Basic_inclusion = "<!DOCTYPE html> <html>\n<head><meta name=\"viewport\"\
		content=\"width=device-width, initial-scale=2.0, user-scalable=no\">\n\
		<title>TUK_LOCKER</title>\n</head>\n<body>\n<h1>TUK_Locker is god</h1>\n</body></html>";

char *lockeroncall = "<!DOCTYPE html> <html>\n<head><meta name=\"viewport\"\
		content=\"width=device-width, initial-scale=2.0, user-scalable=no\">\n\
		<title>TUK_LOCKER_IN1</title>\n</head>\n<body>\n<h1>IN1</h1>\n</body></html>";

char *lockeron = "<!DOCTYPE html> <html>\n<head><meta name=\"viewport\"\
		content=\"width=device-width, initial-scale=2.0, user-scalable=no\">\n\
		<title>TUK_LOCKER_IN2</title>\n</head>\n<body>\n<h1>IN2</h1>\n</body></html>";

char *lockeroffcall = "<!DOCTYPE html> <html>\n<head><meta name=\"viewport\"\
		content=\"width=device-width, initial-scale=2.0, user-scalable=no\">\n\
		<title>TUK_LOCKER_EX1</title>\n</head>\n<body>\n<h1>EX1</h1>\n</body></html>";

char *lockeroff = "<!DOCTYPE html> <html>\n<head><meta name=\"viewport\"\
		content=\"width=device-width, initial-scale=2.0, user-scalable=no\">\n\
		<title>TUK_LOCKER_EX2</title>\n</head>\n<body>\n<h1>EX2</h1>\n</body></html>";


/*****************************************************************************************************************************************/

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
   Uart_sendstring("AT+CIPSTA_CUR=\"192.168.0.222\",\"255.255.255.0\",\"192.168.0.1\"\r\n",wifi_uart);
   while (!(Wait_for("AT+CIPSTA_CUR=\"192.168.0.222\",\"255.255.255.0\",\"192.168.0.1\"\r\r\n\r\nOK\r\n", wifi_uart)));
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






int Server_Send (char *str, int Link_ID)
{
	int len = strlen (str);
	char data[80];
	sprintf (data, "AT+CIPSEND=%d,%d\r\n", Link_ID, len);
	Uart_sendstring(data, wifi_uart);
	while (!(Wait_for(">", wifi_uart)));
	Uart_sendstring (str, wifi_uart);
	while (!(Wait_for("SEND OK", wifi_uart)));
	sprintf (data, "AT+CIPCLOSE=5\r\n");
	Uart_sendstring(data, wifi_uart);
	while (!(Wait_for("OK\r\n", wifi_uart)));
	return 1;
}

void Server_Handle (char *str, int Link_ID)
{
	char datatosend[1024] = {0};
	if (!(strcmp (str, "/lockeroncall")))
	{
		sprintf (datatosend, lockeroncall);
		Server_Send(datatosend, Link_ID);
	}
	else if (!(strcmp (str, "/lockeron")))
	{
		sprintf (datatosend, lockeron);
		Server_Send(datatosend, Link_ID);
	}
	else if (!(strcmp (str, "/lockeroffcall")))
	{
		sprintf (datatosend, lockeroffcall);
		Server_Send(datatosend, Link_ID);
	}
	else if (!(strcmp (str, "/lockeroff")))
	{
		sprintf (datatosend, lockeroff);
		Server_Send(datatosend, Link_ID);
	}
	else
	{
		sprintf (datatosend, Basic_inclusion);
		Server_Send(datatosend, Link_ID);
	}

}


void Server_Start (void)
{
	char buftocopyinto[64] = {0};
	char Link_ID;




	while (!(Get_after("+IPD,", 1, &Link_ID, wifi_uart)));
	Link_ID -= 48;
	while (!(Copy_upto(" HTTP/1.1", buftocopyinto, wifi_uart)));


	if (Look_for("/lockeron", buftocopyinto) == 1 )
	{

		if(Look_for("/lockeroncall1", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][0]=1;
			Locker_Move_Flag=1;
			Server_Handle("/lockeroncall",Link_ID);
		}
		else if(Look_for("/lockeron1", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][0]=2;
			Locker_Move_Flag=1;
			Server_Handle("/lockeron",Link_ID);
		}


		else if(Look_for("/lockeroncall2", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][1]=1;
			Locker_Move_Flag=2;
			Server_Handle("/lockeroncall",Link_ID);
		}
		else if(Look_for("/lockeron2", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][1]=2;
			Locker_Move_Flag=2;
			Server_Handle("/lockeron",Link_ID);
		}

		else if(Look_for("/lockeroncall3", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][2]=1;
			Locker_Move_Flag=3;
			Server_Handle("/lockeroncall",Link_ID);
		}
		else if(Look_for("/lockeron3", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][2]=2;
			Locker_Move_Flag=3;
			Server_Handle("/lockeron",Link_ID);
		}

		else if(Look_for("/lockeroncall4", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][3]=1;
			Locker_Move_Flag=4;
			Server_Handle("/lockeroncall",Link_ID);
		}
		else if(Look_for("/lockeron4", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][3]=2;
			Locker_Move_Flag=4;
			Server_Handle("/lockeron",Link_ID);
		}

		else if(Look_for("/lockeroncall5", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][0]=1;
			Locker_Move_Flag=5;
			Server_Handle("/lockeroncall",Link_ID);
		}
		else if(Look_for("/lockeron5", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][0]=2;
			Locker_Move_Flag=5;
			Server_Handle("/lockeron",Link_ID);
		}

		else if(Look_for("/lockeroncall6", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][1]=1;
			Locker_Move_Flag=6;
			Server_Handle("/lockeroncall",Link_ID);
		}
		else if(Look_for("/lockeron6", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][1]=2;
			Locker_Move_Flag=6;
			Server_Handle("/lockeron",Link_ID);
		}

		else if(Look_for("/lockeroncall7", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][2]=1;
			Locker_Move_Flag=7;
			Server_Handle("/lockeroncall",Link_ID);
		}
		else if(Look_for("/lockeron7", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][2]=2;
			Locker_Move_Flag=7;
			Server_Handle("/lockeron",Link_ID);
		}

		else if(Look_for("/lockeroncall8", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][3]=1;
			Locker_Move_Flag=8;
			Server_Handle("/lockeroncall",Link_ID);
		}
		else if(Look_for("/lockeron8", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][3]=2;
			Locker_Move_Flag=8;
			Server_Handle("/lockeron",Link_ID);
		}

		else
		{
			Server_Handle("/ ", Link_ID);
			return;
		}


	}

	else if (Look_for("/lockeroff", buftocopyinto) == 1)
	{


		if(Look_for("/lockeroffcall1", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][0]=3;
			Locker_Move_Flag=1;
			Server_Handle("/lockeroffcall",Link_ID);
		}
		else if(Look_for("/lockeroff1", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][0]=4;
			Locker_Move_Flag=1;
			Server_Handle("/lockeroff",Link_ID);
		}


		else if(Look_for("/lockeroffcall2", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][1]=3;
			Locker_Move_Flag=2;
			Server_Handle("/lockeroffcall",Link_ID);
		}
		else if(Look_for("/lockeroff2", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][1]=4;
			Locker_Move_Flag=2;
			Server_Handle("/lockeroff",Link_ID);
		}

		else if(Look_for("/lockeroffcall3", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][2]=3;
			Locker_Move_Flag=3;
			Server_Handle("/lockeroffcall",Link_ID);
		}
		else if(Look_for("/lockeroff3", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][2]=4;
			Locker_Move_Flag=3;
			Server_Handle("/lockeroff",Link_ID);
		}

		else if(Look_for("/lockeroffcall4", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][3]=3;
			Locker_Move_Flag=4;
			Server_Handle("/lockeroffcall",Link_ID);
		}
		else if(Look_for("/lockeroff4", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[0][3]=4;
			Locker_Move_Flag=4;
			Server_Handle("/lockeroff",Link_ID);
		}

		else if(Look_for("/lockeroffcall5", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][0]=3;
			Locker_Move_Flag=5;
			Server_Handle("/lockeroffcall",Link_ID);
		}
		else if(Look_for("/lockeroff5", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][0]=4;
			Locker_Move_Flag=5;
			Server_Handle("/lockeroff",Link_ID);
		}

		else if(Look_for("/lockeroffcall6", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][1]=3;
			Locker_Move_Flag=6;
			Server_Handle("/lockeroffcall",Link_ID);
		}
		else if(Look_for("/lockeroff6", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][1]=4;
			Locker_Move_Flag=6;
			Server_Handle("/lockeroff",Link_ID);
		}

		else if(Look_for("/lockeroffcall7", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][2]=3;
			Locker_Move_Flag=7;
			Server_Handle("/lockeroffcall",Link_ID);
		}
		else if(Look_for("/lockeroff7", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][2]=4;
			Locker_Move_Flag=7;
			Server_Handle("/lockeroff",Link_ID);
		}

		else if(Look_for("/lockeroffcall8", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][3]=3;
			Locker_Move_Flag=8;
			Server_Handle("/lockeroffcall",Link_ID);
		}
		else if(Look_for("/lockeroff8", buftocopyinto) == 1)
		{
			if(Locker_Move_Flag==0)
				Locker_Flag[1][3]=4;
			Locker_Move_Flag=8;
			Server_Handle("/lockeroff",Link_ID);
		}


		else
		{
			Server_Handle("/ ", Link_ID);
			return;
		}


	}
	else if (Look_for("/favicon.ico", buftocopyinto) == 1);

	else
		Server_Handle("/ ", Link_ID);

}
