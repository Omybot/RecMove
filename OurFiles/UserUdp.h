/************************************************
*
* UserUdp.h
* Fichier permettant de g�rer une communication UDP
* Couche interm�diaire entre le main et le fichier udp.c
*
*
*****************************************************/

#ifndef __USER_UDP__
#define __USER_UDP__

//#include "TCPIP Stack/UDP.h"
#include "TCPIP Stack/TCPIP.h"

typedef struct Trame
{
	BYTE* message;
	int nbChar;
} Trame;

#define UDP_ID 0xC1
#define portReception 12311
#define portEmission 12321

void InitUserUdp();
Trame ReceptionUserUdp();
void EnvoiUserUdp();
void EnvoiStringUdp(const char *string);
int HasClient();
void memclr(void * dest, WORD size);

#endif
