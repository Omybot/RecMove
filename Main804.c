#define THIS_IS_STACK_APPLICATION
#include "TCPIP Stack/TCPIP.h"
#include "TCPIP Stack/UDPPerformanceTest.h"
#include "Main804.h"
#include "OurFiles/UserUdp.h"
#include "OurFiles/asser.h"
#include "OurFiles/init.h"
#include <stdio.h>
#include <stdlib.h>
#include <uart.h>
#include <math.h>
#include <p33FJ128MC804.h>
#include "OurFiles/Pilotage.h"
#include "OurFiles/CDS5516.h"
#include "OurFiles/FonctionsUc.h"

#define UART_BUFFER_SIZE	100
#define IDCAPTEUR_BAS 0
#define IDCAPTEUR_HAUT 1

// Bits configuration
_FOSCSEL(FNOSC_FRC)
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON)
_FPOR(FPWRT_PWR1)
_FWDT(FWDTEN_OFF)
_FICD(ICS_PGD3 & JTAGEN_OFF)

APP_CONFIG AppConfig;

static void InitAppConfig(void);

extern double pos_x,pos_y,pos_teta;

unsigned int periode_tour;
char Fin_de_tour, Fin_d_angle_bas = 0,Fin_d_angle_haut = 0;
char capteurHautPrec = 0;
char capteurBasPrec = 0;
char oldMagnet = 0;
unsigned char nombre_angles[2]; // n angles (8 maxi)
unsigned char buffer_angles[2][2*8]; // debut_1_MSB; debut_1_LSB; fin_1_MSB; fin_1_LSB; debut_2_MSB; debut_2_LSB; fin_2_MSB; fin_2_LSB; ...;debut_n_MSB; debut_n_LSB; fin_n_MSB; fin_n_LSB; 
unsigned char nombre_fronts[2]; // n fronts (8 maxi)
unsigned int buffer_fronts[2][8]; // Valeurs bruts TMR2
unsigned int buffer_fronts_temp[2][8]; // Valeurs bruts TMR2
unsigned int front_rapide[2][2];
unsigned int ptr_fronts_bas; // ptr pour l'enregistrement
unsigned int ptr_fronts_haut; // 
unsigned int hall_front;
unsigned int motor_speed,Cpt_20ms=0;
float angle;

unsigned int IR_result;

unsigned int cpt_balise,pwm_balise;
extern unsigned int ADC_Results[8],cpu_status;
extern double cons_pos[N];
extern double real_pos[N];
extern unsigned char scan;
unsigned char flag_envoi_position;
unsigned int prd_envoi_position = 100;
unsigned char jackAvant = 0;
unsigned char motor_flag=0,datalogger_blocker=0;
double position_lock;
unsigned int datalogger_counter=0,flag=0,courrier=0,PID_ressource_used;
unsigned char flag_envoi=0,flag_blocage=0,flag_calage=0;

//LIDAR
unsigned char Demande_lidar = 0,offset_premiere_trame=0;
unsigned char timeout_lidar=0;
unsigned char nbr_char_to_send=0,Buffer_passerelle_udpuart[250];
unsigned char ptr_write_buffer_uart_rec=0,save_write;
unsigned char ptr_read_buffer_uart_rec=0,save_read;
unsigned char flag_envoi_uart,buffer_envoi_uart[UART_BUFFER_SIZE],ptr_write_buffer_uart;
unsigned char ptr_read_buffer_uart=0;


void _ISR __attribute__((__no_auto_psv__)) _AddressError(void)
{
    Nop();
	Nop();
}
void _ISR __attribute__((__no_auto_psv__)) _StackError(void)
{
    Nop();
	Nop();
}

int main(void)
{
	unsigned char i;
	unsigned char etatCouleur = 2;
	static DWORD dwLastIP = 0;
	
	Trame trame;		

	Trame Jack;
	static BYTE Presence[2];
	Jack.nbChar = 2;
	Presence[0] = UDP_ID;
	Presence[1] = CMD_DEPART_JACK;
	Jack.message = Presence;

	Trame Couleur_Equipe;
	static BYTE Couleur[3];
	Couleur_Equipe.nbChar = 3;
	Couleur[0] = UDP_ID;
	Couleur[1] = CMD_REPONSE_COULEUR_EQUIPE;
	Couleur[2] = PORTAbits.RA7;
	Couleur_Equipe.message = Couleur;

	Trame envoiFin;
	static BYTE mess[2];
	mess[0] = UDP_ID;
	mess[1] = CMD_FINDEPLACEMENT;
	envoiFin.message = mess;
	envoiFin.nbChar = 2;
		
	Trame envoiBlocage;
	static BYTE messblocage[2];
	messblocage[0] = UDP_ID;
	messblocage[1] = 0x13;
	envoiBlocage.message = messblocage;
	envoiBlocage.nbChar = 2;
	
	Trame envoiCalage;
	static BYTE messcalage[2];
	messcalage[0] = UDP_ID;
	messcalage[1] = CMD_FINRECALLAGE;
	envoiCalage.message = messcalage;
	envoiCalage.nbChar = 2;

	Trame envoiUART;
	static BYTE messUART[250];
	messUART[0] = UDP_ID;
	messUART[1] = TRAME_UART2_RECEPTION;
	messUART[2] = ID_HOKUYO_SOL;
	envoiUART.message = messUART;
	envoiUART.nbChar = 53;
	
	
	Trame envoiTest;
	static BYTE messTest[19];
	messTest[0] = UDP_ID;
	messTest[1] = UDP_ID;
	messTest[2] = 7;
	messTest[3] = 'M';
	messTest[4] = 'S';
	messTest[5] = '0';
	messTest[6] = '0';
	messTest[7] = '0';
	messTest[8] = '0';
	messTest[9] = '0';
	messTest[10] = '7';
	messTest[11] = '2';
	messTest[12] = '5';
	messTest[13] = '0';
	messTest[14] = '0';
	messTest[15] = '0';
	messTest[16] = '0';
	messTest[17] = '1';
	messTest[18] = '\n';

	//MS0000072500001
	
	envoiTest.message = messTest;
	envoiTest.nbChar = 19;
	// V V [LF] 0 0 P [LF] 
	
		Trame envoiBalise;
	static BYTE messbalise[60];
	messbalise[0] = UDP_ID;
	messbalise[1] = TRAME_DETECTION_BALISE;
	messbalise[2] = ID_BALISE;
	envoiBalise.message = messbalise;
	envoiBalise.nbChar = 30;

	Trame envoiBaliserapide;
	static BYTE messbaliserapide[7];
	messbaliserapide[0] = UDP_ID;
	messbaliserapide[1] = TRAME_DETECTION_BALISE_RAPIDE;
	envoiBaliserapide.message = messbaliserapide;
	envoiBaliserapide.nbChar = 7;


	InitClk(); 		// Initialisation de l'horloge
	InitPorts(); 	// Initialisation des ports E/S
    MOT1L = 0;
	MOT1H = 0;
	MOT2L = 0;
	MOT2H = 0;
	MOT3L = 0;
	MOT3H = 0;
	MOT4L = 0;
	MOT4H = 0;
	
	LATAbits.LATA8 = 0; // (moteur balise)

	Init_Timer2();	// Initialisation Timer2
	Init_Timer4();	// Initialisation Timer4
	Init_Timer5();
	InitQEI(); 		// Initialisation des entrées en quadrature
	InitPWM();		// Configuration du module PWM 
        
	InitProp();
	
	// Initialize stack-related hardware components that may be 
	// required by the UART configuration routines
    TickInit();
	#if defined(STACK_USE_MPFS) || defined(STACK_USE_MPFS2)
		MPFSInit();
	#endif
	
	// Initialize Stack and application related NV variables into AppConfig.
	InitAppConfig();
	
	UDPInit();
    StackInit();	
 	UDPPerformanceTask();
	InitUserUdp();
	
	Init_Interrupt_Priority();							
	InitUART2();	
	InitADC();
	InitDMA();

	// Init Timer3
	T3CONbits.TCKPS = 0b11;	// 1:256 Prescaler
	PR3 = 0xFFFF;			// Time to autoreload
	IFS0bits.T3IF = 0;		// Interrupt flag cleared
	IEC0bits.T3IE = 0;		// Interrupt disabled
	T3CONbits.TON = 1;		// Timer enabled

		// Initialize the Input Capture Module
	IC1CONbits.ICM = 0b00; // Disable Input Capture 1 module
	IC1CONbits.ICTMR = 0; // Select Timer3 as the IC1 Time base
	IC1CONbits.ICI = 0b00; // Interrupt on every capture event
	IC1CONbits.ICM = 0b011; // Generate capture event on every Rising edge
	// Enable Capture Interrupt And Timer3
	IPC0bits.IC1IP = 3; // Setup IC1 interrupt priority level
	IFS0bits.IC1IF = 0; // Clear IC1 Interrupt Status Flag
	IEC0bits.IC1IE = 1; // Enable IC1 interrupt
	// Initialize the Input Capture Module
	IC2CONbits.ICM = 0b00; // Disable Input Capture 1 module
	IC2CONbits.ICTMR = 0; // Select Timer3 as the IC1 Time base
	IC2CONbits.ICI = 0b00; // Interrupt on every capture event
	IC2CONbits.ICM = 0b011; // Generate capture event on every edge // change
	// Enable Capture Interrupt And Timer3
	IPC1bits.IC2IP = 2; // Setup IC1 interrupt priority level
	IFS0bits.IC2IF = 0; // Clear IC1 Interrupt Status Flag
	IEC0bits.IC2IE = 1; // Enable IC1 interrupt
	// Initialize the Input Capture Module
	IC7CONbits.ICM = 0b00; // Disable Input Capture 1 module
	IC7CONbits.ICTMR = 0; // Select Timer3 as the IC1 Time base
	IC7CONbits.ICI = 0b00; // Interrupt on every capture event
	IC7CONbits.ICM = 0b011; // Generate capture event on every edge // change
	// Enable Capture Interrupt And Timer3
	IPC5bits.IC7IP = 1; // Setup IC1 interrupt priority level
	IFS1bits.IC7IF = 0; // Clear IC1 Interrupt Status Flag
	IEC1bits.IC7IE = 1; // Enable IC1 interrupt


	DelayMs(500); 

	

	/*while(1)
  	{	
    	pwm(MOTEUR_4,4000);		// Moteur benne
	}*/
	while(1)
	{
		// Gestion Balise
		if(Fin_d_angle_bas == 1)
		{
			Fin_d_angle_bas = 0;
			messbaliserapide[0] = UDP_ID;
			messbaliserapide[1] = CMD_REPONSE_CAPTEUR;
			messbaliserapide[2] = ID_CAPTEUR_BALISE_1;

			envoiBaliserapide.nbChar = 3;
			for(i=0;i<2;i++)
			{
				angle = (float)(front_rapide[IDCAPTEUR_BAS][i]) / (float)(periode_tour) * 36000;
				messbaliserapide[envoiBaliserapide.nbChar+i*2] = (unsigned char)((unsigned int)angle >> 8) & 0xFF;
				messbaliserapide[envoiBaliserapide.nbChar+1+i*2] = (unsigned char)((unsigned int)angle     ) & 0xFF;
			}
			envoiBaliserapide.nbChar = 7;	
			//EnvoiUserUdp(envoiBaliserapide);
		}

		if(Fin_d_angle_haut == 1)
		{
			Fin_d_angle_haut = 0;
			messbaliserapide[0] = UDP_ID;
			messbaliserapide[1] = CMD_REPONSE_CAPTEUR;
			messbaliserapide[2] = ID_CAPTEUR_BALISE_2;

			envoiBaliserapide.nbChar = 3;
			for(i=0;i<2;i++)
			{
				angle = (float)(front_rapide[IDCAPTEUR_HAUT][i]) / (float)(periode_tour) * 36000;
				messbaliserapide[envoiBaliserapide.nbChar+i*2] = (unsigned char)((unsigned int)angle >> 8) & 0xFF;
				messbaliserapide[envoiBaliserapide.nbChar+1+i*2] = (unsigned char)((unsigned int)angle     ) & 0xFF;
			}
			envoiBaliserapide.nbChar = 7;	
			//EnvoiUserUdp(envoiBaliserapide);
		}


		// Gestion Balise
		if(Fin_de_tour == 1)
		{
			Fin_de_tour = 0;
			
			for(i=0;i<nombre_angles[IDCAPTEUR_HAUT];i++)
			{
				angle = (float)(buffer_fronts[IDCAPTEUR_HAUT][i]) / (float)(periode_tour) * 36000;
				buffer_angles[IDCAPTEUR_HAUT][2*i]   = (unsigned char)((unsigned int)angle >> 8) & 0xFF;
				buffer_angles[IDCAPTEUR_HAUT][2*i+1] = (unsigned char)((unsigned int)angle     ) & 0xFF;
			}
			for(i=0;i<nombre_angles[IDCAPTEUR_BAS];i++)
			{
				angle = (float)(buffer_fronts[IDCAPTEUR_BAS][i]) / (float)(periode_tour) * 36000;
				buffer_angles[IDCAPTEUR_BAS][2*i]   = (unsigned char)((unsigned int)angle >> 8) & 0xFF;
				buffer_angles[IDCAPTEUR_BAS][2*i+1] = (unsigned char)((unsigned int)angle     ) & 0xFF;
			}

			messbalise[0] = UDP_ID;
			messbalise[1] = TRAME_DETECTION_BALISE;
			messbalise[2] = ID_BALISE;
			
			//messbalise[3] = (int)(pos_x * 10)>>8;
			//messbalise[4] = (int)(pos_x * 10)&0x00FF;
			//messbalise[5] = (int)(pos_y * 10)>>8;
			//messbalise[6] = (int)(pos_y * 10)&0x00FF;
			//messbalise[7] = (unsigned int)(pos_teta*36000/(2*PI)+18000)>>8;
			//messbalise[8] = (unsigned int)(pos_teta*36000/(2*PI)+18000)&0x00FF;

			messbalise[2+1+0] = (periode_tour >> 8) & 0x00FF;
			messbalise[3+1+0] = periode_tour & 0x00FF;
			
			
			messbalise[4+1+0] = nombre_angles[IDCAPTEUR_HAUT];
			messbalise[5+1+0] = nombre_angles[IDCAPTEUR_BAS];
       		
			envoiBalise.nbChar = 6+1+0;
			
       		for(i = 0; i < nombre_angles[IDCAPTEUR_HAUT]; i++)
			{
				messbalise[envoiBalise.nbChar+i*2] = buffer_angles[IDCAPTEUR_HAUT][2*i];	//MSB
       			messbalise[envoiBalise.nbChar+1+i*2] = buffer_angles[IDCAPTEUR_HAUT][2*i+1]; //LSB
			}
			
			envoiBalise.nbChar += nombre_angles[IDCAPTEUR_HAUT]*2;
			
			for(i = 0; i < nombre_angles[IDCAPTEUR_BAS]; i++)
			{
				messbalise[envoiBalise.nbChar+i*2] = buffer_angles[IDCAPTEUR_BAS][2*i];	 //MSB
       			messbalise[envoiBalise.nbChar+1+i*2] = buffer_angles[IDCAPTEUR_BAS][2*i+1]; //LSB
			}
			
			envoiBalise.nbChar += nombre_angles[IDCAPTEUR_BAS]*2;
			
			EnvoiUserUdp(envoiBalise);
			envoiBalise.nbChar = 0;
		}
		// Fin gestion balise
		//Fin Gestion LIDAR	
		if(Demande_lidar)	
		{
			Demande_lidar=0;
			EnvoiUART(envoiTest);
			messUART[3] = (int)(pos_x * 10)>>8;
			messUART[4] = (int)(pos_x * 10)&0x00FF;
			messUART[5] = (int)(pos_y * 10)>>8;
			messUART[6] = (int)(pos_y * 10)&0x00FF;
			messUART[7] = (unsigned int)(pos_teta*36000/(2*PI)+18000)>>8;
			messUART[8] = (unsigned int)(pos_teta*36000/(2*PI)+18000)&0x00FF;
			offset_premiere_trame=6;
		}
		
		save_write = ptr_write_buffer_uart_rec;
		save_read = ptr_read_buffer_uart_rec;
		if((save_write != save_read))
		{
			if(save_write < save_read)
				nbr_char_to_send = 241 - save_read + save_write;
			else
				nbr_char_to_send = save_write - save_read;
		}
		else
		{
			nbr_char_to_send = 0;
		}	

		if(((nbr_char_to_send > 200) || (nbr_char_to_send !=0 && timeout_lidar > 50)))
		{	
			for(i=0;i<nbr_char_to_send;i++)
			{
				messUART[i+3+offset_premiere_trame]=Buffer_passerelle_udpuart[save_read++];
				if(save_read>240)
					save_read=0;
			}
			
			timeout_lidar=0;
			envoiUART.nbChar = nbr_char_to_send+3+offset_premiere_trame;
			offset_premiere_trame=0;
			EnvoiUserUdp(envoiUART); // Bon ok 1 pt pour kryss
			ptr_read_buffer_uart_rec = save_write;
		}			

		if((ptr_write_buffer_uart != ptr_read_buffer_uart) && U2STAbits.TRMT != 0)
		{
			// Gestion envoi trame
			U2TXREG = buffer_envoi_uart[ptr_read_buffer_uart++];
			if(ptr_read_buffer_uart >= UART_BUFFER_SIZE)
				ptr_read_buffer_uart=0;
		}
		//Fin Gestion LIDAR	
	  	if(PORTAbits.RA10 && jackAvant)
	  	{
		  	EnvoiUserUdp (Jack);
		  	jackAvant = 0;
		}
		if(etatCouleur != PORTAbits.RA7)
		{
			Couleur[2] = PORTAbits.RA7;
  			EnvoiUserUdp (Couleur_Equipe);
  			etatCouleur = PORTAbits.RA7;
  		}
		if(flag_envoi) 
		{	
			scan=0;
			EnvoiUserUdp(envoiFin);
			flag_envoi = 0;
		}
		if(flag_blocage)
		{
			EnvoiUserUdp(envoiBlocage);
			flag_blocage = 0;
		}
		if(flag_calage)
		{
			EnvoiUserUdp(envoiCalage);
			flag_calage = 0;
		}
		if(flag_envoi_position)
		{
			EnvoiUserUdp(PilotePositionXYT());
			flag_envoi_position = 0;
		}
		

		StackTask();
		trame = ReceptionUserUdp();
		if(trame.nbChar != 0)
		{
			trame = AnalyseTrame(trame);
			EnvoiUserUdp(trame);
		}
        StackApplications();

		if(dwLastIP != AppConfig.MyIPAddr.Val)
		{
			dwLastIP = AppConfig.MyIPAddr.Val;
			
			#if defined(STACK_USE_UART)
				putrsUART((ROM char*)"\r\nNew IP Address: ");
			#endif

			DisplayIPValue(AppConfig.MyIPAddr);

			#if defined(STACK_USE_UART)
				putrsUART((ROM char*)"\r\n");
			#endif


			#if defined(STACK_USE_ANNOUNCE)
				AnnounceIP();
			#endif
		}
	}
}

// Writes an IP address to the LCD display and the UART as available
void DisplayIPValue(IP_ADDR IPVal)
{
//	printf("%u.%u.%u.%u", IPVal.v[0], IPVal.v[1], IPVal.v[2], IPVal.v[3]);
    BYTE IPDigit[4];
	BYTE i;
#ifdef USE_LCD
	BYTE j;
	BYTE LCDPos=16;
#endif

	for(i = 0; i < sizeof(IP_ADDR); i++)
	{
	    uitoa((WORD)IPVal.v[i], IPDigit);

		#if defined(STACK_USE_UART)
			putsUART(IPDigit);
		#endif

		#ifdef USE_LCD
			for(j = 0; j < strlen((char*)IPDigit); j++)
			{
				LCDText[LCDPos++] = IPDigit[j];
			}
			if(i == sizeof(IP_ADDR)-1)
				break;
			LCDText[LCDPos++] = '.';
		#else
			if(i == sizeof(IP_ADDR)-1)
				break;
		#endif

		#if defined(STACK_USE_UART)
			while(BusyUART());
			WriteUART('.');
		#endif
	}

	#ifdef USE_LCD
		if(LCDPos < 32u)
			LCDText[LCDPos] = 0;
		LCDUpdate();
	#endif
}

//#pragma romdata MACROM=0x1FFF0
static ROM BYTE SerializedMACAddress[6] = {MY_DEFAULT_MAC_BYTE1, MY_DEFAULT_MAC_BYTE2, MY_DEFAULT_MAC_BYTE3, MY_DEFAULT_MAC_BYTE4, MY_DEFAULT_MAC_BYTE5, MY_DEFAULT_MAC_BYTE6};
//#pragma romdata

static void InitAppConfig(void)
{
	AppConfig.Flags.bIsDHCPEnabled = TRUE;
	AppConfig.Flags.bInConfigMode = TRUE;
	memcpypgm2ram((void*)&AppConfig.MyMACAddr, (ROM void*)SerializedMACAddress, sizeof(AppConfig.MyMACAddr));
//	{
//		_prog_addressT MACAddressAddress;
//		MACAddressAddress.next = 0x157F8;
//		_memcpy_p2d24((char*)&AppConfig.MyMACAddr, MACAddressAddress, sizeof(AppConfig.MyMACAddr));
//	}
	AppConfig.MyIPAddr.Val = MY_DEFAULT_IP_ADDR_BYTE1 | MY_DEFAULT_IP_ADDR_BYTE2<<8ul | MY_DEFAULT_IP_ADDR_BYTE3<<16ul | MY_DEFAULT_IP_ADDR_BYTE4<<24ul;
	AppConfig.DefaultIPAddr.Val = AppConfig.MyIPAddr.Val;
	AppConfig.MyMask.Val = MY_DEFAULT_MASK_BYTE1 | MY_DEFAULT_MASK_BYTE2<<8ul | MY_DEFAULT_MASK_BYTE3<<16ul | MY_DEFAULT_MASK_BYTE4<<24ul;
	AppConfig.DefaultMask.Val = AppConfig.MyMask.Val;
	AppConfig.MyGateway.Val = MY_DEFAULT_GATE_BYTE1 | MY_DEFAULT_GATE_BYTE2<<8ul | MY_DEFAULT_GATE_BYTE3<<16ul | MY_DEFAULT_GATE_BYTE4<<24ul;
	AppConfig.PrimaryDNSServer.Val = MY_DEFAULT_PRIMARY_DNS_BYTE1 | MY_DEFAULT_PRIMARY_DNS_BYTE2<<8ul  | MY_DEFAULT_PRIMARY_DNS_BYTE3<<16ul  | MY_DEFAULT_PRIMARY_DNS_BYTE4<<24ul;
	AppConfig.SecondaryDNSServer.Val = MY_DEFAULT_SECONDARY_DNS_BYTE1 | MY_DEFAULT_SECONDARY_DNS_BYTE2<<8ul  | MY_DEFAULT_SECONDARY_DNS_BYTE3<<16ul  | MY_DEFAULT_SECONDARY_DNS_BYTE4<<24ul;


	// SNMP Community String configuration
	#if defined(STACK_USE_SNMP_SERVER)
	{
		BYTE i;
		static ROM char * ROM cReadCommunities[] = SNMP_READ_COMMUNITIES;
		static ROM char * ROM cWriteCommunities[] = SNMP_WRITE_COMMUNITIES;
		ROM char * strCommunity;
		
		for(i = 0; i < SNMP_MAX_COMMUNITY_SUPPORT; i++)
		{
			// Get a pointer to the next community string
			strCommunity = cReadCommunities[i];
			if(i >= sizeof(cReadCommunities)/sizeof(cReadCommunities[0]))
				strCommunity = "";

			// Ensure we don't buffer overflow.  If your code gets stuck here, 
			// it means your SNMP_COMMUNITY_MAX_LEN definition in TCPIPConfig.h 
			// is either too small or one of your community string lengths 
			// (SNMP_READ_COMMUNITIES) are too large.  Fix either.
			if(strlenpgm(strCommunity) >= sizeof(AppConfig.readCommunity[0]))
				while(1);
			
			// Copy string into AppConfig
			strcpypgm2ram((char*)AppConfig.readCommunity[i], strCommunity);

			// Get a pointer to the next community string
			strCommunity = cWriteCommunities[i];
			if(i >= sizeof(cWriteCommunities)/sizeof(cWriteCommunities[0]))
				strCommunity = "";

			// Ensure we don't buffer overflow.  If your code gets stuck here, 
			// it means your SNMP_COMMUNITY_MAX_LEN definition in TCPIPConfig.h 
			// is either too small or one of your community string lengths 
			// (SNMP_WRITE_COMMUNITIES) are too large.  Fix either.
			if(strlenpgm(strCommunity) >= sizeof(AppConfig.writeCommunity[0]))
				while(1);

			// Copy string into AppConfig
			strcpypgm2ram((char*)AppConfig.writeCommunity[i], strCommunity);
		}
	}
	#endif

	// Load the default NetBIOS Host Name
	memcpypgm2ram(AppConfig.NetBIOSName, (ROM void*)MY_DEFAULT_HOST_NAME, 16);
	FormatNetBIOSName(AppConfig.NetBIOSName);

	#if defined(ZG_CS_TRIS)
		// Load the default SSID Name
		if (sizeof(MY_DEFAULT_SSID_NAME) > sizeof(AppConfig.MySSID))
		{
		    ZGSYS_DRIVER_ASSERT(5, (ROM char *)"AppConfig.MySSID[] too small.\n");
		}
		memcpypgm2ram(AppConfig.MySSID, (ROM void*)MY_DEFAULT_SSID_NAME, sizeof(MY_DEFAULT_SSID_NAME));
	#endif

	#if defined(EEPROM_CS_TRIS)
	{
		BYTE c;
		
	    // When a record is saved, first byte is written as 0x60 to indicate
	    // that a valid record was saved.  Note that older stack versions
		// used 0x57.  This change has been made to so old EEPROM contents
		// will get overwritten.  The AppConfig() structure has been changed,
		// resulting in parameter misalignment if still using old EEPROM
		// contents.
		XEEReadArray(0x0000, &c, 1);
	    if(c == 0x60u)
		    XEEReadArray(0x0001, (BYTE*)&AppConfig, sizeof(AppConfig));
	    else
	        SaveAppConfig();
	}
	#elif defined(SPIFLASH_CS_TRIS)
	{
		BYTE c;
		
		SPIFlashReadArray(0x0000, &c, 1);
		if(c == 0x60u)
			SPIFlashReadArray(0x0001, (BYTE*)&AppConfig, sizeof(AppConfig));
		else
			SaveAppConfig();
	}
	#endif
}

#if defined(EEPROM_CS_TRIS) || defined(SPIFLASH_CS_TRIS)
void SaveAppConfig(void)
{
	// Ensure adequate space has been reserved in non-volatile storage to 
	// store the entire AppConfig structure.  If you get stuck in this while(1) 
	// trap, it means you have a design time misconfiguration in TCPIPConfig.h.
	// You must increase MPFS_RESERVE_BLOCK to allocate more space.
	#if defined(STACK_USE_MPFS) || defined(STACK_USE_MPFS2)
		if(sizeof(AppConfig) > MPFS_RESERVE_BLOCK)
			while(1);
	#endif

	#if defined(EEPROM_CS_TRIS)
	    XEEBeginWrite(0x0000);
	    XEEWrite(0x60);
	    XEEWriteArray((BYTE*)&AppConfig, sizeof(AppConfig));
    #else
	    SPIFlashBeginWrite(0x0000);
	    SPIFlashWrite(0x60);
	    SPIFlashWriteArray((BYTE*)&AppConfig, sizeof(AppConfig));
    #endif
}
#endif

void __attribute__ ((interrupt, no_auto_psv)) _T4Interrupt(void) 
{
	static unsigned int cpt_asser_canon=0,cpt_envoi_position=0;
	static unsigned char k=0;
	unsigned char i=0;
	double temp;
	unsigned int lol,truc;
	static double tab_vitesse_canon[16];
	static unsigned int puissance;
	static unsigned char etat_canon=0;	
	static double vitesse_canon_brut;

	flag = 0;
	courrier = 1;
	cpt_balise=0;
	if(timeout_lidar<200) timeout_lidar++;
	motor_flag = Motors_Task(); // Si prend trop de ressource sur l'udp, inclure motortask dans le main	
		
	if(cpt_envoi_position++>prd_envoi_position)
	{
		if(prd_envoi_position !=0) flag_envoi_position=1;
		cpt_envoi_position=0;
	}

	if(motor_flag)
	{
		switch(motor_flag)
		{
			case FLAG_ENVOI:
				flag_envoi=1;
				break;
			case FLAG_CALAGE:
				flag_calage=1;
				break;
			case FLAG_BLOCAGE:
				flag_blocage=1;
				break;
		}
		motor_flag=0;
	}
	
	cpu_status = (TMR4); //Previous value TMR4
	IFS1bits.T4IF = 0;
}

void __attribute__((interrupt,auto_psv)) _U2RXInterrupt(void)
{
	static unsigned etat_rx=0;
	static unsigned int recu,recu_ptr=0;

	IFS1bits.U2RXIF = 0; 		// clear RX interrupt flag
	
	if(U2STAbits.URXDA == 1)
	{
		Buffer_passerelle_udpuart[ptr_write_buffer_uart_rec++] = U2RXREG;
		if(ptr_write_buffer_uart_rec>240)
			ptr_write_buffer_uart_rec=0;
	}
			
}

void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(void) // 3.2µs (312 itérations sur 1ms)
{
	if(cpt_balise<10000)
		cpt_balise++;
	if(cpt_balise<pwm_balise)
		LATAbits.LATA8 = 1;
	else
		LATAbits.LATA8 = 0;
	
	IFS0bits.T2IF = 0; 		//Clear Timer1 Interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt(void)
{
	unsigned char i;
	TMR3=0;
	IC2CONbits.ICM = 0b011;
	IC7CONbits.ICM = 0b011;
	periode_tour = IC1BUF;
	
	IFS0bits.IC1IF=0;

	nombre_angles[IDCAPTEUR_HAUT] = ptr_fronts_haut;
	nombre_angles[IDCAPTEUR_BAS] = ptr_fronts_bas;

	for(i=0;i<nombre_angles[IDCAPTEUR_HAUT];i++)
	{
		buffer_fronts[IDCAPTEUR_HAUT][i] = 	buffer_fronts_temp[IDCAPTEUR_HAUT][i];
	}
	for(i=0;i<nombre_angles[IDCAPTEUR_BAS];i++)
	{
		buffer_fronts[IDCAPTEUR_BAS][i] = 	buffer_fronts_temp[IDCAPTEUR_BAS][i];
	}
	
	ptr_fronts_haut=0;
	ptr_fronts_bas=0;
	Fin_de_tour=1;
	
}

void __attribute__((interrupt, no_auto_psv)) _IC2Interrupt(void)
{
	buffer_fronts_temp[IDCAPTEUR_BAS][ptr_fronts_bas]=IC2BUF;
	if(ptr_fronts_bas < 8) ptr_fronts_bas++;
	 
	if(ptr_fronts_bas %2 == 0)
	{
		front_rapide[IDCAPTEUR_BAS][1] = buffer_fronts_temp[IDCAPTEUR_BAS][ptr_fronts_bas-1]; // Front descendant
		Fin_d_angle_bas = 1;
	}
	else
	{
		front_rapide[IDCAPTEUR_BAS][0] = buffer_fronts_temp[IDCAPTEUR_BAS][ptr_fronts_bas-1]; // Front montant
	}
	IC2CONbits.ICM = 0b001;
	IFS0bits.IC2IF=0;
}

void __attribute__((interrupt, no_auto_psv)) _IC7Interrupt(void)
{
	buffer_fronts_temp[IDCAPTEUR_HAUT][ptr_fronts_haut]=IC7BUF;		
	if(ptr_fronts_haut < 8) ptr_fronts_haut++;
	if(ptr_fronts_haut %2 == 0)
	{
		front_rapide[IDCAPTEUR_HAUT][1] = buffer_fronts_temp[IDCAPTEUR_HAUT][ptr_fronts_haut-1]; // Front descendant
		Fin_d_angle_haut = 1;
	}
	else
	{
		front_rapide[IDCAPTEUR_HAUT][0] = buffer_fronts_temp[IDCAPTEUR_HAUT][ptr_fronts_haut-1]; // Front montant
	}
	IC7CONbits.ICM = 0b001;
	IFS1bits.IC7IF=0;
}

void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void)
{
	static int cpt_IR=0;
	IFS1bits.INT1IF=0;
	if(PORTBbits.RB2 == 1)
	{
		INTCON2bits.INT1EP = 1; // Check negative edge
		TMR5=0;
	}
	else
	{
		INTCON2bits.INT1EP = 0; // Check positive edge
		IR_result = TMR5;
	}
	cpt_IR++;
}