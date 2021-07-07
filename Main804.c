#include "TCPIP Stack/TCPIP.h"
#include "OurFiles/UserUdp.h"
#include "OurFiles/asser.h"
#include "OurFiles/init.h"
#include <stdio.h>
#include <stdlib.h>
#include <uart.h>
#include <math.h>
#include <p33FJ128MC804.h>
#include "OurFiles/Pilotage.h"

#define UART_BUFFER_SIZE	100
#define IDCAPTEUR_BAS 0
#define IDCAPTEUR_HAUT 1

//Define Capteur Couleur
#define LED LATAbits.LATA8
#define S2 LATAbits.LATA10
#define S3 LATAbits.LATA7 

// Bits configuration
_FOSCSEL(FNOSC_FRC)
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON)
_FPOR(FPWRT_PWR1)
_FWDT(FWDTEN_OFF)
_FICD(ICS_PGD3 & JTAGEN_OFF)

APP_CONFIG AppConfig;

static void InitAppConfig(void);

extern double pos_x,pos_y,pos_teta;

unsigned int IR_result;

extern unsigned int ADC_Results[8],cpu_status;
extern double cons_pos[N];
extern double real_pos[N];
unsigned char flag_envoi_position;
unsigned int envoiPositionInterval = 100;
unsigned char jackAvant = 0;
unsigned char motor_flag=0,datalogger_blocker=0;
unsigned int datalogger_counter=0,flag=0,courrier=0;
unsigned char flag_envoi=0,flag_blocage=0,flag_calage=0;

//LIDAR
unsigned char flagDemandeLidar = 0, offset_premiere_trame = 0;
unsigned char timeout_lidar = 0;
unsigned char nbr_char_to_send = 0; 
unsigned char ptr_write_buffer_uart_rec = 0, save_write;
unsigned char ptr_read_buffer_uart_rec = 0, save_read;
unsigned char ptr_write_buffer_uart;
unsigned char ptr_read_buffer_uart = 0;
unsigned char Buffer_passerelle_udpuart[240];
unsigned char buffer_envoi_uart[UART_BUFFER_SIZE];

//Variable Capteur Couleur
unsigned int Cpt_Tmr2_Capteur_Couleur = 0;
unsigned int Tab_Capteur_Couleur[8] = {0};
unsigned char etat_Capteur_Couleur = 0,alim_capteur_couleur=1;

unsigned char jack_value=0,jack_value_old=0,jack_cpt=0;


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
	static DWORD dwLastIP = 0;

	Trame trame;

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
	
	Trame envoiDemandeHokuyo;
	static BYTE messDemandeHokuyo[19];
	messDemandeHokuyo[0] = UDP_ID;
	messDemandeHokuyo[1] = UDP_ID;
	messDemandeHokuyo[2] = 7;
	messDemandeHokuyo[3] = 'M';
	messDemandeHokuyo[4] = 'S';
	messDemandeHokuyo[5] = '0';
	messDemandeHokuyo[6] = '0';
	messDemandeHokuyo[7] = '0';
	messDemandeHokuyo[8] = '0';
	messDemandeHokuyo[9] = '0';
	messDemandeHokuyo[10] = '7';
	messDemandeHokuyo[11] = '2';
	messDemandeHokuyo[12] = '5';
	messDemandeHokuyo[13] = '0';
	messDemandeHokuyo[14] = '0';
	messDemandeHokuyo[15] = '0';
	messDemandeHokuyo[16] = '0';
	messDemandeHokuyo[17] = '1';
	messDemandeHokuyo[18] = '\n';

	//MS0000072500001
	
	envoiDemandeHokuyo.message = messDemandeHokuyo;
	envoiDemandeHokuyo.nbChar = 19;

	

	Trame envoiBaudrateHokuyo;
	static BYTE messBaudrateHokuyo[12];
	messBaudrateHokuyo[0] = UDP_ID;
	messBaudrateHokuyo[1] = UDP_ID;
	messBaudrateHokuyo[2] = 7;
	messBaudrateHokuyo[3] = 'S';
	messBaudrateHokuyo[4] = 'S';
	messBaudrateHokuyo[5] = '0';
	messBaudrateHokuyo[6] = '1';
	messBaudrateHokuyo[7] = '9';
	messBaudrateHokuyo[8] = '2';
	messBaudrateHokuyo[9] = '0';
	messBaudrateHokuyo[10] = '0';
	messBaudrateHokuyo[11] = '\n';

	//SS115200

	envoiBaudrateHokuyo.message = messBaudrateHokuyo;
	envoiBaudrateHokuyo.nbChar = 12;



	// V V [LF] 0 0 P [LF] 
	
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

	jack_value = PORTBbits.RB2;
	jack_value_old = jack_value;
	
	
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
	Init_Input_Capture();	
	UDPInit();
    StackInit();	
	InitUserUdp();
	
	Init_Interrupt_Priority();							
	InitUART2();	
	InitADC();
	InitDMA();

	//EnvoiUART(envoiBaudrateHokuyo);
	//U2BRG = 19;			// Chgt de baudrate apres avoir repassé le lidar en 19200 // 86 pour 115200 // 520 pour 19200
	DelayMs(500); 




	while(1)
	{
		if(jack_cpt == 50)
		{
			trame=Retour_Capteur_Onoff(JACK_DEMARRAGE);
			trame.message[3] = jack_value;
			EnvoiUserUdp(trame);
			jack_cpt=0;
			jack_value_old = jack_value;
		}
		
		if(flagDemandeLidar)	
		{
			flagDemandeLidar=0;
			EnvoiUART(envoiDemandeHokuyo);
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

		if(((nbr_char_to_send >= 200) || (nbr_char_to_send !=0 && timeout_lidar > 50)))
		{	
			for(i=0;i<nbr_char_to_send && i<200;i++)
			{
				messUART[i+3+offset_premiere_trame]=Buffer_passerelle_udpuart[save_read++];
				if(save_read>240)
					save_read=0;
			}
			
			timeout_lidar=0;
			envoiUART.nbChar = i+3+offset_premiere_trame;
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
		if(flag_envoi) 
		{	
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

		if(dwLastIP != AppConfig.MyIPAddr.Val)
		{
			dwLastIP = AppConfig.MyIPAddr.Val;
		}
	}
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
	static unsigned int cpt_envoi_position=0;
	
	if (HasClient())
	{
		jack_value = !PORTBbits.RB2;
		if(jack_value_old != jack_value)
		{
			if(jack_cpt<50) jack_cpt++;
		}
		else
		{
			jack_cpt = 0;
		}	
	}

	flag = 0;
	courrier = 1;
	if(timeout_lidar<200) timeout_lidar++;
	motor_flag = Motors_Task(); // Si prend trop de ressource sur l'udp, inclure motortask dans le main	
		
	if(cpt_envoi_position++>envoiPositionInterval)
	{
		if(envoiPositionInterval !=0) flag_envoi_position=1;
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
	
	// Gestion capteur de couleur
	if(alim_capteur_couleur)
	{
		Cpt_Tmr2_Capteur_Couleur++;
		if(Cpt_Tmr2_Capteur_Couleur == 20)
		{
			Cpt_Tmr2_Capteur_Couleur = 0;
			switch(etat_Capteur_Couleur++)
			{	
				case 0:
					S2  = 0;
					S3  = 0; 
					LED = 0;
				case 1: // Capture RED filter without led
					Tab_Capteur_Couleur[0] = Send_Variable_Capteur_Couleur(); // Capture de S2=0, S3=0 et LED=0
					S2  = 0;
					S3  = 1; 
					LED = 0;
				break;
				case 2: // Capture BLUE filter without led
					Tab_Capteur_Couleur[1] = Send_Variable_Capteur_Couleur(); // Capture de S2=0, S3=1 et LED=0	
					S2  = 1;
					S3  = 0; 
					LED = 0;
				break;
				case 3: // Capture Clear filter without led
					Tab_Capteur_Couleur[2] = Send_Variable_Capteur_Couleur(); // Capture de S2=1, S3=0 et LED=0
					S2  = 1; 
					S3  = 1;
					LED = 0;
				break;
				case 4: // Capture GREEN filter without led
					Tab_Capteur_Couleur[3] = Send_Variable_Capteur_Couleur(); // Capture de S2=1, S3=1 et LED=0
					S2  = 0;
					S3  = 0; 
					LED = 1;
				break;
				case 5: // Capture RED filter with LED
					Tab_Capteur_Couleur[4] = Send_Variable_Capteur_Couleur(); // Capture de S2=0, S3=0 et LED=1 
					S2  = 0;
					S3  = 1; 
					LED = 1;
				break;
				case 6: // Capture BLUE filter with LED
					Tab_Capteur_Couleur[5] = Send_Variable_Capteur_Couleur(); // Capture de S2=0, S3=1 et LED=1 
					S2  = 1;
					S3  = 0; 
					LED = 1;
				break;
				case 7: // Capture Clear filter with LED
					Tab_Capteur_Couleur[6] = Send_Variable_Capteur_Couleur(); // Capture de S2=1, S3=0 et LED=1 
					S2  = 1;
					S3  = 1; 
					LED = 1;
				break;
				case 8: // Capture GREEN filter with LED
					Tab_Capteur_Couleur[7] = Send_Variable_Capteur_Couleur(); // Capture de S2=1, S3=1 et LED=1 
					S2  = 0;
					S3  = 0; 
					LED = 1; // On saute l'étape 4
					etat_Capteur_Couleur = 5;
				break;
			}
		}
	}
	else
	{
		//LED = 0;
		S2  = 0;
		S3  = 0; 
		etat_Capteur_Couleur=4;
	}

	cpu_status = (TMR4); //Previous value TMR4
	IFS1bits.T4IF = 0;
}

void __attribute__((interrupt,auto_psv)) _U2RXInterrupt(void)
{
	IFS1bits.U2RXIF = 0; 		// clear RX interrupt flag
	
	if(U2STAbits.URXDA == 1)
	{
		timeout_lidar=0;
		Buffer_passerelle_udpuart[ptr_write_buffer_uart_rec++] = U2RXREG;
		if(ptr_write_buffer_uart_rec>240)
			ptr_write_buffer_uart_rec=0;
	}			
}

void __attribute__((__interrupt__,__auto_psv__)) _T2Interrupt(void) // 3.2µs (312 itérations sur 1ms)
{	
	IFS0bits.T2IF = 0; 		//Clear Timer1 Interrupt flag
}
