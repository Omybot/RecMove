#include "Pilotage.h"
#include "asser.h"
#include "uart2.h"
#include <math.h>

// ATTENTION /!\ Ces fonctions ne doivent pas être bloquantes

#define UART_BUFFER_SIZE 100

//	Extern Capteur Couleur
extern unsigned int Tab_Capteur_Couleur[8];
extern unsigned char alim_capteur_couleur;

//Variable Capteur Couleur
unsigned int Valeur_Capteur_Couleur = 24;

extern unsigned int IR_result;
extern unsigned char buffer_envoi_uart[UART_BUFFER_SIZE], ptr_write_buffer_uart;
extern unsigned char flagDemandeLidar;
extern unsigned int pointsPolaire[2][300];
extern unsigned int nbPointsPolaire;
extern unsigned int envoiPositionInterval;
extern unsigned char jackAvant;
extern unsigned int ADC_Results[8];
extern unsigned int position_buffer[6];
extern unsigned char buff_position_ptr, last_send_ptr;
extern long buff_position[N][2];
extern unsigned char buff_status_ptr, last_send_status_ptr;
extern unsigned int buff_status[3][64];
extern double cons_pos[N], pwm_cor[N];
extern double pos_x, pos_y, pos_teta;
extern double offset_teta;
extern double kp_cap, ki_cap, kd_cap;
extern double kp_vit, ki_vit, kd_vit;

unsigned int Send_Variable_Capteur_Couleur(void)
{
	return Valeur_Capteur_Couleur;
}

Trame CouleurRGB(int Id)
{
	Rgb RgbVal;
	Trame RgbMessage;	
	double freqClear,freqRed,freqGreen,freqBlue;	
	static BYTE Couleur[6];
	
	RgbMessage.nbChar = 6;
	Couleur[0] = UDP_ID;
	Couleur[1] = CMD_REPONSE_CAPTEUR_COULEUR;
	Couleur[2] = Id;

	freqClear = period2frequency(Tab_Capteur_Couleur[6]);
	freqRed   = period2frequency(Tab_Capteur_Couleur[4]);
	freqGreen = period2frequency(Tab_Capteur_Couleur[7]);
	freqBlue  = period2frequency(Tab_Capteur_Couleur[5]);

	RgbVal = frequency2RGB(freqClear, freqRed, freqGreen, freqBlue);

	Couleur[3] = RgbVal.red;
	Couleur[4] = RgbVal.green;
	Couleur[5] = RgbVal.blue;

	RgbMessage.message = Couleur;
	return RgbMessage;
}

double period2frequency(unsigned int period)
{
	double frequency;
	double fcy = 40000000;   // 40MHz
	BYTE timerPrescaler = 8; // Timer TCKPS prescaler value
	BYTE captureEventConfig = 16; // Event captured on every 16th rising edge
	double timeBaseFreq = fcy/timerPrescaler;
	double timeBasePeriod = 1/(timeBaseFreq);   // period of 1 tick of the time base
	
	// frequency of the timer value captured by IC mdoule
	frequency = 1/((period/captureEventConfig)*timeBasePeriod);
	return frequency;
}

Rgb frequency2RGB(double freqClear, double freqRed, double freqGreen, double freqBlue)
{
	Rgb rgbVal;
	rgbVal.red = (freqRed/freqClear)*255;
	rgbVal.green = (freqGreen/freqClear)*255;
	rgbVal.blue = (freqBlue/freqClear)*255;
	
	return rgbVal;  
}

Trame Retour_Capteur_Onoff(unsigned char id_capteur)
{
	Trame Etat_Valeurs;
	static BYTE Valeurs[4];
	Etat_Valeurs.nbChar = 4;
	
	Valeurs[0] = UDP_ID;
	Valeurs[1] = CMD_REPONSE_CAPTEUR_ONOFF;
	Valeurs[2] = id_capteur;
	switch(id_capteur)
	{
		case VACUOSTAT_BACK:
			Valeurs[3] = PORTCbits.RC1;
			break;
		case VACUOSTAT_FRONT:
			Valeurs[3] = PORTCbits.RC2;
			break;
		case JACK_DEMARRAGE:
			Valeurs[3] = !PORTBbits.RB2;
			break;
	}
	
	Etat_Valeurs.message = Valeurs;

	return Etat_Valeurs;
}

void PiloteMoteurPosition(unsigned char id, unsigned int position)
{
	//pwm à régler
}

void EnvoiUART(Trame t)
{
	unsigned char i;
	// Copier trame dans buffer circulaire d'envoi
	for(i=0;i<t.nbChar-3;i++)
	{
		buffer_envoi_uart[ptr_write_buffer_uart++]=t.message[i+3];
		if(ptr_write_buffer_uart >= UART_BUFFER_SIZE)
			ptr_write_buffer_uart=0;
	}
}

Trame Retour_Valeurs_Analogiques(void)
{
	Trame Etat_Valeurs;
	static BYTE Valeurs[14];
	Etat_Valeurs.nbChar = 14;
	
	Valeurs[0] = UDP_ID;
	Valeurs[1] = CMD_REPONSE_VALEURS_ANALOGIQUES;
	Valeurs[2] = ADC_Results[0] >> 8;
	Valeurs[3] = ADC_Results[0] & 0xFF;	
	Valeurs[4] = ADC_Results[1] >> 8;
	Valeurs[5] = ADC_Results[1] & 0xFF;	
	Valeurs[6] = ADC_Results[2] >> 8;
	Valeurs[7] = ADC_Results[2] & 0xFF;	
	Valeurs[8] = ADC_Results[3] >> 8;
	Valeurs[9] = ADC_Results[3] & 0xFF;	
	Valeurs[10] = ADC_Results[4] >> 8;
	Valeurs[11] = ADC_Results[4] & 0xFF;	
	Valeurs[12] = ADC_Results[5] >> 8;
	Valeurs[13] = ADC_Results[5] & 0xFF;	
	
	Etat_Valeurs.message = Valeurs;

	return Etat_Valeurs;
}

Trame Retour_Valeurs_Numeriques(void)
{
	Trame Etat_Valeurs;
	static BYTE Valeurs[8];
	Etat_Valeurs.nbChar = 8;
	
	Valeurs[0] = UDP_ID;
	Valeurs[1] = CMD_REPONSE_VALEURS_NUMERIQUES;
	Valeurs[2] = PORTA>>8;
	Valeurs[3] = PORTA&0xFF;
	Valeurs[4] = PORTB>>8;
	Valeurs[5] = PORTB&0xFF;
	Valeurs[6] = PORTC>>8;
	Valeurs[7] = PORTC&0xFF;
	
	
	Etat_Valeurs.message = Valeurs;

	return Etat_Valeurs;
}

void PiloteActionneurOnOff(unsigned char id, OnOff onOff)
{
	switch (id)
	{
		case ALIMENTATION_CAPTEUR_COULEUR:
			alim_capteur_couleur = onOff;
			break;

		case MAKEVACUUM_BACK:
			if(onOff)
			{
				PWM2CON1bits.PEN1L = 1;
				MOT1L=0;
				P2DC1 = 4000;
			}
			else
			{
				PWM2CON1bits.PEN1L = 0;
				MOT1L=1;
				P2DC1 = 4000;
			}
			break;

		case MAKEVACUUM_FRONT:
			if(onOff)
			{
				PWM2CON1bits.PEN1H = 1;
				MOT1H=0;
				P2DC1 = 4000;
			}
			else
			{
				PWM2CON1bits.PEN1H = 0;
				MOT1H=1;
				P2DC1 = 4000;
			}
			break;

		case OPENVACUUM_BACK:
			if(onOff)
			{
				PWM1CON1bits.PEN1L = 1;
				MOT4L=0;
				P1DC1 = 4000;
			}
			else
			{
				PWM1CON1bits.PEN1L = 0;
				MOT4L=1;
				P1DC1 = 4000;
			}
			break;

		case OPENVACUUM_FRONT:
			if(onOff)
			{
				PWM1CON1bits.PEN1H = 1;
				MOT4H=0;
				P1DC1 = 4000;
			}
			else
			{
				PWM1CON1bits.PEN1H = 0;
				MOT4H=1;
				P1DC1 = 4000;
			}
			break;
	}
}

void delay(void)
{
    long i = 10; 
    while(i--);
}

void delayms(void) 
{
	long i = 1600000; //400ms
    while(i--);
}

void delays(void) 
{
	long i = 4000000; //seconde
    while(i--);
}

// DEBUG

Trame PiloteDebug(Trame t, int debugNo)
{
	return t;
}

Trame StatusMonitor(void)
{
	Trame trame;
	static BYTE tableau[512];
	unsigned char i,current_send_ptr,nbr_to_send;
	
	tableau[0] = UDP_ID; // identifiant trame
	tableau[1] = CMD_REPONSE_BUFF_STATUS;

	if(buff_status_ptr > last_send_status_ptr)
		nbr_to_send = buff_status_ptr - last_send_status_ptr;
	else
		nbr_to_send = 64 - last_send_status_ptr + buff_status_ptr;

	//nbr_to_send = (buff_status_ptr - last_send_status_ptr)%64;

	last_send_status_ptr=buff_status_ptr;
	if(nbr_to_send>35) nbr_to_send=35;
	tableau[2] = nbr_to_send;
	trame.nbChar = nbr_to_send*6+4;

	current_send_ptr = last_send_status_ptr + 1;

	for(i=0;i<nbr_to_send;i++)
	{
		tableau[1+2+(i*6)] = buff_status[0][current_send_ptr]>>8; // Status
		tableau[1+3+(i*6)] = buff_status[0][current_send_ptr]&0x00FF;		
		tableau[1+4+(i*6)] = buff_status[1][current_send_ptr]>>8; // PWM gauche
		tableau[1+5+(i*6)] = buff_status[1][current_send_ptr]&0x00FF;
		tableau[1+6+(i*6)] = buff_status[2][current_send_ptr]>>8; // PWM droite
		tableau[1+7+(i*6)] = buff_status[2][current_send_ptr]&0x00FF;
		current_send_ptr = (current_send_ptr + 1)%64;
	}
	
	trame.message = tableau;
	
	return trame;
}

Trame PilotePositionXYT()
{
	//double x,y,teta;
	Trame trame;
	static BYTE tableau[8];
	trame.nbChar = 8;
	
	tableau[0] = UDP_ID;
	tableau[1] = CMD_RETOURPOSITION;
	tableau[2] = (int)(pos_x * 10)>>8;
	tableau[3] = (int)(pos_x * 10)&0x00FF;
	tableau[4] = (int)(pos_y * 10)>>8;
	tableau[5] = (int)(pos_y * 10)&0x00FF;
	tableau[6] = (unsigned int)(pos_teta*36000/(2*PI)+18000)>>8;
	tableau[7] = (unsigned int)(pos_teta*36000/(2*PI)+18000)&0x00FF;
	
	trame.message = tableau;
	
	return trame;
}

Trame ReponseConnexion()
{
	Trame trame;
	static BYTE tableau[2];
	trame.nbChar = 2;

	tableau[0] = UDP_ID;
	tableau[1] = TRAME_TEST_CONNEXION;
	
	trame.message = tableau;
	
	return trame;
}

void PilotePIDCoeffs(unsigned int new_kp, unsigned int new_ki, unsigned int new_kd)
{
	double coeffs[N*3]={DEFAULT_KP,DEFAULT_KI,DEFAULT_KD,DEFAULT_KP,DEFAULT_KI,DEFAULT_KD};
	coeffs[0] = (double)new_kp;
	coeffs[1] = (double)new_ki;
	coeffs[2] = (double)new_kd;
	coeffs[3] = (double)new_kp;
	coeffs[4] = (double)new_ki;
	coeffs[5] = (double)new_kd;
	
	set_pid(coeffs);
}

// A faire : envoie consigne posiiton brut en pas codeur 2 parametres : [sens]8u ; [pascodeur]16u ;

Trame PiloteGetBuffPosition()
{
	Trame trame;
	static BYTE tableau[512];
	unsigned char i,current_send_ptr,nbr_to_send;
	
	tableau[0] = UDP_ID; // identifiant trame
	tableau[1] = CMD_REPONSE_BUFF_POSITION;
	nbr_to_send = buff_position_ptr - last_send_ptr;
	last_send_ptr=buff_position_ptr;
	if(nbr_to_send>60) nbr_to_send=60;
	tableau[2] = nbr_to_send;
	trame.nbChar = nbr_to_send*8+4;
	for(i=0;i<nbr_to_send;i++)
	{
		current_send_ptr = buff_position_ptr-nbr_to_send+i;
		tableau[1+2+(i*8)] = buff_position[0][current_send_ptr]>>24;
		tableau[1+3+(i*8)] = buff_position[0][current_send_ptr]>>16;
		tableau[1+4+(i*8)] = buff_position[0][current_send_ptr]>>8;
		tableau[1+5+(i*8)] = buff_position[0][current_send_ptr]&0x00FF;
		
		tableau[1+6+(i*8)] = buff_position[1][current_send_ptr]>>24;
		tableau[1+7+(i*8)] = buff_position[1][current_send_ptr]>>16;
		tableau[1+8+(i*8)] = buff_position[1][current_send_ptr]>>8;
		tableau[1+9+(i*8)] = buff_position[1][current_send_ptr]&0x00FF;
	}
	
	trame.message = tableau;
	
	return trame;
}


int PiloteAcceleration(int acceleration)
{
	Motors_SetAcceleration(acceleration,MOTEUR_GAUCHE);
	Motors_SetAcceleration(acceleration,MOTEUR_DROIT);
	return 1;
}

int PiloteAvancer(double distance)
{
	Avance(distance,0);
	return 1;
}

// Recule de la distance spécifiée
// distance : distance à reculer
int PiloteReculer(double distance)
{
	Avance(-distance,0);
	return 1;
}

// Pivote de l'angle spécifié
// angle : angle de rotation (en degrés)
// direction : coté du pivot (Gauche ou Droite)
int PilotePivoter(double angle, Cote direction)
{
	if(direction==Gauche)	Pivot( angle/100.0,0);
	else					Pivot(-angle/100.0,0);
	return 1;
}

// Effectuer un virage
// angle : angle de rotation (en degrés)
// rayon : rayon du virage
// direction : coté du virage (Gauche ou Droite)
int PiloteVirage(unsigned char reculer, unsigned char direction, double rayon, double angle)
{
	if(reculer) Virage(direction, rayon, angle/100, 0);
	else	 	Virage(direction, rayon, -angle/100, 0);

	return 1;
}

// Stoppe le robot
// mode : mode de stop (Abrupt, Smooth, Freely)
int PiloteStop(unsigned char stopmode)
{
	/*int distanceRestante;
	Trame envoiReste;
	static BYTE messReste[2];
	messReste[0] = UDP_ID;
	messReste[1] = 0x60;
	envoiReste.nbChar = 4;
	*/
	Stop(stopmode);	

	//envoiReste.message = messReste;

	//while(Motors_IsRunning(MOTEUR_GAUCHE) || Motors_IsRunning(MOTEUR_DROIT));

	//EnvoiUserUdp(envoiReste);
	
	return 1;
}

// Recallage du robot
// s : sens du recallage (Avant ou Arriere)
int PiloteRecallage(Sens s)
{	
	Calage(s);

	return 1;
}

int PiloteOffsetAsserv(double x, double y, double teta)
{
	double toto;
	pos_x = -y;
	pos_y = -x;
	toto = ((teta)/180*PI) / 100.0;
	offset_teta = toto - pos_teta + offset_teta;

	return 1;
}

// Analyse la trame recue et renvoie vers la bonne fonction de pilotage
// Trame t : Trame ethernet recue
Trame AnalyseTrame(Trame t)
{
	Trame retour;
	unsigned int param1, param2, param3, param4, i;
	
	retour = t;

	// Les messages ne commencant pas par UDP_ID ne nous sont pas adressés (RecMove)
	if(t.message[0] != UDP_ID)
	{
		t.message[0] = UDP_ID;
		return t;
	}

	switch(t.message[1])
	{
		case CMD_DEBUG:
			param1 = t.message[3];							// Numero
			retour = PiloteDebug(t, param1);
			break;
		
		case CMD_AVANCER:
			param1 = t.message[2];							// Sens
			param2 = t.message[3] * 256 + t.message[4];		// Distance
			if(param1)
				PiloteAvancer(param2);
			else
				PiloteReculer(param2);
			break;

		case CMD_RECALLAGE:
			param1 = t.message[2];							// Sens
			PiloteRecallage(param1);
			break;

		case CMD_PIVOTER:
			param1 = t.message[3] * 256 + t.message[4];		// Angle
			param2 = t.message[2];							// Sens
			PilotePivoter(param1, (Cote)param2);
			break;

		case CMD_VIRAGE:
			param1 = t.message[4] * 256 + t.message[5];		// Rayon
			param2 = t.message[6] * 256 + t.message[7];		// Angle
			param3 = t.message[3];							// Direction
			param4 = t.message[2];							// Sens
			PiloteVirage((unsigned char)param4,(unsigned char)param3, (double)param1, (double)param2);
			break;

		case CMD_STOP:
			param1 = t.message[2];							// StopMode
			PiloteStop((unsigned char)param1);
			break;

		case CMD_VITESSE_PIVOT:
			param1 = t.message[2] * 256 + t.message[3];		// Vitesse
			Motors_SetSpeed_Pivot(param1);
			break;

		case CMD_VITESSE_LIGNE:
			param1 = t.message[2] * 256 + t.message[3];		// Vitesse
			Motors_SetSpeed_Ligne((double)param1);
			break;

		case CMD_ACCELERATION_PIVOT:
			param1 = t.message[2] * 256 + t.message[3];		// Accélération
			Motors_SetAcceleration_Pivot((double)param1);
			break;

		case CMD_ACCELERATION_LIGNE:
			param1 = t.message[2] * 256 + t.message[3];		// Accélération début
			param2 = t.message[4] * 256 + t.message[5];		// Accélération fin
			Motors_SetAcceleration_Ligne((double)param1, (double)param2);
			break;			

		case CMD_ENVOI_PID:
			param1 = t.message[2]*256+t.message[3];			// P
			param2 = t.message[4]*256+t.message[5];			// I
			param3 = t.message[6]*256+t.message[7];			// D
			PilotePIDCoeffs(param1,param2,param3);
			break;

		case CMD_ENVOI_PID_CAP:
			kp_cap = ((double)(t.message[2]*256+t.message[3]))*100;			// P
			ki_cap = (double)(t.message[4]*256+t.message[5]);			// I
			kd_cap = ((double)(t.message[6]*256+t.message[7]))*100;			// D
			break;

		case CMD_ENVOI_PID_VITESSE:
			kp_vit = (double)(t.message[2]*256+t.message[3]);			// P
			ki_vit = (double)(t.message[4]*256+t.message[5]);			// I
			kd_vit = (double)(t.message[6]*256+t.message[7]);			// D
			break;

		case TRAME_TEST_CONNEXION:
			retour = ReponseConnexion();
			break;
			
		case CMD_OFFSETASSERV:
			param1 = t.message[2]*256+t.message[3];			// X
			param2 = t.message[4]*256+t.message[5];			// Y
			param3 = t.message[6]*256+t.message[7];			// TETA
			PiloteOffsetAsserv((double)param1,(double)param2,(double)param3);
			break;

		case CMD_DEMANDEPOSITION: // Demande POS X Y TETA
			retour = PilotePositionXYT();
			break;

		case CMD_CONSIGNE_POSITION:
			if(t.message[2] == AVANT)
			{
				cons_pos[0] += MM_SCALER * (t.message[3] * 256 + t.message[4]);
				cons_pos[1] += MM_SCALER * (t.message[3] * 256 + t.message[4]) * COEFF_ROUE_DIFF;
			}
			else
			{
				cons_pos[0] -= MM_SCALER * (t.message[3] * 256 + t.message[4]);
				cons_pos[1] -= MM_SCALER * (t.message[3] * 256 + t.message[4]) * COEFF_ROUE_DIFF;
			}
			break;

		case CMD_DEMANDE_BUFF_POSITION:
			return PiloteGetBuffPosition();

		case CMD_DEMANDE_BUFF_STATUS:
			return StatusMonitor();

		case CMD_PRD_ENVOI_POSITION:
			envoiPositionInterval = 10*(unsigned int)t.message[2];
			break;

		case CMD_DEMANDE_VALEURS_ANALOGIQUES:
			return Retour_Valeurs_Analogiques();

		case CMD_DEMANDE_VALEURS_NUMERIQUES:
			return Retour_Valeurs_Numeriques();

		case CMD_DEPLACEMENT_POLAIRE:
		 	nbPointsPolaire = t.message[3] * 256 + t.message[4];
			for(i=0;i<nbPointsPolaire;i++)
			{
				pointsPolaire[0][i+1] = t.message[5+i*4] * 256 + t.message[6+i*4]; // X
				pointsPolaire[1][i+1] = t.message[7+i*4] * 256 + t.message[8+i*4]; // Y
			}
			pointsPolaire[0][0]=pos_x;
			pointsPolaire[1][0]=pos_y;
			Deplacement_Polaire();			
			break;

		case TRAME_UART2_ENVOI:
			flagDemandeLidar = 1;
			break;

		case CMD_MOTEUR_POSITION:
			param1 = t.message[2];						// Id moteur
			param2 = t.message[3] * 256 + t.message[4];	// Position
			PiloteMoteurPosition(param1, param2);
			break;

		case TRAME_PILOTAGE_ONOFF:
			param1 = t.message[2];						// Id actionneur
			param2 = t.message[3];						// OnOff
			PiloteActionneurOnOff(param1, (OnOff)param2);
			break;
		
		case CMD_DEMANDE_CAPTEUR_ONOFF:
			return Retour_Capteur_Onoff(t.message[2]);

		case CMD_DEMANDE_CAPTEUR_COULEUR:
      		retour = CouleurRGB(t.message[2]);
			break;

		default :
			param1 = t.message[2];						// ID
			break;
	}
	return retour;
}

//Interruption Input Capture
//Interruption induced on every 16th Rising Edge
void __attribute__((__interrupt__,__auto_psv__)) _IC1Interrupt(void)
{
	unsigned int t1,t2;
	t1=IC1BUF;
	t2=IC1BUF;

	IFS0bits.IC1IF=0;

	if(t2>t1)
		Valeur_Capteur_Couleur = t2-t1;
	else
		Valeur_Capteur_Couleur = (PR3 - t1) + t2;
}
