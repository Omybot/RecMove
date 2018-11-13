#ifndef __PILOTAGE_H__
#define __PILOTAGE_H__

#include "Types.h"
#include "UserUdp.h"

// Moteur sens
#define SENS_DROITE 3
#define SENS_GAUCHE 2

void EnvoiUART(Trame t);

Trame PiloteGotoXY(int x, int y, unsigned char x_negatif, unsigned char y_negatif);
Trame ReponseEcho(void);
Trame StatusMonitor(void);

Trame PilotePIDRessource();
void PilotePIDInit(void);
Trame PiloteGetPosition(unsigned char cote);
Trame PiloteGetRawPosition(void);
Trame PiloteGetLongPosition(void);
Trame PiloteGetBuffPosition(void);
void PilotePIDCoeffs(unsigned int new_kp, unsigned int new_ki, unsigned int new_kd);
void PilotePIDManual(unsigned int gauche,unsigned int droite);
Trame PilotePIDErreurs(void);
void PilotePIDFeedforward(unsigned int value);
Trame Retour_Valeurs_Analogiques(void);
Trame Retour_Valeurs_Numeriques(void);

Trame PilotePositionXYT(void);
Trame PiloteDemandeCapteurs(char numCapteur);
Trame PiloteDemandeCapteur(char numCapteur);
void PiloteLevagePosition(char avantOuArriere, int position);
Trame PiloteCapteurs(char cote);

int PiloteVitesse(unsigned int id, unsigned int sens, unsigned int vitesse);
Trame Retour_Pattern();

// Constantes des fonctions des actionneurs
#define ON  1
#define OFF 0

#define RISING_EDGE 1
#define FALLING_EDGE 0 


//Delay
void delay(void);
void delays(void);
void delayms(void);
//ASSERVISSEMENT

int PiloteAcceleration(int acceleration);
int PiloteAvancer(double distance);
int PiloteReculer(double distance);
int PilotePivoter(double angle, Cote direction);
int PiloteVirage(unsigned char reculer, unsigned char direction, double rayon, double angle);
int PiloteStop(unsigned char stopmode);
int PiloteRecallage(Sens s);
int PiloteAvancerEtapes(int nombreEtapes, Etape etape);
int PiloteValiderEtapes(int numEtape);
int PiloteOffsetAsserv(double x, double y, double teta);

// DEBUG
Trame PiloteDebug0(Trame t);
Trame PiloteDebug1(Trame t);
Trame PiloteDebug2(Trame t);
Trame PiloteDebug3(Trame t);
Trame PiloteDebug4(Trame t);
Trame PiloteDebug5(Trame t);
Trame PiloteDebug6(Trame t);
Trame PiloteDebug7(Trame t);
Trame PiloteDebug8(Trame t);
Trame PiloteDebug9(Trame t);

//Analyse Trame
Trame AnalyseTrame(Trame t);

#endif // __PILOTAGE_H__

// Deplacements
#define	CMD_AVANCER						0x01
#define	CMD_PIVOTER						0x03
#define	CMD_VIRAGE						0x04
#define	CMD_STOP						0x05
#define	CMD_GOTOXY						0x06
#define	CMD_RECALLAGE					0x10
#define CMD_FINRECALLAGE				0x11
#define CMD_FINDEPLACEMENT				0x12
#define CMD_DEPLACEMENT_POLAIRE			0x20

// Asservissement
#define CMD_DEMANDEPOSITION				0x30
#define CMD_RETOURPOSITION				0x31
#define	CMD_VITESSE_LIGNE				0x32
#define	CMD_ACCELERATION_LIGNE			0x33
#define	CMD_VITESSE_PIVOT				0x34
#define	CMD_ACCELERATION_PIVOT			0x35
#define CMD_ENVOI_PID 					0x36
#define CMD_OFFSETASSERV				0x37
#define CMD_ENVOI_PID_CAP 				0x38
#define CMD_ENVOI_PID_VITESSE			0x39

// Debug asservissement
#define CMD_DEMANDE_BUFF_POSITION		0x43
#define CMD_REPONSE_BUFF_POSITION		0x44
#define CMD_CONSIGNE_POSITION			0x45
#define CMD_DEMANDE_BUFF_STATUS			0x46
#define CMD_REPONSE_BUFF_STATUS			0x47
#define CMD_PRD_ENVOI_POSITION			0x48

// Actionneurs
#define CMD_VITESSE_MOTEUR				0x67

// Capteurs
#define CMD_DEPART_JACK					0x71	
#define CMD_DEMANDE_COULEUR_EQUIPE		0x72
#define CMD_REPONSE_COULEUR_EQUIPE		0x73
#define CMD_ARME_JACK					0x70

// Diagnostic
#define	CMD_DEBUG						0xEE
#define	CMD_ECHO						0xF0
#define	CMD_RESET_CARTE					0xF1
#define CMD_DEMANDE_PRESENCE_JACK		0xF3
#define CMD_REPONSE_PRESENCE_JACK		0xF4

#define CMD_DEMANDE_VALEURS_ANALOGIQUES	0x76
#define CMD_REPONSE_VALEURS_ANALOGIQUES	0x77

#define CMD_DEMANDE_VALEURS_NUMERIQUES	0x78
#define CMD_REPONSE_VALEURS_NUMERIQUES	0x79

#define TRAME_UART2_ENVOI 0xA2
#define TRAME_UART2_RECEPTION 0xA3

#define TRAME_DETECTION_BALISE			0xE4
#define TRAME_DETECTION_BALISE_RAPIDE	0xE5

//2017 à trier
#define ID_MOTEUR_BALISE				0x10
#define CMD_MOTEUR_POSITION				0x66
#define CMD_DEMANDE_CAPTEUR				0x50
#define CMD_REPONSE_CAPTEUR				0x51
#define ID_BALISE 						0x01
#define ID_CAPTEUR_BALISE_1				0x02
#define ID_CAPTEUR_BALISE_2				0x03


//Diagnostic
#define TRAME_TEST_CONNEXION 0xF0

#define ID_HOKUYO_SOL 0x00

#define DEMANDE_PATTERN 0x23
#define REPONSE_PATTERN 0x24