#ifndef __PILOTAGE_H__
#define __PILOTAGE_H__

#include "Types.h"
#include "UserUdp.h"

// Constantes des fonctions des actionneurs
#define ON  1
#define OFF 0

#define RISING_EDGE 1
#define FALLING_EDGE 0 

// Moteur sens
#define SENS_DROITE 3
#define SENS_GAUCHE 2

// Capteur de couleur
typedef struct Rgb
{
  unsigned char red;
  unsigned char green;
  unsigned char blue;
}Rgb;
unsigned int Send_Variable_Capteur_Couleur(void);
Trame CouleurRGB(int Id);
double period2frequency(unsigned int period);
Rgb frequency2RGB(double freqClear, double freqRed, double freqGreen, double freqBlue);

void EnvoiUART(Trame t);

Trame Retour_Capteur_Onoff(unsigned char id_capteur);
Trame PiloteGotoXY(int x, int y, unsigned char x_negatif, unsigned char y_negatif);
Trame ReponseConnexion(void);
Trame StatusMonitor(void);

Trame PiloteGetBuffPosition(void);
void PilotePIDCoeffs(unsigned int new_kp, unsigned int new_ki, unsigned int new_kd);
Trame Retour_Valeurs_Analogiques(void);
Trame Retour_Valeurs_Numeriques(void);

Trame PilotePositionXYT(void);
Trame PiloteDemandeCapteur(char numCapteur);
Trame PiloteCapteurs(char cote);
void PiloteActionneurOnOff(unsigned char id, OnOff onOff);
void PiloteMoteurPosition(unsigned char id, unsigned int position);

//ASSERVISSEMENT
int PiloteAcceleration(int acceleration);
int PiloteAvancer(double distance);
int PiloteReculer(double distance);
int PilotePivoter(double angle, Cote direction);
int PiloteVirage(unsigned char reculer, unsigned char direction, double rayon, double angle);
int PiloteStop(unsigned char stopmode);
int PiloteRecallage(Sens s);
int PiloteOffsetAsserv(double x, double y, double teta);
Trame PiloteDebug(Trame t, int debugNo);

//Analyse Trame
Trame AnalyseTrame(Trame t);

#endif // __PILOTAGE_H__

// Deplacements
#define	CMD_AVANCER						0x01
#define	CMD_PIVOTER						0x03
#define	CMD_VIRAGE						0x04
#define	CMD_STOP						0x05
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

// Capteur de couleur
#define CMD_DEMANDE_CAPTEUR_COULEUR 	0x52
#define CMD_REPONSE_CAPTEUR_COULEUR 	0x53

// Actionneurs
#define TRAME_PILOTAGE_ONOFF 			0x65

// Liste des actionneurs
#define ALIMENTATION_CAPTEUR_COULEUR	0x00
#define MAKEVACUUM_BACK					0x14
#define MAKEVACUUM_FRONT				0x12
#define OPENVACUUM_BACK					0x23
#define OPENVACUUM_FRONT				0x21

#define VACUOSTAT_BACK					0x14
#define VACUOSTAT_FRONT					0x12
#define JACK_DEMARRAGE					0x10
// Capteurs onOff
#define CMD_DEMANDE_CAPTEUR_ONOFF		0x74
#define CMD_REPONSE_CAPTEUR_ONOFF		0x75

// Diagnostic
#define	CMD_DEBUG						0xEE

#define CMD_DEMANDE_VALEURS_ANALOGIQUES	0x76
#define CMD_REPONSE_VALEURS_ANALOGIQUES	0x77

#define CMD_DEMANDE_VALEURS_NUMERIQUES	0x78
#define CMD_REPONSE_VALEURS_NUMERIQUES	0x79

#define TRAME_UART2_ENVOI 				0xA2
#define TRAME_UART2_RECEPTION 			0xA3

#define CMD_MOTEUR_POSITION				0x66


//Diagnostic
#define TRAME_TEST_CONNEXION 0xF0

#define ID_HOKUYO_SOL 0x00
