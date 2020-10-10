#include <p33FJ128MC804.h>
#include "init.h"
#include "Pilotage.h"

unsigned int  BufferA[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));
unsigned int  BufferB[MAX_CHNUM+1][SAMP_BUFF_SIZE] __attribute__((space(dma),aligned(256)));
unsigned int ADC_Results[8],DmaBuffer = 0;

void InitClk(void)
{	
	PLLFBD = 38;				// Multiply by 40 for 160MHz VCO output (8MHz XT oscillator)
	CLKDIV = 0x0000;			// FRC: divide by 2, PLLPOST: divide by 2, PLLPRE: divide by 2
	
	__builtin_write_OSCCONH(0x03);
	__builtin_write_OSCCONL(OSCCON | 0x01);
	while(OSCCONbits.COSC != 0b011);
	while(OSCCONbits.LOCK != 1);
}

void Init_Interrupt_Priority(void)
{
	IPC14bits.QEI1IP = 7;			// Quad Encoder Interrupt
	IPC18bits.QEI2IP = 7;			// Quad Encoder Interrupt
	IPC7bits.U2RXIP = 6;
	IPC2bits.T3IP   = 5;			// Timer 3 used by Input Capture (IC1)
	IPC1bits.T2IP = 4; 				//Set Timer2 Interrupt Priority Level
	IPC15bits.DMA5IP  = 3;			// ADC Interrupt
	IPC0bits.T1IP    = 2;			// Timer 1 used by Ethernet (Default value = 2)
	IPC6bits.T4IP    = 1;			// Timer 4 Used by Asser
}

void InitUART2() // UART2 Gere le LIDAR
{
    U2BRG = 86;			// 86 pour 115200 // 520 pour 19200
	U2MODEbits.UARTEN = 1;		// UART2 is Enabled
	U2MODEbits.USIDL = 0;		// Continue operation at Idlestate
	U2MODEbits.IREN = 0;		// IrDA En/Decoder is disabled
	U2MODEbits.RTSMD = 0; 		// flow control mode
	U2MODEbits.UEN = 0b00;		// UTX, RTX, are enable and on use.
	U2MODEbits.WAKE = 0;		// Wake-up on start bit is enabled
	U2MODEbits.LPBACK = 0;		// Loop-back is disabled
	U2MODEbits.ABAUD = 0;		// auto baud is disabled
	U2MODEbits.URXINV = 0;		// No RX inversion
	U2MODEbits.BRGH = 1;		// High baud rate
	U2MODEbits.PDSEL = 0b00; 	// 8bit no parity
	U2MODEbits.STSEL = 0;		// one stop bit

	U2STAbits.UTXISEL1 = 0b00;
	U2STA &= 0xDFFF;			// clear TXINV by bit masking
	U2STAbits.UTXBRK = 0;		// sync break tx is disabled
	U2STAbits.UTXEN = 1;		// transmit  is enabled
	U2STAbits.URXISEL = 0b00;	// interrupt flag bit is set when RXBUF is filled whith 1 character
	U2STAbits.ADDEN = 0;		// address detect mode is disabled

//  IPC3bits.U2TXIP = 2;         // set UART Tx interrupt priority
	IFS1bits.U2TXIF = 0;         // clear UART Tx interrupt flag
	IEC1bits.U2TXIE = 0;         // enable UART Tx interrupt

	IFS1bits.U2RXIF = 0;		 // clear interrupt flag of rx
	IEC1bits.U2RXIE = 1;		 // enable rx recieved data
//  IPC2bits.U2RXIP = 2;
}


void InitPorts() 
{
	TRISAbits.TRISA0=1; // MOT4_I - Moteurs
	TRISAbits.TRISA1=1; // MOT2_I - Moteurs
	TRISAbits.TRISA2=1; // OSC - Oscillateur
	TRISAbits.TRISA3=1; // RA3 - Mezzanine /* MODIF ! PISTE COUPEE */
	TRISAbits.TRISA4=0; // RST - Ethernet RST
	TRISAbits.TRISA7=0; // RA7 - Mezzanine
	TRISAbits.TRISA8=0; // RA8 - Mezzanine // => Capteur couleur LED
	TRISAbits.TRISA9=0; // CS - Ethernet CS
	TRISAbits.TRISA10=0; // RA10 - Mezzanine
	
	TRISBbits.TRISB0=1; // MOT1_I - Moteurs
	TRISBbits.TRISB1=1; // MOT3_I - Moteurs
	TRISBbits.TRISB2=1; // RB2 - Mezzanine ==> 2018 : Recepteur IR
	TRISBbits.TRISB3=1; // RB3 - Mezzanine
	TRISBbits.TRISB4=1; // RB4 - Mezzanine
	TRISBbits.TRISB5=1; // PGD - JTAG 
	TRISBbits.TRISB6=1; // PGC - JTAG 
	TRISBbits.TRISB7=1; // INT - Ethernet INT
	TRISBbits.TRISB8=1; // CODEUR1B - Codeurs
	TRISBbits.TRISB9=1; // CODEUR1A - Codeurs
	TRISBbits.TRISB10=0; // MOT2_PWMH - Moteurs
	TRISBbits.TRISB11=0; // MOT2_PWML - Moteurs
	TRISBbits.TRISB12=0; // MOT3_PWMH - Moteurs
	TRISBbits.TRISB13=0; // MOT3_PWML - Moteurs
	TRISBbits.TRISB14=0; // MOT4_PWMH - Moteurs
	TRISBbits.TRISB15=0; // MOT4_PWML - Moteurs
	
	TRISCbits.TRISC0=0; // RC0 - Mezzanine
	TRISCbits.TRISC1=1; // RC1 - Mezzanine
	TRISCbits.TRISC2=1; // RC2 - Mezzanine
	TRISCbits.TRISC3=0; // SCK1 - Ethernet MSCK
	TRISCbits.TRISC4=0; // SDO1 - Ethernet MOSI
	TRISCbits.TRISC5=1; // SDI1 - Ethernet MISO
	TRISCbits.TRISC6=0; // MOT1_PWMH - Moteurs
	TRISCbits.TRISC7=0; // MOT1_PWML - Moteurs
	TRISCbits.TRISC8=1; // CODEUR2A - Codeurs
	TRISCbits.TRISC9=1; // CODEUR2B - Codeurs

	AD1PCFGL=0xFFFF;	//Tous les ports Analogiques configur�s en num�rique
	
	RPINR14bits.QEA1R = 9;			// QEIA1	<==> RP9 
	RPINR14bits.QEB1R = 8;			// QEIB1	<==> RP8 
	RPINR16bits.QEA2R = 24;			// QEIA2	<==> RP24 
	RPINR16bits.QEB2R = 25;			// QEIB2	<==> RP25

	//Configuration des ports pour la liaison SPI avec le module Ethernet
	RPOR9bits.RP19R   = 0b01000; // SCK1 		<==> RP19 RC3
	RPOR10bits.RP20R  = 0b00111; // SDO1 		<==> RP20 RC4
	RPINR20bits.SDI1R = 21     ; // SDI1 		<==> RP21 RC5

	//Confguration des ports pour le module UART2 (LIDAR)
	RPOR8bits.RP16R = 0b00101;	//TX RP16
    RPINR19bits.U2RXR = 3; //RX RP3

	// Capteur de couleur OUT  
	RPINR7bits.IC1R 	= 4;		//RP4
}

void Init_Timer2(void)		
{
	T2CONbits.TON 	= 0;	//Stops the timer
	T2CONbits.TSIDL = 0;
	T2CONbits.TGATE = 0;
	T2CONbits.TCS	= 0;
	T2CONbits.T32	= 0;
	T2CONbits.TCKPS = 0b10; //Prescaler set to 1:64
	
	TMR2 = 0; 				//Clear timer register
	PR2  = 1;				//Load the period value (1 = 3.2us)

	IFS0bits.T2IF = 0; 		//Clear Timer2 Interrupt Flag
	IEC0bits.T2IE = 1; 		//Enable Timer2 interrupt
	T2CONbits.TON = 1;		//Timer enabled
}

void Init_Input_Capture(void)
{
	// Use Timer 3 for IC1
	Init_Timer3();

	// Set IC1 to Capture
	IC1CONbits.ICM		= 0;		// Disable Input Capture 1 Module
	IC1CONbits.ICTMR	= 0;		// Select Timer 3 as the time base
	IC1CONbits.ICI		= 0b01;		// Interrupt on every 2nd capture event
	IC1CONbits.ICM		= 0b101;	// Capture mode, every 16th rising edge	

	IFS0bits.IC1IF = 0;
	IEC0bits.IC1IE = 1;
}

void Init_Timer3(void)
{
//	T3CONbits.TON 	= 0;	//Stops the timer
//	T3CONbits.TSIDL = 0;
//	T3CONbits.TGATE = 0;
//	T3CONbits.TCS	= 0;
	T3CONbits.TCKPS = 0b01; //Prescaler set to 1:8

	IFS0bits.T3IF = 0; 		//Clear Timer3 Interrupt Flag
	IEC0bits.T3IE = 0; 		//Disable Timer3 interrupt	
	
	T3CONbits.TON = 1;		//Starts the timer
}


void Init_Timer4(void)
{
	//--Timer4
	T4CONbits.TON 	= 0;	//Stops the timer
	T4CONbits.TSIDL = 0;
	T4CONbits.TGATE = 0;
	T4CONbits.TCS	= 0;
	T4CONbits.TCKPS = 0b01; //Prescaler set to 1:8
	
	TMR4 = 0; 				//Clear timer register
	PR4  = 5000; 			//Load the period value (Pas) 1/(40e6/8/1250) = 1ms

//	IPC6bits.T4IP = 6; 		//Set Timer4 Interrupt Priority Level
	IFS1bits.T4IF = 0; 		//Clear Timer4 Interrupt Flag
	IEC1bits.T4IE = 1; 		//Enable Timer4 interrupt
	
	T4CONbits.TON = 1;		//Starts the timer
}

void Init_Timer5(void)
{
	//--Timer5
	T5CONbits.TON 	= 0;	//Stops the timer
	T5CONbits.TSIDL = 0;
	T5CONbits.TGATE = 0;
	T5CONbits.TCS	= 0;
	T5CONbits.TCKPS = 0b10; //Prescaler set to 1:64
	
	TMR5 = 0; 				//Clear timer register
	PR5  = (unsigned int)65536; 			//Load the period value (Pas) 1/(40e6/64/65536) = 104.8576 ms

	T5CONbits.TON = 1;		//Starts the timer
}

void InitPWM(void)
{
	// Attention Omybot's touch :
	// MOT1 est g�r� par  PWM2 paire 1 => Hacheur n�1 (IC2)
	// MOT2 est g�r� par  PWM1 paire 3 => Hacheur n�1 (IC2)
	// MOT3 est g�r� par  PWM1 paire 2 => Hacheur n�2 (IC3)
	// MOT4 est g�r� par  PWM1 paire 1 => Hacheur n�2 (IC3)
	// 

	P1TCONbits.PTEN = 1; 		// PWM Time base is On
	P1TPER = 4000 - 1; 			// 10kHz PWM (4000 counts @40MIPS)
	PWM1CON1bits.PEN1L = 0;		// PWM1L1 pin is disabled for PWM output
	PWM1CON1bits.PEN1H = 0;		// PWM1H1 pin is disabled for PWM output
	PWM1CON1bits.PEN2L = 0;		// PWM1L2 pin is disabled for PWM output
	PWM1CON1bits.PEN2H = 0;		// PWM1H2 pin is disabled for PWM output
	PWM1CON1bits.PEN3L = 0;		// PWM1L3 pin is disabled for PWM output
	PWM1CON1bits.PEN3H = 0;		// PWM1H3 pin is disabled for PWM output

	P1DC1 = 0; // 0		==> PWM 0%
	P1DC2 = 0; // 4000	==> PWM 50%
	P1DC3 = 0; // 8000	==> PWM 100%

	P2TCONbits.PTEN = 1; 		// PWM Time base is On
	P2TPER = 4000 - 1; 			// 10kHz PWM (4000 counts @40MIPS)
	PWM2CON1bits.PEN1L = 0;		// PWM2L1 pin is disabled for PWM output
	PWM2CON1bits.PEN1H = 0;		// PWM2L2 pin is disabled for PWM output

	P2DC1 = 4000;
}

void InitQEI(void)
{
	QEI1CONbits.QEIM  = 0b111;
	QEI2CONbits.QEIM  = 0b111;
	QEI1CONbits.SWPAB = 1;
	QEI2CONbits.SWPAB = 0;
	POS1CNT = 0x0000;
	POS2CNT = 0x0000;
	IFS3bits.QEI1IF = 0;
	IFS4bits.QEI2IF = 0;
	IEC3bits.QEI1IE = 1;
	IEC4bits.QEI2IE = 1;
}

void InitADC(void)
{
	AD1CON1bits.FORM   = 0;		// Data Output Format: Integer
	AD1CON1bits.SSRC   = 7;		// Sample Clock Source: Conversion autostart
	AD1CON1bits.ASAM   = 1;		// ADC Sample Control: Sampling begins immediately after conversion
	AD1CON1bits.AD12B  = 1;		// 12-bit ADC operation

	AD1CON2bits.CSCNA = 1;		// Scan Input Selections for CH0+ during Sample A bit
	AD1CON2bits.CHPS  = 0;		// Converts CH0

	AD1CON3bits.ADRC = 0;		// ADC Clock is derived from Systems Clock
	AD1CON3bits.ADCS = 63;		// ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
								// ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us	

	AD1CON1bits.ADDMABM = 0; 	// DMA buffers are built in scatter/gather mode
	AD1CON2bits.SMPI    = (NUM_CHS2SCAN-1);	// 6 ADC Channel is scanned
	AD1CON4bits.DMABL   = 3;	// Each buffer contains 8 words

	//AD1CSSH/AD1CSSL: A/D Input Scan Selection Register
	AD1CSSLbits.CSS0=1;		// Enable AN0 for channel scan
	AD1CSSLbits.CSS1=1;		// Enable AN1 for channel scan
	AD1CSSLbits.CSS2=1;		// Enable AN2 for channel scan
	AD1CSSLbits.CSS3=1;		// Enable AN3 for channel scan
	AD1CSSLbits.CSS6=1;		// Enable AN6 for channel scan
	AD1CSSLbits.CSS7=1;		// Enable AN7 for channel scan
	
 	//AD1PCFGH/AD1PCFGL: Port Configuration Register
	AD1PCFGL=0xFFFF;
	AD1PCFGLbits.PCFG0 = 0;	// AN0 as Analog Input
	AD1PCFGLbits.PCFG1 = 0;	// AN1 as Analog Input 
 	//AD1PCFGLbits.PCFG2 = 0;	// AN2 as Analog Input
	//AD1PCFGLbits.PCFG3 = 0;	// AN3 as Analog Input 
	//AD1PCFGLbits.PCFG6 = 0;	// AN6 as Analog Input
	//AD1PCFGLbits.PCFG7 = 0;	// AN7 as Analog Input 
	
	IFS0bits.AD1IF   = 0;		// Clear the A/D interrupt flag bit
	IEC0bits.AD1IE   = 0;		// Do Not Enable A/D interrupt 
	AD1CON1bits.ADON = 1;		// Turn on the A/D converter
}

void InitDMA(void)
{
	DMA5CONbits.AMODE = 2;			// Configure DMA for Peripheral indirect mode
	DMA5CONbits.MODE  = 2;			// Configure DMA for Continuous Ping-Pong mode
	DMA5PAD=(int)&ADC1BUF0;
	DMA5CNT = (SAMP_BUFF_SIZE*NUM_CHS2SCAN)-1;					
	DMA5REQ = 13;					// Select ADC1 as DMA Request source

	DMA5STA = __builtin_dmaoffset(BufferA);		
	DMA5STB = __builtin_dmaoffset(BufferB);

	IFS3bits.DMA5IF = 0; //Clear the DMA interrupt flag bit
	IEC3bits.DMA5IE = 1; //Set the DMA interrupt enable bit
	
	DMA5CONbits.CHEN=1;				// Enable DMA
}

void __attribute__((interrupt, no_auto_psv)) _DMA5Interrupt(void)
{
	unsigned char i;
	ADC_Results[0]=0;
	ADC_Results[1]=0;
	ADC_Results[2]=0;
	ADC_Results[3]=0;
	ADC_Results[4]=0;
	ADC_Results[5]=0;
	if(DmaBuffer == 0)
	{
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[0] += BufferA[0][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[1] += BufferA[1][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[2] += BufferA[2][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[3] += BufferA[3][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[4] += BufferA[6][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[5] += BufferA[7][i];
		ADC_Results[0] /= SAMP_BUFF_SIZE;
		ADC_Results[1] /= SAMP_BUFF_SIZE;
		ADC_Results[2] /= SAMP_BUFF_SIZE;
		ADC_Results[3] /= SAMP_BUFF_SIZE;
		ADC_Results[4] /= SAMP_BUFF_SIZE;
		ADC_Results[5] /= SAMP_BUFF_SIZE;
	}
	else
	{
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[0] += BufferB[0][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[1] += BufferB[1][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[2] += BufferB[2][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[3] += BufferB[3][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[4] += BufferB[6][i];
		for(i=0;i<SAMP_BUFF_SIZE;i++)	ADC_Results[5] += BufferB[7][i];
		ADC_Results[0] /= SAMP_BUFF_SIZE;
		ADC_Results[1] /= SAMP_BUFF_SIZE;
		ADC_Results[2] /= SAMP_BUFF_SIZE;
		ADC_Results[3] /= SAMP_BUFF_SIZE;
		ADC_Results[4] /= SAMP_BUFF_SIZE;
		ADC_Results[5] /= SAMP_BUFF_SIZE;
	}
	
	DmaBuffer ^= 1;

	IFS3bits.DMA5IF = 0;		// Clear the DMA0 Interrupt Flag
}
