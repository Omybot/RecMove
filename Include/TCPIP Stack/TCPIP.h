/*********************************************************************
 *
 *  Microchip TCP/IP Stack Include File
 *
 *********************************************************************
 * FileName:        TCPIP.h
 * Dependencies:    
 * Processor:       PIC18, PIC24F, PIC24H, dsPIC30F, dsPIC33F, PIC32
 * Compiler:        Microchip C32 v1.05 or higher
 *					Microchip C30 v3.12 or higher
 *					Microchip C18 v3.30 or higher
 *					HI-TECH PICC-18 PRO 9.63PL2 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2009 Microchip Technology Inc.  All rights
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and
 * distribute:
 * (i)  the Software when embedded on a Microchip microcontroller or
 *      digital signal controller product ("Device") which is
 *      integrated into Licensee's product; or
 * (ii) ONLY the Software driver source files ENC28J60.c, ENC28J60.h,
 *		ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device
 *		used in conjunction with a Microchip ethernet controller for
 *		the sole purpose of interfacing with the ethernet controller.
 *
 * You should refer to the license agreement accompanying this
 * Software for additional information regarding your rights and
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *
 * Author               Date    	Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Howard Schlunder		12/20/06	Original
 ********************************************************************/
#ifndef __TCPIP_HITECH_WORKAROUND_H
#define __TCPIP_HITECH_WORKAROUND_H

#define VERSION 		"v5.10"		// TCP/IP stack version

#include <string.h>
#include <stdlib.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"

// RESERVED FEATURE -- do not change from current value of 1u as this is not 
// fully implemented yet.
// Defines the number of different network interfaces to support (ex: 2 for 
// Wifi and Ethernet simultaneously).
#define NETWORK_INTERFACES		(1u)	

/*******************************************************************
 * Memory Configuration
 *   The following section sets up the memory types for use by
 *   this application.
 *******************************************************************/
	// Represents data stored in Ethernet buffer RAM
	#define TCP_ETH_RAM	0u
	// The base address for TCP data in Ethernet RAM
	#define TCP_ETH_RAM_BASE_ADDRESS			(BASE_TCB_ADDR)
	// Represents data stored in local PIC RAM
	#define TCP_PIC_RAM	1u
	// The base address for TCP data in PIC RAM
	#define TCP_PIC_RAM_BASE_ADDRESS			((PTR_BASE)&TCPBufferInPIC[0])
	// Represents data stored in external SPI RAM
	#define TCP_SPI_RAM	2u

/*******************************************************************
 * User Configuration
 *   Load the user-specific configuration from TCPIPConfig.h
 *******************************************************************/
#include "TCPIPConfig.h"

/*******************************************************************
 * Configuration Rules Enforcement
 *   The following section enforces requirements for modules based 
 *   on configurations selected in TCPIPConfig.h
 *******************************************************************/

	// When IP Gleaning is enabled, ICMP must also be enabled.
	#if defined(STACK_USE_IP_GLEANING)
	    #if !defined(STACK_USE_ICMP_SERVER)
	        #define STACK_USE_ICMP_SERVER
	    #endif
	#endif
	
	// If TCP is not enabled, clear all memory allocations
	#if !defined(STACK_USE_TCP)
		#undef TCP_ETH_RAM_SIZE
		#undef TCP_PIC_RAM_SIZE
		#undef TCP_SPI_RAM_SIZE
		#define TCP_ETH_RAM_SIZE 0u
		#define TCP_PIC_RAM_SIZE 0u
		#define TCP_SPI_RAM_SIZE 0u
	#endif
	
	// Enable the LCD if configured in the hardware profile
	#if defined(LCD_DATA_IO) || defined(LCD_DATA0_IO)
		#define USE_LCD
	#endif
	
#include "TCPIP Stack/StackTsk.h"
#include "TCPIP Stack/Helpers.h"
#include "TCPIP Stack/Delay.h"
#include "TCPIP Stack/Tick.h"
#include "TCPIP Stack/MAC.h"
#include "TCPIP Stack/IP.h"
#include "TCPIP Stack/ARP.h"

#if defined(STACK_USE_UDP)
	#include "TCPIP Stack/UDP.h"
#endif

#if defined(STACK_USE_ICMP_SERVER) || defined(STACK_USE_ICMP_CLIENT)
	#include "TCPIP Stack/ICMP.h"
#endif

#endif //__TCPIP_HITECH_WORKAROUND_H
