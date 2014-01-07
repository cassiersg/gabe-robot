/*********************************************************************
 *
 *********************************************************************
 * 			OLIMEX PIC-32MX TEST
 *********************************************************************
 *
 * This file is the main file of a progrqm intended to control servo
 * which will be assembled together to form a robot
 * 
 * The servo are connected on RD0 .. RD7; RE0 .. RE4
 * The program also control a LED through RF1
 *
 * timer2 is used to give a time information
 * timer3 is controlling the servo position
 * timer1 is controlling the blinking status LED (using external OSC)
 *
 * A console (UART2) is available to control the program from outside
 *
 *
 * Oscillator Configuration Bit Settings:
 * 		- Oscillator Selection Bits = 	Primary Osc w/PLL (XT+HS+EC+PLL)
 *		- Primary Oscillator Config = 	HS osc mode
 *		- PLL Input Divider			=	2x Divider
 *		- PLL Multiplier			=	18x Multiplier
 *
 * Notes:
 *		- Timer1 clock 		= SOSC/PRESCALE
 *							= 32768/1 = 32.768KHz
 *		- To toggle led once every half second, PR1 of Timer1 is loaded with 16384 = 0x4000
 *
 ********************************************************************/
#include <plib.h>
#include <stdio.h>
#include "basic_types.h"
#include "robot_time.h"
#include "robot_console.h"
#include "motor_control.h"
#include "pod_control.h"
#include "status_led.h"
#include "historical.h"
#include "movement.h"

// Configuration Bit settings
// SYSCLK = 72 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 36 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// FSOSCEN ON - this enables the SOSC to work independantly
// Other options are don't care
//
#pragma config FPLLMUL = MUL_18, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF, FSOSCEN = ON
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_2

#define FOSC           		72E6

#define DESIRED_BAUDRATE    	(115200)      //The desired BaudRate




int main(void)
{
	int i;
	int	pbClk;
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //STEP 1. Configure cache, wait states and peripheral bus clock
	// Configure the device for maximum performance.
	// This macro sets flash wait states, PBCLK divider and DRM wait states based on the specified
	// clock frequency. It also turns on the cache mode if avaialble.
	// Based on the current frequency, the PBCLK divider will be set at 1:2. This knoweldge
	// is required to correctly set UART baud rate, timer reload value and other time sensitive
	// setting.

	pbClk = SYSTEMConfigPerformance(FOSC);

    // Enable multi-vector interrupts
    INTEnableSystemMultiVectoredInt();

	console_init(pbClk,DESIRED_BAUDRATE);
	time_init();
	status_led_init();
	motor_init();
    historical_init();
	move_init();
    pod_init();

	// Time delay  => was inside the inital example - do not know why:::
	i = 512*512;		
	while(i) {i--;}
	
	// Send 'Hello' message through RS232
    putsUART("\n*** WELCOME TO THE ROBOT'S WORLD ***\r\n");
    putsUART("*** Type help is needed ***\r\n");
	putsUART(">>> ");

    while(1)
	{
	    // while(mPORTDRead() & 0x0100);		// Wait until button pushed 
		// while(!(mPORTDRead() & 0x0100));	// Wait until button released 
		// printf("hello this is the time %i\r\n", getTime()/112500);
        // PORTSetBits(IOPORT_F, BIT_1);	// Turn on LED
		console_process();
		process_move();
	}  
}
