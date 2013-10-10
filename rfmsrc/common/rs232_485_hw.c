/*
 *  Open HR20
 *
 *  target:     ATmega169 in Honnywell Rondostat HR20E / ATmega32
 *
 *  compiler:   WinAVR-20071221
 *              avr-libc 1.6.0
 *              GCC 4.2.2
 *
 *  copyright:  2008 Juergen Sachs (juergen-sachs-at-gmx-dot-de)
 *				2008 Jiri Dobry (jdobry-at-centrum-dot-cz)
 *
 *  license:    This program is free software; you can redistribute it and/or
 *              modify it under the terms of the GNU Library General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later version.
 *
 *              This program is distributed in the hope that it will be useful,
 *              but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *              GNU General Public License for more details.
 *
 *              You should have received a copy of the GNU General Public License
 *              along with this program. If not, see http:*www.gnu.org/licenses
 */

/*!
 * \file       rs232_485_hw.c
 * \brief      hardware layer of the rs232 and rs485
 * \author     Juergen Sachs (juergen-sachs-at-gmx-dot-de); Jiri Dobry <jdobry-at-centrum-dot-cz>
 * \date       $Date$
 * $Rev$
 */

// AVR LibC includes
#include <stdint.h>
#include <avr/io.h>
#include <stdio.h>

// HR20 Project includes
#include "config.h"
#include "main.h"
#include "com.h"

/* The following must be AFTER the last include line */
#if (defined COM_RS232) || (defined COM_RS485)

#if defined (_AVR_IOM169_H_) || defined (_AVR_IOM32_H_)
#define UDR0 UDR
#define RXEN0 RXEN
#define RXCIE0 RXCIE
#define UCSR0A UCSRA
#define UCSR0B UCSRB
#define TXC0 TXC
#define TXCIE0 TXCIE
#define UDRIE0 UDRIE
#define TXEN0 TXEN
#define UBRR0H UBRRH
#define UBRR0L UBRRL
#define UCSR0C UCSRC
#define UCSZ00 UCSZ0
#define UCSZ01 UCSZ1
#define U2X0 U2X
#define URSEL0 URSEL
#endif

#if defined COM_RS485

extern void clearRx(void);

/************************************************************************/
/* Control RTS output                                                   */
/************************************************************************/
// enable > 0 to enable transmitter, disable receiver.
// Done this way so polarity of output can easily be changed if needed.
// 
//  NOTE: This is called within ISR, so other use of PORTE needs to take account of it -  @TODO:
//
void serialOutput(unsigned char enable)
{
	if (enable)
	{
		PORTE &= ~(1<<2);		// Take bit low
	}
	else
	{
		PORTE |= (1<<2);		// Take bit high
	}
}


extern void serialInit(uint8_t isRS232);
extern void COM_txInit(void);

#endif



/*
 *	Enable the UART to send data which is already in Tx buffer
 */
void startUARTSend(void)
{
	COM_txInit();								// Initialise transmit pointers
	if ((UCSR0B & _BV(UDRIE0))==0) 
	{
		UCSR0B &= ~(_BV(TXCIE0));				// Clear 'Transmit complete' interrupt enable
		UCSR0A |= _BV(TXC0);					// clear 'Transmit complete' interrupt flag - just in case
		UCSR0B |= _BV(UDRIE0) | _BV(TXEN0);		// Enable transmitter, USART data empty interrupt
	}
}



#if defined COM_RS485
/*
 *	Routines associated with sending RS-485 data.
 *	A sequence is triggered by calling RS_start_send():
 *		1. Timer 1 is started, with a 5ms delay to interrupt (uses OC1A)
 *		2. On timer interrupt, RTS set active. Output compare also resets the timer count
 *		3. On next timer interrupt:
 *			- timer stopped
 *			- UART enabled to send data
 *		4. When all data sent:
 *			- UART disabled
 *			- Timer 1 is enabled with interrupt on OC1B after about 2.5ms
 *		5. On OC1B interrupt:
 *			- RTS disabled
 *			- Timer 1 stopped
 */
volatile unsigned char serialState;
#define SERIAL_STATE_IDLE	0
#define SERIAL_STATE_DELAY	1		/* Wait for distant end to disable transmitter after we've got a complete Rx message */
#define SERIAL_STATE_SETUP	2		/* Setup time - RTS active, not transmitting */
#define SERIAL_STATE_SENDING 3		/* Transmission time - do nothing */
#define SERIAL_STATE_HOLD	4		/* Hold RTS active after Tx complete */
#define SERIAL_STATE_COMPLETE 5		/* RTS ready to go inactive */
#define SERIAL_STATE_RECEIVING 10	/* Receiving data - timer is monitoring inter-character gap */

//volatile uint8_t rxTimeoutCounter = 0;	// Debug aid - can go later

/*
 *	Start timer 1 - used for receive timeout on protocol-driven serial
 */
void startRxTimer(void)
{
//	rxTimeoutCounter = 0;		// Reset counter when first character received
	PRR &= ~(1<<PRTIM1);				// Enable timer/counter
	TCNT1 = 0;							// Set counter value to zero
	TCCR1B= _BV(CS11) | _BV(WGM12); 	// clk/8 CTC mode - start timer (resets count on OC1A match)
	TIMSK1 |= _BV(OCIE1A);
	serialState = SERIAL_STATE_RECEIVING;
}


void retriggerRxTimer(void)
{
	if (serialState != SERIAL_STATE_RECEIVING) return;
	TCNT1 = 0;							// Set counter value to zero
}


void stopRxTimer(void)
{
	TCCR1B = 0;								// Stop counter
	TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B));	// Disable interrupts
	PRR |= (1<<PRTIM1);						// Disable timer/counter
	serialState = SERIAL_STATE_IDLE;		// Force idle condition	
	#if !defined(_AVR_IOM16_H_) && !defined(_AVR_IOM32_H_)
	PCMSK0 |= (1<<PCINT0);					// should already be active, but make sure
	#endif
}


void startTxSendTimer(void)
{
	PRR &= ~(1<<PRTIM1);				// Enable timer/counter
	serialState = SERIAL_STATE_DELAY;
   	TIFR1  = _BV(OCF1A) | _BV(OCF1B); 	// clear interrupt-flags
	TCNT1 = 0;							// Set counter value to zero
	TCCR1B= _BV(CS11) | _BV(WGM12); 	// clk/8 CTC mode - start timer (resets count on OC1A match)
	TIMSK1 |= _BV(OCIE1A);				// Enable interrupt from OC1A
}

void startTxHoldTimer(void)
{
	PRR &= ~(1<<PRTIM1);				// Enable timer/counter
	serialState = SERIAL_STATE_HOLD;
   	TIFR1  = _BV(OCF1A) | _BV(OCF1B); 	// clear interrupt-flags
	TCNT1 = 0;							// Set counter value to zero
	TCCR1B= _BV(CS11) | _BV(WGM12); 	// clk/8 CTC mode - start timer again
	TIMSK1 |= _BV(OCIE1B);				// Enable compare B - shorter time
}

/*
	Called to disable RTS and turn off counter 1.
 */
void txDisable(void)
{
	if (serialState != SERIAL_STATE_HOLD) return;
	TCCR1B = 0;								// Stop counter
	TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B));	// Disable interrupts
	PRR |= (1<<PRTIM1);						// Disable timer/counter
	serialOutput(0);						// Turn off RTS
	serialState = SERIAL_STATE_IDLE;		// Force idle condition
}


/*
 Called to signal that no more characters are available to transmit - initiate transmitter disable after current character sent
*/
void txUartDisable(void)
{
	UCSR0B &= ~(_BV(UDRIE0));		// Disable 'Tx Data Register empty' interrupt
	UCSR0A |= _BV(TXC0);			// clear interrupt flag
	UCSR0B |= (_BV(TXCIE0));		// Enable 'Transmission complete' flag
}
/************************************************************************/
/*			Timer 1 ISR                                                 */
/************************************************************************/

// Output compare A - triggered twice in succession - once to turn on RTS, once to start sending
ISR(TIMER1_COMPA_vect)
{
	switch (serialState)
	{
		case SERIAL_STATE_DELAY :
			serialOutput(1);						// Enable RTS
			serialState = SERIAL_STATE_SETUP;
			break;
		case SERIAL_STATE_SETUP :
			TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B));	// Disable interrupts
			TCCR1B = 0;								// Stop timer while sending
			serialState = SERIAL_STATE_SENDING;
			// Start the actual send here
			startUARTSend();
			break;
		case SERIAL_STATE_RECEIVING :				// Receive inter-character gap expired
			//rxTimeoutCounter++;					// Note that timeout occurred
			TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B));	// Disable interrupts
			TCCR1B = 0;								// Stop timer
			clearRx();								// Abandon any receive in progress (stops timer again, as well)
			break;
	}
}



// Output compare B - triggered once at the end of Tx hold time, to turn off RTS
ISR(TIMER1_COMPB_vect)
{
	txDisable();
}
#endif



/*!
 *******************************************************************************
 *  Interrupt for receiving bytes from serial port
 *
 *  \note
 *	When character has been received, disables UART receive and enables the edge detect again
 ******************************************************************************/
#if defined (_AVR_IOM169P_H_) || defined (_AVR_IOM169_H_) || defined (_AVR_IOM329_H_)
ISR(USART0_RX_vect)
#elif defined(_AVR_IOM16_H_) || defined(_AVR_IOM32_H_)
ISR(USART_RXC_vect)
#endif
{
	COM_rx_char_isr(UDR0);	// Add char to input buffer
	#if (!defined(MASTER_CONFIG_H) && !defined(COM_RECEIVE_ACTIVE))
		UCSR0B &= ~(_BV(RXEN0)|_BV(RXCIE0));	// disable receive
	#endif
	#if !defined(_AVR_IOM16_H_) && !defined(_AVR_IOM32_H_)
	    PCMSK0 |= (1<<PCINT0);			// activate RxD edge detect input again
	#endif
}

/*!
 *******************************************************************************
 *  Interrupt, we can send one more byte to serial port
 *
 *  \note
 *  - We send one byte
 *  - If we have send all, disable interrupt
 ******************************************************************************/
#if defined (_AVR_IOM169P_H_) || defined(_AVR_IOM329_H_)
ISR(USART0_UDRE_vect)
#elif defined(_AVR_IOM169_H_) || defined(_AVR_IOM16_H_)  || defined(_AVR_IOM32_H_)
ISR(USART_UDRE_vect)
#endif
{
	#if COM_RS485
	UDR0 = COM_tx_char_isr();			// Transmitter disable handled differently
	#else
	char c;
	if ((c=COM_tx_char_isr())!='\0')
	{
			UDR0 = c;
	}
	else	// no more chars, disable Interrupt
	{
		UCSR0B &= ~(_BV(UDRIE0));
		UCSR0A |= _BV(TXC0); // clear interrupt flag
		UCSR0B |= (_BV(TXCIE0));
	}
	#endif
}

/*!
 *******************************************************************************
 *  Interrupt for transmit done
 *
 *  \note
 ******************************************************************************/
#if defined (_AVR_IOM169P_H_) || defined (_AVR_IOM169_H_) || defined(_AVR_IOM329_H_)
ISR(USART0_TX_vect)
#elif defined(_AVR_IOM16_H_) || defined(_AVR_IOM32_H_)
ISR(USART_TXC_vect)
#endif
{
	#if defined COM_RS485
		startTxHoldTimer();						// Delay before we disable RS-485 buffer
	#endif
	UCSR0A &= ~(_BV(TXC0));						// Disable 'Tx Buffer empty' interrupt
	UCSR0B &= ~(_BV(TXCIE0)|_BV(TXEN0));		// Disable transmitter
}



/*!
 *******************************************************************************
 *  Initialise serial, setup Baudrate
 *
 *	For RS-485, initialise what we can for timer 1
 *
 *  \note
 *  - set Baudrate
 ******************************************************************************/
void RS_Init(void)
{
	#if defined COM_RS485
		txDisable();						// Make sure RTS inactive. Set Tx state machine and timer to idle.
	
		// Set up timer 1 ready to use for RS-485 control
		PRR &= ~(1<<PRTIM1);				// Enable timer/counter
		TCCR1A = 0;							// Should be anyway
		TCCR1C = 0;							// Should be anyway
    	//OCR1A = 6250-1; 					// 5ms period	(value gave twice that expected)
    	//OCR1B = 3125 - 1;					// 2.5ms approx
    	OCR1A = 3125-1; 					// 5ms period
    	OCR1B = 1563 - 1;					// 2.5ms approx
    	TIFR1  = _BV(OCF1A) | _BV(OCF1B); 	// clear interrupt-flags
		serialInit(0);						// Initialise protocol handler
		PRR |= (1<<PRTIM1);					// Disable timer/counter
		
		
		serialInit(1);					// Allow RS-232 protocol as well, for now. TODO: Delete this line!
	#endif
	// Baudrate
	
	#if defined COM_FAST_BAUD_CLOCK
	uint16_t ubrr_val = ((F_CPU)/(COM_BAUD_RATE*16L)-1);		// Option to use 16X UART clock instead of 8X
	UCSR0A = 0;
	#else
	uint16_t ubrr_val = ((F_CPU)/(COM_BAUD_RATE*8L)-1);
	UCSR0A = _BV(U2X0);
	#endif

	UBRR0H = (unsigned char)(ubrr_val>>8);
	UBRR0L = (unsigned char)(ubrr_val & 0xFF);
	#if defined(_AVR_IOM16_H_) || defined(_AVR_IOM32_H_)
        UCSR0C = (1<<URSEL0) | (_BV(UCSZ00) | _BV(UCSZ01));     // Asynchron 8N1
        #if defined(MASTER_CONFIG_H)
			UCSR0B = _BV(RXCIE0) | _BV(RXEN0);
		#endif
    #else 
        UCSR0C = (_BV(UCSZ00) | _BV(UCSZ01));     // Asynchron 8N1
		#if defined(COM_RECEIVE_ACTIVE)
			UCSR0B = _BV(RXCIE0) | _BV(RXEN0);
		#endif
    #endif

	#if !defined(_AVR_IOM16_H_) && !defined(_AVR_IOM32_H_) && !defined(COM_RECEIVE_ACTIVE)
	    PCMSK0 |= (1<<PCINT0);				// activate interrupt to detect falling edge on RxData
	#endif
}


/*!
 *******************************************************************************
 *  Starts sending the content of the output buffer
 *
 *  \note
 *  - we send the first char to serial port
 *  - start the interrupt
 ******************************************************************************/
void RS_startSend(void)
{
	cli();							// Probably not too critical for RS-485 mode
	#if defined COM_RS485
		startTxSendTimer();			// Initially, wait a bit for line to turn round
	#else
		startUARTSend();
	#endif
	sei();
}

#endif /* COM_RS232 */
