/************************************************************************/
/*        Serial comms handler for RS-485 mode                          */
/************************************************************************/

// Only included if comms enabled in RS-485 mode.

#include <stdint.h>
#include <avr/io.h>

//#include "config.h"
//#include "common.h"

//#include "defaults.h"

// Externally called
void serialInit(uint8_t isRS232);
void txUartDisable(void);
void clearRx(void);
void resetRx(void);


// External routines used
extern void startRxTimer(void);
extern void retriggerRxTimer(void);
extern void stopRxTimer(void);
extern void txDisable(void);





// Standard ASCII Characters
#define	CHAR_SOH	1
#define CHAR_STX	2
#define CHAR_ETX	3
#define CHAR_ACK	6
#define	CHAR_LF		10
#define	CHAR_CR		13
#define CHAR_NAK	0x15
#define CHAR_OB		0x7b
#define CHAR_CB		0x7d


// Protocol types (IBM Protocol)
#define ACK_STD		1
#define ACK_ACK		2
#define ACK_NAK		3


#define RX_BUFFER_SIZE		50
#define TX_BUFFER_SIZE		250

// Configuration values
unsigned char serialFlags = 0;		// Bit 0 - enable CR terminator protocol


enum rxProtocolType {
	protNone = 0,
	protCR = 1,
	protIBM = 2,
	protASCII = 3
	};

typedef enum rxProtocolType	TrxProtocolType;


// Most of the following variables are also used by the transmit state machine.
volatile TrxProtocolType rxProtocol;		// Note received protocol
volatile unsigned char rxPointer;			// Next free space in receive buffer
volatile uint8_t rxCount;					// number of received characters in current message
volatile unsigned char rxAddress;			// Address character received
volatile unsigned char rxTerminator;		// Terminator character to look for

volatile char lastRxCancel = 0;				// For debugging

volatile unsigned char protState;			// state counter - bit 7 set indicates a transmit state
volatile uint8_t cksAcc;					// checksum accumulator

volatile uint8_t txCounter;					// Number of characters (left) in Tx buffer
volatile uint8_t txPointer;					// Next character to send in Tx buffer

char rxBuffer[RX_BUFFER_SIZE];
char txBuffer[TX_BUFFER_SIZE];


/*
 *	Initialise serial protocol driver (called from rs232_485_hw.c
 *
 * In RS-232 mode, can use line-based protocol
 */
void serialInit(uint8_t isRS232)
{
	if (isRS232) serialFlags |= 1;	// Enable simple 'CR' protocol in RS-232 mode (as well as the usual addressed protocol)
	clearRx();						// Set up all the receive buffer stuff
}



/*
 *	Get character from receive buffer
 */
static char COM_getchar(void)
{
	if (rxPointer == rxCount) return '\0';
	return rxBuffer[rxPointer++];
}


/*
 *	Add character to transmit buffer
 */
void COM_putchar(char c)
{
	if (txPointer >= TX_BUFFER_SIZE) return;
	txBuffer[txPointer++] = c;
}


/*
 *	Set pointers ready to transmit a message
 */
void COM_txInit(void)
{
	protState = 0x80;
	txCounter = txPointer;
	txPointer = 0;
}
/************************************************************************/
/*      Transmit ISR - called to request next character                 */
/************************************************************************/
char COM_tx_char_isr(void)
{
	switch (protState)
	{
		case 0x80:		// First character
			switch (rxProtocol)
			{
				case protIBM :
					protState = 0x92;
					cksAcc = CHAR_ACK ^ CHAR_ETX;
					return CHAR_ACK;
				case protASCII :
					protState = 0xa2;
					cksAcc = CHAR_OB + CHAR_CB - 0x40 - 95;
					return CHAR_OB;
				default : ;
			}
			// If not a protocol, intentionally fall through to send data
		case 0x81 :		// Send data
			if (--txCounter == 0)
			{
				protState = 0x82;
			}
			return txBuffer[txPointer++];
		case 0x82 :		// Sent last character - send CR
			protState = 0x83;
			return CHAR_CR;
		case 0x83 :			// Sent CR - send LF, and transmit done
			txUartDisable();
			protState = 0;
			return CHAR_LF;
		case 0x92 :		// Send address for IBM protocol
			protState = 0x93;
			cksAcc ^= (config.RS485_address + '0');
			return config.RS485_address + '0';
		case 0x93 :		// Send data
			cksAcc ^= txBuffer[txPointer];
			if (--txCounter == 0)
			{
				protState = 0x94;
			}
			return txBuffer[txPointer++];
		case 0x94 :				// Send terminator (added into checksum earlier)
			protState = 0x95;
			return CHAR_ETX;
		case 0x95 :				// Send checksum, done
			protState = 0;
			txUartDisable();
			return cksAcc;
		case 0xa2 :
			protState = 0xa3;
			cksAcc += config.RS485_address + '@' - 0x20;
			return config.RS485_address + '@';
		case 0xa3 :		// Send data
			cksAcc += txBuffer[txPointer] - 0x20;
			if (cksAcc >= 95) cksAcc -= 95;
			if (--txCounter == 0)
			{
				protState = 0xa4;
			}
			return txBuffer[txPointer++];
		case 0xa4 :				// Send terminator (added into checksum earlier)
			protState = 0xa5;
			return CHAR_CB;
		case 0xa5 :				// Send checksum, done
		default:
			protState = 0;
			txUartDisable();
			if (cksAcc >= 95) cksAcc -= 95;
			return cksAcc + 0x20;
	}
	return '?';		// In theory, will never get here
}


/************************************************************************/
/*      Receive ISR                                                     */
/************************************************************************/

/*
Receive characters. If the first one is '{' or STX, its a checksummed protocol. Otherwise its a simple line protocol (if enabled)
	When the first character is received, it initialises the serial timer to about 100ms to act as a message timeout
	When a complete and valid message is received, a processing flag is set and the timer retriggered for a delay of about 5ms.
	If a complete but invalid message (i.e. bad checksum, or not addressed to us) is received, the receive side is cleared down.
*/

void COM_rx_char_isr(char c)
{
	char rxCancel = 0;				// Flag set if receive to be reset for any reason
	char rxDone = 0;				// Flag set if complete message to pass on
	int tempAddr;

	
	switch (protState)
	{
		case 0 :
		case 0x10 :					// Received message which hasn't required a response can leave this state pending
			cksAcc = 0;				// Clear this down at start of message
			txDisable();			// Make sure RTS is inactive and timer stopped (in case serial is capable of full duplex)
			switch (c)
			{
				case CHAR_OB :
					rxTerminator = CHAR_CB;
					protState++;
					rxProtocol = protASCII;
					rxPointer = 0;
					startRxTimer();
					break;
				case CHAR_STX :
					rxTerminator = CHAR_ETX;
					protState++;
					rxProtocol = protIBM;
					rxPointer = 0;
					startRxTimer();
					break;
				default :
					if ((serialFlags & 1) == 0) break;		// Ignore other characters if not enabled.
															// Note - no inter-character receive timeout on non-protocol modes
					rxBuffer[0] = c;						// Store first received character - its data
					rxPointer = 1;
					rxTerminator = CHAR_CR;
					rxProtocol = protCR;
					protState = 2;							// Skip address character
			}
			break;
			
		case 1 :			// Receive and check address character
			switch (rxProtocol)
			{
				case protASCII :
					tempAddr = c - '@';
					break;
				case protIBM :
					tempAddr = c - '0';
					break;
				default:
					tempAddr = -1;						// This forces an error
			}
			if ((tempAddr < 0) || (tempAddr >= 64))
			{
				rxCancel = 1;
				break;
			}
			rxAddress = tempAddr;
			protState = 2;					// Go get data
			break;
			
		case 2 :			// Accumulate data until terminator received
			if (c == rxTerminator)
			{
				if (rxProtocol == protCR)
				{
					rxDone = 1;
					break;			// Delete break, to store the CR in the receive buffer
				}
				else
				{
					protState = 3;		// Next character is checksum
					break;
				}
			}
			if (rxPointer >= RX_BUFFER_SIZE)
			{
				rxCancel = 2;			// Overflow
				break;
			}
			rxBuffer[rxPointer++] = c;
			break;
			
		case 3 :			// Checksum received - verify and act
			switch (rxProtocol)
			{
				case protASCII :
					if (cksAcc != (c - 0x20)) rxCancel = 3;
					break;
				case protIBM :
					if (cksAcc != c) rxCancel = 4;
					break;
				default:
					rxCancel = 5;			// Bad checksum
			}
			if ((rxAddress == 0) || (rxAddress == config.RS485_address))
			{
				rxDone = 1;
				break;
			}
			rxCancel = 6;					// Message not for us
			break;
	}
	
	if (rxCancel)
	{
		lastRxCancel = rxCancel;			// May be useful for stats/debugging
		clearRx();
	}
	else
	{
		if (rxDone)
		{
			task |= TASK_COM;
			COM_requests++;
			rxCount = rxPointer;
			rxPointer = 0;				// Used to pull data from receive buffer
			protState = 0x10;
			stopRxTimer();
		}
		else
		{
			retriggerRxTimer();
			switch (rxProtocol)
			{
				case protASCII :
					cksAcc += c - 0x20;
					if (cksAcc >= 95) cksAcc -= 95;
					break;
				case protIBM :
					cksAcc ^= c;
					break;
				default : ;	
			}
		}
	}
}


/************************************************************************/
/*      Clear down all receive-related items, disable serial timer      */
/************************************************************************/

void clearRx(void)
{
	rxProtocol = protNone;
	protState = 0;
	rxPointer = 0;
	cksAcc = 0;
	stopRxTimer();
}


// Just clear rx pointer after using receive message
void resetRx(void)
{
	rxPointer = 0;
}