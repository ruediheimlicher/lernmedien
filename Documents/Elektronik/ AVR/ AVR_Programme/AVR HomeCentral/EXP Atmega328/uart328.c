#include "kbd.h"
#define FOSC   7372000

#define BAUD 19200

#define UART_PORT		PORTC
#define UART_PIN		PINC
#define UART_DDR		DDRC
#define RTS_PIN      5
#define CTS_PIN      4

//defines max coordinates for checking overflow
#define MAX_X 80
#define MAX_Y 25

enum COLORS {
   BLACK = 0,          /* dark colors */
   RED,
   GREEN,
   BROWN,
   BLUE,
   MAGENTA,
   CYAN,
   LIGHTGRAY,
   DARKGRAY,       /* light colors */
   LIGHTRED,
   LIGHTGREEN,
   YELLOW,
   LIGHTBLUE,
   LIGHTMAGENTA,
   LIGHTCYAN,
   WHITE
};



//#define USR UCSR0A


void uart_init (void)
{
   uint8_t sreg = SREG;
   
   /*
    uint16_t ubrr = (uint16_t) ((uint32_t) FOSC/(16UL*BAUDRATE) - 1);
    
    UBRRH = (uint8_t) (ubrr>>8);
    UBRRL = (uint8_t) (ubrr);
    */
   UBRR0H = (((FOSC/16)/BAUD-1)>>8);  // The high byte, UBRR0H
   UBRR0L = ((FOSC/16)/BAUD-1);       // The low byte, UBRR0L
   
   
   // Interrupts kurz deaktivieren
   
   // UART Receiver und Transmitter anschalten, Receive-Interrupt aktivieren
   // Data mode 8N1, asynchron
   UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
   UCSR0C =  (1 << UCSZ01) | (1 << UCSZ00);
   
   // Flush Receive-Buffer (entfernen evtl. vorhandener ungültiger Werte)
   do
   {
      // UDR auslesen (Wert wird nicht verwendet)
      UDR0;
   }
   while (UCSR0A & (1 << RXCIE0));
   
   // Rücksetzen von Receive und Transmit Complete-Flags
   UCSR0A = (1 << RXC0) | (1 << TXC0);
   
   // Global Interrupt-Flag wieder herstellen
   SREG = sreg;
   
   // FIFOs für Ein- und Ausgabe initialisieren
   //   fifo_init (&infifo,   inbuf, BUFSIZE_IN);
   //   fifo_init (&outfifo, outbuf, BUFSIZE_OUT);
   
   UART_DDR &= ~(1<<RTS_PIN); // Eingang
   UART_PORT |= ~(1<<RTS_PIN);
   
   UART_DDR |= (1<< CTS_PIN); // Ausgang
   UART_PORT |= ~(1<<CTS_PIN); // HI
}

void _putch (char ch)
{
   
	while (UART_PIN & (1<<RTS_PIN));
   // _delay_ms(1);
#ifdef USR
	while(!(USR & (1<<UDRE))); //transmit buffer is ready to receive data
	
	UDR = ch;    // send character
	while(!(USR & (1<<TXC))); //wait for char to be send
   
	USR &= ~(1<<TXC || 1<<UDRE);
#else //2 UART MCU
	while(!(UCSR0A & (1<<UDRE0))); //transmit buffer is ready to receivce data
	
	UDR0 = ch;    // send character
	while(!(UCSR0A & (1<<TXC0))); //wait for char to be send
   
	UCSR0A &= ~(1<<TXC0 || 1<<UDRE0);
   
#endif
}



// Einen 0-terminierten String übertragen.
void _puts (const char *s)
{
   do
   {
      _putch (*s);
      
   }
   while (*s++);
}

void clrscr(void)
{
   _delay_ms(1);
   _putch('\033');
   _putch('[');
   
   _putch('2');
   _putch('J');
}

void gotoxy(char x, char y)
{
   if (x>MAX_X || y>MAX_Y)
      return;
   
   x--;
   y--;
   
   _putch(0x1B);
   _putch('[');
   _putch((y/10)+'0');
   _putch((y%10)+'0');
   _putch(';');
   _putch((x/10)+'0');
   _putch((x%10)+'0');
   _putch('f');
}

void _textcolor(int color)
{
   _putch('\033');
   _putch('[');
   if (color & 0x8)
   {
      _putch('1');
   }
   else
   {
      _putch('2');
   }
   
   _putch('m');
   _putch('\033');
   
   _putch('[');
   _putch('3');
   _putch(((color&0x7)%10)+'0');
   _putch('m');
}


void _textbackground(int color)
{
   _putch('\033');
   _putch('[');
   if (color & 0x8)
      _putch('5');
   else _putch('6');
   _putch('m');
   
   _putch('\033');
   _putch('[');
   _putch('4');
   _putch((color&0x7)+'0');
   _putch('m');
}

void _newline(void)
{
   _putch(KB_CR);
   _putch(KB_LF);
   _putch(KB_SPACE);
   _putch(KB_SPACE);
   
   
}

void cursoroff(void)
{
   _putch('\033');
   _putch('[');
   _putch('2');
   _putch('5');
   _putch('l');
}



