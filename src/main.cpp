//#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//#include "sysclock.h"

//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <Wire.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include "defines.h"
#include <stdint.h>
#include <util/twi.h>

#include "lcd.c"
#include "adc.c"





#define LOOPLEDPORT		PORTD
#define LOOPLEDDDR		DDRD
#define LOOPSTEP			0x0FFF

// Define fuer Slave:
#define LOOPLED			6

#define  Error   0x01
#define  Success 0x02
#define SLAVE_COMMAND 2
char     TransmitState = 0x00;
//char*    TextString    = "AVR communicating via the SPI"+0x00;
#define BUFSIZE 8

#define SPI_CONTROL_DDR			DDRB
#define SPI_CONTROL_PORT		PORTB

#define SPI_CONTROL_DD			PORTB1  // pin for sending 2 bytes
#define SPI_CONTROL_CS			PORTB2	//CS fuer HomeCentral Master
#define SPI_CONTROL_MOSI		PORTB3
#define SPI_CONTROL_MISO		PORTB4
#define SPI_CONTROL_SCK			PORTB5



#define waitspi() while(!(SPSR&(1<<SPIF)))

volatile unsigned char incoming[BUFSIZE];

volatile uint8_t in_data[BUFSIZE];
volatile uint8_t in_code[BUFSIZE];

volatile uint8_t incomingcode = 0;
volatile uint8_t incomingdata = 0;


volatile uint8_t displaytask = 0;

volatile uint8_t in_counter = 0;

volatile uint8_t received=0;
volatile uint8_t spistatus = 0;
volatile uint8_t joystickoncounter = 0;


volatile uint8_t analogtastaturstatus = 0;
#define RECEIVED	0
#define SPISTART	1
#define SPIEND		2
#define SPIBYTE0		3
#define SPIBYTE1		4



#define SPI_ABSTAND	500
#define SPI_PACKETSIZE 8

#define SPI_INT0	2

#define SPI_MONO	2 // input mono

volatile uint8_t datapos = 0;
//volatile uint32_t spidistanz = 0; 

volatile uint8_t int0counter = 0; 

uint16_t loopcount0=0;
uint16_t loopcount1=0;
uint16_t loopcount2=0;

uint16_t timercount0=0;
// U8X8_SSD1327_EA_W128128_HW_I2C u8x8(U8X8_PIN_NONE,A5,A6);

uint8_t pfeilrechts[] = 
{
  0x00,
  0x00,
  0x08,
  0x0C,
  0x0E,
  0x0C,
  0x08,
  0x00
};
uint8_t pfeillinks[] = {
  0x00,
  0x02,
  0x06,
  0x0E,
  0x06,
  0x02,
  0x00,
  0x00
};

uint8_t pfeilauf[] = 
{
  0x00,
  0x00,
  0x04,
  0x0E,
  0x1F,
  0x00,
  0x00,
  0x00
};
uint8_t pfeilab[] = 
{
  0x00,
  0x00,
  0x1F,
  0x0E,
  0x04,
  0x00,
  0x00,
  0x00
};


uint8_t kreuz[] = 
{
  0x00,
  0x11,
  0x0A,
  0x04,
  0x04,
  0x0A,
  0x11,
  0x00
};

uint8_t rahmen[] = {
  0x00,
  0x1F,
  0x11,
  0x11,
  0x11,
  0x11,
  0x1F,
  0x00
};

byte gitter[] = {
    0x15,
  0x00,
  0x11,
  0x00,
  0x11,
  0x00,
  0x15,
  0x00
};

// Timer0 overflow interrupt service routine
ISR(TIMER0_OVF_vect) {
    //timer0_overflows++;
    // += 256; // Increment by 256 microseconds for each overflow
}
// Interrupt service routine for Timer0 compare match A
ISR(TIMER0_COMPA_vect) 
{
  // Toggle the output pin
  //PORTD ^= (1<<PORTD6);
	timercount0++;
}

// Initialize Timer0
void timer0_init() 
{
    // Set Timer0 to normal mode (no CTC)
    //TCCR0A = 0;
		TCCR0A = (1 << WGM01);

    // Set the prescaler to 64
    TCCR0B |= (1 << CS01) ;//| (1 << CS00);

    // Enable Timer0 overflow interrupt
		// Enable Timer0 compare match A interrupt
    TIMSK0 |= (1 << OCIE0A);

    // Initialize timer counter
    TCNT0 = 0;
		OCR0A = 50;
    // Enable global interrupts
    sei();
}

unsigned long micros() 
{
    unsigned long m;
    uint8_t oldSREG = SREG; // Save the status register
    cli(); // Disable interrupts
    //m = timer0_micros + TCNT0; // Add the current timer value
		sei();
    SREG = oldSREG; // Restore the status register
    return m;
}



// Intialization RoutineSlave Mode (interrupt controlled)
void Init_Slave_IntContr (void)
{
	volatile char IOReg;
	// Set PB6(MISO) as output 
	
	/*
	SPI_CONTROL_DDR    |= (1<<SPI_CONTROL_MISO);
	SPI_CONTROL_PORT    |= (1<<SPI_CONTROL_MISO);	// MISO als Output
	
	SPI_CONTROL_DDR	&= ~(1<<SPI_CONTROL_CS);	// Chip Select als Eingang
	//SPI_CONTROL_PORT |=(1<<SPI_CONTROL_CS);		// HI
	SPI_CONTROL_DDR	&= ~(1<<SPI_CONTROL_SCK);	// SCK Eingang
	//SPI_CONTROL_PORT |=(1<<SPI_CONTROL_SCK);		// HI
	//SPI_CONTROL_DDR	&= ~(1<<SPI_CONTROL_MOSI);	// MOSI als Eingang
	//SPI_CONTROL_PORT |=(1<<SPI_CONTROL_MOSI);		// HI
	*/
	// Enable SPI Interrupt and SPI in Slave Mode with SCK = CK/4
	SPCR  = (1<<SPIE)|(1<<SPE);
	IOReg   = SPSR;                         // Clear SPIF bit in SPSR
	IOReg   = SPDR;

	
	//DDRD	= 0xFF;	
	// Set Port D as output
	sei(); // Enable global interrupts

}

void INT0_init(void) // Dauer von 2 bytes detekieren
{
	DDRD &= ~(1<<SPI_INT0);
	DDRB |= (1 << PB0);
	// Enable falling edge interrupt on INT0
    EICRA |= (1 << ISC01);
    EICRA &= ~(1 << ISC00);
    
    // Enable INT0 interrupt
    EIMSK |= (1 << INT0);
}

ISR(INT0_vect) 
{
    // Toggle LED on PB0
		int0counter++;
    PORTB |= (1 << PB0); 
		spistatus &= ~(1<<SPIBYTE0);
		spistatus &= ~(1<<SPIBYTE1); // bereit fuer neues paket
}



unsigned char spi_tranceiver (unsigned char data)
{
    // Load data into the buffer
    SPDR = data;
 
    //Wait until transmission complete
    while(!(SPSR & (1<<SPIF)));   // Return received data

  return(SPDR);
}

uint8_t received_from_spi(uint8_t data)
{
  SPDR = data;
  return SPDR;
}

void parse_message()
{

 switch(incoming[0]) 
 {
 case SLAVE_COMMAND:
   //flash_led(incoming[1])
	;
   break;
 default:
   PORTD ^=(1<<1);//LED 1 toggeln
	;
 }

}

// Interrupt Routine Slave Mode (interrupt controlled)
 
// called by the SPI system when there is data ready.
// Just store the incoming data in a buffer, when we receive a
// terminating byte (0x00) call parse_message to process the data received
ISR( SPI_STC_vect )
{
	//PORTD |=(1<<0);//LED 0 ON
	PORTD &= ~(1<<0);//LED 0 OFF
	LOOPLEDPORT |= (1<<LOOPLED);
	uint8_t data = SPDR;
	SPDR = datapos;
	spistatus |= (1<<RECEIVED);

	if (spistatus & (1<<SPIBYTE0)) // zweites byte angekommen, data
	{

		spistatus |= (1<<SPIBYTE1);
		if (in_counter > 7)
		{
			//in_counter = 0;
		}
		//spistatus &= ~(1<<SPIBYTE0);
		in_data[in_counter] = data;
		incomingdata = data;
		in_counter++;
		

	}
	else
	{
		PORTB &= ~(1 << PB0); // 
		if(data == 0xFF)
		{
			in_counter = 0;
		}

		//in_counter &= 0x03;
		spistatus |= (1<<SPIBYTE0); // erstes byte, code
		displaytask = in_counter;
		in_code[in_counter] = data;
		incomingcode = data;
		
	}





/*
	if ((PINB & (1<<0)) == 0) // Mono  noch nicht gesetzt
	{
		in_code[in_counter] = data;
		in_counter++;
	}
	else
	{
		//in_data[in_counter] = data;
		//in_counter++;
		
	}
	*/
	//in_counter &= 0x03;
	
	//



/*
	if (!(spistatus & (1<<RECEIVED))) // Start mit neuem Bytpaket
	{
  	spistatus |= (1<<RECEIVED);
		spistatus |= (1<<SPIBYTE0); // erstes byte
		in_code[in_counter] = data;
	}
	else if(spistatus & (1<<SPIBYTE0))
	{
		spistatus &= ~(1<<SPIBYTE0);
		spistatus |= (1<<SPIBYTE1);// 2. Byte
		in_data[in_counter] = data;
		in_counter++;
		spistatus &= ~(1<<SPIBYTE1);
		spistatus &= ~(1<<SPIBYTE0);
	}
*/		
		if (data == 0xFF) // sync
		{
			received = 0;
		}
		
		//spidistanz= micros();
		incoming[received] = data;
		received++;
		
		
	PORTD |=(1<<0);
	
} // ISR


void slaveinit(void)
{
	DDRD &= ~(1<<SPI_MONO); // Eingang vom Mono
	DDRD |= (1<<DDD0); // ISR

	//DDRB |= (1<<DDB0); // Ausgang fuer Mono
   /*
 			//Pin 0 von PORT D als Ausgang fuer Schalter: ON		
	DDRD |= (1<<DDD1);		//Pin 1 von PORT D als Ausgang fuer Schalter: OFF
	DDRD |= (1<<DDD2);		//Pin 2 von PORT D als Ausgang fuer Buzzer
 	DDRD |= (1<<DDD3);		//Pin 3 von PORT D als Ausgang fuer LED TWI
	DDRD |= (1<<DDD4);		//Pin 4 von PORT D als Ausgang fuer LED
    */
  LOOPLEDDDR |= (1<<LOOPLED);		//Pin 5 von PORT D als Ausgang fuer LED Loop
	LOOPLEDPORT |= (1<<LOOPLED);		//Pin 5 von PORT D als Ausgang fuer LED Loop

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
/*
	DDRC &= ~(1<<DDC0);	//Pin 0 von PORT C als Eingang fuer ADC
//	PORTC |= (1<<DDC0); //Pull-up
	DDRC &= ~(1<<DDC1);	//Pin 1 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC1); //Pull-up
	DDRC &= ~(1<<DDC2);	//Pin 2 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC3); //Pull-up
	DDRC &= ~(1<<DDC3);	//Pin 3 von PORT C als Eingang fuer Tastatur 	
//	PORTC |= (1<<DDC3); //Pull-up
*/

	
	
}


int main (void) 
{
	  slaveinit();
		
	  // initialize the LCD 
	  lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
		_delay_ms(10);

		//lcd_custom();

		lcd_write_custom(0,kreuz);
		lcd_write_custom(1,rahmen);
		lcd_write_custom(2,pfeilauf);
		lcd_write_custom(3,pfeilab);
		lcd_write_custom(4,pfeillinks);
		lcd_write_custom(5,pfeilrechts);
		lcd_write_custom(6,gitter);


		
		
		

		lcd_gotoxy(0,0);
		lcd_puts("SPI Slave LCD");
	  _delay_ms(1000);
	 lcd_clr_line(0);
	 
	 Init_Slave_IntContr();

		//	timer0_init();
	
		INT0_init();

    uint8_t Tastenwert=0;
    uint8_t TastaturCount=0;
    
    uint16_t TastenStatus=0;
    uint16_t Tastencount=0;
    uint16_t Tastenprellen=0x01F;
    
    //initADC(TASTATURPIN);
	
	//uint16_t distanz = 0;
    

		uint8_t spi_input = 0;
    //uint16_t startdelay1=0;

    //uint8_t twierrcount=0;
    //LOOPLEDPORT |=(1<<LOOPLED);


 //u8x8.begin();
 
	   //timer2();
		
		
		 DDRD |= (1<<7);
		 PORTD |=(1<<0);
   //spidistanz = micros();
	 /*
		lcd_gotoxy(0,1);
		
		lcd_putc(0);
		lcd_putc(' ');		
		lcd_putc(1);
		lcd_putc(' ');				
		lcd_putc(2);
		lcd_putc(' ');	
		lcd_putc(3);
		lcd_putc(' ');	
		lcd_putc(4);
		lcd_putc(' ');		
		lcd_putc(5);
		lcd_putc(' ');		
		lcd_putc(6);
*/
	 sei();
	while (1)
	{
      //PORTD ^= (1<<7);
      //_delay_ms(50);
     // PORTD &= ~(1<<0);
		//Blinkanzeige
      wdt_reset();


			if (spistatus & (1<<RECEIVED))
			{
				

				LOOPLEDPORT &= ~(1<<LOOPLED);
				spistatus &= ~(1<<RECEIVED);
				if(spistatus & (1<<SPIBYTE1)) // zweites byte gelesen
				{
					spistatus &= ~(1<<SPIBYTE0);
					spistatus &= ~(1<<SPIBYTE1);
				}
				
				
											//lcd_puts("Joystick");
				//uint8_t linepos = (received / 4) + 2; // Zeilenwechsel nach 3
				//lcd_gotoxy(0,0);
        //lcd_putint12(spidistanz );
				//lcd_gotoxy(6,0);
				//lcd_putint(spistatus);
				//spidistanz = 0;
				//lcd_gotoxy(16,0);
        //lcd_putint(received);

				//lcd_gotoxy(4* (received % 4),linepos);
				//lcd_putint(incoming[received]);
					/*
					lcd_gotoxy(0,1);
					lcd_putint(in_counter);
					lcd_putc(' ');
					lcd_putc('c');
					lcd_putc(' ');
					lcd_putint(in_code[in_counter]);
					lcd_gotoxy(10,1);
					lcd_putc('d');
					lcd_putc(' ');
					lcd_putint(in_data[in_counter]);
					*/
					if (PINB & (1<<0)) // Mono ist HI, SPI-input ist data
					{
			
					}
					else
					{
							//in_data[in_counter] = data;
					}

						//lcd_gotoxy(17,2);
						//lcd_putint(incomingcode);
						//incomingcode = 101;
						switch (incomingcode)
						{
								case 101: // Tastatur
								{
									lcd_gotoxy(12,1);
									lcd_putint(incomingcode);
									lcd_putc(':');
									lcd_putc(' ');
									//lcd_gotoxy(12,1);
									lcd_putint(incomingdata);
									
								}break;
								case 102:
								{
									lcd_gotoxy(12,2);
									lcd_putint(incomingcode);
									lcd_putc(':');
									lcd_putc(' ');
									lcd_putint(incomingdata);
									if(incomingdata & 0x80)
									{	

										analogtastaturstatus |= (1<<JOYSTICK_ON);
										joystickoncounter++;
											//lcd_gotoxy(0,0);
											//lcd_puts("Joystick");
									}// incomingdata
									else
									{
										analogtastaturstatus &= ~(1<<JOYSTICK_ON);
										//lcd_gotoxy(0,0);
										//	lcd_puts("        ");
									}

									if(incomingdata & 0x40)
									{	

										analogtastaturstatus |= (1<<CALIB_ON);
										joystickoncounter++;
											//lcd_gotoxy(0,0);
											//lcd_puts("Joystick");
									}// incomingdata
									else
									{
										analogtastaturstatus &= ~(1<<CALIB_ON);
										//lcd_gotoxy(0,0);
										//	lcd_puts("        ");
									}


								}break;

								case 103:
								{
									lcd_gotoxy(12,3);
									lcd_putint(incomingcode);
									lcd_putc(':');
									lcd_putc(' ');
									lcd_putint(incomingdata);
								}break;

						}// switch

						/*
						uint8_t linepos = displaytask%4;
						lcd_gotoxy(0,linepos);
						lcd_putint(in_code[linepos]);
						lcd_putc(':');
						lcd_putc(' ');
						lcd_putint(in_data[linepos]);
						*/
						/*
						lcd_gotoxy(0,0);
						lcd_putint(in_code[0]);
						lcd_putc(':');
						lcd_putc(' ');
						lcd_putint(in_data[0]);

						lcd_gotoxy(0,1);
						lcd_putint(in_code[1]);
						lcd_putc(':');
						lcd_putc(' ');
						lcd_putint(in_data[1]);

						lcd_gotoxy(0,2);
						lcd_putint(in_code[2]);
						lcd_putc(':');
						lcd_putc(' ');
						lcd_putint(in_data[2]);

						lcd_gotoxy(0,3);
						lcd_putint(in_code[3]);
						lcd_putc(':');
						lcd_putc(' ');
						lcd_putint(in_data[3]);
						*/
						/*
						// spalte 2
						lcd_gotoxy(9,0);
						lcd_putint(in_code[4]);
						lcd_putc(':');
						lcd_putc(' ');
						lcd_putint(in_data[4]);

						lcd_gotoxy(9,1);
						lcd_putint(in_code[5]);
						lcd_putc(':');
						lcd_putc(' ');
						lcd_putint(in_data[5]);

						lcd_gotoxy(10,2);
						lcd_putint(in_code[6]);
						lcd_putc(':');
						lcd_putc(' ');
						lcd_putint(in_data[6]);

						lcd_gotoxy(10,3);
						lcd_putint(in_code[7]);
						lcd_putc(':');
						lcd_putc(' ');
						lcd_putint(in_data[7]);
						*/
						
						/*
						lcd_gotoxy(0,0);
						lcd_putint(in_code[0]);
						lcd_putc(' ');
						lcd_putint(in_code[1]);
						lcd_putc(' ');
						lcd_putint(in_code[2]);
						lcd_putc(' ');
						lcd_putint(in_code[3]);
				
						lcd_gotoxy(0,1);
						lcd_putint(in_data[0]);
						lcd_putc(' ');
						lcd_putint(in_data[1]);
						lcd_putc(' ');
						lcd_putint(in_data[2]);
						lcd_putc(' ');
						lcd_putint(in_data[3]);
						*/
						/*
						lcd_gotoxy(0,2);
						lcd_putint(in_code[4]);
						lcd_putc(' ');
						lcd_putint(in_code[5]);
						lcd_putc(' ');
						lcd_putint(in_code[6]);
						lcd_putc(' ');
						lcd_putint(in_code[7]);
				
						lcd_gotoxy(0,3);
						lcd_putint(in_data[4]);
						lcd_putc(' ');
						lcd_putint(in_data[5]);
						lcd_putc(' ');
						lcd_putint(in_data[6]);
						lcd_putc(' ');
						lcd_putint(in_data[7]);
						*/
						/*
						lcd_gotoxy(0,2);
						lcd_putint(incoming[0]);
						lcd_putc(' ');
						lcd_putint(incoming[2]);
						lcd_putc(' ');
						lcd_putint(incoming[4]);
						lcd_putc(' ');
						lcd_putint(incoming[6]);
				
						lcd_gotoxy(0,3);
						lcd_putint(incoming[1]);
						lcd_putc(' ');
						lcd_putint(incoming[3]);
						lcd_putc(' ');
						lcd_putint(incoming[5]);
						lcd_putc(' ');
						lcd_putint(incoming[7]);
						*/
				//PORTD &= ~(1<<7);
				
				

				//sei();

			}
		loopcount0++;
		if (loopcount0>LOOPSTEP)
		{
			

			loopcount0=0;
			
      loopcount1++;
      if(loopcount1 > 0x1F)
         {
            //LOOPLEDPORT ^=(1<<LOOPLED);
				if (analogtastaturstatus & (1<<JOYSTICK_ON))
				{
					
					lcd_gotoxy(0,0);
					lcd_puts("JOYSTICK");

				}
				else
				{
					lcd_gotoxy(0,0);
					lcd_puts("        ");
				}

				if (analogtastaturstatus & (1<<CALIB_ON))
				{
					
					lcd_gotoxy(0,1);
					lcd_puts("CALIB");

				}
				else
				{
					lcd_gotoxy(0,1);
					lcd_puts("     ");
				}

            loopcount1 = 0;
            loopcount2++;
						//cli();
						lcd_gotoxy(0,3);
						lcd_putint(joystickoncounter);
						lcd_gotoxy(6,3);
						lcd_putint(analogtastaturstatus);
						
						/*
						lcd_gotoxy(0,2);
						lcd_putint(incoming[0]);
						lcd_putc(' ');
						lcd_putint(incoming[2]);
						lcd_putc(' ');
						lcd_putint(incoming[4]);
						lcd_putc(' ');
						lcd_putint(incoming[6]);

						lcd_gotoxy(0,3);
						lcd_putint(incoming[1]);
						lcd_putc(' ');
						lcd_putint(incoming[3]);
						lcd_putc(' ');
						lcd_putint(incoming[5]);
						lcd_putc(' ');
						lcd_putint(incoming[7]);
						//lcd_gotoxy(0,1);
        		//lcd_putint(received);
						*/
            //lcd_gotoxy(10,1);
            //lcd_putint16(loopcount2);
						//lcd_putc(' ');
						//lcd_putint(incoming[received]);
						//lcd_putc(' ');
						//lcd_putint(received);
						//lcd_putc('*');
						//sei();
						
         }
		}
		

		
	}//while

 return 0;
}// main
