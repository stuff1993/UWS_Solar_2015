/*****************************************************
 *
 * LCD Driver.c
 * Author: Tony Baz
 *
 ****************************************************/


#include "lpc17xx.h"
#include "lcd.h"
#include "timer.h"

void delay10US(int count)
{
	volatile int j=0,i=0;

	for(j=0;j<count;j++)
	{
		for(i=0;i<6;i++);
	}
}

void lcd_command_write(unsigned char command)
{
	unsigned char TEMP = command;
	RS_L;		// Register Select Low for Command

	if(TEMP & 0b10000000){D3_H;} else{D3_L;}
	if(TEMP & 0b01000000){D2_H;} else{D2_L;}
	if(TEMP & 0b00100000){D1_H;} else{D1_L;}
	if(TEMP & 0b00010000){D0_H;} else{D0_L;}
	E_H; delay10US(5); E_L;		// Pulse the Enable Pin

	if(TEMP & 0b00001000){D3_H;} else{D3_L;}
	if(TEMP & 0b00000100){D2_H;} else{D2_L;}
	if(TEMP & 0b00000010){D1_H;} else{D1_L;}
	if(TEMP & 0b00000001){D0_H;} else{D0_L;}
	E_H; delay10US(5); E_L;		// Pulse the Enable Pin
}


void lcd_data_write( unsigned char data )
{
	unsigned char TEMP = data;
	RS_H;		// Register Select High for Character

	if(TEMP & 0b10000000){D3_H;} else{D3_L;}
	if(TEMP & 0b01000000){D2_H;} else{D2_L;}
	if(TEMP & 0b00100000){D1_H;} else{D1_L;}
	if(TEMP & 0b00010000){D0_H;} else{D0_L;}
	E_H; delay10US(5); E_L;	// Pulse the Enable Pin

	if(TEMP & 0b00001000){D3_H;} else{D3_L;}
	if(TEMP & 0b00000100){D2_H;} else{D2_L;}
	if(TEMP & 0b00000010){D1_H;} else{D1_L;}
	if(TEMP & 0b00000001){D0_H;} else{D0_L;}
	E_H; delay10US(5); E_L;	// Pulse the Enable Pin
}

void lcd_clear( void)
{
	lcd_command_write( 0x01 );
	delay10US(5);
}


void lcd_gotoxy( unsigned int x, unsigned int y)
{
	if( x==0 ){lcd_command_write( 0x80 + y );}		        // command - position cursor at 0x80 + y
	else if( x==1 ){lcd_command_write( 0xC0 + y );}		    // command - position cursor at 0xC0 + y
	else if( x==2 ){lcd_command_write( 0x80 + y + 20 );}  // command - position cursor at 0x94 + y
	else if( x==3 ){lcd_command_write( 0xC0 + y + 20 );}	// command - position cursor at 0xD4 + y
}

void lcd_putchar( int c )
{
	lcd_data_write( c );
}

void displayTEMP(void)
{
	lcd_data_write( 0b11011111 );
	lcd_data_write('C');
}

void lcd_putstring_custom( unsigned char line, unsigned char Pos, char *string, int len) // only use when printing custom characters to screen, does not check for nulls
{
	uint8_t _len = ((MAX_CHAR_IN_ONE_LINE - Pos) < len) ? (MAX_CHAR_IN_ONE_LINE - Pos) : len;

	lcd_gotoxy( line, Pos );

	while(_len--){lcd_putchar( *string++ );}
}

void lcd_putstring( unsigned char line, unsigned char Pos, char *string )
{
	uint8_t _len = MAX_CHAR_IN_ONE_LINE - Pos;

	lcd_gotoxy( line, Pos );

	while(*string != '\0' && _len--){lcd_putchar( *string++ );}
}

void setLCD( void )
{
	delayMs(1,50);

	/*lcd_command_write(0b00101001);	//   function set - 4-bit interface
	delay10US(10);
	lcd_command_write(0b00001000);	//   display off
	delay10US(10);
	lcd_clear();
	lcd_command_write(0b00000110);	//   cursor move direction
	delay10US(10);




	lcd_command_write(0b00000010);	// cursor home
	delay10US(50);
	lcd_command_write(0b00001100);	// display on
	delay10US(50);

	lcd_gotoxy(0, 0);
	delay10US(10);
	lcd_clear();*/

	lcd_command_write(0b00000010);     /*   cursor home                                 */
	delayMs(1,2);
	lcd_command_write(0b00000110);     /*   cursor move direction                       */
	delayMs(1,2);
	lcd_command_write(0b00001100);    /*   display on      */
	delayMs(1,2);
	lcd_command_write(0b00101000);     /*   4-bit interface, two line, 5X7 dots.        */
	delayMs(1,2);

	// setup custom characters
	lcd_command_write(0b01000000); delay10US(50);	// move to first CGRAM location
	lcd_data_write(0b00000000);	delay10US(50); // 0x00
	lcd_data_write(0b00000011);	delay10US(50);
	lcd_data_write(0b00000100);	delay10US(50);
	lcd_data_write(0b00001000);	delay10US(50);
	lcd_data_write(0b00010000);	delay10US(50);
	lcd_data_write(0b00010000);	delay10US(50);
	lcd_data_write(0b00010000);	delay10US(50);
	lcd_data_write(0b00010000);	delay10US(50);

	lcd_data_write(0b00010000);	delay10US(50); // 0x01
	lcd_data_write(0b00010000);	delay10US(50);
	lcd_data_write(0b00010000);	delay10US(50);
	lcd_data_write(0b00010000);	delay10US(50);
	lcd_data_write(0b00010000);	delay10US(50);
	lcd_data_write(0b00001000);	delay10US(50);
	lcd_data_write(0b00000111);	delay10US(50);
	lcd_data_write(0b00000000);	delay10US(50);

	lcd_data_write(0b00011111);	delay10US(50); // 0x02
	lcd_data_write(0b00000000);	delay10US(50);
	lcd_data_write(0b00000000);	delay10US(50);
	lcd_data_write(0b00000000);	delay10US(50);
	lcd_data_write(0b00000000);	delay10US(50);
	lcd_data_write(0b00000000);	delay10US(50);
	lcd_data_write(0b00000000);	delay10US(50);
	lcd_data_write(0b00001111);	delay10US(50);

	lcd_data_write(0b00010010);	delay10US(50); // 0x03
	lcd_data_write(0b00001001);	delay10US(50);
	lcd_data_write(0b00001001);	delay10US(50);
	lcd_data_write(0b00001001);	delay10US(50);
	lcd_data_write(0b00001001);	delay10US(50);
	lcd_data_write(0b00010010);	delay10US(50);
	lcd_data_write(0b00001100);	delay10US(50);
	lcd_data_write(0b00000000);	delay10US(50);

	lcd_data_write(0b00011111);	delay10US(50); // 0x04
	lcd_data_write(0b00000000);	delay10US(50);
	lcd_data_write(0b00000000);	delay10US(50);
	lcd_data_write(0b00000000);	delay10US(50);
	lcd_data_write(0b00000000);	delay10US(50);
	lcd_data_write(0b00000000);	delay10US(50);
	lcd_data_write(0b00000000);	delay10US(50);
	lcd_data_write(0b00011111);	delay10US(50);

	lcd_data_write(0b00011100);	delay10US(50);  // 0x05
	lcd_data_write(0b00000010);	delay10US(50);
	lcd_data_write(0b00010001);	delay10US(50);
	lcd_data_write(0b00000001);	delay10US(50);
	lcd_data_write(0b00010001);	delay10US(50);
	lcd_data_write(0b00000001);	delay10US(50);
	lcd_data_write(0b00000010);	delay10US(50);
	lcd_data_write(0b00011100);	delay10US(50);


	lcd_gotoxy(0, 0);

	delayMs(1,2);
	lcd_clear();
	delayMs(1,2);
}
