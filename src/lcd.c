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
	E_H; delayMs(1,1); E_L;		// Pulse the Enable Pin

	if(TEMP & 0b00001000){D3_H;} else{D3_L;}
	if(TEMP & 0b00000100){D2_H;} else{D2_L;}
	if(TEMP & 0b00000010){D1_H;} else{D1_L;}
	if(TEMP & 0b00000001){D0_H;} else{D0_L;}
	E_H; delayMs(1,1); E_L;		// Pulse the Enable Pin
}


void lcd_data_write( unsigned char data )
{
	unsigned char TEMP = data;
	RS_H;		// Register Select High for Character

	if(TEMP & 0b10000000){D3_H;} else{D3_L;}
	if(TEMP & 0b01000000){D2_H;} else{D2_L;}
	if(TEMP & 0b00100000){D1_H;} else{D1_L;}
	if(TEMP & 0b00010000){D0_H;} else{D0_L;}
	E_H; delayMs(1,1); E_L;		// Pulse the Enable Pin

	if(TEMP & 0b00001000){D3_H;} else{D3_L;}
	if(TEMP & 0b00000100){D2_H;} else{D2_L;}
	if(TEMP & 0b00000010){D1_H;} else{D1_L;}
	if(TEMP & 0b00000001){D0_H;} else{D0_L;}
	E_H; delayMs(1,1); E_L;		// Pulse the Enable Pin
}

void lcd_clear( void)
{
	lcd_command_write( 0x01 );
	delayMs(1,2);
}


void lcd_gotoxy( unsigned int x, unsigned int y)
{
	if( x == 0 )
	{
		lcd_command_write( 0x80 + y );		/* command - position cursor at 0x00 (0x80 + 0x00 ) */
	}
	else if( x==1 )
	{
		lcd_command_write( 0xC0 + y );		/* command - position cursor at 0x40 (0x80 + 0x00 ) */
	}
	else if( x==2 )
	{
		lcd_command_write( 0x80 + y + 20 );		/* command - position cursor at 0x40 (0x80 + 0x00 ) */
	}
	else if( x==3 )
	{
		lcd_command_write( 0xC0 + y + 20);		/* command - position cursor at 0x40 (0x80 + 0x00 ) */
	}
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

void lcd_putstring( unsigned char line, unsigned char Pos, char *string )
{
	unsigned char len = MAX_CHAR_IN_ONE_LINE - Pos;

	lcd_gotoxy( line, Pos );

	while(*string++ != '\0' && len--)
	{
		lcd_putchar( *string );
	}
}

void setLCD( void )
{
	delayMs(1,50);

	lcd_command_write(0b00101001);	//   function set - 4-bit interface
	delayMs(1,2);
	lcd_command_write(0b00001000);	//   display off
	delayMs(1,2);
	lcd_clear();
	lcd_command_write(0b00000110);	//   cursor move direction
	delayMs(1,2);

	// setup custom characters
	lcd_command_write(0b01000000);	// move to first CGRAM location
	delayMs(1,2);

	lcd_data_write(0b00000000);	delayMs(1,2); // 0x00
	lcd_data_write(0b00000011);	delayMs(1,2);
	lcd_data_write(0b00000100);	delayMs(1,2);
	lcd_data_write(0b00001000);	delayMs(1,2);
	lcd_data_write(0b00010000);	delayMs(1,2);
	lcd_data_write(0b00010000);	delayMs(1,2);
	lcd_data_write(0b00010000);	delayMs(1,2);
	lcd_data_write(0b00010000);	delayMs(1,2);

	lcd_data_write(0b00010000);	delayMs(1,2); // 0x01
	lcd_data_write(0b00010000);	delayMs(1,2);
	lcd_data_write(0b00010000);	delayMs(1,2);
	lcd_data_write(0b00010000);	delayMs(1,2);
	lcd_data_write(0b00010000);	delayMs(1,2);
	lcd_data_write(0b00001000);	delayMs(1,2);
	lcd_data_write(0b00000111);	delayMs(1,2);
	lcd_data_write(0b00000000);	delayMs(1,2);

	lcd_data_write(0b00011111);	delayMs(1,2); // 0x02
	lcd_data_write(0b00000000);	delayMs(1,2);
	lcd_data_write(0b00000000);	delayMs(1,2);
	lcd_data_write(0b00000000);	delayMs(1,2);
	lcd_data_write(0b00000000);	delayMs(1,2);
	lcd_data_write(0b00000000);	delayMs(1,2);
	lcd_data_write(0b00000000);	delayMs(1,2);
	lcd_data_write(0b00001111);	delayMs(1,2);

	lcd_data_write(0b00010010);	delayMs(1,2); // 0x03
	lcd_data_write(0b00001001);	delayMs(1,2);
	lcd_data_write(0b00001001);	delayMs(1,2);
	lcd_data_write(0b00001001);	delayMs(1,2);
	lcd_data_write(0b00001001);	delayMs(1,2);
	lcd_data_write(0b00010010);	delayMs(1,2);
	lcd_data_write(0b00001100);	delayMs(1,2);
	lcd_data_write(0b00000000);	delayMs(1,2);

	lcd_data_write(0b00011111);	delayMs(1,2); // 0x04
	lcd_data_write(0b00000000);	delayMs(1,2);
	lcd_data_write(0b00000000);	delayMs(1,2);
	lcd_data_write(0b00000000);	delayMs(1,2);
	lcd_data_write(0b00000000);	delayMs(1,2);
	lcd_data_write(0b00000000);	delayMs(1,2);
	lcd_data_write(0b00000000);	delayMs(1,2);
	lcd_data_write(0b00011111);	delayMs(1,2);

	lcd_data_write(0b00011100);	delayMs(1,2);  // 0x05
	lcd_data_write(0b00000010);	delayMs(1,2);
	lcd_data_write(0b00000001);	delayMs(1,2);
	lcd_data_write(0b00000111);	delayMs(1,2);
	lcd_data_write(0b00000001);	delayMs(1,2);
	lcd_data_write(0b00000001);	delayMs(1,2);
	lcd_data_write(0b00000010);	delayMs(1,2);
	lcd_data_write(0b00011100);	delayMs(1,2);

	lcd_command_write(0b00000010);	// cursor home
	delayMs(1,2);
	lcd_command_write(0b00001100);	// display on
	delayMs(1,2);

	lcd_gotoxy(0, 0);
	delayMs(1,2);
	lcd_clear();
}
