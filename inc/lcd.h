
#ifndef LCD_H_
#define LCD_H_

#define MAX_CHAR_IN_ONE_LINE 	20

#define RS_H	LPC_GPIO0->FIOSET = (1<<15);
#define RS_L	LPC_GPIO0->FIOCLR = (1<<15);

#define E_H		LPC_GPIO0->FIOSET = (1<<16);
#define E_L		LPC_GPIO0->FIOCLR = (1<<16);


#define D0_H	LPC_GPIO2->FIOSET = (1<<9);
#define D0_L	LPC_GPIO2->FIOCLR = (1<<9);

#define D1_H	LPC_GPIO2->FIOSET = (1<<8);
#define D1_L	LPC_GPIO2->FIOCLR = (1<<8);

#define D2_H	LPC_GPIO2->FIOSET = (1<<7);
#define D2_L	LPC_GPIO2->FIOCLR = (1<<7);

#define D3_H	LPC_GPIO2->FIOSET = (1<<6);
#define D3_L	LPC_GPIO2->FIOCLR = (1<<6);



void setLCD(void);
void lcd_putstring( unsigned char line, unsigned char Pos, char *string );
void lcd_clear( void);
void lcd_backlight_on(void);
void lcd_backlight_off(void);
void lcd_gotoxy(unsigned int x, unsigned int y);
void lcd_putchar(int c);
void displayTEMP(void);

#endif // LCD_H_
