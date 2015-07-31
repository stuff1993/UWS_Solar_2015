/*
 * menu.h
 *
 *  Created on: 29 May 2015
 *      Author: Stuff
 */

#ifndef MENU_H_
#define MENU_H_

#define	MAX_MENU_ITEMS 15
#define ERROR_ITEMS 4

#ifndef FLAG
#define FLAG(x) unsigned int x :1;
#endif // FLAG(x)

#define EROW 	"                    "

struct MENUS
{
	void (*menus[MAX_MENU_ITEMS]) (void);
	void (*errors[ERROR_ITEMS]) (void);
	uint8_t DRIVER;
	uint8_t MENU_ITEMS;
	uint8_t MENU_POS;
	uint8_t SUBMENU_ITEMS;
	uint8_t SUBMENU_POS;
	FLAG(SELECTED)
}MENU;

// startup screens
void lcd_display_errOnStart(void);
void lcd_display_driver(void);
void lcd_display_intro(void);

// menus array
void lcd_display_info(void);
void lcd_display_escBus(void);
void lcd_display_home(void);
void lcd_display_drive(void);
void lcd_display_cruise(void);
void lcd_display_MPPT1(void);
void lcd_display_MPPT2(void);
void lcd_display_MPPTPower(void);
void lcd_display_motor(void);
void lcd_display_debug(void);
void lcd_display_errors(void);
void lcd_display_options(void);
void lcd_display_peaks(void);
void lcd_display_runtime(void);
void lcd_display_odometer(void);

// errors array
void lcd_display_SWOC(void);
void lcd_display_HWOC(void);
void lcd_display_COMMS(void);

void _lcd_putTitle(char*);
void _lcd_padding (int row, int pos, int len);

void menuInit (void);

#endif /* MENU_H_ */
