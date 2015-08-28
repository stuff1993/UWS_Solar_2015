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
	uint8_t driver;
	uint8_t menu_items;
	uint8_t menu_pos;
	uint8_t submenu_items;
	uint8_t submenu_pos;
	uint8_t flags;
}menu;

#define MENU_SELECTED       ((menu.flags & 0x01) >> 0)
#define MENU_DEC_DWN        ((menu.flags & 0x02) >> 1)
#define MENU_INC_DWN        ((menu.flags & 0x04) >> 2)
#define MENU_LEFT_DWN       ((menu.flags & 0x08) >> 3)
#define MENU_RIGHT_DWN      ((menu.flags & 0x10) >> 4)
#define MENU_SEL_DWN        ((menu.flags & 0x20) >> 5)
#define MENU_UNUSED_1       ((menu.flags & 0x40) >> 6)
#define MENU_UNUSED_2       ((menu.flags & 0x80) >> 7)

#define SET_MENU_SELECTED   menu.flags |= 0x01;
#define SET_MENU_DEC_DWN    menu.flags |= 0x02;
#define SET_MENU_INC_DWN    menu.flags |= 0x04;
#define SET_MENU_LEFT_DWN   menu.flags |= 0x08;
#define SET_MENU_RIGHT_DWN  menu.flags |= 0x10;
#define SET_MENU_SEL_DWN    menu.flags |= 0x20;
#define SET_MENU_UNUSED_1   menu.flags |= 0x40;
#define SET_MENU_UNUSED_2   menu.flags |= 0x80;

#define CLR_MENU_SELECTED   menu.flags &= 0xFE;
#define CLR_MENU_DEC_DWN    menu.flags &= 0xFD;
#define CLR_MENU_INC_DWN    menu.flags &= 0xFB;
#define CLR_MENU_LEFT_DWN   menu.flags &= 0xF7;
#define CLR_MENU_RIGHT_DWN  menu.flags &= 0xEF;
#define CLR_MENU_SEL_DWN    menu.flags &= 0xDF;
#define CLR_MENU_UNUSED_1   menu.flags &= 0xBF;
#define CLR_MENU_UNUSED_2   menu.flags &= 0x7F;

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
void _lcd_padding(int row, int pos, int len);
void _buffer_rotate_right(char *_buf, int _len);
void _buffer_rotate_left(char *_buf, int _len);

void menu_inc(uint8_t *_pos, uint8_t _total);
void menu_dec(uint8_t *_pos, uint8_t _total);

uint8_t btn_release_select(void);
uint8_t btn_release_increment(void);
uint8_t btn_release_decrement(void);
uint8_t btn_release_left(void);
uint8_t btn_release_right(void);

void menu_init (void);

#endif /* MENU_H_ */
