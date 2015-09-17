/*
 * menu.h
 *
 *  Created on: 29 May 2015
 *      Author: Stuart G
 */

#ifndef MENU_H_
#define MENU_H_

#define	MAX_MENU_ITEMS 15
#define ERROR_ITEMS 4

#define EROW  "                    "
#define SELECTOR ">>"
#define DESELECTOR "  "

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
  uint32_t counter;
}menu;

#define MENU_SELECTED       ((menu.flags & 0x01) >> 0)
#define MENU_UNUSED_1       ((menu.flags & 0x02) >> 1)
#define MENU_UNUSED_2       ((menu.flags & 0x04) >> 2)
#define MENU_UNUSED_3       ((menu.flags & 0x08) >> 3)
#define MENU_UNUSED_4       ((menu.flags & 0x10) >> 4)
#define MENU_UNUSED_5       ((menu.flags & 0x20) >> 5)
#define MENU_UNUSED_6       ((menu.flags & 0x40) >> 6)
#define MENU_UNUSED_7       ((menu.flags & 0x80) >> 7)

#define SET_MENU_SELECTED   menu.flags |= 0x01;
#define SET_MENU_UNUSED_1   menu.flags |= 0x02;
#define SET_MENU_UNUSED_2   menu.flags |= 0x04;
#define SET_MENU_UNUSED_3   menu.flags |= 0x08;
#define SET_MENU_UNUSED_4   menu.flags |= 0x10;
#define SET_MENU_UNUSED_5   menu.flags |= 0x20;
#define SET_MENU_UNUSED_6   menu.flags |= 0x40;
#define SET_MENU_UNUSED_7   menu.flags |= 0x80;

#define CLR_MENU_SELECTED   menu.flags &= 0xFE;
#define CLR_MENU_UNUSED_1   menu.flags &= 0xFD;
#define CLR_MENU_UNUSED_2   menu.flags &= 0xFB;
#define CLR_MENU_UNUSED_3   menu.flags &= 0xF7;
#define CLR_MENU_UNUSED_4   menu.flags &= 0xEF;
#define CLR_MENU_UNUSED_5   menu.flags &= 0xDF;
#define CLR_MENU_UNUSED_6   menu.flags &= 0xBF;
#define CLR_MENU_UNUSED_7   menu.flags &= 0x7F;

// startup screens
void menu_errOnStart(void);
void menu_driver(void);
void menu_intro(void);

// menus array
void menu_info(void);
void menu_escBus(void);
void menu_home(void);
void menu_controls(void);
void menu_cruise(void);
void menu_MPPT1(void);
void menu_MPPT2(void);
void menu_MPPTPower(void);
void menu_motor(void);
void menu_debug(void);
void menu_config(void);
void menu_errors(void);
void menu_options(void);
void menu_peaks(void);
void menu_runtime(void);
void menu_odometer(void);

// errors array
void menu_swoc(void);
void menu_hwoc(void);
void menu_comms(void);
void menu_battery_error(void);

void _lcd_putTitle(char*);
void _lcd_padding(int row, int pos, int len);
void _buffer_rotate_right(char *_buf, int _len);
void _buffer_rotate_left(char *_buf, int _len);

void menu_inc(uint8_t *_pos, uint8_t _total);
void menu_dec(uint8_t *_pos, uint8_t _total);

void menu_init(void);

#endif /* MENU_H_ */
