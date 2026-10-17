#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_
#include <stdint.h>

typedef enum { BTN_MENU, BTN_BACK, BTN_UP, BTN_DOWN, BTN_COUNT } Button;

void    Buttons_Task(void);          /* call every 10 ms */
uint8_t Buttons_Pressed(Button b);   /* 1 once per press, then clears */

#endif