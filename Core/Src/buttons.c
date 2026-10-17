#include "buttons.h"
#include "main.h"

#define DEBOUNCE_SAMPLES 3

static GPIO_TypeDef *const s_ports[BTN_COUNT] = {
    Button_1_GPIO_Port, Button_2_GPIO_Port, Button_3_GPIO_Port,
    Button_4_GPIO_Port};

static const uint16_t s_pins[BTN_COUNT] = {Button_1_Pin, Button_2_Pin,
                                           Button_3_Pin, Button_4_Pin};

static uint8_t s_stable[BTN_COUNT] = {1, 1, 1,
                                      1}; /* 1 = released, 0 = pressed */

static uint8_t s_count[BTN_COUNT] = {0, 0, 0, 0}; /* debounce counter */

static uint8_t s_pressed[BTN_COUNT] = {0, 0, 0,
                                       0}; /* 1 = pressed since last check */

void Buttons_Task(void) {
  for (uint8_t b = 0; b < BTN_COUNT; b++) {
    uint8_t raw = HAL_GPIO_ReadPin(s_ports[b], s_pins[b]);

    if (raw == s_stable[b]) {
      s_count[b] = 0;
    } else {
      s_count[b]++;
      if (s_count[b] >= DEBOUNCE_SAMPLES) {
        s_stable[b] = raw;
        s_count[b] = 0;
        if (s_stable[b] == 0) {
          s_pressed[b] = 1;
        }
      }
    }
  }
}

uint8_t Buttons_Pressed(Button b) {
    uint8_t was = s_pressed[b];
    s_pressed[b] = 0;
    return was;
}