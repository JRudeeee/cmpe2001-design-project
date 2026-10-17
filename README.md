# CMPE2001 Design Project – Heart Rate Monitor

STM32 NUCLEO-L476RG heart-rate monitor with erratic-rhythm alarm.
Built with STM32CubeMX (CMake toolchain) and the STM32Cube extension for VS Code.

## Hardware

| Part | Pins |
|---|---|
| 16x2 LCD (I2C1) | SCL D15 (PB8), SDA D14 (PB9) |
| DFRobot MAX30102 HR/SpO2 module (I2C3) | SCL A5 (PC0), SDA A4 (PC1), 3V3 |
| Piezo (TIM2 CH1 PWM) | A0 (PA0) |
| Alert LED | D6 (PB10) |
| Buttons (to GND, internal pull-ups) | Menu D7 (PA8), Back D2 (PA10), Up D3 (PB3), Down D4 (PB5) |

## Use

- **Menu** opens the menu / toggles the selected item / acknowledges an alarm.
- **Back** returns to the home screen. **Up/Down** move the cursor.
- Menu items: LED alert on/off, Sound alert on/off, Simulate on/off.
- **Simulate** feeds a scripted erratic heart rate to the detector (alarm after ~20–25 s).
- Acknowledging an alarm silences the piezo for 30 s; the LED keeps flashing while still erratic.

## Code layout

| File | Purpose |
|---|---|
| `Core/Src/main.c` | Main loop, menu, display, alarm outputs (all in USER CODE blocks) |
| `Core/Src/buttons.c` | Debounced buttons, press-edge flags |
| `Core/Src/oximeter.c` | MAX30102 module driver (from the unit support material) |
| `Core/Src/sss_detect.c` | Erratic heart-rate detector |
| `Core/Src/LCD1602_I2C.c` | **Home LCD driver** (Waveshare AiP31068 @ 0x3E) |

## Notes

- **Lab LCD:** replace `LCD1602_I2C.h/.c` with the lab's PCF8574 version – same API, nothing else changes.
- `sss_detect.h`: reading period set to 4000 ms (the sensor module only updates every ~4 s) and stale timeout to 10000 ms.
- After regenerating from CubeMX, check the user sources are still listed in `CMakeLists.txt` (`target_sources`).
- If the LCD is blank after flashing, power-cycle the board (I2C bus can lock up mid-transfer).
