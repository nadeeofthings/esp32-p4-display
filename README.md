# ESP32-P4 Meatrol ME231 Industrial Modbus Display

High-visibility industrial power measurement display application built for the **Waveshare ESP32-P4-WIFI6-Touch-LCD-7B** development board (7-inch 1024x600 MIPI DSI LCD, GT911 Touch Screen, Onboard RS485 transceiver).

The application reads live Voltage and Current values from a **Meatrol ME231** Power Meter over Modbus RTU (RS485) at 10Hz and displays them in large, ultra-crisp, high-contrast typography designed for long-distance legibility.

---

## Key Features

- **RGB Screen Spill Test**:
  - Automatically runs a 0.5-second full-screen color test sequence (**Red** $\rightarrow$ **Green** $\rightarrow$ **Blue**) on startup for display hardware verification.

- **Animated Splash Screen**:
  - Displays `"INDUSTRIAL ELECTRONIC SYSTEMS"` centered in solid white text across 3 rows using a high-resolution **140px** bold font.
  - Smooth **1.0-second fade-in**, **5-second hold**, and **1.0-second fade-out** transition.

- **High-Visibility Measurement UI**:
  - Full-screen dual-row layout with a **320px** native anti-aliased font and a +20px vertical offset for optimal balance.
  - **Top Row**: Average Voltage in configurable high-saturation color (default: **Yellow** `0.0V`).
  - **Bottom Row**: Average Current in configurable high-saturation color (default: **Cyan** `0.0A`).

- **Touchscreen Settings & Color Palette Selection**:
  - **Triple-Tap Gesture**: Tap the screen 3 times in quick succession to open the **Font Color Settings** modal.
  - **High-Saturation Color Swatches**: Interactive color palette buttons (Red, Orange, Yellow, Green, Cyan, Blue, Magenta, Amber, White) with real-time text preview.
  - **NVS Non-Volatile Persistence**: Selected font colors are automatically saved to ESP32-P4 NVS flash memory and restored on reboot.

- **Modbus RTU RS485 Communication**:
  - Connects to the Meatrol ME231 power meter over RS485 (19200 Baud, 8N1).
  - Polls 22 holding registers starting at address `1006` every **100ms** (10Hz).
  - Parses 32-bit IEEE 754 floating-point values for Average Current (Registers `1006-1007`) and Average Voltage (Registers `1026-1027`).
  - **Failsafe Logic**: Automatically retains last known good values during packet drops or meter offline states, defaulting gracefully to `0.0V` and `0.0A`.

---

## Hardware Configuration & Pinout

- **Board**: Waveshare ESP32-P4-WIFI6-Touch-LCD-7B (ESP32-P4, 16MB Flash, 32MB PSRAM)
- **Display**: 7.0-inch MIPI DSI LCD (1024x600 Resolution, EK79007 driver)
- **Touch**: GT911 I2C Capacitive Touch Panel (`mirror_x = 0`, `mirror_y = 0` 1:1 screen mapping)
- **RS485 Interface**:
  - **UART Port**: UART1
  - **TXD Pin**: GPIO 27
  - **RXD Pin**: GPIO 26
  - **Power Supply**: Internal LDO channel VO4 (3.3V) dynamically enabled via ESP32-P4 LDO driver

---

## Software Dependencies & Project Structure

- **Framework**: ESP-IDF v6.0.2
- **Graphics Engine**: LVGL v8 (`esp_lvgl_port`)
- **Board Support Package**: `esp32_p4_function_ev_board`

### Project Layout
```text
esp32-p4-display/
├── CMakeLists.txt              # Root CMake build script
├── main/
│   ├── CMakeLists.txt          # Main component build script
│   ├── main.c                  # Core application logic, NVS, UI & FreeRTOS tasks
│   ├── font_bold_320.c         # Native 320px C font for measurement text
│   ├── font_splash_140.c       # Native 140px C font for splash screen text
│   ├── font_menu_32.c          # Native 32px C font for settings menu UI
│   └── idf_component.yml       # ESP-IDF component dependencies
├── components/
│   └── bsp_board_extra/        # Board Support Package extra drivers
├── partitions.csv              # Custom partition table (16MB Flash support)
├── sdkconfig.defaults          # ESP32-P4 default configurations
└── README.md                   # Project documentation
```

---

## Build & Flash Instructions

### Prerequisites
1. Install ESP-IDF v6.0.2 and export environment variables:
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

### Building the Project
```bash
idf.py set-target esp32p4
idf.py build
```

### Flashing to Target Board
Connect the Waveshare ESP32-P4 board via USB (UART/JTAG port) and run:
```bash
idf.py -p /dev/ttyACM0 flash monitor
```

---

## Serial Console Output Example
```log
I (1139) ME231_DISPLAY: Starting Waveshare ESP32-P4 Meatrol ME231 Display Project
I (1150) ME231_DISPLAY: Loaded colors from NVS: Voltage=0x0000FF, Current=0x00FFFF
I (1154) ME231_DISPLAY: LDO VO4 enabled at 3300mV for RS485 transceiver
I (1160) ME231_DISPLAY: RS485 UART initialized (19200 Baud, 8N1, TX Pin 27, RX Pin 26)
I (1430) ME231_DISPLAY: Starting RGB Screen Spill Test...
I (2935) ME231_DISPLAY: Starting Splash Screen Fade In (140px font, White)...
I (3966) ME231_DISPLAY: Splash Screen visible (holding 5s)...
I (8966) ME231_DISPLAY: Starting Splash Screen Fade Out...
I (9996) ME231_DISPLAY: Main UI created with triple-tap gesture listener enabled.
I (9996) ME231_DISPLAY: Startup sequence complete. Starting Modbus Reader task...
```