/*
Pico Pinout
from https://gabmus.org/posts/raspberry_pi_pico_pinout_in_your_terminal/


                                              ┌╌ LED (GP25)
                                              ┆ ┏━━━━┓
                                          ┌─────┃    ┃─────┐
UART0 TX | I2C0 SDA | SPI0 RX  | GP0  | 01│●  ┆ ┗━━━━┛    ●│40 | VBUS
UART0 RX | I2C0 SCL | SPI0 CSn | GP1  | 02│●  ⏄           ●│39 | VSYS
                                 GND  | 03│■              ■│38 | GND
           I2C1 SDA | SPI0 SCK | GP2  | 04│●    ╭─╮       ●│37 | 3V3_EN
           I2C1 SCL | SPI0 TX  | GP3  | 05│●    │ │       ●│36 | 3V3(OUT)
UART1 TX | I2C0 SDA | SPI0 RX  | GP4  | 06│●    ╰─╯       ●│35 |          | ADC_VREF
UART1 RX | I2C0 SCL | SPI0 CSn | GP5  | 07│●              ●│34 | GP28     | ADC2
                                 GND  | 08│■              ■│33 | GND      | AGND
           I2C1 SDA | SPI0 SCK | GP6  | 09│●   ┌─────┐    ●│32 | GP27     | ADC1     | I2C1 SCL
           I2C1 SCL | SPI0 TX  | GP7  | 10│●   │     │    ●│31 | GP26     | ADC0     | I2C1 SDA
UART1 TX | I2C0 SDA | SPI1 RX  | GP8  | 11│●   │     │    ●│30 | RUN
UART1 RX | I2C0 SCL | SPI1 CSn | GP9  | 12│●   └─────┘    ●│29 | GP22
                                 GND  | 13│■              ■│28 | GND
           I2C1 SDA | SPI1 SCK | GP10 | 14│●              ●│27 | GP21     |          | I2C0 SCL
           I2C1 SCL | SPI1 TX  | GP11 | 15│●              ●│26 | GP20     |          | I2C0 SDA
UART0 TX | I2C0 SDA | SPI1 RX  | GP12 | 16│●              ●│25 | GP19     | SPI0 TX  | I2C1 SCL
UART0 RX | I2C0 SCL | SPI1 CSn | GP13 | 17│●              ●│24 | GP18     | SPI0 SCK | I2C1 SDA
                                 GND  | 18│■              ■│23 | GND
           I2C1 SDA | SPI1 SCK | GP14 | 19│●              ●│22 | GP17     | SPI0 CSn | I2C0 SCL | UART0 RX
           I2C1 SCL | SPI1 TX  | GP15 | 20│●     ● ■ ●    ●│21 | GP16     | SPI0 RX  | I2C0 SDA | UART0 TX
                                          └────────────────┘
                                                 ┆ ┆ ┆
                                                 ┆ ┆ └╌ SWDIO
                                                 ┆ └╌╌╌ GND
                                                 └╌╌╌╌╌ SWCLK
*/

/* Configuration */

#define UART_BAUDRATE 115200
#define CAN_BAUDRATE
#define RTC_I2C_CLK 400000
#define LOG_SAMPLE_RATE 10      // How often to sample data from VESC (in Hz)


/* Pinout */

// ESC UART communication
#define UART_TX_GPIO 12             // UART0 TX
#define UART_RX_GPIO 13             // UART0 RX

// I2C bus for DS1307 RTC
#define RTC_SDA_GPIO 20             // I2C0 SDA
#define RTC_SCL_GPIO 21             // I2C0 SCL

// ESC CAN bus transceiver
#define CAN_TX_GPIO 8               // UART1 TX
#define CAN_RX_GPIO 9               // UART1 RX

// ST75256 LCD SPI interface
#define DISPLAY_SDA_GPIO 3          // SPI0 SDA (MISO)
#define DISPLAY_SCL_GPIO 4          // SPI0 SCL (CLK)
#define DISPLAY_RST_GPIO 5          // Reset
#define DISPLAY_CS_GPIO 6           // Chip Select
#define DISPLAY_BACKLIGHT_GPIO 2    // Active-low PMOS backlight driver

// SD card SPI bus
#define SD_MISO_GPIO 16
#define SD_MOSI_GPIO 19
#define SD_SCK_GPIO 18
#define SD_SS_GPIO 17
#define SD_DETECT_GPIO 15

// Pushbutton GPIO
#define PB_LEFT_GPIO 1
#define PB_RIGHT_GPIO 2
#define PB_CENTER_GPIO 22

// External load control
#define EXT_LOAD_A_GPIO 11
#define EXT_LOAD_B_GPIO 10

// WS2812 LED signal output
#define WS_LED_GPIO 7

// Spare GPIO for debug
#define DEBUG_GPIO 14

// ADC pin numbers
#define ADC_0_GPIO 26
#define ADC_1_GPIO 27
#define ADC_2_GPIO 28

// CYW43 controlled IO
#define CYW43_LED_GPIO 0
#define CYW43_SMPS_PS_GPIO 1


/* Prototypes */

void initialize_gpio(void);
