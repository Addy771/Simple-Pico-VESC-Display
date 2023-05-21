#define UART_TX_GPIO 0          // UART0 TX default pin is GP0 (Pico pin 1)
#define UART_RX_GPIO 1          // UART0 RX default pin is GP1 (Pico pin 2)

#define OLED_SDA_GPIO 4
#define OLED_SCK_GPIO 5
#define OLED_RST_GPIO 15

#define RTC_SDA_GPIO 10
#define RTC_SCK_GPIO 11
#define RTC_CE_GPIO 9

#define DEBUG_GPIO 2            // Timing measurement debug output

#define SD_MISO_GPIO 16
#define SD_MOSI_GPIO 19
#define SD_SCK_GPIO 18
#define SD_SS_GPIO 17
#define SD_DETECT_GPIO 14

#define PB_LEFT_GPIO 21         // Left-side pushbutton
#define PB_RIGHT_GPIO 20        // Right-side pushbutton

#define UART_BAUDRATE 115200
#define LOG_SAMPLE_RATE 10      // How often to sample data from VESC (in Hz)