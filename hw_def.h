#define UART_BAUDRATE 115200
#define UART_TX_GPIO 0          // UART0 TX default pin is GP0 (Pico pin 1)
#define UART_RX_GPIO 1          // UART0 RX default pin is GP1 (Pico pin 2)
#define DEBUG_GPIO 2            // Timing measurement debug output
#define PB_LEFT_GPIO 21         // Left-side pushbutton
#define PB_RIGHT_GPIO 20        // Right-side pushbutton

#define log_sample_rate 10      // How often to sample data from VESC (in Hz)