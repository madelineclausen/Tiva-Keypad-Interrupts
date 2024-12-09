#include <stdint.h>
#include <stdbool.h>

// Define register addresses for peripherals
#define SYSCTL_RCGCGPIO    (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGCTIMER   (*((volatile uint32_t *)0x400FE604))
#define SYSCTL_RCGCUART    (*((volatile uint32_t *)0x400FE618))
#define GPIO_PORTA_DATA    (*((volatile uint32_t *)0x400043FC))
#define GPIO_PORTA_DIR     (*((volatile uint32_t *)0x40004400))
#define GPIO_PORTA_PUR     (*((volatile uint32_t *)0x40004510))
#define GPIO_PORTA_AFSEL_R (*((volatile uint32_t *)0x40004420))
#define GPIO_PORTA_PCTL_R  (*((volatile uint32_t *)0x4000452C))
#define GPIO_PORTA_DEN_R   (*((volatile uint32_t *)0x4000451C))
#define GPIO_PORTB_DATA    (*((volatile uint32_t *)0x400053FC))
#define GPIO_PORTB_DIR     (*((volatile uint32_t *)0x40005400))
#define GPIO_PORTB_PUR     (*((volatile uint32_t *)0x40005510))
#define GPIO_PORTB_DEN     (*((volatile uint32_t *)0x4000551C))
#define GPIO_PORTB_IS      (*((volatile uint32_t *)0x40005404))
#define GPIO_PORTB_IEV     (*((volatile uint32_t *)0x4000540C))
#define GPIO_PORTB_IM      (*((volatile uint32_t *)0x40005410))
#define GPIO_PORTB_IBE     (*((volatile uint32_t *)0x40005408))
#define GPIO_PORTB_ICR     (*((volatile uint32_t *)0x4000541C))
#define GPIO_PORTC_DATA    (*((volatile uint32_t *)0x400063FC))
#define GPIO_PORTC_DIR     (*((volatile uint32_t *)0x40006400))
#define GPIO_PORTC_PUR     (*((volatile uint32_t *)0x40006510))
#define GPIO_PORTC_DEN     (*((volatile uint32_t *)0x4000651C))
#define GPIO_PORTF_DATA    (*((volatile uint32_t *)0x400253FC))
#define GPIO_PORTF_DIR     (*((volatile uint32_t *)0x40025400))
#define GPIO_PORTF_PUR     (*((volatile uint32_t *)0x40025010))
#define GPIO_PORTF_DEN     (*((volatile uint32_t *)0x4002551C))


#define UART0_DR_R             (*((volatile uint32_t *)0x4000C000))
#define UART0_FR_R             (*((volatile uint32_t *)0x4000C018))
#define UART0_IBRD_R           (*((volatile uint32_t *)0x4000C024))
#define UART0_FBRD_R           (*((volatile uint32_t *)0x4000C028))
#define UART0_LCRH_R           (*((volatile uint32_t *)0x4000C02C))
#define UART0_CTL_R            (*((volatile uint32_t *)0x4000C030))
#define UART0_CC_R             (*((volatile uint32_t *)0x4000CFC8))

#define UART0_RX    (1U << 0)   // uart rx is at pa0 (usb)
#define UART0_TX    (1U << 1)   // uart tx is at pa1 (usb)
#define UART_FR_RXFE 0x10       // uart flag (fifo?)
#define UART_CTL_UARTEN 0x301    // enables uart

#define TIMER0_CTL         (*((volatile uint32_t *)0x4003000C))
#define TIMER0_CFG         (*((volatile uint32_t *)0x40030000))
#define TIMER0_TAMR        (*((volatile uint32_t *)0x40030004))
#define TIMER0_TAILR       (*((volatile uint32_t *)0x40030028))
#define TIMER0_ICR         (*((volatile uint32_t *)0x40030024))
#define TIMER0_IMR         (*((volatile uint32_t *)0x40030018))

// Define interrupt control registers
#define NVIC_EN0          (*((volatile uint32_t *)0xE000E100))
#define NVIC_PRI4         (*((volatile uint32_t *)0xE000E410))

// Define bit positions
#define LED_PIN           (1U << 2) // PF2
#define PUSHBUTTON_PIN    (1U << 5) // PB5
#define ROW_MASK          (0x3C)    // PA2-PA5 (Rows)
#define COLUMN_MASK_A     (0xC0)    // PA6, PA7 (Columns on Port A)
#define COLUMN_MASK_C     (0xC0)    // PC6, PC7 (Columns on Port C)

// Keypad configuration
#define ROWS 4
#define COLS 4
char keypad[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

// Function prototypes
void InitUART(void);
void InitGPIO(void);
void InitTimer(void);
void UARTSendChar(char c);
char GetKeypadInput(void);
void DelayMs(uint32_t delay);

// Global variables
volatile char lastKeyPressed = 0;

int main(void) {
    InitUART();     // Initialize UART
    InitGPIO();     // Initialize GPIO for keypad and pushbutton
    InitTimer();    // Initialize Timer for LED blinking

    while (1) {
//        GPIO_PORTB_DIR &= ~(1U << 6);
//        GPIO_PORTB_DATA &= ~(1U << 6);
//        GPIO_PORTB_DIR &= ~(1U << 5);
//        GPIO_PORTB_DATA &= ~(1U << 5);

        uint32_t buttonStatus = GPIO_PORTB_DATA & PUSHBUTTON_PIN;
        // Check for keypad input
        char key = GetKeypadInput();
        if (key != 0) {  // If a key is pressed
            UARTSendChar(key);  // Send the corresponding character to UART
            key = 0;
        }
        // Implement a small delay to debounce
        DelayMs(100);
    }
}

// UART initialization
void InitUART(void) {
    SYSCTL_RCGCUART |= (1U << 0); // Enable clock for Port A (PA0, PA1 for UART0 RX, TX)
    volatile int i;
    for (i = 0; i < 1000; i++); // Simple delay

    UART0_CTL_R &= ~UART_CTL_UARTEN; // disable uart first
    UART0_IBRD_R = 104;              // integer brd
    UART0_FBRD_R = 11;               // fractional brd
    UART0_LCRH_R = 0x60;             // 8 data bits, 1 stop bit, no parity
    UART0_CC_R = 0;                  // system clock for uart
    UART0_CTL_R |= UART_CTL_UARTEN;  // re-enable uart
}

// GPIO initialization
void InitGPIO(void) {
    // Enable clock for GPIO ports
    SYSCTL_RCGCGPIO |= (1U << 5); // Enable Port F for LED
    SYSCTL_RCGCGPIO |= (1U << 1); // Enable Port B for Pushbutton
    SYSCTL_RCGCGPIO |= (1U << 0); // Enable Port A for Keypad
    SYSCTL_RCGCGPIO |= (1U << 2); // Enable Port C for Keypad

    GPIO_PORTA_AFSEL_R |= UART0_RX | UART0_TX;   // set up alternate function on pa0/pa1 -> usb
    GPIO_PORTA_DEN_R |= UART0_RX | UART0_TX;     // make pa0/pa1 digital
    GPIO_PORTA_PCTL_R |= (1U << 0) | (1U << 4);  // configure the pa pins for uart

    // Set up LED on PF2
    GPIO_PORTF_DIR |= LED_PIN;    // Set PF2 as output
    GPIO_PORTF_PUR |= LED_PIN;    // Enable pull-up on PF2
    GPIO_PORTF_DEN |= LED_PIN;     // make led digital

    // Set up pushbutton on PB5
    GPIO_PORTB_DIR &= ~PUSHBUTTON_PIN; // Set PB5 as input
    GPIO_PORTB_PUR |= PUSHBUTTON_PIN;  // Enable pull-up resistor on PB5
    GPIO_PORTB_DEN |= PUSHBUTTON_PIN;  // make button digital

    // Configure interrupt settings for PB5
    GPIO_PORTB_IS &= ~PUSHBUTTON_PIN;  // Configure PB5 as edge-sensitive
    GPIO_PORTB_IBE &= ~PUSHBUTTON_PIN; // Trigger on single edge
    GPIO_PORTB_IEV &= ~PUSHBUTTON_PIN; // Trigger on falling edge
    GPIO_PORTB_IM |= PUSHBUTTON_PIN;    // Unmask interrupt for PB5
    NVIC_EN0 |= (1 << 1); // Enable interrupt for Port B

    // Set up keypad rows (PA2-PA5) as outputs
    GPIO_PORTA_DIR |= ROW_MASK;        // Set PA2-PA5 as output
    GPIO_PORTA_PUR |= ROW_MASK;        // Enable pull-up resistors
    GPIO_PORTA_DEN_R |= ROW_MASK;      // Enable digital functionality for PA2-PA5

    // Set up keypad columns (PA6, PA7, PC6, PC7) as inputs
    GPIO_PORTA_DIR &= ~COLUMN_MASK_A;  // Set PA6, PA7 as input
    GPIO_PORTA_PUR |= COLUMN_MASK_A;   // Enable pull-up resistors
    GPIO_PORTA_DEN_R |= COLUMN_MASK_A; // Enable digital functionality for PA6, PA7

    GPIO_PORTC_DIR &= ~COLUMN_MASK_C;  // Set PC6, PC7 as input
    GPIO_PORTC_PUR |= COLUMN_MASK_C;   // Enable pull-up resistors
    GPIO_PORTC_DEN |= COLUMN_MASK_C;   // Enable digital functionality for PC6, PC7
}

// Function to get keypad input
// Keypad input
char GetKeypadInput(void) {
    int row, col;

    // Set all rows high initially
    GPIO_PORTA_DATA |= ROW_MASK; // Set all rows high

    // Iterate over each row
    for (row = 0; row < ROWS; row++) {
        // Deactivate all rows first
        GPIO_PORTA_DATA |= ROW_MASK; // Set all rows high

        // Activate the current row
        GPIO_PORTA_DATA &= ~(1U << (2 + row)); // Set the current row low

        // Small delay to stabilize the signal
        DelayMs(10); // Adjusted to a shorter time for faster response

        // Check PA6, PA7 (columns on Port A)
        if (!(GPIO_PORTA_DATA & (1U << 6))) {  // Check column PA6
            DelayMs(20);  // Debounce delay
            // Confirm key press after debounce
            if (!(GPIO_PORTA_DATA & (1U << 6))) {
                // Wait for release
                while (!(GPIO_PORTA_DATA & (1U << 6)));
                return keypad[row][0];  // Return the key in row x column 1
            }
        }
        if (!(GPIO_PORTA_DATA & (1U << 7))) {  // Check column PA7
            DelayMs(20);  // Debounce delay
            // Confirm key press after debounce
            if (!(GPIO_PORTA_DATA & (1U << 7))) {
                // Wait for release
                while (!(GPIO_PORTA_DATA & (1U << 7)));
                return keypad[row][1];  // Return the key in row x column 2
            }
        }

        // Check PC6, PC7 (columns on Port C)
        if (!(GPIO_PORTC_DATA & (1U << 6))) {  // Check column PC6
            DelayMs(20);  // Debounce delay
            // Confirm key press after debounce
            if (!(GPIO_PORTC_DATA & (1U << 6))) {
                // Wait for release
                while (!(GPIO_PORTC_DATA & (1U << 6)));
                return keypad[row][2];  // Return the key in row x column 3
            }
        }
        if (!(GPIO_PORTC_DATA & (1U << 7))) {  // Check column PC7
            DelayMs(20);  // Debounce delay
            // Confirm key press after debounce
            if (!(GPIO_PORTC_DATA & (1U << 7))) {
                // Wait for release
                while (!(GPIO_PORTC_DATA & (1U << 7)));
                return keypad[row][3];  // Return the key in row x column 4
            }
        }
    }

    return 0;  // No key pressed
}

// Timer initialization
void InitTimer(void) {
    SYSCTL_RCGCTIMER |= (1U << 0);    // Enable Timer 0
    TIMER0_CTL = 0;                    // Disable Timer 0 during configuration
    TIMER0_CFG = 0x00000000;           // 32-bit timer
    TIMER0_TAMR = 0x00000002;          // Periodic timer
    TIMER0_TAILR = 16000000 - 1;       // Load value for 1 second (16MHz)
    TIMER0_ICR = 0x00000001;           // Clear Timer A timeout flag
    TIMER0_IMR |= 0x00000001;          // Enable timeout interrupt
    TIMER0_CTL |= 0x00000001;          // Enable Timer 0
    NVIC_EN0 |= (1 << (19));           // Enable Timer0A interrupt in NVIC
}

// UART send character function
void UARTSendChar(char c) {
    while (UART0_FR_R & 0x20); // Wait until transmit FIFO is not full
    UART0_DR_R = c;            // Send the character
}

// Delay function
void DelayMs(uint32_t delay) {
    int i;
    for (i = 0; i < delay * 3180; i++);
}
