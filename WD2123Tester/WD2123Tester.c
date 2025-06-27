#include "stdio.h"
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"

/*
    ------------------------------------------------------------------------------------------------------------------
    From Michel Wurtz
    ------------------------------------------------------------------------------------------------------------------

    The WD2123 is a nice chip, but a little tricky because it use only CS1, CS2, CS3 and C/D 
    to select more than 6 registers... Typically, C/D is tied to Address line 0.

        Only one CSx should be low at a time

            CS1 is for accessing channel A
            CS2 is for accessing channel B

                C/D low is for reading or writing on the serial connection ;
                C/D high is for writing the mode and the command registers or reading the status register for each channel.

            CS3 is for setting the baud rate, (It's 2 write only registers)

                C/D low for channel A, 
                C/D high for channel B. 

    Since there is only one address for the mode and command register, the sequence is as follows:
         After a master reset, you can write the mode register, after writing the mode register,
         all the next writes will be to the command register... (this is true for both channels)

    To write the mode register at again (to change the mode), you must put the bit 6 of the command 
    register to 1 (0x40). This is the master reset bit in the command register. The next write will
    affect the MODE register.

    The subsequent writes will again only modify the command register...

    ------------------------------------------------------------------------------------------------------------------
    From David Rumball
    ------------------------------------------------------------------------------------------------------------------

    There are three logical blocks to the WD2123, channel A, channel B 
    and the baud rate gen and each has it’s own chip select 

    These are each assigned a four byte IO slot in the MB2 address map 
    which is further qualified by A0 which drives C/~D so the resultant
    address map is

        * Channel A data and command/status register select
        ACIAD1 EQU $FF08        ; CS1 low C/D low
        ACIAC1 EQU $FF09        ; CS1 low C/D high

        * Channel B data and command/status register select
        ACIAD2 EQU $FF04        ; CS2 low C/D low
        ACIAC2 EQU $FF05        ; CS2 low C/D high

        BAUD1 EQU $FF0C         ; CS3 low C/D low
        BAUD2 EQU $FF0D         ; CS3 low C/D high

        the init code is thus :-

        * Set up acia's.
        SUACIA  LDA     #$CE       <-   2 stop bits/no parity/8 bit data/16x sampling
                STA     ACIAC1
                STA     ACIAC2

                LDA     #$27       <-   force ~RTS low, RX/Tx clock common and enable Tx/Rx
                STA     ACIAC1
                STA     ACIAC2

        * The RTC PRAM stores the baud rates for the WD2123 

                 LDB    #$0F       <- address of saved baud rates in PRAM (each is a four bit field)
                 JSR    GETRTC
                 STA    BAUD1
                 LSRA
                 LSRA
                 LSRA
                 LSRA
                 STA    BAUD2

    ------------------------------------------------------------------------------------------------------------------

                                HOOKUP diagram

                           ----------------------------
                           | 1  GPIO0   | |   VBUS 40 | 
                           | 2  GPIO1   |_|   VSYS 39 | 
                           | 3  GND            GND 38 |
                           | 4  GPIO2       3V3_EN 37 |
                           | 5  GPIO3     3V3(OUT) 36 |
               TXD on FTDI | 6  GPIO4     ADC_VREF 35 |
               RXD on FTDI | 7  GPIO5       GPIO28 34 |
               GND on FTDI | 8  GND            GND 33 |
                           | 9  GPIO6       GPIO27 32 |
                  |--------| 10 GPIO7       GPIO26 31 |
                |-|--------| 11 GPIO8          RUN 30 |
              |-|-|--------| 12 GPIO9       GPIO22 29 |-----------|
              | | |    GND | 13 GND            GND 28 |           |
            |-|-|-|--------| 14 GPIO10      GPIO21 27 |-----------|---|
          |-|-|-|-|--------| 15 GPIO11      GPIO20 26 |---------| |   |
        |-|-|-|-|-|--------| 16 GPIO12      GPIO19 25 |---------|-|---|-|
      |-|-|-|-|-|-|--------| 17 GPIO13      GPIO18 24 |---------|-|-| | |
    |-|-|-|-|-|-|-|--------| 18 GND            GND 23 |         | | | | |
    |-|-|-|-|-|-|-|--------| 19 GPIO14      GPIO17 22 |-----|   | | | | |
    | | | | | | | |    |---| 20 GPIO15      GPIO16 21 |---| |   | | | | |
    | | | | | | | |    |   ----------------------------   | |   | | | | |
    | | | | | | | |    |         Raspberry Pi pico        | |   | | | | |
    | | | | | | | |    |----------------------------------|-|-| | | | | |
    | | | | | | | |                                       | | | | | | | |
    | | | | | | | | |-------------------------------------|-|-|-|-| | | |
    | | | | | | | | | |-----------------------------------|-| | |   | | |
    | | | | | | | | | | |---------------------------------|---|-|   | | |
    | | | | | | | | | | |  ----------------------------   |   |     | | |
    | | | | | | | | | | |  | 1  NC         TXRDY-B 40 |   |   |     | | |
    | | | | | | | | | | |  | 2  TXD-B      RXRDY-B 39 |   |   |     | | |
    | | | | | | | | | | |  | 3  RXD-B        TXE-B 38 |   |   |     | | |
    | | | | | | | | | | |--| 4  /RE       BRKDET-B 37 |   |   |     | | |
    | | | | | | | | | |----| 5  /CS1        /RTS-B 36 |   |   |     | | |
    | | | | | | | | |------| 6  C/D         /CTS-B 35 |   |   |     | | |
    | | | | | | | |--------| 7  D0        SELCLK-B 34 |   |   |     | | |
    | | | | | | |----------| 8  D1       XCI/BCO-B 33 |   |   |     | | |
    | | | | | |------------| 9  D2           XTAL2 32 |   |   |     | | |
    | | | | |              | 10 VSS          XTAL1 31 |---|   |     | | |
    | | | | |--------------| 11 D3             VCC 30 | +5V   |     | | |
    | | | |----------------| 12 D4              MR 29 |-------|     | | |
    | | |------------------| 13 D5       XCI/BCO-A 28 |             | | |
    | |--------------------| 14 D6        SELCLK-A 27 |             | | |
    |----------------------| 15 D7          /CTS-A 26 |             | | |
               |-----------| 16 /CS2        /RTS-A 25 |             | | |
               | |---------| 17 /WE       BRKDET-A 24 |             | | |
               | | |-------| 18 /CS3         TXE-A 23 |             | | |
               | | |       | 19 RXD-A      RXRDY-A 22 |             | | |
               | | |       | 20 TXD-A      TXRDY-A 21 |             | | |
               | | |       ----------------------------             | | |
               | | |         Western Digital WD 2123                |-|-|
               | | |------------------------------------------------|-|-|
               | |--------------------------------------------------|-|
               |----------------------------------------------------|

    All signals go through level converters. They are not shown here to avoid making
    the diagram too crowded. The high side of the level converts go to the WD2123 and 
    the low side go to the pico.

    The pico USB cable and debug probe connect to the development machine. The uart
    from the pico (pins 4, 5 and 6), connect to an FTDI/USB device that connects to
    the PC with TV950 for operator entertainment

*/

// since we are going to use pins 6, 7 and 8 (GPIO4, GPIO5 and GND) for RS232 uart 1 start
// the WD2123 control pints at GP17 instead of GP2 
//
// the following GPIO pins are still available 
//     name       pin
//      GPIO0      1
//      GPIO1      2
//      GPIO2      4
//      GPIO3      5
//      GPIO6      9
//      GPIO22    29
//      GPIO26    31
//      GPIO27    32
//      GPIO28    34
//                  GPIO    pin number on pico  pin number on WD2123
#define PIN_CS1     17      //  22              5
#define PIN_CS2     18      //  24              
#define PIN_CS3     19      //  25              
#define PIN_RD      20      //  26              
#define PIN_WR      21      //  27              
#define PIN_A0      22      // 

// the data line will use 7 through 14
#define PIN_D0       7      //   10, 11, 12, 14, 15, 16, 17, 19

#define PIN_MR      15      //  20
#define PIN_XTAL1   16      //  21  PWM output for ~1.8432 MHz clock

#define DATA_WIDTH   8

// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
#define UART_TX_PIN  4   // this is TX from the pico (RX on the FTDI)
#define UART_RX_PIN  5   // this is RX from the pico (TX on the FTDI)

int pulse_width = 3;        // a value of 3 will give approcimately a 1ms pulse on the read and write lines

bool verbose = true;

absolute_time_t until = 0;
bool timeReached = false;

// We cannot use the pico sleep_ms and spleep_us functions since we are using a pico W and I am
// having trouble getting those functions to work for some reason. If this is re-compiled for
// a standard pico and the actually sleep_ms and sleep_us library functions are used, the 
// pulse width variable should be set to 1. The sleep times are very approximate here.
void my_sleep_ms(uint32_t ms) 
{
    until = make_timeout_time_ms(ms);

    do 
    {
        timeReached = time_reached(until);
        until--;
        if (timeReached)
            break;
    } while (!timeReached);
}

void my_sleep_us(uint32_t us) 
{
    until = us;

    do 
    {
        timeReached = time_reached(until);
        until--;
        if (timeReached)
            break;
    } while (!timeReached);
}

void gpio_setup() 
{
    // Set all the control lines as outputs
    gpio_init(PIN_CS1); gpio_set_dir(PIN_CS1, GPIO_OUT); gpio_put(PIN_CS1, 1);  // channel A not selected
    gpio_init(PIN_CS2); gpio_set_dir(PIN_CS2, GPIO_OUT); gpio_put(PIN_CS2, 1);  // channel B not selected
    gpio_init(PIN_CS3); gpio_set_dir(PIN_CS3, GPIO_OUT); gpio_put(PIN_CS3, 1);  // baud registers not selected
    gpio_init(PIN_RD);  gpio_set_dir(PIN_RD,  GPIO_OUT); gpio_put(PIN_RD,  1);  // not reading
    gpio_init(PIN_WR);  gpio_set_dir(PIN_WR,  GPIO_OUT); gpio_put(PIN_WR,  1);  // not writing
    gpio_init(PIN_MR);  gpio_set_dir(PIN_MR,  GPIO_OUT); gpio_put(PIN_MR,  0);  // not in master reset

    // initially set the Data bus as inputs to the pico
    for (int i = 0; i < DATA_WIDTH; i++) 
    {
        gpio_init(PIN_D0 + i);
        gpio_set_dir(PIN_D0 + i, GPIO_IN);
    }

    // and then set the XTAL1 input to the WD2123 as PWM output from the pico
    gpio_set_function(PIN_XTAL1, GPIO_FUNC_PWM);
}

// set up the pico to provide a PWM signal on PIN_XTAL1 (GPIO16)
void setup_xtal_pwm() 
{
    uint slice = pwm_gpio_to_slice_num(PIN_XTAL1);

    // Target: set up for a ~1.8432 MHz signal from a 125 MHz system clock ()
    uint32_t wrap = 67;                             // Gives: 125 MHz / (wrap + 1) = ~1.838 MHz
    pwm_set_wrap(slice, wrap);                      // actually: 1.838235294117647
    pwm_set_clkdiv(slice, 1.0f);                    // No division
    pwm_set_gpio_level(PIN_XTAL1, wrap / 2);        // 50% duty
    pwm_set_enabled(slice, true);
}

// sets the GPIO7 through GPIO14 lines as either input or output
//      true for output
//      false for input
void set_data_bus_dir(bool output) 
{
    for (int i = 0; i < DATA_WIDTH; i++) 
    {
        gpio_set_dir(PIN_D0 + i, output ? GPIO_OUT : GPIO_IN);
    }
}

void write_data_bus(uint8_t value) 
{
    for (int i = 0; i < DATA_WIDTH; i++) {
        gpio_put(PIN_D0 + i, (value >> i) & 1);
    }
}

uint8_t read_data_bus(void) 
{
    uint8_t val = 0;
    for (int i = 0; i < DATA_WIDTH; i++) {
        if (gpio_get(PIN_D0 + i)) val |= (1 << i);
    }
    return val;
}

// set the proper CSx line to low and the others to high
void set_cs(int cs) 
{
    // before we decide which one to set low, make sure they are all high first,
    // since only one should ever be low at the same time.

    gpio_put(PIN_CS1, 1);
    gpio_put(PIN_CS2, 1);
    gpio_put(PIN_CS3, 1);

    // now set the proper one to low
    switch (cs)
    {
        case 1: gpio_put(PIN_CS1, 0); break;    // select channel A
        case 2: gpio_put(PIN_CS2, 0); break;    // select channel A
        case 3: gpio_put(PIN_CS3, 0); break;    // select baud registers
        default: break;
    }
}

void wd_write(int cs, uint8_t value, bool is_data) 
{
    set_cs(cs);
    gpio_put(PIN_A0, is_data ? 1 : 0);  // Set A0: HIGH for data, LOW for control
    set_data_bus_dir(true);
    write_data_bus(value);
    gpio_put(PIN_WR, 0);
    my_sleep_us(pulse_width);       // may require adjustment
    gpio_put(PIN_WR, 1);
    set_cs(0);
}

uint8_t wd_read(int cs, bool is_data) 
{
    set_cs(cs);
    gpio_put(PIN_A0, is_data ? 1 : 0);  // Set A0: HIGH for data, LOW for status
    set_data_bus_dir(false);
    gpio_put(PIN_RD, 0);
    my_sleep_us(pulse_width);       // may require adjustment
    uint8_t val = read_data_bus();
    gpio_put(PIN_RD, 1);
    set_cs(0);

    return val;
}

// toggle the signal to the MR line high for 1 ms and then low to
// perform a master reset to the WD2123 and put the MODE/COMMAND
// register back in the state to set the mode.
void wd_reset() 
{
    gpio_put(PIN_MR, 1);   // Active high
    my_sleep_ms(pulse_width);
    gpio_put(PIN_MR, 0);
    my_sleep_ms(pulse_width);
}

// Since the WD2123 uses shared mode/control registers we only do this for
// the chip - not the channels
void setup_channel() 
{
    wd_write(3, 0x16, false);       // Mode register: 8 data, no parity, 1 stop, x16 clock
    wd_write(3, 0x03, false);       // Command register: Enable TX and RX
}

void test_channel(const char *label, int cs) 
{
    // the cs value passed in will be used to select either CS1 or CS2
    // for testing

    if (verbose)
        printf("Testing channel %s...\n", label);

    wd_write(cs, 0xA5, true);  // Transmit test byte
    my_sleep_ms(10);

    uint8_t status = wd_read(cs, false);
    if (status & 0x01) 
    {  // RXRDY
        uint8_t val = wd_read(cs, true);
        if (verbose)
            printf  ("Received: 0x%02X\n", val);
        if (verbose)
        {
            if (val == 0xA5)
                printf("PASS: Channel %s loopback verified.\n", label);
            else
                printf("FAIL: Incorrect data on channel %s.\n", label);
        }
    }
    else
    {
        if (verbose)
            printf("FAIL: No data received on channel %s.\n", label);
    }
}

void setup_uart1() 
{
    // Initialize UART1 with 115200 baud rate
    uart_init(UART_ID, 115200);

    // Set the GPIO functions for UART1 TX (GP4) and RX (GP5)
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);  // TX
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);  // RX

    // Redirect printf to UART1
    stdio_uart_init_full(UART_ID, 115200, UART_TX_PIN, UART_RX_PIN);
}

int main() 
{
    set_sys_clock_khz(125000, true);

    stdio_init_all();
    setup_uart1();
    
    // tell the operator we are initializing and starting up
    printf("WD2123 UART Test Initializing...\n");

    // and verify for the user that the clock signals are correct
    // should report 125000000 HZ and 48000000 HZ
    printf("clk_sys = %lu Hz\n", clock_get_hz(clk_sys));
    printf("clk_peri = %lu Hz\n", clock_get_hz(clk_peri));

    // now setup the GPIO pins to communicate with the WD2123
    // this must come before starting the PWM
    gpio_setup();

    // start the 1.8432 KHZ clock for the WD2123.
    setup_xtal_pwm();

    // tell the operator we are running
    printf("WD2123 UART Test Starting...\n");

    // reset the WD2123 and setup the channels
    wd_reset();         // toggle the MT line high breifly
    setup_channel();    // For both channels, as WD2123 uses shared control interface

    // loop on testing the ports
    while (1) 
    {
        test_channel("A", 1);   // the 1 says uses CS1
        test_channel("B", 2);   // the 2 says uses CS2
    }
}
