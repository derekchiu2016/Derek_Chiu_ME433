/* ************************************************************************** */
// ME433 Homework 4
// Derek Chiu



#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro

// Define DEVCFG registers
// Refer to: /Microchip/xc32/v1.40/docs/config_docs/32mx250f128b.html

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock (8MHz) to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module


int main() {
   
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
       
    // initialize pushbutton pin 11 as input pin
    TRISBbits.TRISB4 = 1;
    
    // initialize LED pin 12 as an output that is on
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 1;
    
    __builtin_enable_interrupts();
    
    while(1) {
        
        // read pushbutton-if pushed, turn off LED and wait 
        while(PORTBbits.RB4 == 0) {     // when pushed, input is LO
            LATAbits.LATA4 = 0;         // set RA4 lo, turn off LED
        }
                                   
        // set core timer to 0
        _CP0_SET_COUNT(0);
        
        // turn on LED, set RA4 hi
        LATAbits.LATA4 = 1;
                       
        // delay for 0.5 mS, 10,000 x 2 = 20,000 ticks   
        while(_CP0_GET_COUNT() < 20000) {
            ;
        }
        
        // turn off LED, set RA4 lo
        LATAbits.LATA4 = 0;        
    
        // set core timer to 0
        _CP0_SET_COUNT(0);
        
        // delay for 0.5 mS
        while(_CP0_GET_COUNT() < 20000) {
            ;
        }
        
    }
     
}

// SPI DAC

// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
    SPI1BUF = o;                // write to SPI1BUF
    while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
        ;
    }
    return SPI1BUF;             // read SPI1BUF
}     

#define CS LATAbits.LATA0       // chip select pin
// initialize spi1 and the ram module
void initSPI1() {
    // set up the chip select pin (A0) as an output
    // the chip select pin is used by the sram to indicate
    // when a command is beginning (clear CS to low) and when it
    // is ending (set CS high)
    TRISAbits.TRISA0 = 0;
    CS = 1;
    
    // Pin functions, select a pin for SDI/SDO                                   
    SDI1Rbits.SDI1R = 0;        // set A1 as SDI
    RPA1Rbits.RPA1R = 0b0011;   // set SDO1 as A1  

    // Master - SPI1, pins are: SDI(A1), SDO(), SCK1(25)
    // we manually control SS as a digital output 
    // since the pic is just starting, we know that spi is off. We rely on defaults here

    // setup spi1
    SPI1CON = 0;              // turn off the spi module and reset it
    SPI1BUF;                  // clear the rx buffer by reading from it
    SPI1BRG = 0x1;            // baud rate to 12MHz [SPI1BRG = (48MHz/(2*12MHz))-1] -- fastest possible
    SPI1STATbits.SPIROV = 0;  // clear the overflow bit
    SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1;    // master operation
    SPI1CONbits.ON = 1;       // turn on spi 1
                
}
       
void setVoltage(char channel, char voltage) {
    char data = 0x128;       // = 1.65V
    short to_send = data << 4;  // left shift the voltage data bits by 4
    to_send |= (0b1111 << 11)   // left shift the initialization data bits by 11
    CS = 0;                   // select the SPI chip as slave
    spi_io(to_send << 8);             // send the first 8 bits to SDI
    spi_io(to_send);             // send the remaining 8 bits to SDI
    CS = 1;                   // finish the command
}  









// I2C Pin Expander
void someFunction() {
    // Initialize pins GP0 - GP3 as outputs
    TRISAbits.TRISA4 = 0;
    
    // Set pins GP0 - GP3 as off
    LATAbits.LATA4 = 1;
    
    // Initialize pins GP4 - GP7 as inputs
    TRISAbits.TRISA4 = 1;
    LATAbits.LATA4 = 1;
    
}