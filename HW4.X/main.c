/* ************************************************************************** */
// ME433 Homework 4
// Derek Chiu



#include <xc.h>             // processor SFR definitions
#include <sys/attribs.h>    // __ISR macro
#include <math.h>           // for sine function
#define CS LATAbits.LATA0   // chip select pin

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
    
    // initialize SPI1
    initSPI1();
    
    // create arrays for sine and triangle waves 
    float sinetemp[100];            // initialize a sine array for floating math
    unsigned char sine[100];        // initialize a sine array to turn into ints
    float tritemp[100];             // initialize a triangle array for floating math
    unsigned char tri[100];         // initialize a triangle to turn into ints
    
    int i;
    for (i = 0; i < 100; i++) {
        // sin array should update values 100 times every period for a 10 Hz sine wave
        sinetemp[i] = 255.0*((sin((i/100.0)*2.0*3.14159265)+1.0)/2.0);
        sine[i] = (unsigned char) sinetemp[i];      // cast result into integer       
        
        // triangle array should update values 100 times every period for a 10 Hz tri wave
        tritemp[i] = 255.0*(i/99.0);                           
        tri[i] = (unsigned char) tritemp[i];        // cast result into integer
    }
    
    
    while(1) {      
        
             
        // send values to SPI to return voltage values for sine and triangle waves
        int j;
        for (j = 0; j < 100; j++) {
            setVoltage(0,sine[j]);
            setVoltage(1,tri[j]);
            
            // to make the waves run at 10 Hz, use the core timer
            // set core timer to 0
            _CP0_SET_COUNT(0);
            
            // delay for 1mS (100 values * 1ms = 0.1s = 10 Hz wave)
            while(_CP0_GET_COUNT() < 24000) {
                ;
            }  
            
        }
        

/*      HOMEWORK 1 FLASHING LED
  
        // set core timer to 0
        _CP0_SET_COUNT(0);
         
        // read pushbutton-if pushed, turn off LED and wait 
        while(PORTBbits.RB4 == 0) {     // when pushed, input is LO
            LATAbits.LATA4 = 0;         // set RA4 lo, turn off LED
        }                         

        // turn on LED, set RA4 hi
        LATAbits.LATA4 = 1;
                       
        // delay for 0.5 mS, 24,000,000 / 2000   
        while(_CP0_GET_COUNT() < 12000) {
            ;
        }
        
        // turn off LED, set RA4 lo
        LATAbits.LATA4 = 0;        
    
        // set core timer to 0
        _CP0_SET_COUNT(0);
        
        // delay for 0.5 mS
        while(_CP0_GET_COUNT() < 12000) {
            ;
        }                                       */
                      
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


// initialize spi1 and the ram module
void initSPI1() {
    // set up the chip select pin (A0) as an output
    // the chip select pin is used by the sram to indicate
    // when a command is beginning (clear CS to low) and when it
    // is ending (set CS high)
    ANSELAbits.ANSA0 = 0;       // override AN0
    TRISAbits.TRISA0 = 0;       // set A0 as an output
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
    SPI1BRG = 300;              // baud rate to 12MHz [SPI1BRG = (48MHz/(2*12MHz))-1] -- fastest possible
    SPI1STATbits.SPIROV = 0;  // clear the overflow bit
    SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1;    // master operation
    SPI1CONbits.ON = 1;       // turn on spi 1
                
}

// Input directions:
// char channel: 0(A) or 1(B)
// char voltage: 000(0V) to 255(+3.3V))
void setVoltage(unsigned char channel, unsigned char voltage) {
    
    short to_send = 0b0000000000000000;     // initialize short to be sent via spi_io
    to_send |= voltage << 4;        // left shift the voltage data bits by 4
    to_send |= (0b111 << 12);       // left shift the initialization data bits (BUF, GA, SHDN) by 12
    to_send |= (channel << 15);     // left shift the channel select bit all the way (by 15)
    
    CS = 0;                         // select the SPI chip as slave
    spi_io(to_send >> 8);           // send the first 8 bits to SDI (spi_io takes char which has 8 bits)
    spi_io(to_send);                // send the remaining 8 bits to SDI   
    CS = 1;                         // finish the command
}  









//// I2C Pin Expander
//void someFunction() {
//    // Initialize pins GP0 - GP3 as outputs
//    TRISAbits.TRISA4 = 0;
//    
//    // Set pins GP0 - GP3 as off
//    LATAbits.LATA4 = 1;
//    
//    // Initialize pins GP4 - GP7 as inputs
//    TRISAbits.TRISA4 = 1;
//    LATAbits.LATA4 = 1;
//    
//}