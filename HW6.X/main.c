/* ************************************************************************** */
// ME433 Homework 6
// Derek Chiu



#include <xc.h>             // processor SFR definitions
#include <sys/attribs.h>    // __ISR macro
#include <math.h>           // for sine function
#define CS LATAbits.LATA0   // chip select pin

// Function prototypes
unsigned char spi_io(unsigned char o);
void initSPI1();
void setVoltage(unsigned char channel, unsigned char voltage);
void i2c_master_setup(void);
void i2c_master_start(void);
void i2c_master_restart(void);
void i2c_master_send(unsigned char byte);
unsigned char i2c_master_recv(void);
void i2c_master_ack(int val);
void i2c_master_stop(void);
unsigned char getIMU();

/* HW 4 function prototypes
void initExpander();
void setExpander(char pin, char level); 
char getExpander();       */

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
    
    // SPI1 setup
    initSPI1();
    
    // I2C setup
    i2c_master_setup();
    
    /* HW 4 setups
    initExpander();                 // initializes GP0-3 as low outputs, GP4-7 as inputs        */
    
    /* HW 4 sine and triangle wave stuff
    // create arrays for sine and triangle waves for SPI DAC
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
    }                                       */
    
    
    while(1) {      
        
        // HOMEWORK 6 IMU I2C
        
        // read from WHOAMI register to make sure connections and I2C code are OK
        while (getIMU() == 0b01101001) {
            setVoltage(0,255);
        }
      
               
               
        
/*        // HOMEWORK 4 SPI (CREATE SINE AND TRIANGLE WAVES) AND I2C (BUTTON/LED)
        
        // SPI DAC
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
        
      HW 4 I/O expander lighting an LED when button pushed        
        // I2C
        // read GP7 (pushbutton) and output HI on GP0 if GP7 HI (pushed)
        //while ((getExpander() && 0b10000000) == 0b10000000) {       // if GP7 is HI...
        if ((getExpander() & 0b10000000) == 0b10000000) {
            setExpander(0b00000001,1);                              // set GP0 HI
        }
        else {
            setExpander(0b00000001,0);                              // otherwise GP0 LO
        }                                                       */
                      
    }
     
}

//////// SPI DAC initializations and functions ////////

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
    SPI1BRG = 1;              // baud rate to 12MHz [SPI1BRG = 1 = (48MHz/(2*12MHz))-1] -- fastest possible
                              // change to 300 if debugging using nScope
    SPI1STATbits.SPIROV = 0;  // clear the overflow bit
    SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1;    // master operation
    SPI1CONbits.ON = 1;       // turn on spi 1
                
}


// function that takes 0-255 and tells SPI to output voltage 0-3.3V
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
//////// END SPI DAC initializations and functions ////////



//////// I2C initializations and functions ////////

// I2C Master utilities, 100 kHz, using polling rather than interrupts
// The functions must be called in the correct order as per the I2C protocol
// Change I2C1 to the I2C channel you are using
// I2C pins need pull-up resistors, 2k-10k

void i2c_master_setup(void) {
    
    // override the analog functionality of B2 and B3
    ANSELBbits.ANSB2 = 0; 
    ANSELBbits.ANSB3 = 0;
    
    // set baud to 400 kHz
    I2C2BRG = 233;                  // I2CBRG = [1/(2*100kHz) - 104ns]*48MHz - 2 
    I2C2CONbits.ON = 1;             // turn on the I2C2 module
}

// turn on accelerometer, gyro and set sample rates; enable multiple register reading
void initIMU() {
    i2c_master_start();
    i2c_master_send(0b11010110);        // send to IMU address and write
    i2c_master_send(0x10);              // send to CTRL1_XL register (accelerometer)
    i2c_master_send(0b100000__);        // sample rate 1.66 kHz, 2g sensitivity, x filter
    
    i2c_master_send(0x11);              // send to CTRL2_G register (gyro)
    i2c_master_send(0b100000_0);        // sample rate 1.66 kHz, 245 dps sensitivity, x filter
    
    i2c_master_send(0x12);              // send to CTROL3_C register (enable multi-read)
    i2c_master_send(0b00000100);        // make IF_INC bit 1 to enable multi, but leave all others at default
    
    i2c_master_stop();
}

// I2C functions
// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C2CONbits.SEN = 1;            // send the start bit
    while(I2C2CONbits.SEN) { ; }    // wait for the start bit to be sent
}

void i2c_master_restart(void) {     
    I2C2CONbits.RSEN = 1;           // send a restart 
    while(I2C2CONbits.RSEN) { ; }   // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
  I2C2TRN = byte;                   // if an address, bit 0 = 0 for write, 1 for read
  while(I2C2STATbits.TRSTAT) { ; }  // wait for the transmission to finish
  if(I2C2STATbits.ACKSTAT) {        // if this is high, slave has not acknowledged
    // ("I2C2 Master: failed to receive ACK\r\n");
  }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C2CONbits.RCEN = 1;             // start receiving data
    while(!I2C2STATbits.RBF) { ; }    // wait to receive the data
    return I2C2RCV;                   // read and return the data
}

void i2c_master_ack(int val) {        // sends ACK = 0 (slave should send another byte)
                                      // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val;          // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;            // send ACKDT
    while(I2C2CONbits.ACKEN) { ; }    // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // send a STOP:
  I2C2CONbits.PEN = 1;                // comm is complete and master relinquishes bus
  while(I2C2CONbits.PEN) { ; }        // wait for STOP to complete
}

// read from WHO_AM_I register to make sure initialization worked
unsigned char getIMU() {
    i2c_master_start();
    i2c_master_send(0b11010110);     // send POLOLU address and write
    i2c_master_send(0x0F);              // send register address (GPIO) - HI (1) or LO (0)
    i2c_master_restart();
    i2c_master_send(0b11010111);        // send address and read
    char r = i2c_master_recv();         // save the value returned
    i2c_master_ack(1);
    i2c_master_stop();
    return r;
}

/*  ////// HW 4 I/O expander initialization, setExpander, getExpander  //////
void initExpander() {
       
    ////// HW 4 writing to I/O expander chip ///////    
    // initialize pins GP0-3 as outputs and GP4-7 as inputs
    i2c_master_start();
    i2c_master_send(0b01000000);        // send address and write
    i2c_master_send(0x00);              // send register address (IODIR) - input or output         
    i2c_master_send(0b11110000);        // send bits to set GP0-GP3 outputs (0), GP4-GP7 inputs (1)
                                        // Pins: (7 6 5 4 3 2 1 0)
    i2c_master_stop();    

    // set pins GP0-3 as off
    i2c_master_start(); 
    i2c_master_send(0b01000000);        // send address and write
    i2c_master_send(0x0A);              // send register address (OLAT) - HI (1) or LO (0)
    i2c_master_send(0b00000000);        // set pins 0-3 as low
    i2c_master_stop();

}                                       

// Input directions:
// char pin takes 0bxxxxxxxx
// char level takes 1 or 0
// if level is 1, sets selected pins HI, otherwise if level is 0, all pins LO
void setExpander(char pin, char level) {
    char levelbyte = 0b11111111;
    if (level == 0) {
        levelbyte = 0b00000000;
    }    
    i2c_master_start();
    i2c_master_send(0b01000000);        // send address and write
    i2c_master_send(0x0A);              // send register address (OLAT) - HI (1) or LO (0)
    i2c_master_send(pin & levelbyte);   // set selected pins to HI or all pins to LO, depending on level 
    i2c_master_stop();
}

char getExpander() {
    i2c_master_start();
    i2c_master_send(0b01000000);        // send address and write
    i2c_master_send(0x09);              // send register address (GPIO) - HI (1) or LO (0)
    i2c_master_restart();
    i2c_master_send(0b01000001);        // send address and read
    char r = i2c_master_recv();         // save the value returned
    i2c_master_ack(1);
    i2c_master_stop();
    return r;
}                                       */

//////// END I2C initializations and functions ////////