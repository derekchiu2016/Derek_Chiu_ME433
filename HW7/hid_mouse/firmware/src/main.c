/*******************************************************************************
  MPLAB Harmony Project Main Source File

  Company:
    Microchip Technology Inc.
  
  File Name:
    main.c

  Summary:
    This file contains the "main" function for an MPLAB Harmony project.

  Description:
    This file contains the "main" function for an MPLAB Harmony project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state 
    machines of all MPLAB Harmony modules in the system and it calls the 
    "SYS_Tasks" function from within a system-wide "super" loop to maintain 
    their correct operation. These two functions are implemented in 
    configuration-specific files (usually "system_init.c" and "system_tasks.c")
    in a configuration-specific folder under the "src/system_config" folder 
    within this project's top-level folder.  An MPLAB Harmony project may have
    more than one configuration, each contained within it's own folder under
    the "system_config" folder.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

//Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "system/common/sys_module.h"   // SYS function prototypes
#include <xc.h>
#include <sys/attribs.h>    // __ISR macro
#include "ILI9163C.h"       // LCD header

unsigned char spi_io(unsigned char o);
void initSPI1(void);
void i2c_master_setup(void);
void i2c_master_start(void);
void i2c_master_restart(void);
void i2c_master_send(unsigned char byte);
unsigned char i2c_master_recv(void);
void i2c_master_ack(int val);
void i2c_master_stop(void);
void i2c_sendReg(char,char*,char);
void i2c_readReg(char,char,char*,char);
void LCD_drawCharacter(unsigned short x, unsigned short y, char c);
void LCD_drawArray(unsigned short x, unsigned short y, char *string);

// define IMU registers
#define LSM6 0b1101011
#define WHOAMI 0x0F
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define OUT_TEMP_L 0x20
#define OUT_TEMP_H 0x21
#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27
#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D



// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    /* Initialize all MPLAB Harmony modules, including application(s). */
    SYS_Initialize ( NULL );
    
    // initializations from original code
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
        
    // LCD setup
    LCD_init();
    LCD_clearScreen(WHITE);     // clear the screen
    
    // I2C setup
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();

    // IMU setup (Acceleration, Gyro, Read multiple) 
    char vals[2];
    vals[0] = CTRL1_XL; // accel
    vals[1] = 0x80; // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
    i2c_sendReg(LSM6,vals,2);

    vals[0] = CTRL2_G; // Gyro
    vals[1] = 0x80; // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 dps)
    i2c_sendReg(LSM6,vals,2);

    vals[0] = CTRL3_C;
    vals[1] = 0x04; // IF_INC = 1 (automatically increment register address)
    i2c_sendReg(LSM6,vals,2);
    
  

    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );

    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}

//////// I2C initializations and functions ////////

// I2C Master utilities, 100 kHz, using polling rather than interrupts
// The functions must be called in the correct order as per the I2C protocol
// Change I2C1 to the I2C channel you are using
// I2C pins need pull-up resistors, 2k-10k

void i2c_master_setup(void) {
    // set baud to 100 kHz
    I2C2BRG = 233;               // I2CBRG = [1/(2*100kHz) - 104ns]*48MHz - 2 
    I2C2CONbits.ON = 1;             // turn on the I2C2 module
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

void i2c_sendReg(char address, char * values, char len) {
    int i;
    i2c_master_start();
    i2c_master_send(address<<1);
    for (i=0;i<len;i++) {
        i2c_master_send(values[i]);
    }
    i2c_master_stop();
}

void i2c_readReg(char address, char reg, char * values,char len) {
    int i = 0;

    i2c_master_start();
    i2c_master_send(address<<1);
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send((address<<1)|1);
    for (i=0;i<len;i++) {
        values[i] = i2c_master_recv();
        if (i<len-1) {
            i2c_master_ack(0);
        }
    }
    i2c_master_ack(1);
    i2c_master_stop();
}

//////// END I2C initializations and functions ////////

//////// SPI initializations and functions ////////

// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
    SPI1BUF = o;                // write to SPI1BUF
    while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
        ;
    }
    return SPI1BUF;             // read SPI1BUF
}     

// initialize spi1 and the ram module for LCD
void initSPI1(void) {
    // when a command is beginning (clear CS to low) and when it
    // is ending (set CS high)
    // Master - SPI1, pins are: SDI1(B8), SDO1(A1), SCK1(B14), SS(B7), A0(B25)
    
    SDI1Rbits.SDI1R = 0b0100;   // B8 is SDI1
    RPA1Rbits.RPA1R = 0b0011;   // A1 is SDO1
    TRISBbits.TRISB7 = 0;       // SS is B7
    LATBbits.LATB7 = 1;         // SS starts high
    
    // LCD requires an add'l pin to know if byte is a command or is data
    // A0 / DAT pin (B15))
    ANSELBbits.ANSB15 = 0;      // override B15 analog
    TRISBbits.TRISB15 = 0;      // set B15 as an output
    LATBbits.LATB15 = 0;        // set B15 low
    
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

//////// LCD initializations and functions ////////
void LCD_command(unsigned char com) {
    LATBbits.LATB15 = 0; // DAT
    LATBbits.LATB7 = 0; // CS
    spi_io(com);
    LATBbits.LATB7 = 1; // CS
}

void LCD_data(unsigned char dat) {
    LATBbits.LATB15 = 1; // DAT
    LATBbits.LATB7 = 0; // CS
    spi_io(dat);
    LATBbits.LATB7 = 1; // CS
}

void LCD_data16(unsigned short dat) {
    LATBbits.LATB15 = 1; // DAT
    LATBbits.LATB7 = 0; // CS
    spi_io(dat>>8);
    spi_io(dat);
    LATBbits.LATB7 = 1; // CS
}

void LCD_init() {
    int time = 0;
    LCD_command(CMD_SWRESET);//software reset
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/2) {} //delay(500);

	LCD_command(CMD_SLPOUT);//exit sleep
    time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/200) {} //delay(5);

	LCD_command(CMD_PIXFMT);//Set Color Format 16bit
	LCD_data(0x05);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/200) {} //delay(5);

	LCD_command(CMD_GAMMASET);//default gamma curve 3
	LCD_data(0x04);//0x04
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_GAMRSEL);//Enable Gamma adj
	LCD_data(0x01);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_NORML);

	LCD_command(CMD_DFUNCTR);
	LCD_data(0b11111111);
	LCD_data(0b00000110);

    int i = 0;
	LCD_command(CMD_PGAMMAC);//Positive Gamma Correction Setting
	for (i=0;i<15;i++){
		LCD_data(pGammaSet[i]);
	}

	LCD_command(CMD_NGAMMAC);//Negative Gamma Correction Setting
	for (i=0;i<15;i++){
		LCD_data(nGammaSet[i]);
	}

	LCD_command(CMD_FRMCTR1);//Frame Rate Control (In normal mode/Full colors)
	LCD_data(0x08);//0x0C//0x08
	LCD_data(0x02);//0x14//0x08
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_DINVCTR);//display inversion
	LCD_data(0x07);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_PWCTR1);//Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD
	LCD_data(0x0A);//4.30 - 0x0A
	LCD_data(0x02);//0x05
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_PWCTR2);//Set BT[2:0] for AVDD & VCL & VGH & VGL
	LCD_data(0x02);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_VCOMCTR1);//Set VMH[6:0] & VML[6:0] for VOMH & VCOML
	LCD_data(0x50);//0x50
	LCD_data(99);//0x5b
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_VCOMOFFS);
	LCD_data(0);//0x40
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_CLMADRS);//Set Column Address
	LCD_data16(0x00);
    LCD_data16(_GRAMWIDTH);

	LCD_command(CMD_PGEADRS);//Set Page Address
	LCD_data16(0x00);
    LCD_data16(_GRAMHEIGH);

	LCD_command(CMD_VSCLLDEF);
	LCD_data16(0); // __OFFSET
	LCD_data16(_GRAMHEIGH); // _GRAMHEIGH - __OFFSET
	LCD_data16(0);

	LCD_command(CMD_MADCTL); // rotation
    LCD_data(0b00001000); // bit 3 0 for RGB, 1 for GBR, rotation: 0b00001000, 0b01101000, 0b11001000, 0b10101000

	LCD_command(CMD_DISPON);//display ON
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_RAMWR);//Memory Write
}

void LCD_drawPixel(unsigned short x, unsigned short y, unsigned short color) {
    LCD_setAddr(x,y,x+1,y+1);
    LCD_data16(color);
}

void LCD_setAddr(unsigned short x0, unsigned short y0, unsigned short x1, unsigned short y1) {
    LCD_command(CMD_CLMADRS); // Column
    LCD_data16(x0);
	LCD_data16(x1);

	LCD_command(CMD_PGEADRS); // Page
	LCD_data16(y0);
	LCD_data16(y1);

	LCD_command(CMD_RAMWR); //Into RAM
}

void LCD_clearScreen(unsigned short color) {
    int i;
    LCD_setAddr(0,0,_GRAMWIDTH,_GRAMHEIGH);
		for (i = 0;i < _GRAMSIZE; i++){
			LCD_data16(color);
		}
}

// Takes input (x-coord, y-coord, 'L') where 'L' can be any letter (must be within '')
// Only works if x and y are within screen bounds (limit is (123,120)
void LCD_drawCharacter(unsigned short x, unsigned short y, char c) {
    int i;
    int j;
    if ((x < 124) & (y < 121)) {                    // make sure coordinate is within screen bounds
        
        for (i = 0; i < 5; i++) {
        
            char byte = ASCII[c - 0x20][i];          // subtract 0x20 because of ASCII stuff
            for (j = 7; j >= 0; j--) {
            
                if (((byte >> j) & 1) == 1) {
                    LCD_drawPixel(x+i,y+j,0x0000);      // make the pixel black
                }
            
                if (((byte >> j) & 1) == 0) {
                    LCD_drawPixel(x+i,y+j,0xFFFF);      // make the pixel white
                }
            
            }
        
        }
    
    }
    
    else {
        LCD_clearScreen(RED);                   // if coordinates out of bounds, flash red
    }
    
}

// uses LCD_drawCharacter to write a string instead of individual characters
void LCD_drawArray(unsigned short x, unsigned short y, char *string) {
    int i = 0;
    while (string[i]) {                     // enter loop as long as /0 hasn't been reached
        LCD_drawCharacter(x,y,string[i]);   // print each character in the array
        x = x + 7;                          // increment in x-direction
        
        if (x > 123) {                      // if reach edge, roll back to start of next row
            x = 0;
            y = y + 10;
        }
        
        if (y > 120) {                      // if reach bottom, roll back to top
            y = 0;
        }
        
        i++;
        
    }
    
}

/*******************************************************************************
 End of File
*/

