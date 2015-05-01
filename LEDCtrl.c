/*
 *  Copying code from LedControl library for use with ATS file
 *
 *
 */

#include <stdbool.h>    // In order to use 'bool' datatype
#include <stdint.h>     // In order to use 'uint8_t' datatype

/* Some definitions taken from Arduino.h */
#define HIGH 0x1
#define LOW  0x0
#define OUTPUT 0x1
#define MSBFIRST 1

typedef uint8_t byte;   // Define a byte

/*
 * Segments to be switched on for characters and digits on
 * 7-Segment Displays
 */

/* The array for shifting the data to the devices */
byte spidata[16];
/* Send out a single command to the device */
// void spiTransfer(int addr, byte opcode, byte data);

/* We keep track of the led-status for all 8 devices in this array */
byte status[64];
/* Data is shifted out of this pin*/
int SPI_MOSI;
/* The clock is signaled on this pin */
int SPI_CLK;
/* This one is driven LOW for chip selectzion */
int SPI_CS;
/* The maximum number of devices we use */
int maxDevices;

//the opcodes for the MAX7221 and MAX7219
#define OP_NOOP   0
#define OP_DIGIT0 1
#define OP_DIGIT1 2
#define OP_DIGIT2 3
#define OP_DIGIT3 4
#define OP_DIGIT4 5
#define OP_DIGIT5 6
#define OP_DIGIT6 7
#define OP_DIGIT7 8
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15


void spiTransfer(int addr, volatile byte opcode, volatile byte data) {
    //Create an array with the data to shift out
    int i;
    int offset=addr*2;
    int maxbytes=maxDevices*2;

    for(i=0; i<maxbytes; i++) {
        spidata[i] = (byte)0;
    }

    //put our device data into the array
    spidata[offset+1] = opcode;
    spidata[offset] = data;

    //enable the line 
    digitalWrite(SPI_CS,LOW);   // defined in Arduino.h

    //Now shift out the data 
    for(i=maxbytes; i>0; i--) {
        shiftOut(SPI_MOSI,SPI_CLK,MSBFIRST,spidata[i-1]);   // defined in Arduino.h
    }

    //latch the data onto the display
    digitalWrite(SPI_CS,HIGH);  // defined in Arduino.h
}


void setScanLimit(int addr, int limit) {
    if(addr<0 || addr>=maxDevices) { return; }
    if(limit>=0 || limit<8) {
        spiTransfer(addr, OP_SCANLIMIT,limit);
    }
}


void shutdown(int addr, int b) {    // b should be bool
    if (addr < 0 || addr >=maxDevices) { return; }

    if (b) { 
        spiTransfer(addr, OP_SHUTDOWN, 0);
    } else { 
        spiTransfer(addr, OP_SHUTDOWN, 1);
    }
}




// LedControl lc=LedControl(12,10,11,1);
void LedControl(int dataPin, int clkPin, int csPin, int numDevices) {
    int i;
    SPI_MOSI=dataPin;
    SPI_CLK=clkPin;
    SPI_CS=csPin;

    if(numDevices<=0 || numDevices>8) {
        numDevices=8;
    }
    maxDevices=numDevices;

    pinMode(SPI_MOSI,OUTPUT);   // Defined in Arduino.h
    pinMode(SPI_CLK,OUTPUT);
    pinMode(SPI_CS,OUTPUT);
    digitalWrite(SPI_CS,HIGH);
    SPI_MOSI=dataPin;

    for(i=0;i<64;i++) {
        status[i]=0x00;
    }
    for(i=0;i<maxDevices;i++) {
        spiTransfer(i,OP_DISPLAYTEST,0);
        setScanLimit(i,7);      //scanlimit is set to max on startup
        spiTransfer(i,OP_DECODEMODE,0); //decode is done in source
        clearDisplay(i);
        shutdown(i,true);       //we go into shutdown-mode on startup
    }
}


    

void setIntensity(int addr, int intensity) {
    if(addr<0 || addr>=maxDevices) { return; }
    if(intensity>=0 || intensity<16) {   
        spiTransfer(addr, OP_INTENSITY, intensity);
    }
}

void clearDisplay(int addr) {
    int i, offset;

    if(addr<0 || addr>=maxDevices) { return; }

    offset = addr*8;
    for(i=0; i<8; i++) {
        status[offset+i] = 0;
        spiTransfer(addr, i+1, status[offset+i]);
    }
}

void setLed(int addr, int row, int column, int state) {  // state should be bool
    int offset;
    byte val = 0x00;

    if(addr<0 || addr>=maxDevices) { return; }
    if(row<0 || row>7 || column<0 || column>7) { return; }

    offset = addr*8;
    // val = B10000000 >> column;
    val = (1 << 7) >> column;

    if(state) {
        status[offset+row] = status[offset+row]|val;
    } else {
        val = ~val;
        status[offset+row] = status[offset+row]&val;
    }

    spiTransfer(addr, row+1,status[offset+row]);
}

void loopy() {
  int row, col;

  for (row=0; row<8; row++)
  {
    for (col=0; col<8; col++)
    {
      setLed(0,col,row,true); // turns on LED at col, row
      delay(25);
    }
  }
 
  for (row=0; row<8; row++)
  {
    for (col=0; col<8; col++)
    {
      setLed(0,col,row,false); // turns off LED at col, row
      delay(25);
    }
  }

}




