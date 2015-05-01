(*
#
# LEDCtrl
#
*)
(* ****** ****** *)

#define ATS_DYNLOADFLAG 0
#include "share/atspre_define.hats"
#include "{$ARDUINO}/staloadall.hats"

staload UN = "prelude/SATS/unsafe.sats"

(* ****** ****** *)

(*
LedControl lc=LedControl(12,10,11,1); // 
 
// pin 12 is connected to the MAX7219 pin 1
// pin 11 is connected to the CLK pin 13
// pin 10 is connected to LOAD pin 12
// 1 as we are only using 1 MAX7219

*)

%{^
  #define N 8
  typedef char *charptr;
  int theArray[N] = {0,0,0,0,0,0,0,0} ;
%} 

#define N 8
abstype charptr = $extype"charptr"

macdef theArray = $extval(arrayref(int,N),"theArray")



 
(* Declaration of external functions *)
extern fun LedControl(int, int, int, int): void = "ext#"
extern fun shutdown(int, bool): void = "ext#"
extern fun setIntensity(int, int): void = "ext#"
extern fun clearDisplay(int): void = "ext#"
extern fun setLed(int, int, int, bool): void = "ext#"
extern fun loopy(): void = "ext#" // Just a stupid test, to see if things work
extern fun disp_mat(arrayref(int, N)): void = "ext#"    // Display array on dot matrix
extern fun clr_mat(): void = "ext#"     // Clear dot matrix

%{

/*
 *  Copying code from LedControl library for use with ATS file
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

void clearDisplay(int addr) {
    int i, offset;

    if(addr<0 || addr>=maxDevices) { return; }

    offset = addr*8;
    for(i=0; i<8; i++) {
        status[offset+i] = 0;
        spiTransfer(addr, i+1, status[offset+i]);
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

void disp_mat(int A[]) {
  int row;

  for (row=0; row<8; row++)
  {
    setLed(0,A[row]-1,row,true); // turns off LED at col, row
  }
}

void clr_mat() {
  int row, col;

  for (row=0; row<8; row++)
  {
    for (col=0; col<8; col++)
    {
      setLed(0,col,row,false); // turns off LED at col, row
    }
  }
}

%}


fnx find_next{n:pos}
(A: arrayref(int, n), n: int(n)) : bool = let

  fun loop (i: natLte(n)) : natLte(n) =
  ( if i < n
    then
      if A[i] > 0 then loop (i+1) else i
    else n
  )
  
  val i0 = loop (0)
  
in
  
  if i0 < n
  then
    (A[i0] := 1; find2_next (A, n, i0))
  else
    (A[n-1] := A[n-1] + 1; find2_next (A, n, n-1))

end
and

  find2_next {n:pos} (A: arrayref(int, n), n: int(n), i: natLt(n)) : bool = let
    fun test (j: intGte(0)) : bool =
    ( if j >= i
      then true
      else
        ( if A[i] = A[j]
          then false
          else (if (i-j=abs(A[i]-A[j])) then false else test(j+1))
        )
    )
  
  in
  
    if A[i] <= n
    then let
        (* Nothing *)
      in
        if test(0)
        then (if i+1=n then true else find_next(A, n))
        else (A[i] := A[i]+1; find2_next(A, n, i))
      end 

    else let
      val () = A[i] := 0
    in
      if i > 0 then (A[i-1] := A[i-1]+1; find2_next (A, n, i-1)) else false
    end 
  end 
// End of Find_next

macdef LEDPIN = 13
macdef BAUD_RATE = 9600

(* ****** ****** *)

extern fun setup (): void = "mac#"
implement setup () = () where
{
    val () = LedControl(12, 10, 11, 1)
    val () = shutdown(0, false)
    val () = setIntensity(0, 8)
    val () = clearDisplay(0)


  val () = pinMode(LEDPIN, OUTPUT)
  val () = Serial_ptr._begin(BAUD_RATE)
  // val () = clr_mat()
} 

(* ****** ****** *)

implement fprint_val<int> (out, x) = let

  fun ndot(n: int): void =
    if n > 0 then
    (
      Serial_ptr.print(". "); ndot(n-1)
    )
  
  val () = ndot(x-1)
  val () = Serial_ptr.print("Q ")
  val () = ndot(N-x)
  val () = Serial_ptr.println()
  
  // TODO: get row, col
  // val () = setLed(0,col,row,true); // turns on LED at col, row
in
  // nothing
end

(* ****** ****** *)

implement fprint_string (out, x) = Serial_ptr.print(x)
implement fprint_array$sep<> (out) = ()

(* ****** ****** *)
// extern fun disp_mat(A: arrayref(int, N)) : void = "mac#"
(*implement disp_mat(A) = let

  fun disp_row(m: int): void =
    if m < N
    then 
      (setLed(0,A[m],m,true); disp_row(m+1))
    else ()

  (*  if m < N then
    (
      setLed(0,A[m],m,true); disp_row(m+1)
    )
  *)

  val () = disp_row(0)

in
  ((* Nothing *))
end
*)

extern fun loop (): void = "mac#"
implement loop () =
myloop() where
{
  fun myloop(): void = let
  
    val out   = $extval (FILEref, "0")
    val A     = theArray
    val found = find_next (A, N)

    val () =
    if found then
    {
      val () = fprint_arrayref (out, A, i2sz(N))
      val () = Serial_ptr.println()
    }

    val () =
    if ~found then
    {
      val () = fprint_string (out, "All solutions are found!")
      val () = Serial_ptr.println()
    } 

    val () = digitalWrite(LEDPIN, 1)
    val () = delay(250)
    val () = digitalWrite(LEDPIN, 0)
    val () = delay(1000)
    val () = clr_mat()
    val () = loopy()

    val () =
    if found then
    {
      val () = disp_mat(A)
      val () = delay(10000)
    }

  in
    myloop ()
  end 
} 

(* ****** ****** *)

(*
extern fun setup(): void = "mac#"
implement setup() =
{
    val () = LedControl(12, 10, 11, 1)
    val () = shutdown(0, false)
    val () = setIntensity(0, 8)
    val () = clearDisplay(0)
    // val () = pinMode(13, OUTPUT)

}
*)

(* ****** ****** *)

(*
extern fun loop(): void = "mac#"
implement loop() =
{
    //val xs = list_nil()

    val () = loopy()
    // val () = (digitalWrite(13, HIGH) ; delay(200))
    // val () = (digitalWrite(13, LOW ) ; delay(200))
}
*)

(* ****** ****** *)

