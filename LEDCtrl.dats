(*
#
# LEDCtrl
#
*)

#define ATS_DYNLOADFLAG 0
#include "share/atspre_define.hats"
#include "{$ARDUINO}/staloadall.hats"

staload "../../SATS/LedControl/LedControl.sats"

(* ****** ****** *)

%{^
  #include "LedControl.h"

  #define N 8
  int theArray[N] = {0,0,0,0,0,0,0,0} ;
  LedControl theLC = LedControl(12,10,11,1);
%} 

#define N 8
macdef theArray = $extval(arrayref(int,N),"theArray")
macdef lc = $extval(LedControl_ptr, "&theLC")

(* ****** ****** *)

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
  
in
  // nothing
end

implement fprint_string (out, x) = Serial_ptr.print(x)
implement fprint_array$sep<> (out) = ()

(* ****** ****** *)

macdef LEDPIN = 13
macdef BAUD_RATE = 9600

fun disp_mat(A: arrayref(int, N)): void = 
let
  fun disp_row(A: arrayref(int, N), i: natLte(N)): void = 
  if i < N then let 
    val tmp = A[i]
    val () = lc.setLed(0,tmp-1,i,true) // turns off LED at col, row
  in
    disp_row(A, i+1)
  end else ()
in
  disp_row(A, 0)
end

(* ****** ****** *)

extern fun setup (): void = "mac#"
implement setup () = () where
{
  val () = lc.shutdown(0, false)
  val () = lc.setIntensity(0, 8)
  val () = lc.clearDisplay(0)

  val () = pinMode(LEDPIN, OUTPUT)
  val () = Serial_ptr._begin(BAUD_RATE)
}

extern fun loop (): void = "mac#"
implement loop () =
myloop() where
{
  fun myloop(): void = let
    val out   = $extval (FILEref, "0")
    val A     = theArray
    val found = find_next (A, N)

    val () =
    if ~found then
    {
      val () = fprint_string (out, "All solutions are found!")
      val () = Serial_ptr.println()
    }

    // val () = digitalWrite(LEDPIN, 1)
    // val () = delay(250)
    // val () = digitalWrite(LEDPIN, 0)
    // val () = delay(1000)
    // val () = clr_mat()
    // val () = loopy()

    val () = lc.clearDisplay(0)
    val () =
    if found then
    {
      val () = fprint_arrayref (out, A, i2sz(N))
      val () = Serial_ptr.println()

      val () = disp_mat(A)
      val () = delay(10000)
    }
  in
    myloop ()
  end 
} 

(* ****** ****** *)

