#include <SPI.h>

byte c;
void setup() {
  Serial.begin(115200);
  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  // turn on interrupts
  SPI.attachInterrupt();
}
// SPI interrupt routine
ISR (SPI_STC_vect)
{
  c = SPDR;
  SPDR = c+10;
}  // end of interrupt service routine (ISR) for SPI
void loop () { 
  //Serial.println(c);
  //delay(10);
  }
