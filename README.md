Actel A2F200M3F + SunFounder Mifare RC522
=========================================

Modified version of [this arduino
code](http://www.grantgibson.co.uk/blog/2012/04/how-to-get-started-with-the-mifare-mf522-an-and-arduino/)
for use with the Actel A2F200M3F boards.

Check out [`main.c`](main.c) for a quick example program that listens for cards and echos out the ID on contact.

## Quickstart

```
#include <stdio.h>
#include <inttypes.h>
#include "mfrc522.c"

int main() {

  // setup does initial pin config, performs a soft reset on the card, and sets
  // some sane defaults for the SunFounder Mifare RC522 card
  setup();

  // now do what you want

  // if you just wanted to do a simple read or write to a register on the chip,
  // use one of the below commands...
  /*
   * uchar readd = Read_MFRC522(TxControlReg);
   * Write_MFRC522(TxControlReg, 0x03);
  */

  return(0);
}
```
