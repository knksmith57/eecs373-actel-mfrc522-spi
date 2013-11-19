#include <stdio.h>
#include <inttypes.h>
#include "mfrc522.c"

void loop();

int main() {

  // setup does initial pin config, performs a soft reset on the card, and sets
  // some sane defaults for the SunFounder Mifare RC522 card
  setup();

  // if you just wanted to do a simple read or write to a register on the chip,
  // use one of the below commands...
  /*
   * uchar readd = Read_MFRC522(TxControlReg);
   * Write_MFRC522(TxControlReg, 0x03);
  */


  // in this example loop, we keep spinning the CPU until a card is detected
  // within range, then we spit out its ID and keep on rollin'
  while(1) {
    loop();
  }

  // once done, good to put the card into hibernation...
  // MFRC522_Halt();

  return(0);
}

void loop() {
  uchar status, checksum1, str[MAX_LEN];

  // Find cards
  status = MFRC522_Request(PICC_REQIDL, str);
  if (status == MI_OK) {
    printf("~~~ Card detected!\t%x , %x\r\n", str[0], str[1]);
  }

  // Anti-collision, return card serial number == 4 bytes
  status = MFRC522_Anticoll(str);
  if (status == MI_OK) {
    checksum1 = str[0] ^ str[1] ^ str[2] ^ str[3];

    printf("~~~ The card's number is: %x %x %x %x %x\t checksum: %x\r\n", str[0], str[1], str[2], str[3], str[4], checksum1);
  }
}
