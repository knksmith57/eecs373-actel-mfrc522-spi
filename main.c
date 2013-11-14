#include <stdio.h>
#include <inttypes.h>
#include "mfrc522.c"

void loop();

int main() {

  // setup does initial pin config and performs a soft reset on the card
	setup();

  // this init function sets some reasonable defaults for the Sunfounder card
  // that we are using... Still in the works of tweaking these for optimal use
	MFRC522_Init();

  // if you just wanted to do a simple read or write to a register, use one of
  // the below commands...
  /*
	 * uchar readd = Read_MFRC522(TxControlReg);
	 * Write_MFRC522(TxControlReg, 0x03);
  */


  // in this example loop, we keep spinning the CPU until a card is detected
  // within range, then we spit out its ID and keep on rollin'
	while(1) {
		loop();
	}

  return(0);
}

void loop() {
  uchar i,tmp, checksum1;
  uchar status;
  uchar str[MAX_LEN];
  uchar RC_size;
  uchar blockAddr;	//Selection operation block address 0 to 63

  // Find cards, return card type
  status = MFRC522_Request(PICC_REQIDL, str);
  if (status == MI_OK) {
    printf("~~~ Card detected!\t%x , %x\r\n", str[0], str[1]);
  }

  // Anti-collision, return card serial number 4 bytes
  status = MFRC522_Anticoll(str);
  memcpy(serNum, str, 5);
  if (status == MI_OK) {
    checksum1 = serNum[0] ^ serNum[1] ^ serNum[2] ^ serNum[3];

    printf("~~~ The card's number is: %x %x %x %x %x\t checksum: %x\r\n", serNum[0], serNum[1], serNum[2], serNum[3], serNum[4], checksum1);
  }

  // once done, good to put the card into hibernation...
  // MFRC522_Halt();
}
