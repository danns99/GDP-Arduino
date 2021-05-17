#include <Firmata.h>

// Create a union to easily convert float to byte
typedef union{
  float number;
  byte receivedBytes[4]; //byte
} FLOATUNION_t;

// Create the variable you want to send
FLOATUNION_t myValue;

void setup() {
  Firmata.begin(9600);
}

void loop() {
  myValue.number = 10;//recvBytesWithStartEndMarkers();
  Firmata.write(myValue.receivedBytes);
}
