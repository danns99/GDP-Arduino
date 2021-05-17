const byte numBytes = 4;
byte numReceived = 0;

// Create a union to easily convert float to byte
typedef union{
  float number;
  uint8_t receivedBytes[numBytes]; //byte
} FLOATUNION_t;

boolean newData = false;

// Create the variable you want to send
FLOATUNION_t myValue;

void setup() {
  Serial.begin(9600);
}

void loop() {
  myValue.number = recvBytesWithStartEndMarkers();
  Serial.write(myValue.receivedBytes, 4);
  newData = false;
  delay(50);
}

float recvBytesWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    byte startMarker = 0x3C; // "<"
    byte endMarker = 0x3E;  // ">"
    byte rb;
    FLOATUNION_t f;
   

    while (Serial.available() > 0 && newData == false) {
        rb = Serial.read();

        if (recvInProgress == true) {
            if (rb != endMarker) {
                f.receivedBytes[ndx] = rb;
                ndx++;
                if (ndx >= numBytes) {
                    ndx = numBytes - 1;
                }
            }
            else {
                //f.receivedBytes[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                numReceived = ndx;  // save the number for use when printing
                ndx = 0;
                newData = true;
            }
        }

        else if (rb == startMarker) {
            recvInProgress = true;
        }
    }
    return f.number;
}
