// Create a union to easily convert float to byte
typedef union{
  float number;
  uint8_t bytes[4]; //byte
} FLOATUNION_t;

// Create the variable you want to send
FLOATUNION_t myValue;

void setup() {
  // initialize serial, use the same boudrate in the Simulink Config block
  Serial.begin(9600);
}

void loop(){
  myValue.number = getFloat(); // Give your float a value
 
  // Print header: Important to avoid sync errors!
  //Serial.write('A'); 

  
  // Print float data
//  for (int i=0; i<4; i++){
//    Serial.write(myValue.bytes[i]); 
//  }

//  Serial.write((byte*) &myValue.bytes, 1*sizeof(float));
  Serial.write(myValue.bytes, 4); 

  // Print terminator
  //Serial.print('\n');
  
  // Use the same delay in the Serial Receive block
  delay(50);
}


float getFloat(){
    int i = 0;
    FLOATUNION_t f;
    while (i < 4 ){
        f.bytes[i] = Serial.read();
        i += 1;
    }
    return f.number;
}
