// NOTES
// 50kPa = 5.10 mH20 max water column depth (http://www.convert-measurement-units.com/conversion-calculator.php?type=pressure)
// If max v of 4.5v = 50kPa, then sensorValue of 921 (920.7) = 50kPa

int sensorPin = A0;
int sensorValue = 0, sensorMax = 1023, sensorOffset = 34;
float voltage = 0, kpa = 0, voltageMax = 5.0, kpaRangeTopVoltage = 4.7, airspeed, density=1.225;

void setup() {
  Serial.begin(9600);
}

void loop() {
  sensorValue = analogRead(sensorPin) - sensorOffset;  // Read sensor & adjust with offset
  Serial.print("sensorValue (offset ");
  Serial.print(sensorOffset, DEC);
  Serial.print("): ");
  Serial.print(sensorValue, DEC);
  Serial.print("\t");
  voltage = sensorValue * (kpaRangeTopVoltage / sensorMax);
  kpa=(((voltage/kpaRangeTopVoltage)-0.04)/0.018)+2.2;
  Serial.print("kpa: ");
  Serial.print(kpa, 3);
  Serial.print("\t");
  airspeed=pow((2*kpa*1000)/density,0.5);
  Serial.print("airspeed: ");
  Serial.println(airspeed, 1);
  delay(1000);
}
