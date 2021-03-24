#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

float pitch_rate;

void setup(void)
{
  Serial.begin(9600);

  /* Enable auto-ranging */
  gyro.enableAutoRange(true);

  /* Initialise the sensor */
  if (!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while (1);
  }
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  gyro.getEvent(&event);

  pitch_rate = -1* event.gyro.y * 57.2958;
  //Serial.print("Pitch Rate :\t"); 
  Serial.println(pitch_rate);   //Serial.println("deg/s");
  delay(500);
}
