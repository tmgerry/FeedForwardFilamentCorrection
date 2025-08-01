#include <Wire.h>      // Used to establish serial communication on the I2C bus
#include "tmag_5273.h" // Used to send and recieve specific information from our sensor

TMAG5273 sensor; // Initialize hall-effect sensor

// I2C default address
// Alternate i2c address comments to switch between sensors when using more than 1 sensor
// uint8_t i2cAddress = 0x78;
uint8_t i2cAddress = 0x22;

int super_sample = 250;
float z_buffer = 0;

void setup() 
{
  Wire.begin();
  // Start serial communication at 115200 baud
  Serial.begin(115200);
  delay(1000);

  // Begin example of the magnetic sensor code (and add whitespace for easy reading)
  Serial.println("TMAG5273 Example 1: Basic Readings");
  Serial.println("");

  // If begin is successful (0), then start example
  if(sensor.begin(i2cAddress, Wire) == 1)
  {
    Serial.println("Begin");
  }
  else // Otherwise, infinite loop
  {
    Serial.println("Device failed to setup - Freezing code.");
    // while(1); // Runs forever
  }
  sensorSetup();
}


void loop() 
{
  // Checks if mag channels are on - turns on in setup
  if(sensor.getMagneticChannel() != 0) 
  {
    sensor.setTemperatureEn(true);

    // float magX = sensor.getXData();
    // float magY = sensor.getYData();
    z_buffer = 0;
    for (int i = 0; i < super_sample; i++) {
      float magZ = sensor.getZDataRaw();
      // magZ = ((magZ/80*32768)*-0.0000157091914)+1.998755832;
      z_buffer += magZ;
    }
    z_buffer = z_buffer / super_sample;
    Serial.println(z_buffer, 5);
  }
  else
  {
    // If there is an issue, stop the magnetic readings and restart sensor/example
    Serial.println("Mag Channels disabled, stopping..");
    while(1);
  }

  delay(100);
}



void sensorSetup(){
  Serial.print("Averaging Mode: ");
  Serial.println(sensor.getConvAvg());
  sensor.setConvAvg(0X5);
  delay(500);
  Serial.print("Read Mode Set: ");
  Serial.println(sensor.setReadMode(0X0));
  sensor.setMagneticChannel(0X4);
  sensor.setLowPower(0X1);
  sensor.setGlitchFilter(0X0);
  delay(500);
  sensor.setZAxisRange(0X0);
  delay(500);
  sensor.setMagTemp(0x1);
  delay(1000);
  Serial.print("Averaging Mode: ");
  Serial.println(sensor.getConvAvg());
  Serial.print("Read Mode: ");
  Serial.println(sensor.getReadMode());
  Serial.print("Power Mode: ");
  Serial.println(sensor.getLowPower());
  Serial.print("Range: ");
  Serial.println(sensor.getZAxisRange());
  Serial.print("Temp Mode: ");
  Serial.println(sensor.getMagTemp());
  Serial.print("Glitch Filter: ");
  Serial.println(sensor.getGlitchFiler());
}

