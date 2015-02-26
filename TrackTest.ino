  //Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>
#include "config.h"

#include "I2C.h"
#include "MPU6050.h"
#include "Kalman.h" 

//Assign the Chip Select signal to pin 10.
int CS=53;

//ADXL345 Register Addresses
#define	DEVID		0x00	//Device ID Register
#define THRESH_TAP	0x1D	//Tap Threshold
#define	OFSX		0x1E	//X-axis offset
#define	OFSY		0x1F	//Y-axis offset
#define	OFSZ		0x20	//Z-axis offset
#define	DURATION	0x21	//Tap Duration
#define	LATENT		0x22	//Tap latency
#define	WINDOW		0x23	//Tap window
#define	THRESH_ACT	0x24	//Activity Threshold
#define	THRESH_INACT	0x25	//Inactivity Threshold
#define	TIME_INACT	0x26	//Inactivity Time
#define	ACT_INACT_CTL	0x27	//Axis enable control for activity and inactivity detection
#define	THRESH_FF	0x28	//free-fall threshold
#define	TIME_FF		0x29	//Free-Fall Time
#define	TAP_AXES	0x2A	//Axis control for tap/double tap
#define ACT_TAP_STATUS	0x2B	//Source of tap/double tap
#define	BW_RATE		0x2C	//Data rate and power mode control
#define POWER_CTL	0x2D	//Power Control Register
#define	INT_ENABLE	0x2E	//Interrupt Enable Control
#define	INT_MAP		0x2F	//Interrupt Mapping Control
#define	INT_SOURCE	0x30	//Source of interrupts
#define	DATA_FORMAT	0x31	//Data format control
#define DATAX0		0x32	//X-Axis Data 0
#define DATAX1		0x33	//X-Axis Data 1
#define DATAY0		0x34	//Y-Axis Data 0
#define DATAY1		0x35	//Y-Axis Data 1
#define DATAZ0		0x36	//Z-Axis Data 0
#define DATAZ1		0x37	//Z-Axis Data 1
#define	FIFO_CTL	0x38	//FIFO control
#define	FIFO_STATUS	0x39	//FIFO status


 Kalman kalmanX; 
 Kalman kalmanY; 
 Kalman kalmanZ; 
 
 float realAX;
 float realAY; 
 float realAZ;

 float accXangle; // Angle calculate using the accelerometer 
 float accYangle;
 float accZangle;
 float gyroXangle = 180; // Angle calculate using the gyro 
 float gyroYangle = 180;
 float gyroZangle = 180; 
 float compAngleX = 180; // Calculate the angle using a Kalman filter 
 float compAngleY = 180;
 float compAngleZ = 180; 
 float kalAngleX; // Calculate the angle using a Kalman filter 
 float kalAngleY;
 float kalAngleZ; 

 uint32_t timer; 


//This buffer will hold values read from the ADXL345 registers.
char values[10];
char output[20];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x,y,z;
float xg, yg, zg;
char tapType=0;

void setup(){ 
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(115200);
  Serial.println("");
  Serial.println("---");

  I2C::begin();
  MPU6050::init();
   kalmanX.setAngle(0); // Set starting angle 
   kalmanY.setAngle(0); 
   kalmanZ.setAngle(0);
   timer = micros(); 
  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);
  
  //Create an interrupt that will trigger when a tap is detected.
//  attachInterrupt(0, tap, RISING);
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);

  //Send the Tap and Double Tap Interrupts to INT1 pin
  writeRegister(INT_MAP, 0x9F);
  //Look for taps on the Z axis only.
  writeRegister(TAP_AXES, 0x01);
  //Set the Tap Threshold to 3g
  writeRegister(THRESH_TAP, 0x38);
  //Set the Tap Duration that must be reached
  writeRegister(DURATION, 0x10);
  
  //100ms Latency before the second tap can occur.
  writeRegister(LATENT, 0x50);
  writeRegister(WINDOW, 0xFF);
  
  //Enable the Single and Double Taps.
  writeRegister(INT_ENABLE, 0xE0);  
  
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode
//  readRegister(INT_SOURCE, 1, values); //Clear the interrupts from the INT_SOURCE register.
}

void loop(){

    //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
    //The results of the read operation will get stored to the values[] buffer.
    readRegister(DATAX0, 6, values);

    //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
    //The X value is stored in values[0] and values[1].
    x = ((int)values[1]<<8)|(int)values[0];
    //The Y value is stored in values[2] and values[3].
    y = ((int)values[3]<<8)|(int)values[2];
    //The Z value is stored in values[4] and values[5].
    z = ((int)values[5]<<8)|(int)values[4];
    
    //Convert the accelerometer value to G's. 
    //With 10 bits measuring over a +/-4g range we can find how to convert by using the equation:
    // Gs = Measurement Value * (G-range/(2^10)) or Gs = Measurement Value * (8/1024)
    xg = x * 0.0078;
    yg = y * 0.0078;
    zg = z * 0.0078;

  
  if(MPU6050::refresh())
  {
    
      /*==*==*==*==*===* ACC *===*==*==*==*==*/
     float accX = MPU6050::getAccelX();
     float accY = MPU6050::getAccelY();
     float accZ = MPU6050::getAccelZ();
     
      /*==*==*==*==*===* GYRO *===*==*==*==*==*/   
     float gyroX = MPU6050::getGyroX();
     float gyroY = MPU6050::getGyroY();
     float gyroZ = MPU6050::getGyroZ();
      
     /* Calculate the angls based on the different sensors and algorithm */ 
     accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG; 
     accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
     accZangle = (atan2(accX,accY)+PI)*RAD_TO_DEG;   

     float gyroXrate = gyroX; 
     float gyroYrate = -gyroY;
     float gyroZrate = -gyroZ; 
     gyroXangle = kalmanX.getRate()*((float)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate 
     gyroYangle = kalmanY.getRate()*((float)(micros()-timer)/1000000);
     gyroZangle = kalmanZ.getRate()*((float)(micros()-timer)/1000000); 

  //   kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (float)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter 
  //   kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (float)(micros()-timer)/1000000);
  //   kalAngleZ = kalmanZ.getAngle(accZangle, gyroZrate, (float)(micros()-timer)/1000000); 
     
     kalAngleX = kalmanX.getAngle(accXangle, gyroXangle, (float)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter 
     kalAngleY = kalmanY.getAngle(accYangle, gyroYangle, (float)(micros()-timer)/1000000);
     kalAngleZ = kalmanZ.getAngle(accZangle, gyroZangle, (float)(micros()-timer)/1000000);  
     
     timer = micros();  
     
     ApplyRotationMatrix (accXangle, accYangle, accZangle, kalAngleX, kalAngleY, kalAngleZ);

     Serial.print("gyroAngle: ");
     Serial.print(gyroXangle);
     Serial.print(", ");
     Serial.print(gyroYangle);
     Serial.print(", ");
     Serial.print(gyroZangle);
     Serial.println();
     
     Serial.print("accAngle: ");
     Serial.print(accXangle);
     Serial.print(", ");
     Serial.print(accYangle);
     Serial.print(", ");
     Serial.print(accZangle);
     Serial.println();
     Serial.println();
     
     Serial.print("kalAngle: ");
     Serial.print(kalAngleX);
     Serial.print(", ");
     Serial.print(kalAngleY);
     Serial.print(", ");
     Serial.print(kalAngleZ);
     Serial.println();
     
     Serial.print("real: ");
     Serial.print(realAX);
     Serial.print(", ");
     Serial.print(realAY);
     Serial.print(", ");
     Serial.print(realAZ);
     Serial.println();
     Serial.println();

  //    detachInterrupt(0);
  //    delay(500);
  //    attachInterrupt(0, tap, RISING);
  //    int intType=0;
  }
  delay(500); 
}

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}

//void tap(void){
//  //Clear the interrupts on the ADXL345
//  readRegister(INT_SOURCE, 1, values); 
//  if(values[0] & (1<<5))tapType=2;
//  else tapType=1;;
//}

void ApplyRotationMatrix(float accX, float accY, float accZ, float kalAngleX, float kalAngleY, float kalAngleZ) {
  float Roll = kalAngleX,
        Pitch = kalAngleY,
        Yaw = kalAngleZ;
  realAX = cos(Pitch)*cos(Yaw)*accX + ( sin(Pitch)*sin(Roll) - cos(Pitch)*sin(Yaw)*cos(Roll) )*accY + ( cos(Pitch)*sin(Yaw)*sin(Roll) + sin(Pitch)*cos(Roll) ) * accZ;
  realAY = sin(Yaw) * accX + cos(Yaw)*cos(Roll)*accY - cos(Yaw)*sin(Roll)*accZ;
  realAZ = -1*sin(Pitch)*cos(Yaw)*accX + ( sin(Pitch)*sin(Yaw)*cos(Roll) + cos(Pitch)*sin(Roll) )*accY + ( cos(Pitch)*cos(Roll) - sin(Pitch)*sin(Yaw)*sin(Roll) ) * accZ;
}
