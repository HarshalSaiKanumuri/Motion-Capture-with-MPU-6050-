#include <Wire.h>
#include <math.h>
#include <Servo.h>
#define Sample 100;
#define Rad2Deg  180 / M_PI;
float Alpha =  0.95;
float Roll_r, Pitch_r, Yaw_r;
float AccX, AccY, AccZ,GyroX,GyroZ,GyroY;
float AngleRoll, AnglePitch, AngleSide ;
float LoopTimer;
float offset_z_ACC , offset_y_ACC , offset_x_ACC , offset_x_GYR , offset_y_GYR , offset_z_GYR ;

Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position


void acc() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // Register for low-passfilter
  Wire.write(0x05);  // Bandwidth 10Hz
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // register for adjusting scale
  Wire.write(0x03); // +/- 16g Full Scale Range
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  AccX=(AccXLSB/2048) - offset_x_ACC;
  AccY=(AccYLSB/2048) - offset_y_ACC;
  AccZ=(AccZLSB/2048) - offset_z_ACC;
  AngleRoll = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ)) * Rad2Deg;
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ)) * Rad2Deg;
}

void gyro(void){

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // Register for low-passfilter
  Wire.write(0x05);  // Bandwidth 10Hz
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Registor for sensitivity
  Wire.write(0x02); // +/- 1000 degrees per second ( Important for Integration )
  Wire.endTransmission();          

  Wire.beginTransmission(0x68);
  Wire.write(0x43);  // first register of Gyroscope
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();

  Roll_r=GyroX/16.4; // LSB Sensitivity 65.5 LSB/*/s
  Pitch_r=GyroY/16.4;
  Yaw_r=GyroZ/16.4;
}

void offset_Calc(void){
  // Average a number of gyroscope readings to determine the offset
  for (int i = 0; i < 100; i++)
  {
    offset_x_ACC += AccX;
    offset_y_ACC += AccY;
    offset_z_ACC += AccZ;

    offset_x_GYR += GyroX;
    offset_y_GYR += GyroY;
    offset_z_GYR += GyroZ;
  }
  offset_x_ACC /= Sample;
  offset_y_ACC /= Sample;
  offset_z_ACC /= Sample;

  offset_x_GYR /= Sample;
  offset_y_GYR /= Sample;
  offset_z_GYR /= Sample;
   
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B); // Turn on MPU
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);  // for getting ACC values for offset
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();  

  Wire.beginTransmission(0x68);
  Wire.write(0x43);  // first register of Gyroscope
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();

  offset_Calc();
  myservo1.attach(8);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(9);  // attaches the servo on pin 9 to the servo object

  myservo1.write(pos);
  myservo2.write(pos);
  


}
void loop() {

  gyro();
  acc();
  delay(40);
  //Serial.print("Acceleration X [g]= ");
  //Serial.print(AccX);
  //Serial.print(" Acceleration Y [g]= ");
  //Serial.print(AccY);
  //Serial.print(" Acceleration Z [g]= ");
  //Serial.println(AccZ);
  
  //AnglePitch = AnglePitch * Alpha + (1-Alpha) * GyroY * 40 ;
  //AngleRoll = AngleRoll * Alpha + (1-Alpha)  ;

  Serial.print("Pitch Angle : ");
  Serial.print(AnglePitch);
  Serial.print(" Roll Angle : ");
  Serial.print(AngleRoll);
  Serial.print("Angle Side : ");
  AngleSide = AngleSide + GyroZ;
  Serial.println(AngleSide);

  myservo1.write(AnglePitch);
  myservo2.write(AngleRoll);


}
