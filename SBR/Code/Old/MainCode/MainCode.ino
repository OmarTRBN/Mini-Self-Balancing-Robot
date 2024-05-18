#include <Wire.h>
#include <SoftwareSerial.h>
#include <String.h>

#define ENA 3 // PWM control for motor 1
#define IN1 4 // Direction control for motor 1
#define IN2 5 // Direction control for motor 1
#define ENB 6 // PWM control for motor 2
#define IN3 7 // Direction control for motor 2
#define IN4 8 // Direction control for motor 2
int motorSpeed = 0;

bool valuesReceived = false;
const byte rxPin = 12;
const byte txPin = 11;
SoftwareSerial BTSerial(rxPin, txPin); // RX TX

const int MPU = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
float gyroWeight = 0.4;

double Kp=0.0, Ki=0.0, Kd=0.0;
double error=0.0, lastError=0.0, beforeLastError=0.0, outputPID=0.0;
double dt;
double proportional, integral, derivative;
unsigned long lastTime=0.0;

void calculateAngles();
int controllerPID(float, float);
void calculate_IMU_error();
float KALMAN(float);

void setup()
{
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  
//  Serial.begin(9600);
  BTSerial.begin(9600);

  while (!valuesReceived) // Loop until values are received
  {
    if (BTSerial.available())
    {
      String message = BTSerial.readStringUntil('\n'); // Read until newline character
      char messageBuffer[100];
      message.toCharArray(messageBuffer, 100); // Convert String to char array
      
      // Extract values using strtok()
      char *token = strtok(messageBuffer, ":");
      while (token)
      {
        // Serial.println(token);
        if (strstr(token, "kp=") != NULL)
          Kp = atof(token+3);
        else if (strstr(token, "ki=") != NULL)
          Ki = atof(token+3);
        else if (strstr(token, "kd=") != NULL)
          Kd = atof(token+3);
                   
        token = strtok(NULL, ":"); // Get next token
      }
      valuesReceived = true; // Set flag to indicate values have been received
      message = ""; // Clear message 
    }
  }
  BTSerial.print("Kp=");BTSerial.print(Kp);BTSerial.print(":)Ki=");BTSerial.print(Ki);
  BTSerial.print(":)Kd=");BTSerial.println(Kd);
  BTSerial.println("These are your PID values.");

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  calculate_IMU_error();
  delay(20);
}

void loop() 
{
  calculateAngles();
  roll = KALMAN(roll);
  BTSerial.print("roll: ");BTSerial.println(roll);
  
  // Desired angle to be 0 is Roll
  if (roll > 0.0)
  {
    // Set motors to go backward for negative angle
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);  
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else
  {
    // Set motors to go forwards for positive angle
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);  
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  
  motorSpeed = controllerPID(0.0, roll);
  if (motorSpeed < 0)
  {
    motorSpeed *= -1;
  }
  motorSpeed = constrain(motorSpeed, 0, 255);
  BTSerial.print("motorSpeed: ");BTSerial.println(motorSpeed);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
//  Serial.print(roll); Serial.print(" degrees "); Serial.print(motorSpeed); Serial.println(" /255");
  delay(20); // Wait
}

void calculateAngles()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // ~(-1.58)
  
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  
  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
//  GyroY = GyroY - GyroErrorY;
//  GyroZ = GyroZ - GyroErrorZ;
  
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
//  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
//  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = gyroWeight * gyroAngleX + (1.0 - gyroWeight) * accAngleX;
//  pitch = gyroWeight * gyroAngleY + (1.0 - gyroWeight)* accAngleY;
}

int controllerPID(float ref, float actual)
{
  unsigned long nowTime = millis();
  dt = double(nowTime - lastTime)/1000.00;
  lastTime = nowTime;
  
  error = double(ref - actual);
  
  proportional = error;
  integral += error*dt;
  derivative = (3.0*error - 4.0*lastError + beforeLastError) / (2.0*dt);
  
  int sumPID = int((Kp*proportional) + (Ki*integral) + (Kd*derivative));
  beforeLastError = lastError;
  lastError = error;
  
  return sumPID;
}

float KALMAN(float U)
{
  static const double R=40.0; // Noise covariance
  static const double H=1.00;
  static double Q=10.0; // Estimated covariance
  static double P=0.0; // Error covariance
  static double U_hat=0.0; // Initial estimated state
  static double K=0.0; // Initial kalman gain

  K=P*H/(H*P*H+R);
  U_hat=U_hat+K*(double(U)-H*U_hat);

  P=(1-K*H)*P+Q;

  return float(U_hat);   
}

void calculate_IMU_error()
{
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
//  Serial.print("AccErrorX: ");
//  Serial.println(AccErrorX);
//  Serial.print("AccErrorY: ");
//  Serial.println(AccErrorY);
//  Serial.print("GyroErrorX: ");
//  Serial.println(GyroErrorX);
//  Serial.print("GyroErrorY: ");
//  Serial.println(GyroErrorY);
//  Serial.print("GyroErrorZ: ");
//  Serial.println(GyroErrorZ);
}
