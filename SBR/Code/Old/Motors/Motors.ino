// Pin Definitions
#define ENA 6 // PWM control for motor 1
#define IN1 8 // Direction control for motor 1
#define IN2 7 // Direction control for motor 1
int motorSpeed = 128;

void setup()
{
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(A0, INPUT);

  Serial.begin(9600);
}

void loop()
{
//  // Read analog value from A0
//  int pot = analogRead(A0); // Read analog value (0-1023)
//  
//  // Map analog value to PWM range (0-255)
//  int motorSpeed = map(pot, 0, 1023, 0, 255);
//
//  Serial.println(pot);
//  Serial.println(motorSpeed);
//  

  motorSpeed = 255;
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);    
  analogWrite(ENA, motorSpeed); // Adjust speed as needed

  delay(2000);

  motorSpeed = 70;
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  
  analogWrite(ENA, motorSpeed); // Adjust speed as needed
  
  delay(2000); // Wait for 2 seconds
}

// When angle is negative
//digitalWrite(IN1, HIGH);
//  digitalWrite(IN2, LOW);
//  analogWrite(ENA, motorSpeed); // Adjust speed as needed
//  
//  digitalWrite(IN3, LOW);
//  digitalWrite(IN4, HIGH);
