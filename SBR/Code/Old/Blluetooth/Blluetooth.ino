#include <SoftwareSerial.h>
#include <String.h>

float kp=0.0, ki=0.0, kd=0.0;
bool valuesReceived = false; // Flag to indicate if values have been received

const byte rxPin = 12;
const byte txPin = 11;
SoftwareSerial BTSerial(rxPin, txPin); // RX TX

void setup()
{
  // Define pin modes for tx, rx:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(13, OUTPUT);
  
  BTSerial.begin(9600);
  Serial.begin(9600);

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
          kp = atof(token+3);
        else if (strstr(token, "ki=") != NULL)
          ki = atof(token+3);
        else if (strstr(token, "kd=") != NULL)
          kd = atof(token+3);
                   
        token = strtok(NULL, ":"); // Get next token
      }
      valuesReceived = true; // Set flag to indicate values have been received
      message = ""; // Clear message 
    }
  }

  BTSerial.print("Kp=");BTSerial.print(kp);BTSerial.print(":)Ki=");BTSerial.print(ki);
  BTSerial.print(":)Kd=");BTSerial.println(kd);
}

String messageBuffer = "";
String message = "";

void loop()
{
  static int ledState=0;
//  while (BTSerial.available() > 0)
//  {
//    char data = (char) BTSerial.read();
//    messageBuffer += data;
//    if (data == ';')
//    {
//      message = messageBuffer;
//      messageBuffer = "";
//      Serial.print(message); // send to serial monitor
//      message = "You sent " + message;
//      BTSerial.print(message); // send back to bluetooth terminal
//    }
//  }
  Serial.println("Inside of the loop.");
  Serial.print("kp: ");
  Serial.println(kp);
  Serial.print("ki: ");
  Serial.println(ki);
  Serial.print("kd: ");
  Serial.println(kd);
  
  digitalWrite(13, ledState);
  ledState=!ledState;
  
  delay(2000);
}
