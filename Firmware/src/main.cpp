#include <Arduino.h>

// Pin assignments (adjust as needed)
/*
  INPUT A - WHITE - DIRECTION
  INPUT B - BLACK - STEP
  ENABLE  - BLUE -
*/
#define STEP 11    // PWM-capable pin for step signal - BLACK - 11
#define DIR  10    // Simple digital pin for direction - WHITE - 10
#define ENABLE 9 // Enabled high - BLUE - 9
#define TRIGGER 5 // Green
#define AZI_MOVE_DONE 2
#define LED LED_BUILTIN

// Control parameters
#define AZI_DELAY 100
#define TRIGGERING_DELAY 75

int i;
volatile int receivedNumber = 0;  // Global variable for the received number
volatile int direction = 0;
char inputBuffer[6];              // Buffer to store incoming number (-800 to 800, max 5 chars + null)

unsigned long previousMillis = 0; // Stores the last time the action was performed
const unsigned long interval = 100; // Interval (0.1 second)

void stepMotor(unsigned long steps, 
                     bool direction)
{
  // Set the direction pin
  digitalWrite(DIR, direction ? HIGH : LOW);

  for (int i = 0; i < steps; i++)
  //while(true)
  {
    digitalWrite(STEP, HIGH);
    delayMicroseconds(AZI_DELAY);
    digitalWrite(STEP, LOW);
    delayMicroseconds(AZI_DELAY);
  }
  //digitalWrite(LED, LOW);
  //digitalWrite(ENABLE, LOW);
}

void move_done()
{

  Serial.write("1");
  
}

void setup()
{
  Serial.begin(115200);  // High baud rate for fast transmission

  // Set up the direction pin
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(TRIGGER, OUTPUT);
  pinMode(AZI_MOVE_DONE, INPUT_PULLUP);

  digitalWrite(ENABLE, HIGH);
  digitalWrite(LED, HIGH);
  digitalWrite(STEP, LOW);
  //stepMotor(400,true);
  attachInterrupt(digitalPinToInterrupt(AZI_MOVE_DONE), move_done, FALLING);

}

void loop()
{
  digitalWrite(TRIGGER, LOW);
  unsigned long currentMillis = millis(); // Get the current time
  while (Serial.available()) {
      char c = Serial.read();  // Read a character
      Serial.write(c);

      if (c == '\n') {  // End of input, process the number
        inputBuffer[i] = '\0';  // Null-terminate the string
          int num = atoi(inputBuffer);  // Convert to integer
          // Movement Command
          if (num >= -800 && num <= 800) {  // Validate range
              receivedNumber = abs(num);  // Update global variable
              if (num > 0)
              {
                direction = 0;
              }
              else 
              {
                direction = 1;
              }
              digitalWrite(LED, HIGH);
              stepMotor(receivedNumber, direction);
          }

          i = 0;  // Reset index for next input
          
      } 
      else if (c == 'd' || c == 'D')
      {
        digitalWrite(ENABLE, LOW);
      }
      else if (c == 'e' || c == 'E')
      {
        digitalWrite(ENABLE, HIGH);
      }
      else if (c == 't' || c == 'T')
      {
        digitalWrite(TRIGGER, HIGH);
        delay(TRIGGERING_DELAY);
        digitalWrite(TRIGGER, LOW);
      }
      else if (i < sizeof(inputBuffer) - 1) {  // Store character if within buffer size
        inputBuffer[i] = c;
        i++;
      }
  }

  // 0.1 second interrupt
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis; // Update the last action time

  }
}
