#include <Servo.h>  // Include the Servo library

// Create Servo objects for each motor
Servo motor1;
Servo motor2;
Servo motor3;

// Define the servo motor pins
int motor1Pin = 8;  // Motor 1 connected to pin 9
int motor2Pin = 9; // Motor 2 connected to pin 10
int motor3Pin = 10; // Motor 3 connected to pin 11

void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Attach the servos to the correct pins
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);

  // Initial motor positions (home position)
  motor1.write(90);  // Motor 1 at 90 degrees
  motor2.write(90);  // Motor 2 at 90 degrees
  motor3.write(107);  // Motor 3 at 90 degrees

  // Send an initialization message back to MATLAB
  Serial.println("Arduino Ready");
}

void loop() {
  // Check if data is available to read from MATLAB
  if (Serial.available() > 0) {
    // Read the incoming command (example: J1:90, J2:45, J3:60)
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove any extra spaces or newline characters

    // Process the command
    int motorNum = -1;  // Default to invalid
    int angle = -1;     // Default to invalid angle

    // Parse the command in the format "J1:angle"
    if (command.startsWith("J1:")) {
      motorNum = 1;
      angle = command.substring(3).toInt();
    } else if (command.startsWith("J2:")) {
      motorNum = 2;
      angle = command.substring(3).toInt();
    } else if (command.startsWith("J3:")) {
      motorNum = 3;
      angle = command.substring(3).toInt();
    }

    // If valid motor and angle are parsed, rotate the motor
    if (motorNum != -1 && angle != -1) {
      // Ensure the angle is within valid range (0-180 for servo motors)
      angle = constrain(angle, 0, 180);

      // Rotate the correct motor
      if (motorNum == 1) {
        motor1.write(angle);
      } else if (motorNum == 2) {
        motor2.write(angle);
      } else if (motorNum == 3) {
        motor3.write(angle);
      }

      // Send a confirmation message back to MATLAB
      String response = "Motor " + String(motorNum) + " set to " + String(angle) + " degrees";
      Serial.println(response);
    }
  }

  // Add a small delay to prevent excessive processing
  delay(10);
}
