# power switch circuit with auto power off and on.

📋 Overview

This Arduino-based project implements a latching power switch using a pushbutton and a MOSFET transistor, with automatic power-off after a set duration and automatic power-on after a pause. An LED connected via PWM represents partial brightness control.

🧠 Code Explanation
⸻

⚙️ Features
	•	Press the button to turn on the power (via MOSFET).
	•	Device stays on for 10 seconds, then automatically powers off.
	•	After staying off for 5 seconds, it automatically powers on again.
	•	PWM LED shows activity at 25% brightness when power is on.
	•	Serial monitor logs the power state changes.

 const int buttonPin = 2;    // Pushbutton on digital pin 2
const int latchPin = 5;     // Transistor base for MOSFET control
const int pwmPin = 9;       // PWM pin for LED brightness

const unsigned long onDuration = 10000;  // 10 seconds ON
const unsigned long offDuration = 5000;  // 5 seconds OFF

unsigned long lastButtonPress = 0;
bool isPowered = false;

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  digitalWrite(latchPin, LOW);  // Start with power off
  analogWrite(pwmPin, 0);       // LED off
  Serial.begin(9600);
}

void loop() {
  int buttonState = digitalRead(buttonPin);

  // Manual power-on via button
  if (buttonState == HIGH && !isPowered) {
    isPowered = true;
    lastButtonPress = millis();
    digitalWrite(latchPin, HIGH); // Turn on MOSFET
    analogWrite(pwmPin, 64);      // 25% LED brightness
    Serial.println("Power ON");
  }

  // Auto power-off after onDuration
  if (isPowered && millis() - lastButtonPress >= onDuration) {
    digitalWrite(latchPin, LOW);  // Turn off MOSFET
    analogWrite(pwmPin, 0);       // Turn off LED
    isPowered = false;
    lastButtonPress = millis();
    Serial.println("Power OFF");
  }

  // Auto power-on after offDuration
  if (!isPowered && millis() - lastButtonPress >= offDuration) {
    isPowered = true;
    lastButtonPress = millis();
    digitalWrite(latchPin, HIGH); // Turn on MOSFET
    analogWrite(pwmPin, 64);      // 25% LED brightness
    Serial.println("Auto Power ON");
  }
  }
  ------
  🔌 Applications
	•	DIY power timers
	•	Energy-saving circuits
	•	Interactive LED displays
	•	Auto-resetting test benches

