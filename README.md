## 🔌 Latching Power Switch Circuit with Auto Power OFF/ON
### 📋 Overview
This Arduino-based project implements a `latching power switch` using a pushbutton and a MOSFET transistor, with `automatic power-off` after a set duration and `automatic power-on` after a pause. An LED connected via PWM represents partial brightness control.

### ⚙️ Features
- Press the button to `turn on` the power (via MOSFET).
- Device `stays on for 10 seconds`, then automatically powers off.
- After staying off for `5 seconds`, it automatically powers on again.
- `PWM LED` shows activity at 25% brightness when power is on.
- `Serial monitor` logs the power state changes.

### 🧠 Code
```cpp
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
```

### 💡 Applications
- `DIY power timers`  
- `Energy-saving circuits`  
- `Interactive LED displays`  
- `Auto-resetting test benches`  
- `Low-power embedded projects that need periodic activity`
--------------------
  ## 🧠 Digital and Analog Sensor Reading with Arduino

### 📋 Overview
This project demonstrates how to read input from both `digital` and `analog` sensors using an Arduino. It helps beginners understand the difference between reading a button (digital signal) and a sensor like a potentiometer or LDR (analog signal).

---

### ⚙️ Features
- Reads button state using `digitalRead()`.
- Reads analog sensor values using `analogRead()`.
- Outputs values to the Serial Monitor for real-time observation.
- Great for learning input handling on Arduino.

---

### 🔌 Digital Input Example (Button)

```cpp
// C++ code
/*
  DigitalReadSerial
  Reads a digital input on pin 2,
  prints the result to the serial monitor
*/

int buttonState = 0;

void setup() {
  pinMode(2, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Read the input pin
  buttonState = digitalRead(2);

  // Print out the state of the button
  Serial.println(buttonState);

  delay(10); // Small delay to improve simulation performance
}
```

🧪 **Expected Output:**  
`0` when the button is not pressed, `1` when it is pressed.

---

### 🎛️ Analog Input Example (Sensor)

```cpp
// C++ code
/*
  AnalogReadSerial
  Reads an analog input on pin A0,
  prints the result to the serial monitor
*/

int sensorValue = 0;

void setup() {
  pinMode(A0, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Read the analog input
  sensorValue = analogRead(A0);

  // Print out the sensor value
  Serial.println(sensorValue);

  delay(10); // Small delay to improve simulation performance
}
```

🧪 **Expected Output:**  
Values from `0` to `1023` depending on the sensor input (e.g., potentiometer position or light intensity).

---

### 🧭 Comparison Table

| Feature           | Digital Input           | Analog Input              |
|-------------------|--------------------------|----------------------------|
| Function used     | `digitalRead()`          | `analogRead()`             |
| Value range       | `0 or 1` (LOW/HIGH)       | `0 – 1023`                 |
| Example device    | Push button               | Potentiometer / LDR       |
| Input pin         | Digital (e.g., pin 2)     | Analog (e.g., pin A0)     |

---

### 💡 Applications

- Digital: Detecting button presses, motion sensors, limit switches.
- Analog: Reading light, temperature, position, sound level, etc.

---

### 🛠️ Optional Enhancements

- Add an LED to turn on when the sensor exceeds a threshold.
- Plot sensor readings using the Arduino Serial Plotter.
- Log data for further analysis.
