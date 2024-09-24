# Led Clusters

1. Tem piscar a 1 Hz
2. \> 1 W
3. Green  when safe
4. Green Blink 1hz when engaged
5. Red when is shit

```c
#include <FlexCAN_T4.h>  // Library for CAN communication

// Define pins for LEDs
const int greenLED = 3;  // Safe
const int blinkingGreenLED = 4;  // Engaged
const int redLED = 5;  // Error

// Define state variables
enum State { SAFE, ENGAGED, ERROR };
State currentState = SAFE;
State lastSentState = SAFE;  // Keep track of the last sent state for CAN

// Timing for blinking (1Hz = 1000ms period)
unsigned long lastBlinkTime = 0;
bool blinkState = false;
const unsigned long blinkInterval = 1000;

// CAN bus setup
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;  // Initialize CAN1 for Teensy 3.6

// Function declarations
void sendCANMessage(State state);

void setup() {
  // Initialize LED pins as outputs
  pinMode(greenLED, OUTPUT);
  pinMode(blinkingGreenLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  // Start with all LEDs off
  digitalWrite(greenLED, LOW);
  digitalWrite(blinkingGreenLED, LOW);
  digitalWrite(redLED, LOW);

  // Initialize CAN bus with 500 kbps speed
  Can1.begin();
  Can1.setBaudRate(500000);  // 500kbps

  Serial.begin(115200);  // Optional, for debugging
}

void loop() {
  // Update the state machine
  updateState();

  // Handle LED output based on the current state
  handleLEDs();

  // Send CAN message if the state has changed
  if (currentState != lastSentState) {
    sendCANMessage(currentState);
    lastSentState = currentState;  // Update last sent state
  }
}

// Function to update the current state (for testing, you can simulate it here)
void updateState() {
  // In real-world cases, this function should read inputs such as relays, contactors, and IMD relay statuses
  // For now, let's simulate state changes manually.
  
  // Example: Switch states every 5 seconds
  unsigned long currentTime = millis();
  
  if (currentTime > 5000 && currentTime <= 10000) {
    currentState = ENGAGED;
  } else if (currentTime > 10000 && currentTime <= 15000) {
    currentState = ERROR;
  } else {
    currentState = SAFE;
  }
}

// Function to handle the LED states
void handleLEDs() {
  switch (currentState) {
    case SAFE:
      // Green LED ON, others OFF
      digitalWrite(greenLED, HIGH);
      digitalWrite(blinkingGreenLED, LOW);
      digitalWrite(redLED, LOW);
      break;
      
    case ENGAGED:
      // Blink Green LED (1Hz)
      unsigned long currentMillis = millis();
      if (currentMillis - lastBlinkTime >= blinkInterval) {
        lastBlinkTime = currentMillis;
        blinkState = !blinkState;  // Toggle the blink state
        digitalWrite(blinkingGreenLED, blinkState ? HIGH : LOW);
      }
      // Turn off other LEDs
      digitalWrite(greenLED, LOW);
      digitalWrite(redLED, LOW);
      break;
      
    case ERROR:
      // Red LED ON, others OFF
      digitalWrite(redLED, HIGH);
      digitalWrite(greenLED, LOW);
      digitalWrite(blinkingGreenLED, LOW);
      break;
  }
}

// Function to send CAN message with the current state
void sendCANMessage(State state) {
  CAN_message_t msg;
  msg.id = 0x100;  // CAN message ID (you can modify this ID)
  msg.len = 1;     // Message length (1 byte)

  // Assign state to the CAN message data
  msg.buf[0] = (uint8_t)state;

  // Send the message
  Can1.write(msg);

  // For debugging (optional)
  Serial.print("Sent CAN message with state: ");
  switch (state) {
    case SAFE:
      Serial.println("SAFE");
      break;
    case ENGAGED:
      Serial.println("ENGAGED");
      break;
    case ERROR:
      Serial.println("ERROR");
      break;
  }
}
```

Breakdown of Changes:
CAN Bus Initialization:

The CAN bus is initialized in the setup() function with a baud rate of 500 kbps.
We use FlexCAN_T4 for handling CAN communication.
Sending CAN Messages:

The sendCANMessage(State state) function creates a CAN message and sends the state as a single byte.
The CAN_message_t structure is used to define the message, and the state is sent in the first byte (msg.buf[0]).
The state is only sent when it changes (compared to lastSentState), which reduces unnecessary CAN traffic.
Message Content:

The CAN message has an ID of 0x100 (you can change this if needed).
The data length is 1 byte, where the state (SAFE, ENGAGED, or ERROR) is sent.
Debugging:

The script includes Serial.print statements to help debug and view the state changes and CAN messages in the Arduino Serial Monitor.
State-to-CAN Mapping:
SAFE → 0
ENGAGED → 1
ERROR → 2
Requirements:
Teensy 3.6 with CAN transceiver hardware to connect the Teensy to a physical CAN bus.
FlexCAN_T4 library for CAN communication.
Let me know if you need further changes or explanations!

To Do:

- [ ] Define Power Delivery
- [ ] See what uC is available and cheap
- [ ] RGB - BiColor - or 2 LED
