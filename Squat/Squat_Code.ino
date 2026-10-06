/**
 * @brief    Ultrasonic squat detector with BLE for Smart Cues
 * @author   Smart Cues Group
 * @note     Detects squat state using ultrasonic distance and sends state changes over BLE.
 */

#include <ArduinoBLE.h>

const int trigPin = A1;
const int echoPin = A2;

const float squatThreshold = 50.0;  // Distance threshold in centimetres

// Squat state management
bool squatting = false;
bool pendingState = false;
unsigned long stateChangeStart = 0;
const unsigned long stableTime = 1000;  // Required stable time in milliseconds

// BLE configuration
BLEService squService("19B10000-E8F2-537E-4F6C-D104768A1215");

BLEIntCharacteristic squChar(
    "19B10001-E8F2-537E-4F6C-D104768A1215",
    BLERead | BLENotify
);

// Ultrasonic echo timing
volatile unsigned long echoStart = 0;
volatile unsigned long echoEnd = 0;
volatile bool newDistanceReady = false;

unsigned long lastTriggerTime = 0;
const unsigned long measureInterval = 100;

void setup() {
    Serial.begin(115200);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Capture ultrasonic echo timing using an interrupt
    attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);

    // Initialize BLE
    if (!BLE.begin()) {
        Serial.println("BLE initialization failed");
        while (1);
    }

    BLE.setLocalName("Squ_Sender");
    BLE.setAdvertisedService(squService);

    squService.addCharacteristic(squChar);
    BLE.addService(squService);

    squChar.writeValue(0);
    BLE.advertise();

    Serial.println("Squ_Sender ready");
    Serial.print("BLE MAC Address: ");
    Serial.println(BLE.address());
}

void loop() {
    BLE.poll();

    BLEDevice central = BLE.central();
    unsigned long now = millis();

    // Trigger a new ultrasonic measurement at a fixed interval
    if (now - lastTriggerTime >= measureInterval) {
        lastTriggerTime = now;
        triggerUltrasonic();
    }

    // Process a completed ultrasonic measurement
    if (newDistanceReady) {
        newDistanceReady = false;

        unsigned long duration = echoEnd - echoStart;
        float distance = duration * 0.0343 / 2.0;

        // Ignore invalid distance readings
        if (distance > 0 && distance < 1000) {
            bool newSquatState = (distance < squatThreshold);

            // Start timing when a new candidate state is detected
            if (newSquatState != pendingState) {
                pendingState = newSquatState;
                stateChangeStart = millis();
            }

            // Confirm the state only after it remains stable
            if ((millis() - stateChangeStart) >= stableTime) {
                if (newSquatState != squatting) {
                    squatting = newSquatState;

                    if (central && central.connected()) {
                        squChar.writeValue(squatting ? 1 : 0);
                        Serial.println(squatting ? "Squat started" : "Squat ended");
                    }
                }
            }
        }
    }
}

// Trigger one ultrasonic measurement
void triggerUltrasonic() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(trigPin, LOW);
}

// Interrupt service routine for ultrasonic echo timing
void echoISR() {
    if (digitalRead(echoPin) == HIGH) {
        echoStart = micros();
    } else {
        echoEnd = micros();
        newDistanceReady = true;
    }
}
