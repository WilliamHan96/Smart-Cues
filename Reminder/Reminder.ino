/**
 * @brief    Smart Cues activity reminder controller
 * @author   Smart Cues Group
 * @note     Provides reminder logic and local feedback to prompt physical activity.
 */

const int pirPin = 2;
const int buttonPin = 4;
const int buzzerPin = 7;
const int ledSufficient = 8;
const int ledSome = 9;
const int ledNone = 10;

int exerciseCount = 0;

int lastButtonState = HIGH;
int lastPirState = LOW;

unsigned long buttonPressTime = 0;
bool buttonHeld = false;

const unsigned long longPressDuration = 2000;
const unsigned long debounceDelay = 50;

// ---------- Status Control ----------
bool statusTriggered = false;
int beepCount = 0;
int beepTotal = 0;
unsigned long lastBeepTime = 0;
bool buzzerOn = false;
int statusLED = -1;
unsigned long ledStartTime = 0;
const unsigned long ledDuration = 1000;
const unsigned long beepInterval = 400;

void setup() {
  lastButtonState = digitalRead(buttonPin);  // Prevent false trigger on boot
  pinMode(pirPin, INPUT);
  pinMode(buttonPin, INPUT);  // Use with external pull-down resistor
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledSufficient, OUTPUT);
  pinMode(ledSome, OUTPUT);
  pinMode(ledNone, OUTPUT);

  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledSufficient, LOW);
  digitalWrite(ledSome, LOW);
  digitalWrite(ledNone, LOW);

  Serial.begin(9600);
}

void loop() {
  unsigned long currentTime = millis();
  int reading = digitalRead(buttonPin);

  // Button press start
  if (lastButtonState == LOW && reading == HIGH) {
    buttonPressTime = currentTime;
    buttonHeld = false;
  }

  // Long press check
  if (reading == HIGH && !buttonHeld && (currentTime - buttonPressTime >= longPressDuration)) {
    buttonHeld = true;
  }

  // Button released
  if (lastButtonState == HIGH && reading == LOW) {
    if (buttonHeld && (currentTime - buttonPressTime >= longPressDuration)) {
      // Long press → reset
      digitalWrite(ledSufficient, LOW);
      digitalWrite(ledSome, LOW);
      digitalWrite(ledNone, LOW);
      exerciseCount = 0;
      Serial.println("🔄 Long press reset: exerciseCount = 0");

      beepTotal = 1;
      statusLED = ledNone;
      ledStartTime = currentTime;
      statusTriggered = true;
      beepCount = 0;
      buzzerOn = false;
      lastBeepTime = currentTime;
      digitalWrite(statusLED, HIGH);  // Turn on red LED manually
    } else if (!buttonHeld) {
      // Short press → +1, LED only
      exerciseCount++;
      Serial.print("Exercise count +1: ");
      Serial.println(exerciseCount);
      showLedOnly();  // No buzzer
    }
    buttonHeld = false;
  }

  lastButtonState = reading;

  // PIR sensor detects motion
  int pirValue = digitalRead(pirPin);
  if (pirValue == HIGH && lastPirState == LOW) {
    Serial.println("👣 PIR: Detected person");
    prepareStatusFeedback();
  }
  lastPirState = pirValue;

  // LED & buzzer updates
  updateBuzzerAndLED();
}

// ---------- Feedback on PIR or long press ----------
void prepareStatusFeedback() {
  statusTriggered = true;
  beepCount = 0;
  lastBeepTime = millis();
  buzzerOn = false;

  if (exerciseCount == 0) {
    beepTotal = 3;
    statusLED = ledNone;
  } else if (exerciseCount == 1) {
    beepTotal = 2;
    statusLED = ledSome;
  } else {
    beepTotal = 0;
    statusLED = ledSufficient;
  }

  if (statusLED != -1) {
    digitalWrite(statusLED, HIGH);
    ledStartTime = millis();
  }
}

// ---------- LED-only feedback (for button click) ----------
void showLedOnly() {
  if (exerciseCount == 0) {
    statusLED = ledNone;
  } else if (exerciseCount == 1) {
    statusLED = ledSome;
  } else {
    statusLED = ledSufficient;
  }

  if (statusLED != -1) {
    digitalWrite(statusLED, HIGH);
    ledStartTime = millis();
    statusTriggered = true;
    beepTotal = 0;          // no beeps
    beepCount = 0;
    buzzerOn = false;
  }
}

// ---------- Non-blocking buzzer and LED controller ----------
void updateBuzzerAndLED() {
  if (!statusTriggered) return;

  unsigned long currentTime = millis();

  // LED auto off
  if (statusLED != -1 && currentTime - ledStartTime >= ledDuration) {
    digitalWrite(statusLED, LOW);
    statusLED = -1;
  }

  // Buzzer beeping
  if (beepCount < beepTotal) {
    if (currentTime - lastBeepTime >= beepInterval) {
      lastBeepTime = currentTime;

      if (buzzerOn) {
        digitalWrite(buzzerPin, LOW);
        buzzerOn = false;
        beepCount++;
      } else {
        digitalWrite(buzzerPin, HIGH);
        buzzerOn = true;
      }
    }
  }

  // Done with LED & buzzer
  if (beepCount >= beepTotal && statusLED == -1) {
    statusTriggered = false;
  }
  if (beepTotal == 0 && statusLED == -1) {
    statusTriggered = false;
  }
}
