/**
 * @brief    Multi-reader RFID interface for Smart Cues
 * @author   Smart Cues Group
 * @note     Uses four MFRC522 RFID readers, an I2C LCD, LED, and buzzer feedback.
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <MFRC522.h>

// LCD configuration: I2C address 0x27, 16 columns, 2 rows
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pin definitions
#define RST_PIN     9
#define SS_1_PIN    8
#define SS_2_PIN    10
#define SS_3_PIN    7
#define SS_4_PIN    6
#define LED_PIN     2
#define BUZZER_PIN  3

#define NR_OF_READERS 4

byte ssPins[] = {SS_1_PIN, SS_2_PIN, SS_3_PIN, SS_4_PIN};

// Different buzzer frequency for each reader
const int tones[4] = {1000, 1200, 1400, 1600};

// RFID reader instances
MFRC522 mfrc522[NR_OF_READERS];

// Predefined tag UIDs
byte tag1UID[4] = {0xE6, 0xA0, 0x0B, 0x01};
byte tag2UID[4] = {0xD6, 0x16, 0x12, 0x02};

// Compare two 4-byte UIDs
bool isSameUID(byte *uid1, byte *uid2) {
    for (byte i = 0; i < 4; i++) {
        if (uid1[i] != uid2[i]) {
            return false;
        }
    }
    return true;
}

void setup() {
    Serial.begin(9600);
    while (!Serial);

    SPI.begin();

    // Initialize all RFID readers
    for (uint8_t i = 0; i < NR_OF_READERS; i++) {
        mfrc522[i].PCD_Init(ssPins[i], RST_PIN);
    }

    // Initialize LCD
    lcd.init();
    lcd.backlight();
    showWelcome();

    // Initialize feedback outputs
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
    for (uint8_t reader = 0; reader < NR_OF_READERS; reader++) {

        // Check whether a new RFID card is present
        if (mfrc522[reader].PICC_IsNewCardPresent() &&
            mfrc522[reader].PICC_ReadCardSerial()) {

            // LED feedback
            digitalWrite(LED_PIN, HIGH);
            delay(150);
            digitalWrite(LED_PIN, LOW);

            // Buzzer feedback using a reader-specific tone
            tone(BUZZER_PIN, tones[reader], 150);
            delay(150);
            noTone(BUZZER_PIN);

            // Identify the detected tag
            const char* tagLabel = "Unknown";

            if (isSameUID(mfrc522[reader].uid.uidByte, tag1UID)) {
                tagLabel = "Tag 1";
            } else if (isSameUID(mfrc522[reader].uid.uidByte, tag2UID)) {
                tagLabel = "Tag 2";
            }

            // Display reader number and tag identity
            lcd.clear();

            lcd.setCursor(0, 0);
            lcd.print("Reader: ");
            lcd.print(reader);

            lcd.setCursor(0, 1);
            lcd.print(tagLabel);

            delay(2000);

            showWelcome();

            // Stop communication with the current card
            mfrc522[reader].PICC_HaltA();
            mfrc522[reader].PCD_StopCrypto1();
        }
    }
}

// Display the default ready screen
void showWelcome() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("RFID Ready");
}
