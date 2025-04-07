/*
 * Project - Interactive Lightsaber System

  This project creates a reactive lightsaber experience, combining audio feedback, motion detection, proximity sensing, and dynamic lighting effects.
  The lightsaber responds to movement, proximity, and clashes with sound and light effects.

  The program continuously monitors the accelerometer and ultrasonic sensor for movement and proximity changes.
  Based on the sensor input, it triggers corresponding sound effects and animates the LED strip to simulate motion and clash effects. 
  MP3 playback is managed according to sensor inputs.

  Features:
  - **MP3 Playback**: Plays various sounds for activation, motion, and clashes.
  - **Dynamic Lighting**: A NeoPixel LED strip provides glowing blade effects that react to user interactions.
  - **Motion Detection**: Detects g-force changes using an accelerometer/gyroscope for motion-based sound triggers.
  - **Clash Detection**: Uses an ultrasonic sensor to identify clashes and triggers corresponding effects.

  Circuit:
  * INPUTS:
  *     MPU6050 accelerometer/gyroscope
  *     Ultrasonic proximity sensor
  * OUTPUTS:
  *     NeoPixel LED strip for lightsaber blade effects
  *     Speaker for MP3 playback

  Video link: https://youtu.be/Zf5vSd0jumg?si=SsjPrTeG5K9Dqi3n
  Created By:
  Theodore_Dai_Maman #211541594
  Omer_Dan #322952466
  Amit_Kaminsky #207487661
*/

#include <WiFi.h>
#include "SPIFFS.h"
#include "AudioFileSourceSPIFFS.h"
#include "AudioFileSourceID3.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2S.h"
#include <Adafruit_NeoPixel.h>

#include "I2Cdev.h"
#include "MPU6050.h"

// Initialize the MPU6050 accelerometer/gyroscope sensor
MPU6050 accelgyro;

// Variables for accelerometer and gyroscope readings
int16_t ax, ay, az;
int16_t gx, gy, gz;
float baselineAx = 0.0, baselineAy = 0.0, baselineAz = 0.0;

// Constants for movement detection
const int MOVEMENT_DEBOUNCE = 3000; // Minimum time between movement triggers (ms)
const float THRESHOLD = 1.2;        // Movement threshold in g-force
const int ACCEL_SAMPLE_RATE = 50;  // Sampling rate for accelerometer (ms)

#define OUTPUT_READABLE_ACCELGYRO

// Proximity sensor setup
#define TRIGGER_PIN  16              // Trigger pin for ultrasonic sensor
#define ECHO_PIN     17              // Echo pin for ultrasonic sensor
#define MAX_DISTANCE 60              // Maximum distance for sensor in cm
int currentDistance = 0;             // Variable to store proximity reading

// Audio objects for MP3 playback
AudioGeneratorMP3 *mp3;
AudioOutputI2S *out;
AudioFileSourceSPIFFS *currentSource = nullptr;
AudioFileSourceID3 *currentID3 = nullptr;

// Flags for tracking current playback
boolean playingHumming = false;
boolean playingStart = false;
boolean playingMoveLong = false;

// Timers for debouncing and periodic tasks
unsigned long lastAccelRead = 0;
unsigned long lastMovementTime = 0;
unsigned long lastClashTime = 0;

// NeoPixel LED configuration
#define LED_PIN 15
#define LED_COUNT 24
boolean lightsOn = false;
int currentLight = 0;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Function to release audio resources
void cleanupAudio() {
    Serial.println("DEBUG: Cleaning up audio resources");
    if (mp3) {
        Serial.println("DEBUG: Stopping MP3");
    }
    if (currentID3) {
        Serial.println("DEBUG: Deleting ID3");
        delete currentID3;
        currentID3 = nullptr;
    }
    if (currentSource) {
        Serial.println("DEBUG: Deleting audio source");
        delete currentSource;
        currentSource = nullptr;
    }
    Serial.println("DEBUG: Audio cleanup complete");
}

// Function to switch to a specific MP3 file
boolean switchToSound(const char* filename) {
    Serial.printf("DEBUG: Switching to sound file: %s\n", filename);
    cleanupAudio();
    
    Serial.println("DEBUG: Creating new audio source");
    currentSource = new AudioFileSourceSPIFFS(filename);
    if (!currentSource->isOpen()) {
        Serial.printf("ERROR: Failed to open %s\n", filename);
        return false;
    }
    
    Serial.println("DEBUG: Creating ID3 decoder");
    currentID3 = new AudioFileSourceID3(currentSource);
    Serial.println("DEBUG: Starting MP3 playback");
    boolean result = mp3->begin(currentID3, out);
    Serial.printf("DEBUG: MP3 begin result: %d\n", result);
    return result;
}

// Function to calculate g-force from accelerometer readings
float checkForMovement(int16_t ax, int16_t ay, int16_t az) {
    // Convert raw accelerometer values to "g" values
    float gX = ax / 16384.0;
    float gY = ay / 16384.0;
    float gZ = az / 16384.0;

    // Calculate the resultant g-force magnitude
    float gForce = sqrt(gX * gX + gY * gY + gZ * gZ);

    // Debugging
    Serial.printf("DEBUG: g-force magnitude: %.2f\n", gForce);

    return abs(gForce);
}

// Function to set all NeoPixel LEDs to a specified color
void colorWipe(uint32_t color) {
    for (int i = 0; i < strip.numPixels(); i++) { // Iterate through all LEDs
        strip.setPixelColor(i, color);         // Set each LED to the specified color
        strip.show();                          // Apply the color changes
    }
}

// Function to measure distance using the ultrasonic sensor
int ultrasonic_measure(int trigPin, int echoPin, int max_distance) {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(trigPin, LOW);
    int duration = pulseIn(echoPin, HIGH, max_distance * 59);
    return duration / 59; // Convert duration to distance in cm
}

// Setup function to initialize peripherals and baseline data
void setup() {
    // Initialize I2C communication and accelerometer
    Wire.begin(21, 19);
    Serial.begin(115200);
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // Initialize SPIFFS for MP3 playback
    WiFi.mode(WIFI_OFF);
    delay(1000);
    SPIFFS.begin();
    Serial.printf("Sample MP3 playback begins...\n");

    audioLogger = &Serial;
    out = new AudioOutputI2S();
    mp3 = new AudioGeneratorMP3();

    // Calculate accelerometer baseline
    Serial.println("DEBUG: Calculating accelerometer baseline");
    long sumAx = 0, sumAy = 0, sumAz = 0;
    for (int i = 0; i < 50; i++) {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumAx += ax; sumAy += ay; sumAz += az;
        Serial.printf("DEBUG: Baseline sample %d - X: %d, Y: %d, Z: %d\n", i, ax, ay, az);
        delay(5);
    }
    baselineAx = sumAx / 50.0;
    baselineAy = sumAy / 50.0;
    baselineAz = sumAz / 50.0;
    Serial.printf("DEBUG: Baseline values - X: %.2f, Y: %.2f, Z: %.2f\n", baselineAx, baselineAy, baselineAz);

    // Start the system with the initial sound
    Serial.println("DEBUG: Starting initial start sound");
    if (!switchToSound("/start.mp3")) {
        Serial.println("ERROR: Failed to start start sound!");
        while (1) {
            Serial.println("ERROR: System halted - Audio failure");
            delay(1000);
        }
    }
    playingStart = true;

    // Initialize NeoPixel LEDs
    strip.begin();
    strip.show();
    strip.setBrightness(50); // Set LED brightness
}

// Main loop function to handle audio, sensors, and LEDs
void loop() {
    static int loopCount = 0;
    unsigned long currentMillis = millis();
    
    // Handle MP3 playback
    if (mp3->isRunning()) {
        if (currentLight < 2 * LED_COUNT) {
            strip.setPixelColor(currentLight / 2, strip.Color(0, 255, 0)); // Green
            strip.show();
            currentLight++;
        }
        if (!mp3->loop()) {
            mp3->stop();
            Serial.println("                              Switching back to humming");
            switchToSound("/humming.mp3");
        }
    } else {
        Serial.printf("MP3 done\n");
        delay(1000);
    }
    
    // Read accelerometer and proximity sensor at intervals
    if (currentMillis - lastAccelRead >= ACCEL_SAMPLE_RATE) {
        // Proximity reading
        currentDistance = ultrasonic_measure(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
        
        // Accelerometer reading
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);

        lastAccelRead = currentMillis;
        float currentGFORCE = checkForMovement(ax, ay, az);
        Serial.println(currentGFORCE);

        // Check for movement with debouncing
        if (currentGFORCE > THRESHOLD && (currentMillis - lastMovementTime >= MOVEMENT_DEBOUNCE)) {
            Serial.println("DEBUG: Movement detected!");
            lastMovementTime = currentMillis;
            
            if (currentGFORCE > THRESHOLD + 0.5) {
                switchToSound("/move_short.mp3");
                Serial.println("DEBUG: Successfully switched to long sound");
            } else if (currentGFORCE > THRESHOLD) {
                switchToSound("/move_long.mp3");
                Serial.println("DEBUG: Successfully switched to short sound");
            }
        } else if ((currentDistance > 3 && currentDistance < 8) && (currentMillis - lastClashTime >= MOVEMENT_DEBOUNCE)) {
            Serial.println("DEBUG: Clash detected!");
            lastClashTime = currentMillis;
            if (switchToSound("/clash.mp3")) {
                Serial.println("DEBUG: Successfully switched to clash sound");
                colorWipe(strip.Color(255, 255, 255)); // White
                colorWipe(strip.Color(255, 0, 0));     // Red
                colorWipe(strip.Color(255, 255, 255)); // White
                colorWipe(strip.Color(0, 255, 0));     // Green
            }
        }
    }
    // Prevent system overload with a small delay
    delay(10);
}
