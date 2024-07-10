#define BLYNK_TEMPLATE_ID "TMPL6hJghmPfW"
#define BLYNK_TEMPLATE_NAME "Iot Health monitor"
#define BLYNK_AUTH_TOKEN "a7zeYohDYYx1yNDtwv1Lebbsk0fcAfjZ"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "DHT.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define DHTPIN 13 // Pin which is connected to the DHT sensor
#define DHTTYPE DHT22 // DHT 22 (AM2302)

#define RELAY_PIN 12 // GPIO pin connected to the relay module

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Redmi10";
char pass[] = "shami1234";

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MAX30105 particleSensor;
DHT dht(DHTPIN, DHTTYPE);

// Calculate heart rate variables
const byte RATE_SIZE = 4; // Number of samples for heart rate averaging
byte rates[RATE_SIZE]; // Heart rate array
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
float beatAvg;

// Calculate SpO2 variables
double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
double SpO2 = 0;
double ESpO2 = 90.0; // Initial value
double FSpO2 = 0.7; // Filter factor for estimated SpO2
double frate = 0.95; // Low pass filter for IR/red LED value to eliminate AC component
int i = 0;
int Num = 30; // Take 30 samples before calculating once
#define FINGER_ON 7000 // Minimum IR value to detect finger presence
#define MINIMUM_SPO2 90.0 // Minimum SpO2 value

unsigned long lastUpdate = 0;
unsigned long updateInterval = 100; // Update interval for graph movement (in milliseconds)
int graph[SCREEN_WIDTH];

BlynkTimer timer;

void setup() {
  Serial.begin(115200);
  Serial.println("System Start");

  pinMode(RELAY_PIN, OUTPUT); // Initialize relay pin as output
  digitalWrite(RELAY_PIN, LOW); // Ensure the relay is off initially

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("Could not find MAX30102 sensor");
    while (1);
  }

  byte ledBrightness = 0x7F;
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 800;
  int pulseWidth = 215;
  int adcRange = 16384;
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableDIETEMPRDY();

  particleSensor.setPulseAmplitudeRed(0x0A);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) ; // Don't proceed, loop forever
  }

  display.display();
  delay(2000); // Pause for 2 seconds

  display.clearDisplay();

  for (int i = 0; i < SCREEN_WIDTH; i++) {
    graph[i] = 0;
  }

  Blynk.begin(auth, ssid, pass);
  dht.begin();
  timer.setInterval(1000L, sendToBlynk); // Update Blynk every second
}

void loop() {
  Blynk.run(); // Run Blynk
  timer.run(); // Run Blynk timer

  long irValue = particleSensor.getIR();

  if (irValue > FINGER_ON) {
    if (checkForBeat(irValue) == true) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);
      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;

        // Trigger email alert if BPM exceeds 100
        if (beatAvg > 100) {
          Blynk.logEvent("bpm_alert", "Your BPM has exceeded 100!");
        }
      }
    }

    uint32_t ir, red;
    double fred, fir;
    particleSensor.check();
    if (particleSensor.available()) {
      i++;
      ir = particleSensor.getFIFOIR();
      red = particleSensor.getFIFORed();
      fir = (double)ir;
      fred = (double)red;
      aveir = aveir * frate + (double)ir * (1.0 - frate);
      avered = avered * frate + (double)red * (1.0 - frate);
      sumirrms += (fir - aveir) * (fir - aveir);
      sumredrms += (fred - avered) * (fred - avered);

      if ((i % Num) == 0) {
        double R = (sqrt(sumirrms) / aveir) / (sqrt(sumredrms) / avered);
        SpO2 = -23.3 * (R - 0.4) + 100;
        ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;
        if (ESpO2 <= MINIMUM_SPO2) ESpO2 = MINIMUM_SPO2;
        if (ESpO2 > 100) ESpO2 = 99.9;
        sumredrms = 0.0; sumirrms = 0.0; SpO2 = 0;
        i = 0;
      }
      particleSensor.nextSample();
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastUpdate >= updateInterval) {
      lastUpdate = currentMillis;

      for (int i = 0; i < SCREEN_WIDTH - 1; i++) {
        graph[i] = graph[i + 1];
      }
      graph[SCREEN_WIDTH - 1] = irValue;

      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.print("BPM: ");
      display.print((int)beatAvg);
      display.setTextSize(2);
      display.setCursor(0, 15);
      display.print("SpO2: ");
      display.println((int)ESpO2);

      for (int i = 1; i < SCREEN_WIDTH; i++) {
        display.drawLine(i - 1, SCREEN_HEIGHT - map(graph[i - 1], 0, 65535, 0, SCREEN_HEIGHT / 4), i, SCREEN_HEIGHT - map(graph[i], 0, 65535, 0, SCREEN_HEIGHT / 4), SSD1306_WHITE);
      }

      display.display();
    }
  } else {
    for (byte rx = 0; rx < RATE_SIZE; rx++) rates[rx] = 0;
    beatAvg = 0; rateSpot = 0; lastBeat = 0;
    avered = 0; aveir = 0; sumirrms = 0; sumredrms = 0; SpO2 = 0;
  }
}

void sendToBlynk() {
  // Send BPM and SpO2 values to Blynk
  Blynk.virtualWrite(V3, (int)beatAvg);
  Blynk.virtualWrite(V4, (int)ESpO2);

  // Read temperature and humidity
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (!isnan(temperature) && !isnan(humidity)) {
    // Send temperature and humidity to Blynk
    Blynk.virtualWrite(V5, temperature);
    Blynk.virtualWrite(V6, humidity);

    // Control fan based on temperature
    if (temperature > 35.0) {
      digitalWrite(RELAY_PIN, HIGH); // Turn on the relay (and the fan)
    } else {
      digitalWrite(RELAY_PIN, LOW); // Turn off the relay (and the fan)
    }

    // Display temperature and humidity on OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Temp: ");
    display.print(temperature);
    display.print(" C");
    display.setCursor(0, 10);
    display.print("Humidity: ");
    display.print(humidity);
    display.print(" %");

    // Display BPM and SpO2 on OLED
    display.setTextSize(1);
    display.setCursor(0, 30);
    display.print("BPM: ");
    display.print((int)beatAvg);
    display.setCursor(0, 40);
    display.print("SpO2: ");
    display.print((int)ESpO2);

    display.display();

    // Trigger email alert if temperature exceeds 30°C
    if (temperature > 30.0) {
      Blynk.logEvent("temperature_event", "Temperature is above 30°C!");
    }
  }
}
