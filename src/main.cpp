#define BLYNK_TEMPLATE_ID "TMPL3reDBelVx"
#define BLYNK_TEMPLATE_NAME "Smart Agriculture System Using IoT"
#define BLYNK_AUTH_TOKEN "YLh2dO1hKEhl-MX94eGwhT9LyTGF_56j"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <WiFiClient.h>

BlynkTimer timer;

// Define sensor pins
#define DHT_PIN 4        // D32 on ESP32
#define SOIL_MOISTURE 32 // D27 on ESP32
#define PIR 25           // D25 on ESP32
#define RELAY 26         // D26 on ESP32 for the relay
#define VPIN_BUTTON_1 V12
#define BUZZER_PIN 23 // GPIO pin to which the buzzer is connected

int relay1State = LOW;

// Soil moisture threshold (adjust as needed)
int soilMoistureThreshold = 50;

// Initialize DHT sensor
DHT dht(DHT_PIN, DHT11);

int PIR_ToggleValue;

// Set password to "" for open networks.
char ssid[] = "KSP";
char pass[] = "9550421866";

// Create three variables for pressure
double T, P;
char status;

// Get the DHT11 sensor values
void DHT11sensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t))
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Blynk.virtualWrite(V0, t);
  Blynk.virtualWrite(V1, h);
}

// Get the soil moisture values
void soilMoistureSensor()
{
  int value = analogRead(SOIL_MOISTURE);
  value = map(value, 0, 4095, 100, 0);
  // value = (value - 100) * -1;

  Blynk.virtualWrite(V2, value);
  Serial.println(value);

  // Check if soil moisture is below the threshold
  if (value < soilMoistureThreshold)
  {
    // Turn on the relay (motor) if it's not already on
    if (relay1State == LOW)
    {
      digitalWrite(RELAY, HIGH);
      relay1State = HIGH;
      Blynk.virtualWrite(VPIN_BUTTON_1, relay1State);
      Serial.println("Motor turned ON");
    }
  }
  else
  {
    // Turn off the relay (motor) if it's not already off
    if (relay1State == HIGH)
    {
      digitalWrite(RELAY, LOW);
      relay1State = LOW;
      Blynk.virtualWrite(VPIN_BUTTON_1, relay1State);
      Serial.println("Motor turned OFF");
    }
  }
}

// Get the PIR sensor values
void PIRsensor()
{
  bool value = digitalRead(PIR);
  Serial.print("PIR Value: ");
  Serial.println(value);
  if (value)
  {
    Blynk.logEvent("pirmotion", "WARNNG! Motion Detected!"); // Enter your Event Name
    WidgetLED LED(V5);
    LED.on();
    digitalWrite(BUZZER_PIN, HIGH);
  }
  else
  {
    WidgetLED LED(V5);
    LED.off();
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// BLYNK_WRITE(V3)
// {
//   PIR_ToggleValue = param.asInt();
// }

BLYNK_CONNECTED()
{
  // Request the latest state from the server
  Blynk.syncVirtual(VPIN_BUTTON_1);
}

BLYNK_WRITE(VPIN_BUTTON_1)
{
  relay1State = param.asInt();
  digitalWrite(RELAY, relay1State);
}

void setup()
{

  Serial.begin(9600);

  Serial2.begin(9600);
  delay(3000);

  pinMode(PIR, INPUT);

  pinMode(RELAY, OUTPUT);
  pinMode(SOIL_MOISTURE, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // digitalWrite(RELAY, LOW);
  // pinMode(PUSH_BUTTON_1, INPUT_PULLUP);
  // digitalWrite(RELAY, relay1State);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  dht.begin();

  // Call the function
  timer.setInterval(100L, soilMoistureSensor);
  timer.setInterval(100L, DHT11sensor);
  // timer.setInterval(500L, checkPhysicalButton);
}

void loop()
{
  // updateSerial();

  Blynk.run();
  timer.run();
  PIRsensor();
}
