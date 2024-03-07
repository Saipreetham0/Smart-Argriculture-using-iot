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

#include <SoftwareSerial.h>
SoftwareSerial sim800lSerial(16, 17); // RX2, TX2 pins for SoftwareSerial

/* ... (Other includes and definitions) */

#define SIM800L_RX 16 // RX2 pin connected to SIM800L TX
#define SIM800L_TX 17 // TX2 pin connected to SIM800L RX

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
//  Serial.println(value);

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
//  Serial.print("PIR Value: ");
//  Serial.println(value);
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

void sendSMS( String message)
{


   sim800lSerial.println("AT"); // Once the handshake test is successful, it will back to OK
  updateSerial();

  sim800lSerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  sim800lSerial.println("AT+CMGS=\"+916304102761\""); // change ZZ with country code and xxxxxxxxxxx with phone number to sms
//  updateSerial();
//  sim800lSerial.print("Last Minute Engineers | lastminuteengineers.com"); // text content
//  updateSerial();
//  sim800lSerial.write(26);
  // Set the phone number
//  sim800lSerial.println("AT+CMGS=\"" + phonenumber + "\"");
//  delay(1000);
updateSerial();
  // Set the SMS content and send Ctrl+Z to indicate the end
  sim800lSerial.print(message);
  updateSerial();
  sim800lSerial.write(26);

  delay(1000);
}


void processSMS(String command) {
  if (command.equalsIgnoreCase("SENSORS")) {
    // Get sensor details
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    int soilMoisture = analogRead(SOIL_MOISTURE);
    soilMoisture = map(soilMoisture, 0, 4095, 100, 0);

    // Prepare sensor details message
    String sensorDetails = "Sensor Details:\n";
    sensorDetails += "Temperature: " + String(temperature) + " °C\n";
    sensorDetails += "Humidity: " + String(humidity) + " %\n";
    sensorDetails += "Soil Moisture: " + String(soilMoisture) + " %";

    // Send sensor details via SMS
    sendSMS( sensorDetails);
  } else {
    // Handle other commands or provide a response
    // For example, send an acknowledgment message
//    sendSMS("+919550421866", "Command received: " + command);
  }
}



void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    sim800lSerial.write(Serial.read()); // Forward what Serial received to Software Serial Port
  }
  while (sim800lSerial.available())
  {
    Serial.write(sim800lSerial.read()); // Forward what Software Serial received to Serial Port
  }
}

void setup()
{

  Serial.begin(9600);

  sim800lSerial.begin(9600); // Initialize SoftwareSerial for SIM800L
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
//  sendSMS("+919550421866", "KSP SENSORS");


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
//   if (sim800lSerial.available() > 0) {
//    String receivedMessage = sim800lSerial.readStringUntil('\n');
//    processSMS(receivedMessage);
//  }
//    updateSerial();

}