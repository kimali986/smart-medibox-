#include <WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// defining pin numbers
#define DHT_PIN 15
#define BUZZER 12
#define leftLDR 32  
#define rightLDR 33 

// Global variables
WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
float theta_offset = 30; // Initial value for shade angle offset
float gammaValue = 0.75; // Initial value for shade control gamma
float intensity = 0.0;    // Initial value for light intensity
float D;                   // Variable for shade distance adjustment
char tempAr[6];            // Array to store temperature
char lightAr[6];           // Array to store light intensity
DHTesp dhtSensor;          // DHT temperature and humidity sensor instance
Servo servo;               // Servo instance for motor control
const int servoPin = 18;   // Pin for servo motor

bool isScheduledON = false;     // Flag to track scheduled activation
unsigned long scheduledOnTime;  // Variable to store scheduled activation time

void setup() {
  Serial.begin(115200);

  // Setup WiFi and MQTT
  setupWifi();
  setupMqtt();

  // Setup DHT sensor
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);

  // Setup NTP client
  timeClient.begin();
  timeClient.setTimeOffset(5.5 * 3600); // Set time offset

  // Setup servo motor
  servo.attach(servoPin, 500, 2400); // Attach servo motor

  // Setup pins for buzzer and LDR
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  pinMode(leftLDR, INPUT);
  pinMode(rightLDR, INPUT);
}

void loop() {
  // Check MQTT connection
  if (!mqttClient.connected()) {
    connectToBroker();
  }
  mqttClient.loop();

  // Update temperature reading and publish
  updateTemperature();
  Serial.println(tempAr);
  mqttClient.publish("ENTC-KIMALI-TEMP", tempAr);

  // Read LDR values
  int Left = analogRead(leftLDR);
  int Right = analogRead(rightLDR);

  // Scale LDR values
  float leftValue = 1.0 - (map(Left,32,4063,0,1023) /1023.0);
  float rightValue = 1.0 - (map(Right,32,4063,0,1023)/1023.0);

  Serial.println(leftValue);
  Serial.println(rightValue);

  // Determine which LDR has a higher value
  bool isLeftHigher = leftValue > rightValue;

  // Publish the highest value along with the corresponding sensor
  if (isLeftHigher) {
    String(leftValue,2).toCharArray(lightAr,6);
    mqttClient.publish("ENTC-KIMALI-LIGHT", lightAr);
    mqttClient.publish("ENTC-KIMALI-LIGHT-SIDE", "Left side");
    D = 1.5; // if the left is high d is set to 1.5
  } else {
    String(rightValue,2).toCharArray(lightAr,6);
    mqttClient.publish("ENTC-KIMALI-LIGHT", lightAr);
    mqttClient.publish("ENTC-KIMALI-LIGHT-SIDE", "Right side");
    D = 0.5; // if the right is high d is set to 0.5
  }

  // Check scheduled activation
  checkSchedule();
  delay(1000);
}

// Function to get current time from NTP server
unsigned long getTime() {
  timeClient.update();
  return timeClient.getEpochTime();
} 

// Function to update temperature reading
void updateTemperature() {
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  String(data.temperature, 2).toCharArray(tempAr, 6);
}

// Function to setup WiFi connection
void setupWifi() { 
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println("Wokwi-GUEST");
  WiFi.begin("Wokwi-GUEST","");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Function to setup MQTT connection
void setupMqtt() {
  mqttClient.setServer("test.mosquitto.org", 1883);
  mqttClient.setCallback(recieveCallback);
}

// Callback function for MQTT messages
void recieveCallback(char* topic, byte* payload, unsigned int length) {
  float angle;

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]");

  char payloadCharAr[length];
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    payloadCharAr[i] = (char)payload[i];
  }

  Serial.println();
  
  // Handle different MQTT topics
  if (strcmp(topic, "ENTC-KIMALI-MAIN-ON-OFF") == 0) {  //if the topic is main on off
    buzzerOn(payloadCharAr[0] == '1');  // boolean to activate buzzer
  } else if (strcmp(topic, "ENTC-KIMALI-SCH-ON") == 0) { //if the topic is sch on off
    if (payloadCharAr[0] == 'N') {
      isScheduledON = false;
    } else {
      isScheduledON = true;
      scheduledOnTime = atol(payloadCharAr);
    }
  } else if (strcmp(topic, "ENTC-KIMALI-LIGHT-INTENSITY") == 0) { //if the topic is light intensity
    intensity = atof(payloadCharAr);  
    // Check if all parameters are available before calculating motor angle
    if (theta_offset != 0 && gammaValue != 0) {
      angle = calculateMotorAngle();
      rotateMotor(angle);
    }
  } else if (strcmp(topic, "ENTC-KIMALI-SHADE-ANGLE") == 0) {  //if the topic is shade angle
    theta_offset = atof(payloadCharAr);
    // Check if all parameters are available before calculating motor angle
    if (gammaValue != 0 && intensity != 0) {
      angle = calculateMotorAngle();
      rotateMotor(angle);
    }
  } else if (strcmp(topic, "ENTC-KIMALI-SHADE-CONTROL") == 0) {   //if the topic is shade controll
    gammaValue = atof(payloadCharAr);
    // Check if all parameters are available before calculating motor angle
    if (theta_offset != 0 && intensity != 0) {
      angle = calculateMotorAngle();
      rotateMotor(angle);
    }
  }
}

// Function to connect to MQTT broker
void connectToBroker() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("21553263")) {
      Serial.println("connected");
      // Subscribe to relevant topics
      mqttClient.subscribe("ENTC-KIMALI-MAIN-ON-OFF");
      mqttClient.subscribe("ENTC-KIMALI-SCH-ON");
      mqttClient.subscribe("ENTC-KIMALI-SHADE-ANGLE");
      mqttClient.subscribe("ENTC-KIMALI-SHADE-CONTROL");
      mqttClient.subscribe("ENTC-KIMALI-LIGHT-INTENSITY");
    } else {
      Serial.print("failed");
      Serial.print(mqttClient.state());
      delay(5000);
    }
  }
}

// Function to control the buzzer
void buzzerOn(bool on) {
  if (on) {
    tone(BUZZER, 256);
  } else {
    noTone(BUZZER);
  }
}

// Function to check scheduled activation
void checkSchedule() {
  if (isScheduledON) {
    unsigned long currentTime = getTime();
    if (currentTime > scheduledOnTime) {
      buzzerOn(true);
      isScheduledON = false;
      mqttClient.publish("ENTC-KIMALI-MAIN-ON-OFF-ESP", "1");
      mqttClient.publish("ENTC-KIMALI-SCH-ESP-ON", "0");
      Serial.println("Scheduled ON");
    }
  }
}

// Function to calculate motor angle based on shade parameters
float calculateMotorAngle() {
  float theta = min((theta_offset * D) + (180 - theta_offset) * intensity * gammaValue, 180.0f);
  Serial.print("Motor angle: ");
  Serial.println(theta);
  return theta;
}

// Function to rotate the servo motor to a specific angle
void rotateMotor(float theta) {
  static int currentAngle = 0; // Variable to store the current angle
  int targetAngle = map(theta, 0, 180, 0, 180); // Map theta to servo angle range

  // Define the increment step for each iteration
  int step = (targetAngle > currentAngle) ? 1 : -1; // Direction of rotation

  // Change the angle until it reaches the target angle
  for (int pos = currentAngle; pos != targetAngle; pos += step) {
    servo.write(pos);
    delay(5); // Delay for smoother rotation
  }

  // Update the current angle after reaching the target angle
  currentAngle = targetAngle;
}

