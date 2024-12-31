#include <Arduino.h>
#include <ESP32Servo.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

const char *ssid = "GOVAPEMAKER";    // Enter the 2.4 GHz WIFI name
const char *password = "12345678A"; // Enter the 2.4 GHz WIFI password

// Motor pins
#define motor1_pin1 27
#define motor1_pin2 26
#define motor2_pin1 25
#define motor2_pin2 33

#define motor3_pin1 32
#define motor3_pin2 16
#define motor4_pin1 17
#define motor4_pin2 18

// Servo pins
Servo myservo1;
Servo myservo2;
Servo myservo3;

int pos = 0;

WebSocketsServer webSocket = WebSocketsServer(8686);
int carSpeed = 255;

// Initialize pins
void initPin()
{
  pinMode(motor1_pin1, OUTPUT);
  pinMode(motor1_pin2, OUTPUT);
  pinMode(motor2_pin1, OUTPUT);
  pinMode(motor2_pin2, OUTPUT);
  pinMode(motor3_pin1, OUTPUT);
  pinMode(motor3_pin2, OUTPUT);
  pinMode(motor4_pin1, OUTPUT);
  pinMode(motor4_pin2, OUTPUT);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{

  switch (type)
  {
  case WStype_TEXT:
    String json = (char *)payload;
    JsonDocument doc;
    deserializeJson(doc, json);
    String dp = doc["dp"];

    if (dp == "servo")
    {
      int value = doc["value"];
      myservo1.write(value);
      myservo2.write(value);
      myservo3.write(value);
    }

    if (dp == "speed")
    {
      carSpeed = doc["value"];
    }

    if (dp == "direction")
    {
      String value = doc["value"];
      Serial.println(value);

      if (value == "forward")
      {
        digitalWrite(motor1_pin1, LOW);
        analogWrite(motor1_pin2, carSpeed);

        digitalWrite(motor2_pin1, LOW);
        analogWrite(motor2_pin2, carSpeed);

        analogWrite(motor3_pin1, carSpeed);
        digitalWrite(motor3_pin2, LOW);

        analogWrite(motor4_pin1, carSpeed);
        digitalWrite(motor4_pin2, LOW);
      }

      if (value == "backward")
      {
        analogWrite(motor1_pin1, carSpeed);
        digitalWrite(motor1_pin2, LOW);

        analogWrite(motor2_pin2, carSpeed);
        digitalWrite(motor2_pin2, LOW);

        digitalWrite(motor3_pin1, LOW);
        analogWrite(motor3_pin1, carSpeed);

        digitalWrite(motor4_pin1, LOW);
        analogWrite(motor4_pin1, carSpeed);
      }

      if (value == "left")
      {
        analogWrite(motor1_pin1, carSpeed);
        digitalWrite(motor1_pin2, LOW);

        digitalWrite(motor2_pin1, LOW);
        analogWrite(motor2_pin2, carSpeed);

        digitalWrite(motor3_pin1, carSpeed);
        digitalWrite(motor3_pin2, LOW);

        digitalWrite(motor4_pin1, LOW);
        analogWrite(motor4_pin2, carSpeed);
      }

      if (value == "right")
      {
        digitalWrite(motor1_pin1, LOW);
        analogWrite(motor1_pin2, carSpeed);

        analogWrite(motor2_pin1, carSpeed);
        digitalWrite(motor2_pin2, LOW);

        digitalWrite(motor3_pin1, LOW);
        analogWrite(motor3_pin2, carSpeed);

        analogWrite(motor4_pin1, carSpeed);
        digitalWrite(motor4_pin2, LOW);
      }

      if (value == "counterclockwise")
      {
        analogWrite(motor1_pin1, carSpeed);
        digitalWrite(motor1_pin2, LOW);

        digitalWrite(motor2_pin1, LOW);
        analogWrite(motor2_pin2, carSpeed);

        digitalWrite(motor3_pin1, LOW);
        analogWrite(motor3_pin2, carSpeed);

        analogWrite(motor4_pin1, carSpeed);
        digitalWrite(motor4_pin2, LOW);
      }

      if (value == "clockwise")
      {
        digitalWrite(motor1_pin1, LOW);
        analogWrite(motor1_pin2, carSpeed);

        analogWrite(motor2_pin1, carSpeed);
        digitalWrite(motor2_pin2, LOW);

        analogWrite(motor3_pin1, carSpeed);
        digitalWrite(motor3_pin2, LOW);

        digitalWrite(motor4_pin1, LOW);
        analogWrite(motor4_pin2, carSpeed);
      }

      if (value == "forwardLeft")
      {
        digitalWrite(motor2_pin1, LOW);
        analogWrite(motor2_pin2, carSpeed);

        analogWrite(motor3_pin1, carSpeed);
        digitalWrite(motor3_pin2, LOW);
      }

      if (value == "forwardRight")
      {
        digitalWrite(motor1_pin1, LOW);
        analogWrite(motor1_pin2, carSpeed);

        analogWrite(motor4_pin1, carSpeed);
        digitalWrite(motor4_pin2, LOW);
      }

      if (value == "backwardLeft")
      {
        analogWrite(motor1_pin1, carSpeed);
        digitalWrite(motor1_pin2, LOW);

        digitalWrite(motor4_pin1, LOW);
        analogWrite(motor4_pin2, carSpeed);
      }

      if (value == "backwardRight")
      {
        analogWrite(motor2_pin1, carSpeed);
        digitalWrite(motor2_pin2, LOW);

        digitalWrite(motor3_pin1, LOW);
        analogWrite(motor3_pin2, carSpeed);
      }

      if (value == "stop")
      {
        initPin();
        digitalWrite(motor1_pin1, LOW);
        digitalWrite(motor1_pin2, LOW);

        digitalWrite(motor2_pin1, LOW);
        digitalWrite(motor2_pin2, LOW);

        digitalWrite(motor3_pin1, LOW);
        digitalWrite(motor3_pin2, LOW);

        digitalWrite(motor4_pin1, LOW);
        digitalWrite(motor4_pin2, LOW);
      }
    }

    break;
  }
}

void setup()
{
  Serial.begin(115200);

  Serial.println();
  Serial.println("******************************************************");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  Serial.println("If successful, connection address will be printed. Otherwise, dots will keep printing.");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");

  initPin();

  myservo1.attach(13);
  myservo1.attach(2);
  myservo1.attach(4);

  webSocket.begin();                 // Start the WebSocket server
  webSocket.onEvent(webSocketEvent); // Set the WebSocket event handler

  String ipString = WiFi.localIP().toString();
  Serial.println("Socket address: ws://" + ipString + ":8686");
}

void loop()
{
  webSocket.loop(); // Handle WebSocket events
}
