#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define drivo1 4
#define drivo2 17
#define drivo3 16
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 13
#define L298IN1 21
#define L298IN2 22
#define oxygenQuailty 5
#define pHQuailty 79
#define turbidityQuailty 100
#define STEPS_PER_REV 1024
#define pHTopic "/pHSensor"
#define oxygenTopic "/OxygenSensor"
#define turbidityTopic "/TurbiditySensor"


#include <WiFi.h>
#include <PubSubClient.h>

#define MQTT_SERVER "40.81.22.116"
#define MQTT_PORT 1883
#define MQTT_USERNAME ""
#define MQTT_PASSWORD ""
#define MQTT_NAME "ESP32_1"
WiFiClient client;
PubSubClient mqtt(client);

const char* ssid = "vivoV30Pro";
const char* password = "qcu4nqms377figq";

// float pHValue = 0;
// float oxygenValue = 0;
// float turbidity = 0;
int stepDelay = 2;

void connectWiFi();
void mqttTask(void* pvParameters);
void callback(char* topic, byte* payload, unsigned int length);
void stepMotorTask(void* pvParameters);
void motorTask(void* pvParameters);
void stepMotor(int step);


TaskHandle_t wifiTaskHandle;
TaskHandle_t stepMotorTaskHandle;
TaskHandle_t motorTaskHandle;

QueueHandle_t turbidityMailbox;
QueueHandle_t pHMailbox;
QueueHandle_t oxygenMailbox;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  connectWiFi();
  pinMode(drivo1, OUTPUT);
  pinMode(drivo2, OUTPUT);
  pinMode(drivo3, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(L298IN1, OUTPUT);
  pinMode(L298IN2, OUTPUT);
  digitalWrite(drivo1, HIGH);
  digitalWrite(drivo2, HIGH);
  digitalWrite(drivo3, HIGH);

  pHMailbox = xQueueCreate(1, sizeof(float));
  oxygenMailbox = xQueueCreate(1, sizeof(float));
  turbidityMailbox = xQueueCreate(1, sizeof(float));

  xTaskCreate(mqttTask, "MQTT Task", 2048, NULL, 1, &wifiTaskHandle);
  xTaskCreate(stepMotorTask, "Step Motor Task", 2048, NULL, 1, &stepMotorTaskHandle);
  xTaskCreate(motorTask, "Motor Task", 2048, NULL, 1, &motorTaskHandle);
}

void loop() {
  // put your main code here, to run repeatedly:
}


void mqttTask(void* pvParameters) {
  while (1) {
    if (!mqtt.connected()) {
      Serial.println("Reconnecting to MQTT...");
      if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
        Serial.println("MQTT Connected");
        mqtt.subscribe("/OxygenSensor");
        mqtt.subscribe("/TurbiditySensor");
        mqtt.subscribe("/pHSensor");
      } else {
        Serial.println("MQTT Connection Failed");
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Wait 5 seconds before retrying
      }
    }
    mqtt.loop();                          // Process MQTT messages
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Avoid CPU overload
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);  //Optional
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(callback);
}

void callback(char* topic, byte* payload, unsigned int length) {

  float checkValue = 0.0;

  payload[length] = '\0';
  String topic_str = topic, payload_str = (char*)payload;
  // Serial.print("Payload is :");
  // Serial.println("[" + topic_str + "]: " + payload_str);
  float value = payload_str.toFloat();
  if (topic_str == pHTopic) {
    xQueueOverwrite(pHMailbox, &value);


    if (xQueuePeek(pHMailbox, &checkValue, 0) == pdTRUE) {
      if (checkValue == value) {
        Serial.println("Queue update successful!");
      } else {
        Serial.println("Queue update failed!");
      }
    } else {
      Serial.println("Queue read failed!");
    }


    // pHValue = value;
    Serial.print("Payload is :");
    Serial.println("[" + topic_str + "]: " + payload_str);
  } else if (topic_str == oxygenTopic) {
    xQueueOverwrite(oxygenMailbox, &value);


    if (xQueuePeek(oxygenMailbox, &checkValue, 0) == pdTRUE) {
      if (checkValue == value) {
        Serial.println("Queue update successful!");
      } else {
        Serial.println("Queue update failed!");
      }
    } else {
      Serial.println("Queue read failed!");
    }

    // oxygenValue = value;
    Serial.print("Payload is :");
    Serial.println("[" + topic_str + "]: " + payload_str);
  } else if (topic_str == turbidityTopic) {
    xQueueOverwrite(turbidityMailbox, &value);


    if (xQueuePeek(turbidityMailbox, &checkValue, 0) == pdTRUE) {
      if (checkValue == value) {
        Serial.println("Queue update successful!");
      } else {
        Serial.println("Queue update failed!");
      }
    } else {
      Serial.println("Queue read failed!");
    }

    // turbidity = value;
    Serial.print("Payload is :");
    Serial.println("[" + topic_str + "]: " + payload_str);
  }
}

void stepMotorTask(void* pvParameters) {
  while (1) {
    // Rotate stepper motor forward
    for (int i = 0; i < STEPS_PER_REV; i++) {
      stepMotor(i);
      vTaskDelay(pdMS_TO_TICKS(stepDelay));
    }
    vTaskDelay(pdMS_TO_TICKS(10000));
    // Rotate stepper motor backward
    for (int i = STEPS_PER_REV - 1; i >= 0; i--) {
      stepMotor(i);
      vTaskDelay(pdMS_TO_TICKS(stepDelay));
    }

    vTaskDelay(pdMS_TO_TICKS(10000));  // Small delay before next cycle
  }
}

void motorTask(void* pvParameters) {
  float pHValue, oxygenValue, turbidityValue;

  while (1) {
    if (xQueuePeek(turbidityMailbox, &turbidityValue, pdMS_TO_TICKS(100)) == pdTRUE) {
      Serial.print("Latest turbidity value: ");
      Serial.println(turbidityValue);
      digitalWrite(drivo3, turbidityValue > turbidityQuailty ? LOW : HIGH);
    }
    if (xQueuePeek(pHMailbox, &pHValue, pdMS_TO_TICKS(100)) == pdTRUE) {
      Serial.print("Latest pH value: ");
      Serial.println(pHValue);
      if (pHValue < pHQuailty - 1) {
        digitalWrite(drivo2, LOW);
        vTaskDelay(pdMS_TO_TICKS(3000));
        digitalWrite(drivo2, HIGH);
      } else if (pHValue > pHQuailty + 1) {
        digitalWrite(drivo1, LOW);
        vTaskDelay(pdMS_TO_TICKS(3000));
        digitalWrite(drivo1, HIGH);
      }
    }
    if (xQueuePeek(oxygenMailbox, &oxygenValue, pdMS_TO_TICKS(100)) == pdTRUE) {
      Serial.print("Latest oxygen value: ");
      Serial.println(oxygenValue);
      if (oxygenValue < oxygenQuailty) {
        digitalWrite(L298IN1, LOW);
        digitalWrite(L298IN2, HIGH);
      } else {
        digitalWrite(L298IN1, LOW);
        digitalWrite(L298IN2, LOW);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));  // Run every 1 second
  }
}

void stepMotor(int step) {
  switch (step % 4) {
    case 0:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break;
    case 1:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break;
    case 2:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      break;
    case 3:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      break;
  }
}