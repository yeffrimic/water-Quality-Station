/*
 * Monitor de calidad de agua — ESP32
 * Sensores: AHT10 (temp/hum ambiente), DS18B20 (temp agua), sonda pH (ADC), GPS (Serial2)
 * Salidas:  OLED SSD1306 (I2C), MQTT (PubSubClient), buzzer
 * Cambios v2:
 *   - Pantalla migrada de SH1106 a SSD1306
 *   - GPS con 5 decimales en el mensaje MQTT
 *   - Código reestructurado en funciones (lecturas, pantalla, botones, envío)
 *   - Corregido bug de toCharArray (faltaba el byte del terminador nulo)
 */

#include <WiFiManager.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>

// ---------- Pines ----------
#define BUTTON_PIN_SEND 26
#define BUTTON_PIN_STOP 33
#define BUZZER_PIN      4
#define DS18B20_PIN     32
#define ANALOG_PIN      34
#define SD_CS_PIN       5

// ---------- Configuración ----------
#define EEPROM_SIZE     4
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1      // Sin pin de reset dedicado
#define OLED_ADDR       0x3C

const char* mqtt_server = "broker.mqttdashboard.com";
const char* topic       = "/holi/1234";
const long  interval    = 10000;   // Intervalo de envío: 10 s

// Calibración pH (valores ADC medidos)
const int ADC_PH4  = 2523;
const int ADC_PH7  = 2100;
const int ADC_PH10 = 1973;

// ---------- Objetos ----------
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_AHTX0 aht;
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
TinyGPSPlus gps;
WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wifiManager;

// ---------- Estado ----------
struct Lecturas {
  float tempAmb;
  float humAmb;
  float tempAgua;
  float ph;
};
Lecturas datos;

unsigned long previousMillis = 0;
int  sendCount = 0;
bool isSending = false;

// =====================================================
//  SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);   // GPS en Serial2

  pinMode(BUTTON_PIN_SEND, INPUT_PULLUP);
  pinMode(BUTTON_PIN_STOP, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  iniciarPantalla();
  iniciarSensores();

  EEPROM.begin(EEPROM_SIZE);
  sendCount = EEPROM.read(0);

  wifiManager.autoConnect("ESP32_AutoConnect");
  client.setServer(mqtt_server, 1883);

  Serial.println(F("Setup completo"));
}

// =====================================================
//  LOOP
// =====================================================
void loop() {
  leerGPS();
  leerSensores();
  actualizarPantalla();

  if (!client.connected()) reconnect();
  client.loop();

  manejarBotones();

  if (isSending && millis() - previousMillis >= interval) {
    previousMillis = millis();
    enviarDatos();
  }
}

// =====================================================
//  INICIALIZACIÓN
// =====================================================
void iniciarPantalla() {
  // SSD1306: begin(fuente_de_voltaje, dirección I2C)
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.display();          // Splash de Adafruit
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void iniciarSensores() {
  if (!aht.begin()) {
    Serial.println(F("No AHT10 detected"));
    while (1);
  }
  sensors.begin();            // DS18B20
}

// =====================================================
//  LECTURAS
// =====================================================
void leerGPS() {
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }
}

void leerSensores() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  datos.tempAmb = temp.temperature;
  datos.humAmb  = humidity.relative_humidity;

  sensors.requestTemperatures();
  datos.tempAgua = sensors.getTempCByIndex(0);

  datos.ph = leerPH();
}

float leerPH() {
  int adcValue = analogRead(ANALOG_PIN);
  float phValue;

  // Interpolación lineal por tramos entre puntos de calibración
  if (adcValue <= ADC_PH10) {
    phValue = 10 + (15 - 10) * ((float)(adcValue - ADC_PH10) / (4098 - ADC_PH10));
  } else if (adcValue <= ADC_PH7) {
    phValue = 7 + (10 - 7) * ((float)(adcValue - ADC_PH7) / (ADC_PH10 - ADC_PH7));
  } else {
    phValue = 4 + (7 - 4) * ((float)(adcValue - ADC_PH4) / (ADC_PH7 - ADC_PH4));
  }
  return phValue;
}

// =====================================================
//  PANTALLA
// =====================================================
void actualizarPantalla() {
  display.clearDisplay();

  display.setCursor(0, 0);
  display.print(F("GPS lat: "));
  display.print(gps.location.lat(), 4);

  display.setCursor(0, 10);
  display.print(F("GPS long: "));
  display.print(gps.location.lng(), 4);

  display.setCursor(0, 20);
  display.print(F("Temp Amb.: "));
  display.print(datos.tempAmb);
  display.print(F(" C"));

  display.setCursor(0, 30);
  display.print(F("Hum Amb: "));
  display.print(datos.humAmb);
  display.print(F(" %"));

  display.setCursor(0, 40);
  display.print(F("H2O Temp: "));
  display.print(datos.tempAgua);
  display.print(F(" C"));

  display.setCursor(0, 50);
  display.print(F("PH: "));
  display.print(datos.ph);

  display.display();
}

// =====================================================
//  BOTONES
// =====================================================
void manejarBotones() {
  if (digitalRead(BUTTON_PIN_SEND) == LOW) {
    isSending = true;
    tone(BUZZER_PIN, 1000, 100);   // Tono de aceptación
    delay(500);                    // Debounce
  }

  if (digitalRead(BUTTON_PIN_STOP) == LOW) {
    isSending = false;
    tone(BUZZER_PIN, 500, 100);    // Tono de alerta
    delay(500);                    // Debounce
  }
}

// =====================================================
//  MQTT
// =====================================================
void enviarDatos() {
  // Timestamp a partir de fecha/hora del GPS
  tmElements_t tm;
  tm.Year   = gps.date.year() - 1970;
  tm.Month  = gps.date.month();
  tm.Day    = gps.date.day();
  tm.Hour   = gps.time.hour();
  tm.Minute = gps.time.minute();
  tm.Second = gps.time.second();
  time_t timestamp = makeTime(tm);

  // CSV: timestamp,lat,lng,alt,contador,tAgua,tAmb,hum,ph,mac
  String msg = String(timestamp);
  msg += ",";
  msg += String(gps.location.lat(), 5);    // 5 decimales (~1.1 m de precisión)
  msg += ",";
  msg += String(gps.location.lng(), 5);    // 5 decimales
  msg += ",";
  msg += String(gps.altitude.meters());
  msg += ",";
  msg += String(sendCount);
  msg += ",";
  msg += String(datos.tempAgua);
  msg += ",";
  msg += String(datos.tempAmb);
  msg += ",";
  msg += String(datos.humAmb);
  msg += ",";
  msg += String(datos.ph);
  msg += ",";
  msg += WiFi.macAddress();

  if (client.publish(topic, msg.c_str())) {
    sendCount++;
    EEPROM.write(0, sendCount);
    EEPROM.commit();
    Serial.print(F("Mensaje enviado exitosamente. Total de envios: "));
    Serial.println(sendCount);
  } else {
    Serial.println(F("Error al enviar el mensaje"));
  }
}

void reconnect() {
  if (!client.connected()) {
    Serial.print(F("Conectando al broker MQTT..."));
    String clientId = "Ducuchu-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println(F("conectado"));
    } else {
      Serial.print(F("fallo, rc="));
      Serial.print(client.state());
      Serial.println(F(" intentando de nuevo en 5 segundos"));
      delay(5000);
    }
  }
}