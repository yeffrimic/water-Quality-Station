
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Button2.h"
#include <Ticker.h>
#include <Adafruit_AHTX0.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPSPlus.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <TimeLib.h>
#include <EEPROM.h>

#define UUID_ADDRESS 0  // Dirección en la EEPROM donde se almacenará el UUID
#define UUID_SIZE 4     // Tamaño del UUID en bytes (para un entero de 4 bytes)
#define MQTT_SERVER "broker.mqtt-dashboard.com"
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define FORDWARD_BUTTON 25
#define BACKWARD_BUTTON 26
#define ONE_WIRE_BUS 32
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define VOLTAGEPH7 1.89
#define VOLTAGEPH4 2.08
#define PH1 4
#define PH2 7
#define ph_pin 34
#define GPSBaud 9600
#define ss Serial2
#define MSG_BUFFER_SIZE 150


WiFiClient espClient;
PubSubClient client(espClient);

const char* ssid = "Xibalba Hackerspace";
const char* password = "ESTALIBRE";


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_AHTX0 aht;
sensors_event_t humidity, temp;

Button2 fordwardButton, backwardButton;

boolean IdleFlag = true;  // true shows the idle data
boolean showData = true;
boolean sensorsReviewed = false;


TinyGPSPlus gps;

Ticker sensorsRead;
Ticker sensorsData;
Ticker sensorsSend;
float ph;
int h2o, tempe, hume, IDLectura;

byte states = 0;
byte sensors = 0;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature h20sensors(&oneWire);
DeviceAddress insideThermometer;



long lastReconnectAttempt = 0;




void setup() {
  Serial.begin(115200);
  ss.begin(9600);

  setup_wifi();
  // put your setup code here, to run once:
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    //Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  fordwardButton.begin(FORDWARD_BUTTON);
  backwardButton.begin(BACKWARD_BUTTON);
  fordwardButton.setPressedHandler(pressed);
  backwardButton.setPressedHandler(pressed);
  idle();

  client.setServer(MQTT_SERVER, 1883);
  EEPROM.begin(512);

  // Incrementar el UUID y guardarlo en la EEPROM
  uint32_t uuid = incrementarUUID();
  guardarUUID(uuid);

  Serial.print("UUID actualizado: ");
  Serial.println(uuid);
}
void loop() {
  switch (states) {
    case 0:  // idle
      idle();
      fordwardButton.loop();
      break;
    case 1:  // checkSensors
      reviewSensors();
      states = 2;
      break;
    case 2:  // activate ticker
      sensorsData.attach(5, showSensorsData);
      sensorsSend.attach(60, sensorsDataSend);
      states = 3;
      break;
    case 3:  // operating
      if (!client.connected()) {
        long now = millis();
        if (now - lastReconnectAttempt > 5000) {
          lastReconnectAttempt = now;
          // Attempt to reconnect
          if (reconnect()) {
            lastReconnectAttempt = 0;
          }
        }
      }
      fordwardButton.loop();
      backwardButton.loop();

      break;
    case 4:  // reseting ticker


      sensorsData.detach();
      sensorsSend.detach();
      states = 0;
      showData = true;
      break;
  }
}


uint32_t incrementarUUID() {
  // Leer el UUID actual desde la EEPROM
  uint32_t uuid;
  EEPROM.get(UUID_ADDRESS, uuid);

  Serial.println("leido: ");
  Serial.println(uuid);
  // Incrementar el UUID
  uuid++;
  Serial.println("aumentado: ");
  Serial.println(uuid);
  IDLectura = uuid;
  return uuid;
}

void guardarUUID(uint32_t uuid) {
  // Escribir el nuevo UUID en la EEPROM
  EEPROM.put(UUID_ADDRESS, uuid);
  EEPROM.commit();  // Guardar los cambios en la EEPROM
}


void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


boolean reconnect() {
  if (client.connect("ducuhuuuuu")) {
    // Once connected, publish an announcement...
    client.publish("/ducuchu/conexion", "1");
  }
  return client.connected();
}


void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

void displayInfo() {
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 8);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 8);
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}


void GPS() {
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      //  displayInfo();

      Serial.println("gps");
}
void H2O() {
  h20sensors.requestTemperatures();
  h2o = h20sensors.getTempC(insideThermometer);
  if (h2o == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  //Serial.print("Temp C: ");
  //Serial.println(h2o);
}
void PH() {
  int measure = analogRead(ph_pin);
  //Serial.print("Measure: ");
  // Serial.print(measure);

  double voltage = 3.3 / 4098.0 * measure;  //classic digital to voltage conversion
                                            // Serial.print("\tVoltage: ");
  //Serial.print(voltage, 2);

  float PH_step = (VOLTAGEPH7 - VOLTAGEPH4) / (PH1 - PH2);

  float Po = 7 + ((VOLTAGEPH7 - voltage) / PH_step);
  //Serial.print("\tPH: ");
  // Serial.print(Po, 2);
  ph = Po;
  //Serial.println("");
  if (ph > 15) ph = 0;
}
void TEMPNHUM() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);  // populate temp and humidity objects with fresh data
                                   // Serial.print("Temperature: ");
                                   // Serial.print(temp.temperature);
                                   // Serial.println(" degrees C");
                                   // Serial.print("Humidity: ");
                                   // Serial.print(humidity.relative_humidity);
                                   // Serial.println("% rH");
  tempe = int(temp.temperature);
  hume = int(humidity.relative_humidity);
  //Serial.println(tempe);
  //Serial.println(hume);
}
void sensorUpdate() {
  switch (sensors) {
    case 0:
      GPS();
      sensors = 1;
      break;
    case 1:
      H2O();
      sensors = 2;
      break;
    case 2:
      PH();
      sensors = 3;
      break;
    case 3:
      TEMPNHUM();
      sensors = 0;
      break;
  }
}


void pressed(Button2& btn) {
  if (btn == fordwardButton) {
    if (states == 0) states = 1;
  } else if (btn == backwardButton) {
    if (states == 3) states = 4;
  }
}

void idle() {
  if (showData) {
    // Clear the buffer
    display.clearDisplay();
    display.setTextSize(2);               // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);  // Draw white text
    display.setCursor(0, 0);              // Start at top-left corner
    display.print(F("Presione \n \n \n avanzar"));
    display.drawCircle(display.width() / 2, display.height() / 2, 10, SSD1306_WHITE);
    display.display();
    showData = false;
  }
}


void displayData(char* data, int delayTime, byte textSize) {

  display.clearDisplay();
  display.setTextSize(textSize);        // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.print((data));
  display.display();
  delay(delayTime);
}

boolean reviewSensors() {

  displayData("Revisando \nsensores", 2000, 2);
  displayData("Revisando \nGPS", 2000, 2);
  displayData("Revisando \nPH", 2000, 2);
  if (analogRead(ph_pin) < 5 || analogRead(ph_pin) > 4095) ph = -255;

  displayData("Revisando \nTemperatura \nH2O", 2000, 2);
  h20sensors.begin();
  if (!h20sensors.getAddress(insideThermometer, 0)) {
    Serial.println("Unable to find address for Device 0");
    h2o = -255;
  } else {
    h20sensors.setResolution(insideThermometer, 9);
    //Serial.print("Device 0 Resolution: ");
    //Serial.print(h20sensors.getResolution(insideThermometer), DEC);
    //Serial.println();
    // Serial.print("Parasite power is: ");
    if (h20sensors.isParasitePowerMode()) Serial.println("ON");
    else Serial.println("OFF");
  }


  displayData("Revisando \nTemperatura \nHumedad \nAmbiente", 2000, 2);
  if (!aht.begin()) {
    // Serial.println("Could not find AHT? Check wiring");
    tempe = -255;
    hume = -255;
  } else {
    //Serial.println("AHT10 or AHT20 found");
  }


  return true;
}

void showSensorsData() {
  sensorUpdate();
  char data[40];
  snprintf(data, 40, "PH: %.2f\nTEMP: %iC \nH2O: %iC\nHUM: %i%%\n", ph, tempe, h2o, hume);
  //Serial.println(data);
  displayData(data, 20, 2);
  //Serial.println("sensorsloop");
  delay(100);
}

void sensorsDataSend() {

  if (client.connected()) {
    // Client connected
    /*
  TIMESTAMP,
  LATITUD,
  LONGITUD,
  ALTITUD,
  Numero de lectura,
  TEMPERATURA DEL AGUA,
  TEMPERATURA DEL AMBIENTE,
  HUMEDAD DEL AMBIENTE,
  PH*/
    tmElements_t tm;
    tm.Year = gps.date.year() - 1970;
    tm.Month = gps.date.month();
    tm.Day = gps.date.day();
    tm.Hour = gps.time.hour();
    tm.Minute = gps.time.minute();
    tm.Second = gps.time.second();
    time_t timestamp = makeTime(tm);
    String msg = String(timestamp);
    msg += ",";

    msg += dtostrf(gps.location.lat(), 3, 8, "");
    msg += ",";
    msg += dtostrf(gps.location.lng(), 3, 8, "");
    msg += ",";
    msg += String(gps.altitude.meters());
    msg += ",";
    msg += String(IDLectura);
    msg += ",";
    msg += String(h2o);
    msg += ",";
    msg += String(tempe);
    msg += ",";
    msg += String(hume);
    msg += ",";
    msg += String(ph);
    msg += ",";
    msg += WiFi.macAddress();
    //Serial.println(mac);
    char msgchar[msg.length()];
    Serial.println(msg);
    Serial.println(msgchar);
    msg.toCharArray(msgchar, msg.length());
    //Serial.print("Publish message: ");
    //Serial.println(msg);
    client.setBufferSize(150);
    client.publish("/duchucu/agua/datos", msgchar);
    client.loop();
  }

  Serial.println("enviando datos");
}
