/*
 * Monitor de calidad de agua — ESP32
 * Sensores: AHT10 (temp/hum ambiente), DS18B20 (temp agua), sonda pH (ADC), GPS (Serial2)
 * Salidas:  OLED SH1106 (I2C), MQTT (PubSubClient), buzzer
 * Cambios v2:
 *   - GPS con 5 decimales en el mensaje MQTT
 *   - Código reestructurado en funciones (lecturas, pantalla, botones, envío)
 *   - Corregido bug de toCharArray (faltaba el byte del terminador nulo)
 * Cambios v3:
 *   - Revertido a librería SH1106 (Adafruit_SH110X): el panel físico es
 *     SH1106, no SSD1306. Usar la librería SSD1306 en este panel hacía
 *     que arrancara con la pantalla llena de puntos random en vez del
 *     logo de Adafruit.
 * Cambios v4:
 *   - Pantalla de título "Lagos Abiertos" al arrancar.
 *   - Si no hay WiFi guardado/disponible, la pantalla muestra las
 *     instrucciones de conexión (red de configuración e IP del portal)
 *     en vez de quedar en blanco esperando.
 */

#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_AHTX0.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>
#include <LittleFS.h>

// ---------- Logo "Lagos Abiertos" (48x48 px, contornos, 1 bit/pixel) ----------
#define LOGO_WIDTH  48
#define LOGO_HEIGHT 48
static const unsigned char PROGMEM logo_bitmap[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x08, 0x0C, 0x00, 0x00,
  0x00, 0x01, 0x03, 0xC1, 0x80, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
  0x00, 0x11, 0x00, 0x01, 0x90, 0x00, 0x00, 0x24, 0x00, 0x00, 0x40, 0x00,
  0x00, 0x48, 0x00, 0x00, 0x10, 0x00, 0x00, 0x90, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x20, 0x00, 0x00, 0x04, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0x80,
  0x04, 0x80, 0x00, 0x05, 0x00, 0x40, 0x05, 0x00, 0x00, 0x08, 0x81, 0x00,
  0x08, 0x01, 0x00, 0x30, 0x40, 0xA0, 0x02, 0x08, 0x80, 0x40, 0x20, 0x00,
  0x10, 0x10, 0x41, 0x80, 0x1C, 0x50, 0x04, 0x60, 0x24, 0x00, 0x02, 0x00,
  0x01, 0x80, 0x18, 0x00, 0x01, 0x08, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x68,
  0x20, 0x00, 0x00, 0x00, 0x00, 0x28, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x01, 0x04,
  0x00, 0x00, 0x39, 0x80, 0x00, 0x00, 0x00, 0x3F, 0x01, 0x01, 0xFC, 0x00,
  0x0B, 0x78, 0x01, 0x80, 0x00, 0x04, 0x08, 0x00, 0x01, 0x80, 0x00, 0x04,
  0x20, 0x00, 0x01, 0xC0, 0x00, 0x00, 0x20, 0x00, 0x32, 0x44, 0x00, 0x00,
  0x23, 0x3F, 0xFA, 0x4F, 0xFE, 0x00, 0x01, 0x08, 0x06, 0x70, 0x01, 0x08,
  0x04, 0x09, 0x04, 0x30, 0x00, 0x00, 0x10, 0x00, 0x00, 0x30, 0x00, 0x04,
  0x00, 0x00, 0x00, 0x20, 0x00, 0x1C, 0x09, 0xFF, 0xC0, 0x00, 0x00, 0x14,
  0x07, 0x80, 0xF0, 0x00, 0x00, 0x60, 0x04, 0x00, 0x1C, 0x00, 0x01, 0xC8,
  0x00, 0x2E, 0x07, 0x80, 0x07, 0x90, 0x01, 0xEF, 0x00, 0xFE, 0xFC, 0x00,
  0x00, 0x80, 0x70, 0x0C, 0x40, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x10, 0x0C, 0x00, 0x71, 0x00, 0x00, 0x04, 0x00, 0x5F, 0xC2, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x60, 0x00,
  0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

// ---------- Pines ----------
#define BUTTON_PIN_START_STOP_T 26
#define BUTTON_PIN_NAV_N 33
#define BUZZER_PIN      4
#define DS18B20_PIN     32
#define ANALOG_PIN      34
#define SD_CS_PIN       5

// ---------- Configuración ----------
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1      // Sin pin de reset dedicado
#define OLED_ADDR       0x3C

const char* mqtt_server = "broker.mqttdashboard.com";
const char* topic       = "/lagosabiertos/dispositivo";
const long  interval    = 10000;   // Intervalo de envío: 10 s
const unsigned long TIEMPO_MAX_INICIALIZACION = 60000;
const unsigned long TIEMPO_MIN_LOGO = 2000;

const char* APP_SITE = "lagosabiertos.org";
const char* APP_TITLE = "Lagos Abiertos";
const char* AP_SSID    = "Lagos Abiertos";  // Red de configuración WiFi

// Caracteres acentuados (código de página 437, la fuente que usa Adafruit_GFX)
const char TILDE_E = 0x82;  // é
const char TILDE_O = 0xA2;  // ó

const char* ARCHIVO_PENDIENTES = "/pendientes.csv";
const char* ARCHIVO_PENDIENTES_TMP = "/pendientes.tmp";
const size_t MARGEN_MINIMO_LIBRE = 2048;
const float UMBRAL_COLA_LLENA = 0.90;

// Calibración pH (valores ADC medidos, recalibrado 2026-07-31 tras rehidratar la sonda)
const int ADC_PH4    = 3673;  // buffer pH 4.00
const int ADC_PH7    = 3233;  // buffer pH 7.00
const int ADC_PH9_18 = 2929;  // buffer pH 9.18
const float PH_MIN_VALIDO = 0.0;
const float PH_MAX_VALIDO = 14.0;

// ---------- Objetos ----------
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_AHTX0 aht;
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
TinyGPSPlus gps;
WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wifiManager;
Preferences preferences;

// ---------- Estado ----------
struct Lecturas {
  float tempAmb;
  float humAmb;
  float tempAgua;
  float ph;
};
Lecturas datos;

String macAddress;

unsigned long previousMillis = 0;
unsigned long mqttUltimoIntento = 0;
const unsigned long MQTT_REINTENTO_INTERVALO = 5000;
unsigned long wifiUltimoIntento = 0;
const unsigned long WIFI_REINTENTO_INTERVALO = 30000;
unsigned long ultimaSolicitudTemp = 0;
bool solicitudTempPendiente = false;
unsigned long tiempoConversionDS18B20 = 750;
unsigned long ultimoDrenado = 0;
const unsigned long DRENADO_INTERVALO = 2000;
uint32_t sessionId = 0;
bool activo = false;

enum Pantalla { PANTALLA_DATOS, PANTALLA_PENDIENTES, PANTALLA_WIFI };
Pantalla pantallaActual = PANTALLA_DATOS;

// =====================================================
//  SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);   // GPS en Serial2

  pinMode(BUTTON_PIN_START_STOP_T, INPUT_PULLUP);
  pinMode(BUTTON_PIN_NAV_N, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  iniciarPantalla();
  iniciarSensores();

  preferences.begin("wqstation", false);

  if (!LittleFS.begin(true)) {
    Serial.println(F("Error al montar LittleFS"));
  }

  WiFi.begin();
  macAddress = WiFi.macAddress();
  client.setServer(mqtt_server, 1883);

  mostrarLogoEInicializar();

  if (!client.connected()) {
    esperarDecisionSinInternet();
  }

  if (pantallaActual != PANTALLA_WIFI && !gps.location.isValid() && !client.connected()) {
    mostrarPantallaError();
    esperarReinicio();
  }

  Serial.println(F("Setup completo"));
}

// =====================================================
//  LOOP
// =====================================================
void loop() {
  leerGPS();
  leerSensores();

  reconectarWiFiSiHaceFalta();

  if (!client.connected()) reconnect();
  client.loop();

  manejarBotones();
  manejarBotonNav();

  switch (pantallaActual) {
    case PANTALLA_DATOS:
      actualizarPantalla();
      break;
    case PANTALLA_PENDIENTES:
      actualizarPantallaPendientes();
      break;
    case PANTALLA_WIFI:
      wifiManager.process();
      break;
  }

  if (client.connected() && millis() - ultimoDrenado >= DRENADO_INTERVALO) {
    ultimoDrenado = millis();
    enviarPendientesEncolados();
  }

  if (millis() - previousMillis >= interval) {
    previousMillis = millis();

    if (activo) {
      if (client.connected()) {
        enviarDatos();
      } else {
        guardarLecturaEnCola();
      }
    }
  }
}

// =====================================================
//  INICIALIZACIÓN
// =====================================================
void iniciarPantalla() {
  // SH1106: begin(dirección I2C, reset_via_software)
  if (!display.begin(OLED_ADDR, true)) {
    Serial.println(F("SH1106 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
}

void dibujarPantallaLogo(int segundosRestantes) {
  display.clearDisplay();

  int logoX = (SCREEN_WIDTH - LOGO_WIDTH) / 2;
  display.drawBitmap(logoX, 0, logo_bitmap, LOGO_WIDTH, LOGO_HEIGHT, SH110X_WHITE);

  display.setTextSize(1);
  int16_t x1, y1;
  uint16_t textW, textH;
  display.getTextBounds(APP_SITE, 0, 0, &x1, &y1, &textW, &textH);
  display.setCursor((SCREEN_WIDTH - textW) / 2, 48);
  display.print(APP_SITE);

  String textoInicio = "Iniciando (" + String(segundosRestantes) + "s)";
  display.getTextBounds(textoInicio, 0, 0, &x1, &y1, &textW, &textH);
  display.setCursor((SCREEN_WIDTH - textW) / 2, 56);
  display.print(textoInicio);

  display.display();
}

// Se ejecuta automáticamente cuando WiFiManager no logra conectarse a una
// red guardada y abre su propio punto de acceso para que el usuario lo
// configure. Muestra en pantalla cómo conectarse mientras se espera.
void mostrarInstruccionesWiFi(WiFiManager* wm) {
  IPAddress apIP = WiFi.softAPIP();

  display.clearDisplay();

  int16_t x1, y1;
  uint16_t textW, textH;
  display.getTextBounds(APP_TITLE, 0, 0, &x1, &y1, &textW, &textH);
  display.setCursor((SCREEN_WIDTH - textW) / 2, 0);
  display.println(APP_TITLE);

  String lineaConectate = "1.Con";
  lineaConectate += TILDE_E;
  lineaConectate += "ctate a la red:";
  display.setCursor(0, 12);
  display.println(lineaConectate);
  display.setCursor(0, 22);
  display.println(wm->getConfigPortalSSID());
  display.setCursor(0, 33);
  display.print(F("2.Entra a:"));
  display.println(apIP);
  display.setCursor(0, 44);
  display.println(F("3.Ingresa los datos"));
  display.setCursor(0, 54);
  display.println(F("   de tu red"));

  display.display();
}

void iniciarSensores() {
  if (!aht.begin()) {
    Serial.println(F("No AHT10 detected"));
    while (1);
  }
  sensors.begin();            // DS18B20
  sensors.setWaitForConversion(false);
  tiempoConversionDS18B20 = sensors.millisToWaitForConversion();
}

void mostrarLogoEInicializar() {
  unsigned long inicio = millis();
  int ultimoSegundoMostrado = -1;

  while (true) {
    unsigned long transcurrido = millis() - inicio;

    leerGPS();
    reconnect();
    client.loop();

    int segundosRestantes = (TIEMPO_MAX_INICIALIZACION - transcurrido) / 1000 + 1;
    if (segundosRestantes != ultimoSegundoMostrado) {
      dibujarPantallaLogo(segundosRestantes);
      ultimoSegundoMostrado = segundosRestantes;
    }

    bool listo = gps.location.isValid() && client.connected();
    if (listo && transcurrido >= TIEMPO_MIN_LOGO) {
      return;
    }
    if (transcurrido >= TIEMPO_MAX_INICIALIZACION) {
      return;
    }

    delay(10);
  }
}

void mostrarPantallaError() {
  display.clearDisplay();
  display.setTextSize(1);

  int16_t x1, y1;
  uint16_t textW, textH;
  display.getTextBounds(APP_TITLE, 0, 0, &x1, &y1, &textW, &textH);
  display.setCursor((SCREEN_WIDTH - textW) / 2, 0);
  display.println(APP_TITLE);

  String lineaConexion = "GPS ni conexi";
  lineaConexion += TILDE_O;
  lineaConexion += "n.";

  String lineaBoton = "bot";
  lineaBoton += TILDE_O;
  lineaBoton += "n para reiniciar";

  display.setCursor(0, 20);
  display.println(F("No se pudo obtener"));
  display.setCursor(0, 30);
  display.println(lineaConexion);
  display.setCursor(0, 45);
  display.println(F("Presione cualquier"));
  display.setCursor(0, 54);
  display.println(lineaBoton);

  display.display();
}

void esperarReinicio() {
  while (true) {
    if (digitalRead(BUTTON_PIN_START_STOP_T) == LOW || digitalRead(BUTTON_PIN_NAV_N) == LOW) {
      ESP.restart();
    }
    delay(10);
  }
}

void mostrarPantallaSinInternet() {
  display.clearDisplay();
  display.setTextSize(1);

  int16_t x1, y1;
  uint16_t textW, textH;
  display.getTextBounds(APP_TITLE, 0, 0, &x1, &y1, &textW, &textH);
  display.setCursor((SCREEN_WIDTH - textW) / 2, 0);
  display.println(APP_TITLE);

  display.setCursor(0, 12);
  display.println(F("Presiona \"N\" para"));
  display.setCursor(0, 20);
  display.println(F("continuar sin"));
  display.setCursor(0, 28);
  display.println(F("internet."));

  String lineaConexion = "conexi";
  lineaConexion += TILDE_O;
  lineaConexion += "n.";

  display.setCursor(0, 40);
  display.println(F("Presiona \"T\" para ver"));
  display.setCursor(0, 48);
  display.println(F("instrucciones de"));
  display.setCursor(0, 56);
  display.println(lineaConexion);

  display.display();
}

void esperarDecisionSinInternet() {
  mostrarPantallaSinInternet();

  while (true) {
    leerGPS();

    if (digitalRead(BUTTON_PIN_NAV_N) == LOW) {
      delay(500);
      return;
    }

    if (digitalRead(BUTTON_PIN_START_STOP_T) == LOW) {
      delay(500);
      abrirPortalWifi();
      return;
    }

    delay(10);
  }
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

  if (!solicitudTempPendiente) {
    sensors.requestTemperatures();
    ultimaSolicitudTemp = millis();
    solicitudTempPendiente = true;
  } else if (millis() - ultimaSolicitudTemp >= tiempoConversionDS18B20) {
    float lectura = sensors.getTempCByIndex(0);
    if (lectura != DEVICE_DISCONNECTED_C) {
      datos.tempAgua = lectura;
    }
    solicitudTempPendiente = false;
  }

  float phLectura = leerPH();
  if (phLectura >= PH_MIN_VALIDO && phLectura <= PH_MAX_VALIDO) {
    datos.ph = phLectura;
  }
}

float leerPH() {
  int adcValue = analogRead(ANALOG_PIN);
  float phValue;

  // Interpolación lineal por tramos entre puntos de calibración
  if (adcValue >= ADC_PH7) {
    phValue = 4.00 + (7.00 - 4.00) * ((float)(adcValue - ADC_PH4) / (ADC_PH7 - ADC_PH4));
  } else {
    phValue = 7.00 + (9.18 - 7.00) * ((float)(adcValue - ADC_PH7) / (ADC_PH9_18 - ADC_PH7));
  }

  Serial.printf("[PH] adcValue=%d  phCalculado=%.2f\n", adcValue, phValue);

  return phValue;
}

// =====================================================
//  PANTALLA
// =====================================================
String construirTituloEstado() {
  String tituloEstado = String(APP_TITLE);

  if (activo) {
    if (client.connected()) {
      tituloEstado += " ";
      tituloEstado += char(24);
      tituloEstado += " WEB";
    } else {
      tituloEstado += " ";
      tituloEstado += char(25);
      tituloEstado += " MEM";
    }
  } else {
    tituloEstado += " PAUSA";
  }

  return tituloEstado;
}

void dibujarEncabezado() {
  String tituloEstado = construirTituloEstado();

  int16_t x1, y1;
  uint16_t textW, textH;
  display.getTextBounds(tituloEstado, 0, 0, &x1, &y1, &textW, &textH);
  display.setCursor((SCREEN_WIDTH - textW) / 2, 0);
  display.print(tituloEstado);

  display.drawFastHLine(0, 9, SCREEN_WIDTH, SH110X_WHITE);
}

void actualizarPantalla() {
  display.clearDisplay();

  dibujarEncabezado();

  display.setCursor(0, 12);
  display.print(F("GPS lat: "));
  display.print(gps.location.lat(), 4);

  display.setCursor(0, 20);
  display.print(F("GPS long: "));
  display.print(gps.location.lng(), 4);

  display.setCursor(0, 28);
  display.print(F("Temp Amb: "));
  display.print(datos.tempAmb);
  display.print(F(" C"));

  display.setCursor(0, 36);
  display.print(F("Hum Amb: "));
  display.print(datos.humAmb);
  display.print(F(" %"));

  display.setCursor(0, 44);
  display.print(F("Temp Agua: "));
  display.print(datos.tempAgua);
  display.print(F(" C"));

  display.setCursor(0, 52);
  display.print(F("PH: "));
  display.print(datos.ph);

  display.display();
}

void actualizarPantallaPendientes() {
  display.clearDisplay();

  dibujarEncabezado();

  int pendientes = contarPendientes();

  display.setCursor(0, 20);
  if (pendientes > 0) {
    display.print(F("Pendientes: "));
    display.print(pendientes);
  } else if (activo && client.connected()) {
    String lineaTransmision = "Transmisi";
    lineaTransmision += TILDE_O;
    lineaTransmision += "n de";
    display.println(lineaTransmision);
    display.setCursor(0, 28);
    display.print(F("datos activada"));
  } else {
    display.print(F("Sin pendientes"));
  }

  if (colaCercaDeLlenarse()) {
    display.setCursor(0, 40);
    display.print(F("Cola casi llena!"));
  }

  display.display();
}

// =====================================================
//  BOTONES
// =====================================================
void manejarBotones() {
  if (digitalRead(BUTTON_PIN_START_STOP_T) == LOW) {
    activo = !activo;

    if (activo) {
      iniciarNuevaSesion();
      tone(BUZZER_PIN, 1000, 100);
    } else {
      tone(BUZZER_PIN, 500, 100);
    }

    delay(500);                    // Debounce
  }
}

void iniciarNuevaSesion() {
  sessionId = preferences.getUInt("sessionId", 0) + 1;
  preferences.putUInt("sessionId", sessionId);
}

void abrirPortalWifi() {
  pantallaActual = PANTALLA_WIFI;
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.startConfigPortal(AP_SSID);
  mostrarInstruccionesWiFi(&wifiManager);
}

void manejarBotonNav() {
  if (digitalRead(BUTTON_PIN_NAV_N) != LOW) return;

  if (pantallaActual == PANTALLA_DATOS) {
    pantallaActual = PANTALLA_PENDIENTES;
  } else if (pantallaActual == PANTALLA_PENDIENTES) {
    if (client.connected()) {
      pantallaActual = PANTALLA_DATOS;
    } else {
      abrirPortalWifi();
    }
  } else {
    wifiManager.stopConfigPortal();
    pantallaActual = PANTALLA_DATOS;
  }

  tone(BUZZER_PIN, 1500, 100);
  delay(500);                    // Debounce
}

// =====================================================
//  COLA DE PENDIENTES (LittleFS)
// =====================================================
bool hayEspacioEnCola() {
  size_t libre = LittleFS.totalBytes() - LittleFS.usedBytes();
  return libre > MARGEN_MINIMO_LIBRE;
}

bool colaCercaDeLlenarse() {
  return LittleFS.usedBytes() >= (LittleFS.totalBytes() * UMBRAL_COLA_LLENA);
}

void encolarLectura(const String& csv) {
  if (!hayEspacioEnCola()) {
    Serial.println(F("Cola de pendientes llena, no se guarda la lectura"));
    return;
  }

  File archivo = LittleFS.open(ARCHIVO_PENDIENTES, "a");
  if (!archivo) {
    Serial.println(F("No se pudo abrir la cola de pendientes para escribir"));
    return;
  }
  archivo.println(csv);
  archivo.close();
}

int contarPendientes() {
  File archivo = LittleFS.open(ARCHIVO_PENDIENTES, "r");
  if (!archivo) return 0;

  int contador = 0;
  while (archivo.available()) {
    if (archivo.readStringUntil('\n').length() > 0) contador++;
  }
  archivo.close();
  return contador;
}

bool enviarPendientesEncolados() {
  File origen = LittleFS.open(ARCHIVO_PENDIENTES, "r");
  if (!origen) return false;

  String primeraLinea = origen.readStringUntil('\n');
  if (primeraLinea.length() == 0) {
    origen.close();
    return false;
  }

  if (!client.publish(topic, primeraLinea.c_str())) {
    origen.close();
    return false;
  }

  File temporal = LittleFS.open(ARCHIVO_PENDIENTES_TMP, "w");
  if (!temporal) {
    origen.close();
    return false;
  }
  while (origen.available()) {
    temporal.println(origen.readStringUntil('\n'));
  }
  origen.close();
  temporal.close();

  LittleFS.remove(ARCHIVO_PENDIENTES);
  LittleFS.rename(ARCHIVO_PENDIENTES_TMP, ARCHIVO_PENDIENTES);
  return true;
}

// =====================================================
//  MQTT
// =====================================================
const char* MAC_INVALIDA = "00:00:00:00:00:00";

void actualizarMacAddress() {
  if (macAddress == MAC_INVALIDA || macAddress.length() == 0) {
    macAddress = WiFi.macAddress();
  }
}

String construirMensajeCSV() {
  actualizarMacAddress();

  // Timestamp a partir de fecha/hora del GPS
  tmElements_t tm;
  tm.Year   = gps.date.year() - 1970;
  tm.Month  = gps.date.month();
  tm.Day    = gps.date.day();
  tm.Hour   = gps.time.hour();
  tm.Minute = gps.time.minute();
  tm.Second = gps.time.second();
  time_t timestamp = makeTime(tm);

  // CSV: timestamp,lat,lng,alt,sessionId,tAgua,tAmb,hum,ph,mac
  String msg = String(timestamp);
  msg += ",";
  msg += String(gps.location.lat(), 5);    // 5 decimales (~1.1 m de precisión)
  msg += ",";
  msg += String(gps.location.lng(), 5);    // 5 decimales
  msg += ",";
  msg += String(gps.altitude.meters());
  msg += ",";
  msg += String(sessionId);
  msg += ",";
  msg += String(datos.tempAgua);
  msg += ",";
  msg += String(datos.tempAmb);
  msg += ",";
  msg += String(datos.humAmb);
  msg += ",";
  msg += String(datos.ph);
  msg += ",";
  msg += macAddress;

  return msg;
}

void enviarDatos() {
  String msg = construirMensajeCSV();

  if (client.publish(topic, msg.c_str())) {
    Serial.println(F("Mensaje enviado exitosamente."));
  } else {
    Serial.println(F("Error al enviar el mensaje, se guarda en cola"));
    encolarLectura(msg);
  }
}

void guardarLecturaEnCola() {
  String msg = construirMensajeCSV();
  encolarLectura(msg);
}

void reconectarWiFiSiHaceFalta() {
  if (pantallaActual == PANTALLA_WIFI) return;
  if (WiFi.status() == WL_CONNECTED) return;

  unsigned long ahora = millis();
  if (ahora - wifiUltimoIntento < WIFI_REINTENTO_INTERVALO) return;
  wifiUltimoIntento = ahora;

  Serial.println(F("WiFi desconectado, reintentando..."));
  WiFi.reconnect();
}

void reconnect() {
  if (client.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  unsigned long ahora = millis();
  if (ahora - mqttUltimoIntento < MQTT_REINTENTO_INTERVALO) return;
  mqttUltimoIntento = ahora;

  Serial.print(F("Conectando al broker MQTT..."));
  String clientId = "Ducuchu-" + String(random(0xffff), HEX);

  if (client.connect(clientId.c_str())) {
    Serial.println(F("conectado"));
  } else {
    Serial.print(F("falló, rc="));
    Serial.println(client.state());
  }
}
