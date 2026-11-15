// =====================================================================
//  MüllBot v1.4.0  --  e-ink edition (VUSION 2.6" BWR, 152x296)
//  ESP8266 NodeMCU, ядро ESP8266 2.7.x
//
//  Панель выбирается флагом PANEL_BWRY: 0 = VUSION BWR (3 цвета), 1 = SE2266QS0F9 BWRY (4 цвета).
//  Пины (как в рабочем тесте экрана):
//    SCK = D5, MOSI = D7 (аппаратный SPI), BUSY = D6, RES = D4, DC = D0, CS = D8
//    Экранчик (OLED 128x64): SDA = D2, SCL = D1
//  Зарезервировано (флаги ниже, по умолчанию выключено):
//    Сенсорная кнопка -> A0  (читаем по порогу, ТОЛЬКО до 3.3 В!)
//    Диод             -> D3  (включение низким уровнем: анод к 3.3 В через резистор, катод в пин)
//
//  Библиотеки: Adafruit GFX, Adafruit SSD1306, U8g2_for_Adafruit_GFX  (WiFiManager больше не нужен)
//  Serial 115200: команды строкой, ? = список.
// =====================================================================
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <Updater.h>
#include <WiFiUdp.h>
#include <LittleFS.h>
#include <time.h>
#include <math.h>
#include <memory>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <U8g2_for_Adafruit_GFX.h>

// ---------------- Настройки ----------------
#define FW_VERSION      "2.5.5"
#define BUILD_ID        __DATE__ " " __TIME__
#define DEFAULT_TZ      "CET-1CEST,M3.5.0,M10.5.0/3"
#define DEFAULT_NAME    "MüllBot"
#define DEFAULT_LANG    0                 // 0 = Deutsch, 1 = Русский
#define AP_NAME         "MüllBot-Setup"
#define DEFAULT_ROT     1                 // 1 = ландшафт; если вверх ногами: 3 (команда r)
#define PANEL_BWRY      1                 // 0 = VUSION 2.6" BWR (3 цвета), 1 = SE2266QS0F9 BWRY (4 цвета, жёлтый)
#define BWRY_WAIT_MS    24000UL           // 4-цветная панель, если сигнал BUSY не работает: сколько ждать конца обновления (у тебя оно длится ~15-20 с)
#define USE_POWER_OFF   0                 // 1 = слать 0x02 после обновления (проверь на своём экране)
#define BUSY_TIMEOUT    30000UL
#if PANEL_BWRY
#define MIN_RENDER_GAP  60000UL           // 4-цветная панель обновляется ~25 с
#else
#define MIN_RENDER_GAP  20000UL           // не чаще, чем раз в 20 с
#endif
#define CLEAN_HOUR      3                 // раз в сутки в 03:xx чистка от ghosting
#define ACHTUNG_UNTIL   18                // сегодняшний мусор "уходит" после 18:00
#define SERVICE_MS      120000UL          // сервисный экран висит 2 минуты
#define BOOT_WINDOW_MS  120000UL          // окно "оставить сохранённое / настроить заново"

#define ENABLE_BUTTON   1                 // 1 = кнопка включена
#define BUTTON_TYPE     1                 // 0 = сенсорная кнопка на A0, 1 = кнопка FLASH на D3 (GPIO0, замыкает на землю)
#define ENABLE_LED      0                 // 1 = диод на D3 (НЕ вместе с кнопкой FLASH: оба на GPIO0)
#define BUTTON_PIN      A0                // для BUTTON_TYPE 0
#define BUTTON_THRESH   512
const int FLASH_PIN = 0;                  // D3 = GPIO0 = кнопка FLASH на NodeMCU
const int LED_PIN = 0;                    // D3 (GPIO0)
#if PANEL_BWRY
#define MIN_REFRESH_MIN 2                 // минимум минут между обновлениями экрана (4 цвета: обновление ~25 с)
#else
#define MIN_REFRESH_MIN 1
#endif
#define LIGHT_HOLD_MS   3000
#define RESET_HOLD_MS   5000

// ---------------- Пины экрана ----------------
const int BUSY_PIN = 12; // D6
const int RES_PIN  = 2;  // D4
const int DC_PIN   = 16; // D0
const int CS_PIN   = 15; // D8
const int SCK_PIN_BWRY = 14;  // D5 (для bit-bang чтения OTP 4-цветной панели)
const int SDA_PIN_BWRY = 13;  // D7

const int PHYS_W = 152;
const int PHYS_H = 296;
const int PHYS_WIDTH_BYTES = 19;
const uint8_t STABLE_SCAN = 0x03;

const int BAND_ROWS = 37;                       // 296 = 8 полос по 37 строк
#if PANEL_BWRY
const int PHYS_ROW_BYTES = 38;                  // 152 пикселя * 2 бита
#else
const int PHYS_ROW_BYTES = PHYS_WIDTH_BYTES;    // 19 (1 бит на пиксель, два слоя)
#endif
uint8_t planeBuf[PHYS_ROW_BYTES * BAND_ROWS];   // ОДНА ПОЛОСА (0.7 КБ для BWR / 1.4 КБ для BWRY)
bool flipX = false, flipY = false;              // подгонка ориентации (команды x / y)
int curPlane = 0;                               // 0 = чёрный слой, 1 = красный слой
int bandStart = 0;                              // первая физическая строка текущей полосы

#define C_WHITE  0
#define C_BLACK  1
#define C_RED    2
#define C_TRANSP 3   // псевдоцвет: drawPixel его игнорирует (фон букв)
#define C_YELLOW 4   // только для 4-цветной панели; на BWR рисуется красным
#if PANEL_BWRY
#define C_SUN C_YELLOW
#define C_ACC C_YELLOW        // акцентный цвет (жёлтый на 4-цветной, красный на 3-цветной)
#define C_ACC_TXT C_BLACK     // цвет текста на акцентном фоне
#else
#define C_SUN C_RED
#define C_ACC C_RED
#define C_ACC_TXT C_WHITE
#endif

// Геометрия главного экрана (ландшафт 296x152)
const int TOP_H = 22;     // линия под верхней шторкой
const int BOT_Y = 134;    // линия над нижней шторкой
const int DIV_X = 172;    // вертикальный разделитель
const int SEP_Y = 108;    // горизонтальный разделитель слева

// ---------------- Типы ----------------
enum Screen { S_MAIN = 0, S_NEUJAHR, S_SETUP, S_SAVED, S_NOWIFI, S_NOCAL, S_NONTP, S_SERVICE };
enum Banner { B_NONE = 0, B_TODAY, B_TOMORROW, B_BOTH };
enum { FS_XL = 0, FS_L, FS_M, FS_S, FS_XS };

struct Group   { long key; uint8_t cnt; char n[2][26]; };
struct Weather { bool ok; float temp, tmin, tmax; int rain; int code; time_t when; };
struct TzOpt   { const char* label; const char* posix; };

struct ScreenData {
  String dateShort, dateTiny;
  int    bars;
  String dispTime, wxTime, calTime;
  bool   wxShow;
  String city, devName, ip;
  Banner banner;
  bool   done;
  bool   hasNext;
  String n1, n2, dateLine;
  int    days;
  bool   hasLower;
  String lowerLabel, lowerDate, lowerNames;
  bool   wxOk;
  int    temp, tmin, tmax, rain, code;
  int    year;
  String apName, savedSsid, savedCal, savedCity, failedSsid, homeSsid;
  String netName[4];
  int    netRssi[4];
  int    netCount;
};

// ---------------- Шрифты ----------------
// Шрифты U8g2 на ESP8266 занимают ОЗУ, поэтому их мало: 4 латинских + 1 кириллический
const uint8_t* const FONT_LAT[5] = {
  u8g2_font_helvB18_tf, u8g2_font_helvB18_tf, u8g2_font_helvB14_tf, u8g2_font_helvB10_tf, u8g2_font_helvR08_tf
};
const uint8_t* const FONT_CYR[5] = {
  u8g2_font_6x13_t_cyrillic, u8g2_font_6x13_t_cyrillic, u8g2_font_6x13_t_cyrillic, u8g2_font_6x13_t_cyrillic, u8g2_font_6x13_t_cyrillic
};

// ---------------- Экранчик ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 displayOLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledOk = false;
unsigned long oledHoldUntil = 0;
unsigned long oledLastStatus = 0;

// ---------------- Веб ----------------
ESP8266WebServer server(80);
DNSServer dnsServer;
bool webReady = false;
bool portalMode = false;          // точка доступа настройки WiFi активна
String portalNotice = "";
String pendingSsid = "";          // введено в портале, ещё не сохранено в резерв
String pendingPass = "";
String scanSsid[10];             // найденные WiFi-сети (для страницы настройки)
int scanRssi[10];
int scanCount = 0;
String webNotice = "";

// ---------------- Настройки пользователя ----------------
int    g_lang = DEFAULT_LANG;
String deviceName = DEFAULT_NAME;
String tzString = DEFAULT_TZ;
String calUrl = "";
String cityName = "";
double geoLat = 0, geoLon = 0;
bool   geoOk = false;
String geoLabel = "";
String ignoreList = "\n";

String lsel(const __FlashStringHelper* de, const __FlashStringHelper* ru) { return String(g_lang == 1 ? ru : de); }
#define LL(de, ru) lsel(F(de), F(ru))
#define PRF(fmt, ...) Serial.printf_P(PSTR(fmt), ##__VA_ARGS__)

#if ENABLE_BUTTON
#if BUTTON_TYPE == 1
bool buttonIsDown() { return digitalRead(FLASH_PIN) == LOW; }
#else
bool buttonIsDown() { return analogRead(BUTTON_PIN) > BUTTON_THRESH; }
#endif
#endif

// ---------------- Состояние ----------------
time_t calSyncEpoch = 0;
long fakeOffset = 0;
bool trashDoneToday = false;
bool ledMasterEnabled = true;
bool forceRender = false;
Banner curBanner = B_NONE;
Screen curScreen = S_NONTP;
long lastHourKey = -2;
long lastCleanKey = -1;
int  lastYday = -1;
unsigned long lastRenderMs = 0;
unsigned long lastAutoUpdate = 0;
const unsigned long AUTO_UPDATE_INTERVAL = 12UL * 3600UL * 1000UL;
bool blinkState = false;
String lastRenderInfo = "-";
int  renderCount = 0;
unsigned long serviceUntil = 0;
bool bootWindowActive = false;
bool bootNew = false;
int  refreshMin = 2;                         // обновление экрана раз в N минут (по умолчанию 2: тестовый режим, рекомендация 60)
unsigned long previewUntil = 0;              // превью по номерам висит, пока не придёт команда now (или 10 минут)
unsigned long lastWeatherMs = 0;
unsigned long lastWizardActivity = 0;
unsigned long wizConnectStart = 0;
String wizError = "";
bool otaBusy = false;                        // идёт обновление прошивки: экран не трогаем
bool otaOk = false;
String otaErr = "";
String ghRepo = "";                          // GitHub: владелец/репозиторий для кнопки обновления
bool wizNeedNocal = false;
unsigned long wizConnectedAt = 0;           // когда в мастере подключился WiFi (после этого точку доступа выключаем)                   // после подключения WiFi в мастере показать на экране адрес в домашней сети
String savedSsid = "";
int rot = DEFAULT_ROT;

Group gLast, gToday, gNext;
String typeNames[24];
int typeCount = 0;
int evTotal = 0, evShown = 0;

Weather wx = { false, 0, 0, 0, -1, 0, 0 };
bool wxFailed = false;

ScreenData sd;

const TzOpt TZ_OPTS[] = {
  { "Центральная Европа: DE AT CH PL FR IT ES (CET/CEST)", "CET-1CEST,M3.5.0,M10.5.0/3" },
  { "Западная Европа: UK IE PT (GMT/BST)",                 "GMT0BST,M3.5.0/1,M10.5.0" },
  { "Восточная Европа: FI GR RO UA (EET/EEST)",            "EET-2EEST,M3.5.0/3,M10.5.0/4" },
  { "Москва / Минск (UTC+3)",                              "MSK-3" },
  { "Турция (UTC+3)",                                      "<+03>-3" },
  { "UTC",                                                 "UTC0" }
};
const int TZ_OPT_COUNT = sizeof(TZ_OPTS) / sizeof(TZ_OPTS[0]);

// =====================================================================
//  Класс: рисуем сразу в буферы bw/red
// =====================================================================
class EinkGFX : public Adafruit_GFX {
public:
  EinkGFX() : Adafruit_GFX(PHYS_W, PHYS_H) {}

  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (color == C_TRANSP) return;
    if (x < 0 || y < 0 || x >= _width || y >= _height) return;
    int16_t px, py;
    switch (rotation) {
      case 0:  px = x;              py = y;              break;
      case 1:  px = PHYS_W - 1 - y; py = x;              break;
      case 2:  px = PHYS_W - 1 - x; py = PHYS_H - 1 - y; break;
      default: px = y;              py = PHYS_H - 1 - x; break;
    }
    if (flipX) px = PHYS_W - 1 - px;
    if (flipY) py = PHYS_H - 1 - py;
    int row = py - bandStart;
    if (row < 0 || row >= BAND_ROWS) return;      // вне текущей полосы
#if PANEL_BWRY
    int idx = row * PHYS_ROW_BYTES + (px >> 2);
    int sh = 6 - 2 * (px & 3);                    // первый пиксель = старшие биты
    uint8_t v = (color == C_BLACK) ? 0 : (color == C_YELLOW ? 2 : (color == C_RED ? 3 : 1));
    planeBuf[idx] = (planeBuf[idx] & ~(3 << sh)) | (v << sh);
#else
    if (color == C_YELLOW) color = C_RED;
    int idx = row * PHYS_WIDTH_BYTES + (px >> 3);
    uint8_t m = 0x80 >> (px & 7);
    bool on = (curPlane == 0) ? (color == C_BLACK) : (color == C_RED);
    if (on) planeBuf[idx] |= m;
    else    planeBuf[idx] &= ~m;
#endif
  }

  void fillScreen(uint16_t color) override {
#if PANEL_BWRY
    uint8_t v = (color == C_BLACK) ? 0 : (color == C_YELLOW ? 2 : (color == C_RED ? 3 : 1));
    memset(planeBuf, v * 0x55, sizeof(planeBuf));
#else
    if (color == C_YELLOW) color = C_RED;
    bool on = (curPlane == 0) ? (color == C_BLACK) : (color == C_RED);
    memset(planeBuf, on ? 0xFF : 0x00, sizeof(planeBuf));
#endif
  }
};

EinkGFX gfx;
U8G2_FOR_ADAFRUIT_GFX uf;

// =====================================================================
//  Время и даты
// =====================================================================
bool timeValid() { return time(nullptr) > 1767225600L; }   // >= 01.01.2026
time_t nowT() { return time(nullptr) + fakeOffset; }

long daysFromCivil(int y, int m, int d) {
  y -= (m <= 2);
  long era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097L + (long)doe - 719468L;
}
long keyDays(long key) { return daysFromCivil((int)(key / 10000), (int)((key / 100) % 100), (int)(key % 100)); }
int  keyWeekday(long key) { long w = (keyDays(key) + 4) % 7; if (w < 0) w += 7; return (int)w; }
long dayDiffKeys(long a, long b) { return keyDays(b) - keyDays(a); }
long keyOfTm(const struct tm& t) { return (long)(t.tm_year + 1900) * 10000L + (t.tm_mon + 1) * 100L + t.tm_mday; }

const char* wdShort(int w) {
  static const char* const de[7] = { "So", "Mo", "Di", "Mi", "Do", "Fr", "Sa" };
  static const char* const ru[7] = { "Вс", "Пн", "Вт", "Ср", "Чт", "Пт", "Сб" };
  return (g_lang == 1 ? ru : de)[w % 7];
}
const char* wdLong(int w) {
  static const char* const de[7] = { "Sonntag", "Montag", "Dienstag", "Mittwoch", "Donnerstag", "Freitag", "Samstag" };
  static const char* const ru[7] = { "Воскресенье", "Понедельник", "Вторник", "Среда", "Четверг", "Пятница", "Суббота" };
  return (g_lang == 1 ? ru : de)[w % 7];
}

String fmtDate(long key, bool longWd) {
  char b[48];
  snprintf(b, sizeof(b), "%s, %d.%d.%d", longWd ? wdLong(keyWeekday(key)) : wdShort(keyWeekday(key)),
           (int)(key % 100), (int)((key / 100) % 100), (int)(key / 10000));
  return String(b);
}
String fmtShort(long key) {
  char b[32];
  snprintf(b, sizeof(b), "%s %d.%d.", wdShort(keyWeekday(key)), (int)(key % 100), (int)((key / 100) % 100));
  return String(b);
}

// =====================================================================
//  Файлы и строки
// =====================================================================
String readFileRaw(const char* path) {
  if (!LittleFS.exists(path)) return "";
  File f = LittleFS.open(path, "r");
  if (!f) return "";
  String s = f.readString();
  f.close();
  return s;
}
String readFile(const char* path) { String s = readFileRaw(path); s.trim(); return s; }
void writeFile(const char* path, const String& s) {
  File f = LittleFS.open(path, "w");
  if (f) { f.print(s); f.close(); }
}

String esc(const String& s) {
  String o;
  o.reserve(s.length() + 8);
  for (unsigned i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') o += "&amp;";
    else if (c == '<') o += "&lt;";
    else if (c == '>') o += "&gt;";
    else if (c == '"') o += "&quot;";
    else o += c;
  }
  return o;
}

String urlEncode(const String& s) {
  String o;
  char b[4];
  for (unsigned i = 0; i < s.length(); i++) {
    uint8_t ch = (uint8_t)s[i];
    if (isalnum(ch) || ch == '-' || ch == '_' || ch == '.' || ch == '~') o += (char)ch;
    else { snprintf(b, sizeof(b), "%%%02X", ch); o += b; }
  }
  return o;
}

String hostOf(const String& url) {
  int s = url.indexOf("://");
  s = (s < 0) ? 0 : s + 3;
  int e = url.indexOf('/', s);
  String h = (e < 0) ? url.substring(s) : url.substring(s, e);
  int c = h.indexOf(':');
  if (c >= 0) h = h.substring(0, c);
  return h;
}

bool isIgnored(const String& name) { return ignoreList.indexOf("\n" + name + "\n") >= 0; }

// =====================================================================
//  Экранчик
// =====================================================================
void oledDraw(const String& l0, const String& l1, const String& l2, const String& l3, const String& l4) {
  if (!oledOk) return;
  displayOLED.clearDisplay();
  displayOLED.setTextSize(1);
  displayOLED.setTextColor(SSD1306_WHITE);
  displayOLED.setCursor(0, 0);  displayOLED.print(l0);
  displayOLED.setCursor(0, 13); displayOLED.print(l1);
  displayOLED.setCursor(0, 26); displayOLED.print(l2);
  displayOLED.setCursor(0, 39); displayOLED.print(l3);
  displayOLED.setCursor(0, 52); displayOLED.print(l4);
  displayOLED.display();
}

void oledMsg(const String& a, const String& b) {
  Serial.println("[msg] " + a + " | " + b + "   [heap " + String(ESP.getFreeHeap()) + "]");
  oledDraw("MuellBot e-ink v" FW_VERSION, a, b, "", "");
  oledHoldUntil = millis() + 4000;
}

void oledStatus() {
  if (!oledOk) return;
  if (millis() < oledHoldUntil) return;
  if (millis() - oledLastStatus < 1000) return;
  oledLastStatus = millis();

  char b1[32], b3[32];
  if (timeValid()) {
    time_t now = nowT();
    struct tm t = *localtime(&now);
    snprintf(b1, sizeof(b1), "%02d:%02d:%02d %02d.%02d.%02d", t.tm_hour, t.tm_min, t.tm_sec, t.tm_mday, t.tm_mon + 1, t.tm_year % 100);
  } else {
    snprintf(b1, sizeof(b1), "no time (NTP...)");
  }
  if (WiFi.status() == WL_CONNECTED) snprintf(b3, sizeof(b3), "WiFi %ld dBm", (long)WiFi.RSSI());
  else snprintf(b3, sizeof(b3), "WiFi LOST");

  String ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("-");
  oledDraw("MuellBot e-ink v" FW_VERSION, String(b1), "IP " + ip, String(b3), "Scr: " + lastRenderInfo);
}

// =====================================================================
//  Драйвер панели
// =====================================================================
#if !PANEL_BWRY
bool waitBusy() {
  unsigned long st = millis();
  while (digitalRead(BUSY_PIN) == HIGH) {
    if (millis() - st > BUSY_TIMEOUT) return false;
    if (webReady) server.handleClient();
    yield();
  }
  return true;
}

void sendCommand(uint8_t command) {
  digitalWrite(DC_PIN, LOW);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(command);
  digitalWrite(CS_PIN, HIGH);
}

void sendData(uint8_t data) {
  digitalWrite(DC_PIN, HIGH);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(data);
  digitalWrite(CS_PIN, HIGH);
}

void initPanel() {
  digitalWrite(RES_PIN, LOW);
  delay(10);
  digitalWrite(RES_PIN, HIGH);
  delay(100);
  waitBusy();

  sendCommand(0x01);
  sendData((PHYS_H - 1) & 0xFF);
  sendData(((PHYS_H - 1) >> 8) & 0xFF);
  sendData(STABLE_SCAN);

  sendCommand(0x00);
  sendData(0xCF);
  sendData(0x89);

  sendCommand(0x44);
  sendData(0x00);
  sendData(PHYS_WIDTH_BYTES - 1);

  sendCommand(0x45);
  sendData(0x00);
  sendData(0x00);
  sendData((PHYS_H - 1) & 0xFF);
  sendData(((PHYS_H - 1) >> 8) & 0xFF);

  sendCommand(0x4E);
  sendData(0x00);
  sendCommand(0x4F);
  sendData(0x00);
  sendData(0x00);

  sendCommand(0x04);
  waitBusy();
}

void sendBand() {
  for (int i = 0; i < PHYS_WIDTH_BYTES * BAND_ROWS; i++) sendData(planeBuf[i]);
}

void finishRefresh(unsigned long t0) {
  sendCommand(0x12);
  bool ok = waitBusy();
#if USE_POWER_OFF
  sendCommand(0x02);
  waitBusy();
#endif
  PRF("[panel] refresh %lu ms%s\n", millis() - t0, ok ? "" : " (BUSY TIMEOUT!)");
}

// нарисовать экран и отправить оба слоя (0x10 = чёрный, 0x13 = красный)
void pushScreen(Screen id) {
  unsigned long t0 = millis();
  initPanel();
  for (int p = 0; p < 2; p++) {
    curPlane = p;
    sendCommand(p == 0 ? 0x10 : 0x13);
    for (int b = 0; b < PHYS_H / BAND_ROWS; b++) {   // экран рисуется заново для каждой полосы
      bandStart = b * BAND_ROWS;
      drawScreen(id);
      sendBand();
      yield();
    }
  }
  bandStart = 0;
  finishRefresh(t0);
}

// залить весь экран одним цветом
void pushSolid(uint16_t color) {
  unsigned long t0 = millis();
  initPanel();
  for (int p = 0; p < 2; p++) {
    curPlane = p;
    bandStart = 0;
    gfx.fillScreen(color);
    sendCommand(p == 0 ? 0x10 : 0x13);
    for (int b = 0; b < PHYS_H / BAND_ROWS; b++) sendBand();
  }
  finishRefresh(t0);
}
#else
// ---------- PANEL_BWRY: SE2266QS0F9 (Pervasive Spectra 4, 2.66", 152x296, 2 бита на пиксель) ----------
// Параметры инициализации берутся из OTP самой панели (читается 3-проводным bit-bang один раз).
// BUSY здесь обратный: HIGH = готова.
uint8_t otp[48];
bool otpOk = false;
int otpSample = -1;          // -1 = подобрать само
int panelTempC = 25;

bool busyStuck = false;      // BUSY не поднимается: дальше ждём фиксированное время вслепую

int busyFails = 0;

bool waitBusyHigh(unsigned long timeoutMs) {
  if (busyStuck && timeoutMs <= 5000UL) { delay(300); return true; }
  unsigned long t0 = millis();
  while (digitalRead(BUSY_PIN) == LOW) {
    if (millis() - t0 > timeoutMs) {
      if (timeoutMs <= 5000UL && ++busyFails >= 2) {
        busyStuck = true;
        PRF("[bwry] BUSY не работает (2 таймаута подряд): дальше использую фиксированные паузы\n");
      }
      return false;
    }
    if (webReady) { server.handleClient(); if (portalMode) dnsServer.processNextRequest(); }
    delay(1);
  }
  return true;
}

void spiStart() {
  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
}

void writeReg(uint8_t reg, const uint8_t* d, int n) {
  digitalWrite(DC_PIN, LOW);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg);
  digitalWrite(CS_PIN, HIGH);
  if (n > 0) {
    digitalWrite(DC_PIN, HIGH);
    digitalWrite(CS_PIN, LOW);
    for (int i = 0; i < n; i++) SPI.transfer(d[i]);
    digitalWrite(CS_PIN, HIGH);
  }
}
void writeReg1(uint8_t reg, uint8_t v) { writeReg(reg, &v, 1); }

void cogReset() {
  digitalWrite(RES_PIN, HIGH);
  delay(5);
  digitalWrite(RES_PIN, LOW);  delay(20);
  digitalWrite(RES_PIN, HIGH); delay(10);
  digitalWrite(RES_PIN, LOW);  delay(20);
  digitalWrite(RES_PIN, HIGH); delay(10);
  if (!waitBusyHigh(3000)) Serial.println(F("[bwry] BUSY не стал HIGH после сброса"));
  delay(10);
}

void bbWrite(uint8_t v) {
  pinMode(SDA_PIN_BWRY, OUTPUT);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SDA_PIN_BWRY, (v >> i) & 1);
    delayMicroseconds(2);
    digitalWrite(SCK_PIN_BWRY, HIGH);
    delayMicroseconds(3);
    digitalWrite(SCK_PIN_BWRY, LOW);
    delayMicroseconds(2);
  }
}

uint8_t bbRead(int st) {
  pinMode(SDA_PIN_BWRY, INPUT);
  uint8_t v = 0;
  for (int i = 0; i < 8; i++) {
    int b = 0;
    digitalWrite(SCK_PIN_BWRY, HIGH);
    delayMicroseconds(3);
    if (st == 0) b = digitalRead(SDA_PIN_BWRY);
    digitalWrite(SCK_PIN_BWRY, LOW);
    delayMicroseconds(3);
    if (st == 1) b = digitalRead(SDA_PIN_BWRY);
    v = (v << 1) | (b & 1);
  }
  return v;
}

bool readOtpOnce(int st, uint8_t* id, uint8_t* dummy) {
  SPI.end();
  pinMode(SCK_PIN_BWRY, OUTPUT); digitalWrite(SCK_PIN_BWRY, LOW);
  pinMode(SDA_PIN_BWRY, OUTPUT); digitalWrite(SDA_PIN_BWRY, LOW);
  cogReset();
  digitalWrite(CS_PIN, LOW);

  digitalWrite(DC_PIN, LOW);  bbWrite(0x70);
  digitalWrite(DC_PIN, HIGH); id[0] = bbRead(st); id[1] = bbRead(st);

  digitalWrite(DC_PIN, LOW);  bbWrite(0xA4);
  digitalWrite(DC_PIN, HIGH); bbWrite(0x15); bbWrite(0x00); bbWrite(0x01);
  waitBusyHigh(2000);

  digitalWrite(DC_PIN, LOW);  bbWrite(0xA1);
  digitalWrite(DC_PIN, HIGH);
  *dummy = bbRead(st);
  for (int i = 0; i < 48; i++) otp[i] = bbRead(st);

  digitalWrite(RES_PIN, LOW); delay(40);
  digitalWrite(RES_PIN, HIGH); delay(10);
  waitBusyHigh(2000);
  digitalWrite(CS_PIN, HIGH);

  spiStart();
  cogReset();
  return (otp[0] == 0xA5 && otp[11] == 0x04);
}

bool readOtpAuto() {
  uint8_t id[2], dummy;
  int from = 0, to = 1;
  if (otpSample >= 0) { from = otpSample; to = otpSample; }
  for (int k = from; k <= to; k++) {
    PRF("[otp] чтение, вариант %d\n", k);
    bool ok = readOtpOnce(k, id, &dummy);
    PRF("  chip ID: %02X %02X, dummy %02X, OTP[0]=%02X OTP[11]=%02X\n", id[0], id[1], dummy, otp[0], otp[11]);
    if (ok) { otpSample = k; otpOk = true; return true; }
  }
  otpOk = false;
  return false;
}

void cogInit() {
  writeReg1(0xE0, 0x02);
  writeReg1(0xE6, (uint8_t)panelTempC);
  writeReg(0xA5, nullptr, 0);
  waitBusyHigh(3000);
  writeReg1(0x01, otp[16]);
  writeReg(0x00, &otp[17], 2);
  writeReg(0x03, &otp[30], 3);
  writeReg(0x06, &otp[23], 7);
  writeReg1(0x50, otp[39]);
  writeReg(0x60, &otp[40], 2);
  writeReg(0x61, &otp[19], 4);
  writeReg1(0xE7, otp[33]);
  writeReg1(0xE3, otp[42]);
  writeReg1(0x4D, otp[43]);
  writeReg1(0xB4, otp[44]);
  writeReg1(0xB5, otp[45]);
  writeReg1(0xE9, 0x01);
  writeReg1(0x30, 0x08);
}

void panelUpdate(unsigned long t0) {
  writeReg(0x04, nullptr, 0);
  waitBusyHigh(3000);
  writeReg1(0x12, 0x00);
  bool ok = true;
  if (busyStuck) {                                // BUSY не работает: ждём фиксированное время, но следим за уровнем
    unsigned long w0 = millis();
    int last = digitalRead(BUSY_PIN);
    PRF("[bwry] ожидание обновления, BUSY=%d, максимум %lu с\n", last, BWRY_WAIT_MS / 1000UL);
    while (millis() - w0 < BWRY_WAIT_MS) {
      int lvl = digitalRead(BUSY_PIN);
      if (lvl != last) {
        PRF("[bwry] BUSY -> %d через %lu мс\n", lvl, millis() - w0);
        last = lvl;
        if (lvl == HIGH && millis() - w0 > 5000UL) break;      // сигнал всё-таки пришёл: обновление закончено
      }
      if (webReady) { server.handleClient(); if (portalMode) dnsServer.processNextRequest(); }
      delay(5);
    }
  } else {
    ok = waitBusyHigh(60000);
    if (!ok) {
      busyStuck = true;
      PRF("[bwry] BUSY не поднялся за 60 с: считаю сигнал неисправным, дальше ждём фиксированное время\n");
    }
  }
  writeReg1(0x02, 0x00);
  waitBusyHigh(3000);
  PRF("[panel] refresh %lu ms%s\n", millis() - t0, ok ? "" : " (BUSY TIMEOUT!)");
}

void imageBegin() {
  digitalWrite(DC_PIN, LOW);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x10);
  digitalWrite(CS_PIN, HIGH);
  digitalWrite(DC_PIN, HIGH);
  digitalWrite(CS_PIN, LOW);
}
void imageEnd() { digitalWrite(CS_PIN, HIGH); bandStart = 0; }
void sendBandBytes() { for (unsigned i = 0; i < sizeof(planeBuf); i++) SPI.transfer(planeBuf[i]); }

void printOtpDump() {
  PRF("[otp] ok=%d, вариант=%d, chip-проверка: OTP[0]=%02X (A5), OTP[11]=%02X (04)\n", otpOk ? 1 : 0, otpSample, otp[0], otp[11]);
  Serial.print(F("  OTP:"));
  for (int i = 0; i < 48; i++) {
    if (i % 16 == 0) Serial.print(F("\n   "));
    PRF(" %02X", otp[i]);
  }
  Serial.println();
}

bool panelReady() {
  if (otpOk) return true;
  if (readOtpAuto()) return true;
  lastRenderInfo = "OTP FAIL";
  oledMsg("OTP read failed", "see Serial");
  PRF("[bwry] экран НЕ обновлён: OTP панели не прочитан. Команда  o  покажет подробности\n");
  return false;
}

void pushScreen(Screen id) {
  if (!panelReady()) return;
  unsigned long t0 = millis();
  PRF("[bwry] обновление экрана, подожди ~25 с...\n");
  cogReset();
  cogInit();
  imageBegin();
  for (int b = 0; b < PHYS_H / BAND_ROWS; b++) {
    bandStart = b * BAND_ROWS;
    drawScreen(id);
    sendBandBytes();
    yield();
  }
  imageEnd();
  panelUpdate(t0);
}

void pushSolid(uint16_t color) {
  if (!panelReady()) return;
  unsigned long t0 = millis();
  cogReset();
  cogInit();
  imageBegin();
  bandStart = 0;
  gfx.fillScreen(color);
  for (int b = 0; b < PHYS_H / BAND_ROWS; b++) sendBandBytes();
  imageEnd();
  panelUpdate(t0);
}

#endif

// =====================================================================
//  Текст: шрифт выбирается по содержимому (кириллица / латиница)
// =====================================================================
bool hasCyr(const char* s) {
  for (; *s; s++) {
    uint8_t c = (uint8_t)*s;
    if (c == 0xD0 || c == 0xD1) return true;
  }
  return false;
}

const uint8_t* fontFor(int fs, const String& s) {
  return hasCyr(s.c_str()) ? FONT_CYR[fs] : FONT_LAT[fs];
}

int textW(const String& s, int fs) {
  uf.setFont(fontFor(fs, s));
  return uf.getUTF8Width(s.c_str());
}

void drawText(int x, int y, const String& s, int fs, uint16_t col) {
  uf.setFont(fontFor(fs, s));
  uf.setForegroundColor(col);
  uf.setBackgroundColor(C_TRANSP);
  uf.drawUTF8(x, y, s.c_str());
}

void drawTextC(int cx, int y, const String& s, int fs, uint16_t col) {
  drawText(cx - textW(s, fs) / 2, y, s, fs, col);
}

void drawTextR(int rx, int y, const String& s, int fs, uint16_t col) {
  drawText(rx - textW(s, fs), y, s, fs, col);
}

String clipStr(String s, int fs, int maxW) {
  if (textW(s, fs) <= maxW) return s;
  while (s.length() > 1) {
    int n = s.length() - 1;
    while (n > 0 && (((uint8_t)s[n]) & 0xC0) == 0x80) n--;
    s.remove(n);
    String t = s + ".";
    if (textW(t, fs) <= maxW) return t;
  }
  return s;
}

int fitFs(const String& s, int maxW, int startFs) {
  int f = startFs;
  while (f < FS_XS && textW(s, f) > maxW) f++;
  return f;
}

void drawFitL(int x, int y, const String& s, int maxW, int startFs, uint16_t col) {
  int f = fitFs(s, maxW, startFs);
  drawText(x, y, clipStr(s, f, maxW), f, col);
}

void drawFitC(int cx, int y, const String& s, int maxW, int startFs, uint16_t col) {
  int f = fitFs(s, maxW, startFs);
  drawTextC(cx, y, clipStr(s, f, maxW), f, col);
}

// несколько кусков в одну строку, у каждого свой шрифт (умлауты рядом с кириллицей)
int partsW(const String* p, int n, int fs) {
  int w = 0;
  for (int i = 0; i < n; i++) w += textW(p[i], fs);
  return w;
}
int fitFsParts(const String* p, int n, int maxW, int startFs) {
  int f = startFs;
  while (f < FS_XS && partsW(p, n, f) > maxW) f++;
  return f;
}
void drawParts(int x, int y, const String* p, int n, int fs, uint16_t col) {
  for (int i = 0; i < n; i++) {
    drawText(x, y, p[i], fs, col);
    x += textW(p[i], fs);
  }
}

// =====================================================================
//  Иконки
// =====================================================================
void drawBin(int x, int y, int s, uint16_t col, uint16_t bg) {   // 16s x 21s
  gfx.fillRect(x + 5 * s, y,         6 * s,  2 * s,  col);
  gfx.fillRect(x + 1 * s, y + 2 * s, 14 * s, 3 * s,  col);
  gfx.fillRect(x + 2 * s, y + 5 * s, 12 * s, 16 * s, col);
  gfx.fillRect(x + 4 * s, y + 7 * s, 8 * s,  12 * s, bg);
  int sw = (s + 1) / 2;
  gfx.fillRect(x + 6 * s, y + 8 * s, sw, 10 * s, col);
  gfx.fillRect(x + 9 * s, y + 8 * s, sw, 10 * s, col);
}

void drawWifi(int x, int y, int bars, int s, uint16_t col) {      // ширина 8s
  for (int i = 0; i < 3; i++) {
    int h  = (i + 1) * 2 * s;
    int bx = x + i * 3 * s;
    int by = y + 6 * s - h;
    if (i < bars) gfx.fillRect(bx, by, 2 * s, h, col);
    else          gfx.drawRect(bx, by, 2 * s, h, col);
  }
}

void drawSun(int cx, int cy, int r, uint16_t col) {
  gfx.fillCircle(cx, cy, r, col);
  for (int i = 0; i < 8; i++) {
    float a = i * 0.785398f;
    int x1 = cx + (int)(cosf(a) * (r + 3));
    int y1 = cy + (int)(sinf(a) * (r + 3));
    int x2 = cx + (int)(cosf(a) * (r + 7));
    int y2 = cy + (int)(sinf(a) * (r + 7));
    gfx.drawLine(x1, y1, x2, y2, col);
    gfx.drawLine(x1 + 1, y1, x2 + 1, y2, col);
  }
}

void drawCloud(int cx, int cy, uint16_t col) {                    // x: cx-22..cx+22, y: cy-19..cy+10
  gfx.fillCircle(cx - 13, cy,     9,  col);
  gfx.fillCircle(cx,      cy - 7, 12, col);
  gfx.fillCircle(cx + 13, cy,     9,  col);
  gfx.fillRect(cx - 13, cy, 27, 10, col);
}

void drawRain(int cx, int cy, uint16_t col) {
  for (int i = -1; i <= 1; i++) {
    gfx.drawLine(cx + i * 10,     cy, cx + i * 10 - 4, cy + 9, col);
    gfx.drawLine(cx + i * 10 + 1, cy, cx + i * 10 - 3, cy + 9, col);
  }
}

void drawSnow(int cx, int cy, uint16_t col) {
  for (int i = -1; i <= 1; i++) {
    int x = cx + i * 11, y = cy + 4;
    gfx.drawLine(x - 3, y, x + 3, y, col);
    gfx.drawLine(x, y - 3, x, y + 3, col);
    gfx.drawLine(x - 2, y - 2, x + 2, y + 2, col);
    gfx.drawLine(x - 2, y + 2, x + 2, y - 2, col);
  }
}

void drawFog(int cx, int cy, uint16_t col) {
  for (int i = 0; i < 3; i++) gfx.fillRect(cx - 18 + i * 3, cy + i * 5, 36 - i * 6, 2, col);
}

void drawBolt(int cx, int cy, uint16_t col) {
  gfx.fillTriangle(cx + 3, cy, cx - 5, cy + 10, cx + 1, cy + 10, col);
  gfx.fillTriangle(cx - 1, cy + 8, cx + 6, cy + 8, cx - 3, cy + 20, col);
}

void drawWeatherIcon(int code, int cx, int cy) {
  if (code == 0) {
    drawSun(cx, cy, 11, C_SUN);
  } else if (code == 1 || code == 2) {
    drawSun(cx - 12, cy - 8, 7, C_SUN);
    drawCloud(cx + 3, cy + 8, C_BLACK);
  } else if (code == 45 || code == 48) {
    drawCloud(cx, cy - 4, C_BLACK);
    drawFog(cx, cy + 10, C_BLACK);
  } else if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82)) {
    drawCloud(cx, cy - 4, C_BLACK);
    drawRain(cx, cy + 10, C_RED);
  } else if ((code >= 71 && code <= 77) || code == 85 || code == 86) {
    drawCloud(cx, cy - 4, C_BLACK);
    drawSnow(cx, cy + 8, C_RED);
  } else if (code >= 95) {
    drawCloud(cx, cy - 6, C_BLACK);
    drawBolt(cx, cy + 6, C_SUN);
  } else {
    drawCloud(cx, cy + 2, C_BLACK);
  }
}

void drawCheckSmall(int cx, int cy, uint16_t col) {
  for (int t = -2; t <= 2; t++) {
    gfx.drawLine(cx - 9, cy + t,     cx - 3, cy + 7 + t, col);
    gfx.drawLine(cx - 3, cy + 7 + t, cx + 10, cy - 7 + t, col);
  }
}

void drawCheckBig(int cx, int cy, uint16_t col) {
  for (int t = -3; t <= 3; t++) {
    gfx.drawLine(cx - 20, cy + t,      cx - 6,  cy + 14 + t, col);
    gfx.drawLine(cx - 6,  cy + 14 + t, cx + 22, cy - 14 + t, col);
  }
}

// мини-иконки нижней шторки
void drawIconDisplay(int x, int y) {
  gfx.drawRect(x, y, 10, 7, C_BLACK);
  gfx.drawFastHLine(x + 3, y + 9, 4, C_BLACK);
  gfx.drawFastVLine(x + 5, y + 7, 3, C_BLACK);
}
void drawIconCloudMini(int x, int y) {
  gfx.fillCircle(x + 4, y + 5, 3, C_BLACK);
  gfx.fillCircle(x + 8, y + 3, 3, C_BLACK);
  gfx.fillCircle(x + 11, y + 5, 3, C_BLACK);
  gfx.fillRect(x + 4, y + 5, 8, 4, C_BLACK);
}
void drawIconCalendar(int x, int y) {
  gfx.drawRect(x, y, 10, 9, C_BLACK);
  gfx.fillRect(x, y, 10, 3, C_BLACK);
  gfx.fillRect(x + 2, y - 1, 1, 2, C_BLACK);
  gfx.fillRect(x + 7, y - 1, 1, 2, C_BLACK);
}

// =====================================================================
//  Календарь
// =====================================================================
void groupClear(Group& g) { g.key = 0; g.cnt = 0; g.n[0][0] = 0; g.n[1][0] = 0; }

void groupAddName(Group& g, const String& name) {
  if (g.cnt >= 2) return;
  if (g.cnt == 1 && name.equals(String(g.n[0]))) return;
  size_t len = name.length();
  if (len > 25) len = 25;
  while (len > 0 && len < name.length() && (((uint8_t)name[len]) & 0xC0) == 0x80) len--;
  memcpy(g.n[g.cnt], name.c_str(), len);
  g.n[g.cnt][len] = 0;
  g.cnt++;
}

String joinNames(const Group& g) {
  String s = String(g.n[0]);
  if (g.cnt > 1) s += ", " + String(g.n[1]);
  return s;
}

long parseIcsDate(String str) {
  int colonIdx = str.indexOf(':');
  if (colonIdx != -1) str = str.substring(colonIdx + 1);
  str.trim();
  if (str.length() < 8) return 0;
  int year = str.substring(0, 4).toInt();
  int mon  = str.substring(4, 6).toInt();
  int day  = str.substring(6, 8).toInt();
  if (year < 2020 || mon < 1 || mon > 12 || day < 1 || day > 31) return 0;
  return (long)year * 10000L + mon * 100L + day;
}

void noteType(const String& name) {
  for (int i = 0; i < typeCount; i++) if (typeNames[i] == name) return;
  if (typeCount < 24) typeNames[typeCount++] = name;
}

void addEvent(long key, const String& name, long todayKey) {
  if (key < todayKey) {
    if (gLast.cnt == 0 || key > gLast.key) { groupClear(gLast); gLast.key = key; }
    if (key == gLast.key) groupAddName(gLast, name);
  } else if (key == todayKey) {
    gToday.key = key;
    groupAddName(gToday, name);
  } else {
    if (gNext.cnt == 0 || key < gNext.key) { groupClear(gNext); gNext.key = key; }
    if (key == gNext.key) groupAddName(gNext, name);
  }
}

void parseCalendar() {
  groupClear(gLast); groupClear(gToday); groupClear(gNext);
  typeCount = 0; evTotal = 0; evShown = 0;
  if (!LittleFS.exists("/calendar.ics")) return;

  File file = LittleFS.open("/calendar.ics", "r");
  if (!file) return;

  long todayKey = 0;
  bool tv = timeValid();
  if (tv) {
    time_t now = nowT();
    struct tm nt = *localtime(&now);
    todayKey = keyOfTm(nt);
  }

  String curSummary = "";
  long curKey = 0;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();

    if (line.startsWith("BEGIN:VEVENT")) {
      curSummary = "";
      curKey = 0;
    } else if (line.startsWith("SUMMARY")) {
      int c = line.indexOf(':');
      if (c != -1) {
        curSummary = line.substring(c + 1);
        curSummary.replace("\\", "");
        curSummary.trim();
      }
    } else if (line.startsWith("DTSTART")) {
      curKey = parseIcsDate(line);
    } else if (line.startsWith("END:VEVENT")) {
      if (curKey > 0 && curSummary.length() > 0) {
        evTotal++;
        noteType(curSummary);
        if (!isIgnored(curSummary)) {
          evShown++;
          if (tv) addEvent(curKey, curSummary, todayKey);
        }
      }
    }
  }
  file.close();
}

void loadCalSyncEpoch() {
  String s = readFile("/last_download.txt");
  calSyncEpoch = (time_t)s.toInt();
}

void saveCalSyncEpoch() {
  if (!timeValid()) return;
  writeFile("/last_download.txt", String((unsigned long)time(nullptr)));
  calSyncEpoch = time(nullptr);
}

// =====================================================================
//  Сеть: HTTP(S), календарь, погода
// =====================================================================
void prepareTls(BearSSL::WiFiClientSecure& c, const String& host) {
  c.setInsecure();
  c.setBufferSizes(512, 512);      // как в рабочей версии 1.3; проба MFLN валила прошивку при малой куче
}

bool guardUsed = false;            // защита от цикла перезагрузок: первый сетевой запрос после старта помечаем файлом
String crashedOp = "";             // если прошлый запуск упал на сетевом запросе: на какой хост
bool skipNet = false;
unsigned long netBlockedUntil = 0;

// url -> файл (path) или строка (body); HTTPS только если хватает памяти
bool httpFetchRaw(String url, const char* path, String* body) {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (url.startsWith("https://") && ESP.getFreeHeap() < 14000) {
    PRF("[http] мало памяти для HTTPS (%u), запрос пропущен\n", ESP.getFreeHeap());
    return false;
  }
  url.trim();
  if (url.startsWith("webcal://")) url = "https://" + url.substring(9);
  bool ok = false;
  bool began = false;

  // ПОРЯДОК ВАЖЕН: клиенты объявлены ДО HTTPClient, иначе при выходе клиент уничтожается раньше,
  // а деструктор HTTPClient ещё обращается к нему (это и валило прошивку: Exception 9/28)
  std::unique_ptr<BearSSL::WiFiClientSecure> tls;
  WiFiClient plain;
  HTTPClient http;
  http.setTimeout(15000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (url.startsWith("https://")) {
    tls.reset(new BearSSL::WiFiClientSecure);
    prepareTls(*tls, hostOf(url));
    began = http.begin(*tls, url);
  } else {
    began = http.begin(plain, url);
  }

  if (began) {
    int code = http.GET();
    if (code == HTTP_CODE_OK) {
      if (path) {
        File f = LittleFS.open(path, "w");
        if (f) { http.writeToStream(&f); f.close(); ok = true; }
      } else if (body) {
        *body = http.getString();
        ok = body->length() > 0;
      }
    } else {
      PRF("[http] %s -> %d\n", url.c_str(), code);
    }
    http.end();
  }
  return ok;
}

bool httpFetch(String url, const char* path, String* body) {
  bool armed = false;
  if (!guardUsed) {                                  // если на этом запросе всё упадёт, при следующем старте он будет пропущен
    guardUsed = true;
    armed = true;
    writeFile("/guard.txt", hostOf(url));
  }
  bool ok = httpFetchRaw(url, path, body);
  if (armed) LittleFS.remove("/guard.txt");
  return ok;
}

// API погоды: сначала обычный HTTP (без TLS, мало памяти), потом HTTPS
bool apiGet(const String& host, const String& pathQuery, String& body) {
  if (httpFetch("http://" + host + pathQuery, nullptr, &body)) return true;
  return httpFetch("https://" + host + pathQuery, nullptr, &body);
}

// Обновить экран сейчас: сбрасывает удержание сервисного экрана и превью и пропускает паузу между обновлениями
void requestRefresh() {
  serviceUntil = 0;
  previewUntil = 0;
  forceRender = true;
  lastRenderMs = 0;
}

bool downloadCalendar() {
  if (calUrl.length() == 0) return false;
  bool ok = httpFetch(calUrl, "/calendar.ics", nullptr);
  if (ok) {
    saveCalSyncEpoch();
    parseCalendar();                                   // сразу разобрать новый файл, не ждать перезагрузки
    if (evTotal == 0) {
      PRF("[cal] файл загружен, но в нём нет событий (не iCal?): удаляю\n");
      LittleFS.remove("/calendar.ics");
      ok = false;
    }
  }
  requestRefresh();
  return ok;
}

bool jsonNum(const String& s, int from, const char* key, double& out) {
  if (from < 0) return false;
  int i = s.indexOf(key, from);
  if (i < 0) return false;
  i += strlen(key);
  while (i < (int)s.length() && (s[i] == ' ' || s[i] == '[')) i++;
  if (s.startsWith("null", i)) return false;
  out = s.substring(i, i + 14).toFloat();
  return true;
}

String jsonStr(const String& s, int from, const char* key) {
  if (from < 0) return "";
  int i = s.indexOf(key, from);
  if (i < 0) return "";
  i += strlen(key);
  int e = i;
  while (e < (int)s.length() && s[e] != '"') {
    if (s[e] == '\\') e++;
    e++;
  }
  return s.substring(i, e);
}

void saveGeo() {
  writeFile("/geo.txt", String(geoLat, 5) + "\n" + String(geoLon, 5) + "\n" + geoLabel);
}

void loadGeo() {
  geoOk = false;
  String g = readFileRaw("/geo.txt");
  int a = g.indexOf('\n');
  if (a < 0) return;
  int b = g.indexOf('\n', a + 1);
  if (b < 0) return;
  geoLat = g.substring(0, a).toFloat();
  geoLon = g.substring(a + 1, b).toFloat();
  geoLabel = g.substring(b + 1);
  geoLabel.trim();
  geoOk = (geoLat != 0 || geoLon != 0);
}

bool geocodeCity(const String& city) {
  String pq = String("/v1/search?name=") + urlEncode(city) +
              "&count=1&language=" + (g_lang == 1 ? "ru" : "de") + "&format=json";
  String body;
  if (!apiGet("geocoding-api.open-meteo.com", pq, body)) return false;
  int r = body.indexOf("\"results\":[");
  if (r < 0) return false;
  double la, lo;
  if (!jsonNum(body, r, "\"latitude\":", la) || !jsonNum(body, r, "\"longitude\":", lo)) return false;
  String nm = jsonStr(body, r, "\"name\":\"");
  String cc = jsonStr(body, r, "\"country_code\":\"");
  geoLat = la;
  geoLon = lo;
  geoLabel = nm;
  if (cc.length()) geoLabel += " (" + cc + ")";
  geoOk = true;
  saveGeo();
  return true;
}

bool fetchWeather() {
  lastWeatherMs = millis();
  if (lastWeatherMs == 0) lastWeatherMs = 1;
  if (!geoOk || WiFi.status() != WL_CONNECTED) return false;
  if (millis() < netBlockedUntil) return false;
  String pq = String("/v1/forecast?latitude=") + String(geoLat, 4) +
               "&longitude=" + String(geoLon, 4) +
               "&current=temperature_2m,weather_code" +
               "&daily=temperature_2m_max,temperature_2m_min,precipitation_probability_max" +
               "&timezone=auto&forecast_days=1";
  String body;
  if (!apiGet("api.open-meteo.com", pq, body)) { wxFailed = true; return false; }

  int c = body.indexOf("\"current\":{");
  int d = body.indexOf("\"daily\":{");
  double t, code, mx, mn, rain;
  if (!jsonNum(body, c, "\"temperature_2m\":", t) || !jsonNum(body, c, "\"weather_code\":", code) ||
      !jsonNum(body, d, "\"temperature_2m_max\":", mx) || !jsonNum(body, d, "\"temperature_2m_min\":", mn)) {
    wxFailed = true;
    return false;
  }
  if (!jsonNum(body, d, "\"precipitation_probability_max\":", rain)) rain = -1;

  wx.temp = t; wx.tmax = mx; wx.tmin = mn;
  wx.code = (int)code;
  wx.rain = (int)rain;
  wx.ok = true;
  wx.when = timeValid() ? time(nullptr) : 0;
  wxFailed = false;
  return true;
}

// =====================================================================
//  Данные для отрисовки
// =====================================================================
String hm(time_t t) {
  struct tm tt = *localtime(&t);
  char b[8];
  snprintf(b, sizeof(b), "%02d:%02d", tt.tm_hour, tt.tm_min);
  return String(b);
}

void fillCommon() {
  bool tv = timeValid();
  time_t now = tv ? nowT() : time(nullptr);
  struct tm t = *localtime(&now);
  char b[48];

  if (tv) {
    snprintf(b, sizeof(b), "%s, %d.%d.%d", wdShort(t.tm_wday), t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);
    sd.dateShort = String(b);
    snprintf(b, sizeof(b), "%s %d.%d.", wdShort(t.tm_wday), t.tm_mday, t.tm_mon + 1);
    sd.dateTiny = String(b);
    sd.dispTime = hm(now);
  } else {
    sd.dateShort = "--";
    sd.dateTiny = "--";
    sd.dispTime = "--:--";
  }

  if (wx.when > 1700000000L) sd.wxTime = hm(wx.when); else sd.wxTime = "--:--";
  if (calSyncEpoch > 1700000000L) {
    time_t ce = calSyncEpoch;
    struct tm ct = *localtime(&ce);
    snprintf(b, sizeof(b), "%d.%d. %02d:%02d", ct.tm_mday, ct.tm_mon + 1, ct.tm_hour, ct.tm_min);
    sd.calTime = String(b);
  } else {
    sd.calTime = "--";
  }

  sd.wxShow = (cityName.length() > 0 && geoOk);
  sd.city = cityName;
  sd.devName = deviceName;

  sd.bars = 0;
  sd.ip = "-";
  if (WiFi.status() == WL_CONNECTED) {
    long rssi = WiFi.RSSI();
    sd.bars = (rssi > -70) ? 3 : (rssi > -85 ? 2 : 1);
    sd.ip = WiFi.localIP().toString();
  }

  sd.banner = B_NONE;
  sd.done = false;
  sd.hasNext = false;
  sd.n1 = ""; sd.n2 = ""; sd.dateLine = "";
  sd.days = 0;
  sd.hasLower = false;
  sd.lowerLabel = ""; sd.lowerDate = ""; sd.lowerNames = "";
  sd.wxOk = false;
  sd.temp = sd.tmin = sd.tmax = 0;
  sd.rain = -1;
  sd.code = 3;
  sd.year = t.tm_year + 1900;
  sd.apName = AP_NAME;
  sd.homeSsid = (WiFi.status() == WL_CONNECTED) ? WiFi.SSID() : String("");
  sd.netCount = 0;
}

void fillWeather() {
  if (wx.ok) {
    sd.wxOk = true;
    sd.temp = (int)lroundf(wx.temp);
    sd.tmin = (int)lroundf(wx.tmin);
    sd.tmax = (int)lroundf(wx.tmax);
    sd.rain = wx.rain;
    sd.code = wx.code;
  }
}

void setNamesFrom(const Group& g) {
  sd.n1 = String(g.n[0]);
  sd.n2 = (g.cnt > 1) ? String(g.n[1]) : String("");
}

void fillMainReal() {
  fillCommon();
  fillWeather();
  if (!timeValid()) return;

  time_t now = nowT();
  struct tm t = *localtime(&now);
  long todayKey = keyOfTm(t);

  bool todayAct  = (gToday.cnt > 0 && t.tm_hour < ACHTUNG_UNTIL);
  bool morgenAct = (gNext.cnt > 0 && dayDiffKeys(todayKey, gNext.key) == 1);

  if (todayAct && morgenAct) sd.banner = B_BOTH;
  else if (todayAct) sd.banner = B_TODAY;
  else if (morgenAct) sd.banner = B_TOMORROW;
  curBanner = sd.banner;
  sd.done = (trashDoneToday && sd.banner != B_NONE);

  // блок "ближайший вывоз": сегодняшний (до 18:00) или следующий
  const Group* nx = todayAct ? &gToday : (gNext.cnt > 0 ? &gNext : nullptr);
  if (nx) {
    sd.hasNext = true;
    setNamesFrom(*nx);
    sd.dateLine = fmtDate(nx->key, false);
    sd.days = (int)dayDiffKeys(todayKey, nx->key);
  }

  // нижняя строка слева
  if (sd.banner == B_BOTH) {
    sd.hasLower = true;
    sd.lowerLabel = LL("Morgen:", "Завтра:");
    sd.lowerDate = "";
    sd.lowerNames = joinNames(gNext);
  } else if (gToday.cnt > 0 && t.tm_hour >= ACHTUNG_UNTIL) {
    sd.hasLower = true;                       // после 18:00 сегодняшний мусор уже "прошедший"
    sd.lowerLabel = LL("Letzter:", "Последний:");
    sd.lowerDate = fmtShort(gToday.key);
    sd.lowerNames = joinNames(gToday);
  } else if (gLast.cnt > 0) {
    sd.hasLower = true;
    sd.lowerLabel = LL("Letzter:", "Последний:");
    sd.lowerDate = fmtShort(gLast.key);
    sd.lowerNames = joinNames(gLast);
  }
}

// образцы для превью по номерам
void fillSampleMain(int n) {
  fillCommon();
  sd.dateShort = LL("So, 25.10.2026", "Вс, 25.10.2026");
  sd.dateTiny = LL("So 25.10.", "Вс 25.10.");
  sd.dispTime = "14:00";
  sd.wxTime = "14:00";
  sd.calTime = "25.10. 06:00";
  sd.wxShow = true;
  sd.city = "Dortmund";
  sd.bars = 3;

  sd.wxOk = true;
  sd.temp = 12; sd.tmin = 8; sd.tmax = 14; sd.rain = 30; sd.code = 2;

  sd.hasNext = true;
  sd.n1 = "Restmüll"; sd.n2 = "";
  sd.dateLine = LL("Do, 29.10.2026", "Чт, 29.10.2026");
  sd.days = 4;
  sd.hasLower = true;
  sd.lowerLabel = LL("Letzter:", "Последний:");
  sd.lowerDate = LL("Mo 19.10.", "Пн 19.10.");
  sd.lowerNames = "Bioabfall, Papier";

  if (n == 3) {                       // сегодня
    sd.banner = B_TODAY; sd.days = 0; sd.dateLine = LL("So, 25.10.2026", "Вс, 25.10.2026");
    sd.code = 61; sd.rain = 80;
  } else if (n == 4) {                // завтра, два вида
    sd.banner = B_TOMORROW; sd.days = 1; sd.n1 = "Bioabfall"; sd.n2 = "Papier";
    sd.dateLine = LL("Mo, 26.10.2026", "Пн, 26.10.2026");
    sd.code = 3;
  } else if (n == 5) {                // сегодня и завтра
    sd.banner = B_BOTH; sd.days = 0; sd.dateLine = LL("So, 25.10.2026", "Вс, 25.10.2026");
    sd.lowerLabel = LL("Morgen:", "Завтра:"); sd.lowerDate = ""; sd.lowerNames = "Bioabfall";
    sd.code = 0; sd.rain = 0;
  } else if (n == 6) {                // вывезено
    sd.banner = B_TOMORROW; sd.done = true; sd.days = 1; sd.n1 = "Bioabfall";
    sd.dateLine = LL("Mo, 26.10.2026", "Пн, 26.10.2026");
    sd.code = 71; sd.temp = -2; sd.tmin = -5; sd.tmax = 1;
  } else if (n == 13) {               // погоды нет
    sd.wxOk = false; sd.wxShow = false; sd.city = "";
  }
}

// =====================================================================
//  Экраны (ландшафт 296x152)
// =====================================================================
String bannerText(Banner b) {
  if (b == B_TODAY)    return String(LL("HEUTE MÜLL!", "СЕГОДНЯ МУСОР!"));
  if (b == B_TOMORROW) return String(LL("MORGEN MÜLL!", "ЗАВТРА МУСОР!"));
  return String(LL("HEUTE + MORGEN MÜLL!", "СЕГОДНЯ + ЗАВТРА МУСОР!"));
}

String badgeText(int days) {
  if (days <= 0) return String(LL("heute", "сегодня"));
  if (days == 1) return String(LL("morgen", "завтра"));
  if (g_lang == 1) return "через " + String(days) + " дн.";
  return "in " + String(days) + " Tagen";
}

void drawBottomBar() {
  int W = gfx.width();
  gfx.drawFastHLine(0, BOT_Y, W, C_BLACK);
  const int ty = 147;
  int x = 6;

  drawIconDisplay(x, 138);
  x += 14;
  drawText(x, ty, sd.dispTime, FS_XS, C_BLACK);
  x += textW(sd.dispTime, FS_XS) + 10;
  if (sd.wxShow) {
    drawIconCloudMini(x, 138);
    x += 17;
    drawText(x, ty, sd.wxTime, FS_XS, C_BLACK);
    x += textW(sd.wxTime, FS_XS) + 10;
  }
  int leftEnd = x;

  int rw = 14 + textW(sd.calTime, FS_XS);
  int rx = W - 6 - rw;
  drawIconCalendar(rx, 138);
  drawText(rx + 14, ty, sd.calTime, FS_XS, C_BLACK);

  if (sd.city.length()) {
    String lab = String("");                          // подпись "Stadt:/Город:" не влезает, показываем только название
    int avail = rx - 8 - leftEnd - 4 - 12;
    String city = sd.city;
    String parts[2] = { lab, city };
    if (partsW(parts, 2, FS_XS) > avail) {
      parts[1] = clipStr(city, FS_XS, avail - textW(lab, FS_XS));
    }
    int w = partsW(parts, 2, FS_XS) + 12;
    int wxPos = W / 2 - w / 2;
    if (wxPos < leftEnd + 4) wxPos = leftEnd + 4;
    if (wxPos + w > rx - 8) wxPos = rx - 8 - w;
#if PANEL_BWRY
    gfx.fillRoundRect(wxPos, 136, w, 15, 3, C_YELLOW);
#endif
    gfx.drawRoundRect(wxPos, 136, w, 15, 3, C_BLACK);
    drawParts(wxPos + 6, ty, parts, 2, FS_XS, C_BLACK);
  }
}

void drawWeatherPanel() {
  int W = gfx.width();
  int cx = (DIV_X + W) / 2;
  int panelW = W - DIV_X - 10;

  if (!sd.wxOk) {
    drawFitC(cx, 72, String(LL("Keine Wetterdaten", "Нет данных о погоде")), panelW, FS_S, C_BLACK);
    if (sd.city.length() == 0) {
      drawFitC(cx, 92, String(LL("Stadt im Webmenü", "Укажи город")), panelW, FS_XS, C_BLACK);
      drawFitC(cx, 104, String(LL("eintragen", "в веб-меню")), panelW, FS_XS, C_BLACK);
    }
    return;
  }

  drawWeatherIcon(sd.code, 205, 52);

  String ts = String(sd.temp) + "°";
  int tw = W - 240 - 4;
  int f = fitFs(ts, tw, FS_XL);
  drawText(240, 58, ts, f, C_BLACK);

  String mm[4] = { String(LL("min ", "мин ")), String(sd.tmin) + "° / ", String(LL("max ", "макс ")), String(sd.tmax) + "°" };
  int f2 = fitFsParts(mm, 4, panelW, FS_S);
  drawParts(cx - partsW(mm, 4, f2) / 2, 88, mm, 4, f2, C_BLACK);

  if (sd.rain >= 0) {
    String rr[2] = { String(LL("Regen ", "Дождь ")), String(sd.rain) + "%" };
    int f3 = fitFsParts(rr, 2, panelW, FS_S);
    drawParts(cx - partsW(rr, 2, f3) / 2, 108, rr, 2, f3, sd.rain >= 60 ? C_RED : C_BLACK);
  }
}

void scrMain() {
  int W = gfx.width();
  gfx.fillScreen(C_WHITE);

  // ---------- верхняя шторка ----------
  bool ban = (sd.banner != B_NONE);
  uint16_t bg = C_RED, fg = C_WHITE;
  if (!ban) { bg = C_WHITE; fg = C_BLACK; }
  else if (sd.banner == B_TOMORROW) { bg = C_ACC; fg = C_ACC_TXT; }
  if (ban) gfx.fillRect(0, 0, W, TOP_H, bg);
#if PANEL_BWRY
  if (sd.banner == B_BOTH) {                      // двухцветная: слева красная "сегодня", справа жёлтая "завтра"
    gfx.fillRect(W / 2, 0, W - W / 2, TOP_H, C_YELLOW);
    drawFitC(W / 4, 16, String(LL("HEUTE MÜLL!", "СЕГОДНЯ МУСОР!")), W / 2 - 12, FS_M, C_WHITE);
    drawFitC(W / 2 + (W / 2 - 14) / 2, 16, String(LL("MORGEN MÜLL!", "ЗАВТРА МУСОР!")), W / 2 - 30, FS_M, C_BLACK);
    drawWifi(W - 14, 9, sd.bars, 1, C_BLACK);
  } else {
#endif
  drawWifi(W - 14, 9, sd.bars, 1, fg);

  String d = ban ? sd.dateTiny : sd.dateShort;
  drawText(6, 16, d, FS_S, fg);
  int x0 = 6 + textW(d, FS_S) + 10;
  int x1 = W - 14 - 10;
  String mid = ban ? bannerText(sd.banner) : sd.devName;
  if (mid.length() && x1 > x0) {
    bool cy = hasCyr(mid.c_str());
    int f = fitFs(mid, x1 - x0, ban ? FS_M : FS_S);
    String c = clipStr(mid, f, x1 - x0);
    int by = cy ? 17 : 16;
    drawText((x0 + x1) / 2 - textW(c, f) / 2, by, c, f, fg);
  }
#if PANEL_BWRY
  }
#endif
  gfx.drawFastHLine(0, TOP_H, W, C_BLACK);

  // ---------- блок мусора (слева) ----------
  drawText(6, 36, String(LL("Nächster Müll:", "Следующий вывоз:")), FS_XS, C_BLACK);
  if (sd.hasNext) {
    if (sd.done) {
      gfx.fillCircle(24, 67, 18, C_RED);
      drawCheckSmall(24, 67, C_WHITE);
    } else {
      drawBin(8, 46, 2, sd.days <= 1 ? C_RED : C_BLACK, C_WHITE);
    }
    int tx = 48;
    int tw = DIV_X - 6 - tx;
    int badgeY;
    if (sd.n2.length() == 0) {
      drawFitL(tx, 62, sd.n1, tw, FS_L, C_BLACK);
      drawFitL(tx, 78, sd.dateLine, tw, FS_S, C_BLACK);
      badgeY = 84;
    } else {
      drawFitL(tx, 54, sd.n1, tw, FS_M, C_BLACK);
      drawFitL(tx, 70, sd.n2, tw, FS_M, C_BLACK);
      drawFitL(tx, 84, sd.dateLine, tw, FS_S, C_BLACK);
      badgeY = 88;
    }
    String bt = badgeText(sd.days);
    int bw = textW(bt, FS_S) + 14;
    if (bw > tw) bw = tw;
    uint16_t bgc = (sd.days <= 1) ? C_RED : C_ACC;
    uint16_t txc = (sd.days <= 1) ? C_WHITE : C_ACC_TXT;
    gfx.fillRoundRect(tx, badgeY, bw, 18, 4, bgc);
    drawText(tx + 7, badgeY + 14, clipStr(bt, FS_S, bw - 10), FS_S, txc);
  } else {
    drawText(6, 66, String(LL("Keiner", "Нет данных")), FS_L, C_BLACK);
  }

  gfx.drawFastHLine(0, SEP_Y, DIV_X + 1, C_BLACK);

  String lab = sd.hasLower ? sd.lowerLabel : String(LL("Letzter:", "Последний:"));
  drawText(6, 124, lab, FS_XS, C_BLACK);
  int lx = 6 + textW(lab, FS_XS) + 5;
  int avail = DIV_X - 4 - lx;
  if (sd.hasLower) {
    String dt = sd.lowerDate.length() ? sd.lowerDate + " " : String("");
    int dw = textW(dt, FS_XS);
    String nm = clipStr(sd.lowerNames, FS_XS, avail - dw);
    drawText(lx, 124, dt, FS_XS, C_BLACK);
    drawText(lx + dw, 124, nm, FS_XS, C_BLACK);
  } else {
    drawText(lx, 124, String(LL("Keiner", "Нет")), FS_XS, C_BLACK);
  }

  // ---------- вертикальный разделитель: от линии до линии ----------
  gfx.drawFastVLine(DIV_X, TOP_H, BOT_Y - TOP_H + 1, C_BLACK);

  drawWeatherPanel();
  drawBottomBar();
}

void scrNeujahr() {
  int W = gfx.width(), H = gfx.height();
  gfx.fillScreen(C_BLACK);
  gfx.drawRect(4, 4, W - 8, H - 8, C_RED);
  gfx.drawRect(6, 6, W - 12, H - 12, C_RED);
  drawTextC(W / 2, 62, String(LL("Frohes Neues!", "С Новым годом!")), FS_XL, C_WHITE);
  String yr = (g_lang == 1) ? (String(sd.year) + " год!") : ("Jahr " + String(sd.year) + "!");
  drawTextC(W / 2, 98, yr, FS_L, C_SUN);
  drawTextC(W / 2, 128, "* * * * * * * *", FS_M, C_WHITE);
}

void titleBand(const String& title, uint16_t bg, uint16_t fg) {
  int W = gfx.width();
  gfx.fillRect(0, 0, W, 28, bg);
  drawFitC(W / 2, 21, title, W - 16, FS_M, fg);
}

void scrSetup() {
  int W = gfx.width();
  gfx.fillScreen(C_WHITE);
  titleBand(String(LL("WLAN-Einrichtung", "Настройка WiFi")), C_ACC, C_ACC_TXT);
  drawFitL(10, 46, String(LL("1. Handy/PC mit diesem WLAN verbinden:", "1. Подключи телефон или ПК к сети WiFi:")), W - 20, FS_S, C_BLACK);
  gfx.drawRoundRect(30, 52, W - 60, 28, 6, C_BLACK);
  gfx.drawRoundRect(31, 53, W - 62, 26, 5, C_BLACK);
  drawFitC(W / 2, 73, sd.apName, W - 80, FS_M, C_BLACK);
  String s2[2] = { String(LL("2. Browser öffnen: ", "2. Открой в браузере: ")), String("http://192.168.4.1") };
  int f2 = fitFsParts(s2, 2, W - 20, FS_S);
  drawParts(10, 98, s2, 2, f2, C_BLACK);
  drawFitL(10, 114, String(LL("3. WLAN, Stadt, Kalender und Einstellungen eintragen", "3. Укажи WiFi, город, календарь и настройки")), W - 20, FS_XS, C_BLACK);
  drawFitL(10, 128, String(LL("(Die Seite öffnet sich oft von selbst)", "(Страница часто открывается сама)")), W - 20, FS_XS, C_BLACK);
  gfx.drawFastHLine(0, BOT_Y, W, C_BLACK);
  drawText(6, 147, String(LL("Setup-Modus", "Режим настройки")), FS_XS, C_BLACK);
  drawTextR(W - 6, 147, String("192.168.4.1"), FS_XS, C_BLACK);
}

void scrSaved() {
  int W = gfx.width();
  gfx.fillScreen(C_WHITE);
  titleBand(String(LL("Gespeicherte Einstellungen", "Найдены сохранённые настройки")), C_ACC, C_ACC_TXT);
  int y = 44;
  String l1[2] = { String(LL("WLAN: ", "WiFi: ")), sd.savedSsid.length() ? sd.savedSsid : String("-") };
  String l2[2] = { String(LL("Kalender: ", "Календарь: ")), sd.savedCal.length() ? sd.savedCal : String("-") };
  String l3[2] = { String(LL("Stadt: ", "Город: ")), sd.savedCity.length() ? sd.savedCity : String("-") };
  String* lines[3] = { l1, l2, l3 };
  for (int i = 0; i < 3; i++) {
    int w = W - 20 - textW(lines[i][0], FS_S);
    lines[i][1] = clipStr(lines[i][1], FS_S, w);
    drawParts(10, y, lines[i], 2, FS_S, C_BLACK);
    y += 14;
  }
  gfx.drawFastHLine(10, 80, W - 20, C_BLACK);
  String ha[2] = { String(LL("Dein Gerät muss im WLAN sein: ", "Телефон/ПК должен быть в сети: ")), sd.savedSsid };
  int fa = fitFsParts(ha, 2, W - 20, FS_XS);
  drawParts(10, 95, ha, 2, fa, C_BLACK);
  String nw[2] = { String(LL("Neu einrichten: ", "Настроить заново: ")), "http://" + sd.ip };
  int f = fitFsParts(nw, 2, W - 20, FS_S);
  drawParts(10, 112, nw, 2, f, C_BLACK);
  drawFitL(10, 128, String(LL("Alles ok? 2 Min warten oder Taste drücken", "Всё верно? Подожди 2 мин или нажми кнопку")), W - 20, FS_XS, C_BLACK);
  gfx.drawFastHLine(0, BOT_Y, W, C_BLACK);
  drawText(6, 147, String(LL("Dann läuft alles normal weiter", "Затем всё продолжит работать штатно")), FS_XS, C_BLACK);
}

void scrNoWifi() {
  int W = gfx.width();
  gfx.fillScreen(C_WHITE);
  titleBand(String(LL("Keine WLAN-Verbindung", "Нет соединения с WiFi")), C_RED, C_WHITE);

  String f1 = g_lang == 1 ? ("Не удалось подключиться к \"" + sd.failedSsid + "\"")
                          : ("Verbindung zu \"" + sd.failedSsid + "\" fehlgeschlagen");
  drawFitL(10, 46, f1, W - 20, FS_S, C_BLACK);
  drawText(10, 62, String(LL("Bitte WLAN-Daten prüfen", "Проверь данные WiFi")), FS_S, C_BLACK);

  drawText(10, 82, String(LL("Gefundene Netze:", "Найденные сети:")), FS_XS, C_BLACK);
  int y = 96;
  for (int i = 0; i < sd.netCount && i < 4; i++) {
    int bars = (sd.netRssi[i] > -70) ? 3 : (sd.netRssi[i] > -85 ? 2 : 1);
    drawWifi(10, y - 7, bars, 1, C_BLACK);
    String label = clipStr(sd.netName[i], FS_XS, 100);
    drawText(24, y, label, FS_XS, C_BLACK);
    drawText(126, y, String(sd.netRssi[i]) + " dBm", FS_XS, C_BLACK);
    y += 12;
  }
  if (sd.netCount == 0) drawText(10, 96, String(LL("keine Netze gefunden", "сетей не найдено")), FS_XS, C_BLACK);

  gfx.drawRoundRect(178, 78, 108, 52, 5, C_RED);
  gfx.drawRoundRect(179, 79, 106, 50, 4, C_RED);
  drawFitC(232, 92, String(LL("Einrichtung:", "Настройка:")), 96, FS_XS, C_RED);
  drawFitC(232, 106, "WLAN " + String(AP_NAME), 100, FS_XS, C_BLACK);
  drawFitC(232, 120, "192.168.4.1", 100, FS_S, C_BLACK);

  gfx.drawFastHLine(0, BOT_Y, W, C_BLACK);
  drawFitL(6, 147, String(LL("Verbindet sich automatisch, sobald das WLAN wieder da ist", "Подключится само, как только WiFi вернётся")), W - 12, FS_XS, C_BLACK);
}

void scrNoCal() {
  int W = gfx.width();
  gfx.fillScreen(C_WHITE);
  drawText(6, 16, sd.dateShort, FS_S, C_BLACK);
  drawWifi(W - 14, 9, sd.bars, 1, C_BLACK);
  gfx.drawFastHLine(0, TOP_H, W, C_BLACK);

  drawBin(20, 40, 3, C_RED, C_WHITE);
  drawFitL(86, 60, String(LL("Kein Kalender", "Нет календаря")), W - 92, FS_L, C_RED);
  String n1[2] = { String(LL("1. Im WLAN sein: ", "1. Будь в сети WiFi: ")), sd.homeSsid.length() ? sd.homeSsid : String("?") };
  int f1 = fitFsParts(n1, 2, W - 92, FS_S);
  drawParts(86, 80, n1, 2, f1, C_BLACK);
  gfx.drawRoundRect(86, 88, 200, 26, 5, C_BLACK);
  drawFitC(186, 106, "http://" + sd.ip, 190, FS_M, C_BLACK);
  drawFitL(86, 128, String(LL("2. Öffnen und Kalender + Stadt eintragen", "2. Открой и укажи календарь и город")), W - 92, FS_XS, C_BLACK);
  drawBottomBar();
}

void scrNoNtp() {
  int W = gfx.width();
  gfx.fillScreen(C_WHITE);
  drawBin(20, 40, 3, C_RED, C_WHITE);
  drawFitL(86, 66, String(LL("Warte auf Zeit", "Ждём время")), W - 92, FS_L, C_RED);
  drawFitL(86, 88, String(LL("NTP-Synchronisierung läuft...", "Синхронизация по NTP...")), W - 92, FS_S, C_BLACK);
  gfx.drawRoundRect(86, 96, 200, 26, 5, C_BLACK);
  drawFitC(186, 114, sd.ip, 190, FS_M, C_BLACK);
  drawFitL(86, 131, String(LL("Kalender: Adresse im Browser öffnen", "Календарь: открой адрес в браузере")), W - 92, FS_XS, C_BLACK);
  gfx.drawFastHLine(0, BOT_Y, W, C_BLACK);
  drawText(6, 147, String(LL("Zeit & Datum synchronisieren", "Синхронизация времени и даты")), FS_XS, C_BLACK);
}

// ---------- сервисный экран ----------
String uptimeStr() {
  unsigned long s = millis() / 1000UL;
  unsigned long d = s / 86400UL; s %= 86400UL;
  unsigned long h = s / 3600UL;  s %= 3600UL;
  unsigned long m = s / 60UL;
  char b[24];
  if (d > 0) snprintf(b, sizeof(b), "%lud %02luh %02lum", d, h, m);
  else snprintf(b, sizeof(b), "%luh %02lum", h, m);
  return String(b);
}

void kvLine(int x, int y, int w, const String& k, const String& v) {
  String key = k + " ";
  drawText(x, y, key, FS_XS, C_BLACK);
  int kw = textW(key, FS_XS);
  drawText(x + kw, y, clipStr(v, FS_XS, w - kw), FS_XS, C_RED);
}

void scrService() {
  int W = gfx.width();
  gfx.fillScreen(C_WHITE);
  gfx.fillRect(0, 0, W, 17, C_RED);
  drawText(6, 13, String(LL("SERVICE", "СЕРВИС")) + "  v" FW_VERSION, FS_S, C_WHITE);
  drawTextR(W - 6, 13, deviceName, FS_S, C_WHITE);
  gfx.drawFastVLine(148, 19, 120, C_BLACK);

  const int L = 4, R = 154, CW = 140;
  int y = 28;
  const int dy = 12;

  kvLine(L, y, CW, LL("Build:", "Сборка:"), String(__DATE__)); y += dy;
  kvLine(L, y, CW, LL("Sprache:", "Язык:"), g_lang == 1 ? "Русский" : "Deutsch"); y += dy;
  kvLine(L, y, CW, LL("Laufzeit:", "Аптайм:"), uptimeStr()); y += dy;
  kvLine(L, y, CW, LL("Speicher:", "Память:"), String(ESP.getFreeHeap() / 1024) + " KB"); y += dy;
  bool conn = (WiFi.status() == WL_CONNECTED);
  kvLine(L, y, CW, "WLAN:", conn ? WiFi.SSID() : String("-")); y += dy;
  kvLine(L, y, CW, LL("Signal:", "Сигнал:"), conn ? (String(WiFi.RSSI()) + " dBm") : String("-")); y += dy;
  kvLine(L, y, CW, "IP:", conn ? WiFi.localIP().toString() : String("-")); y += dy;
  String nowS = "--";
  if (timeValid()) { time_t n = nowT(); nowS = hm(n); }
  kvLine(L, y, CW, LL("Zeit:", "Время:"), nowS); y += dy;
  kvLine(L, y, CW, LL("Zone:", "Пояс:"), tzString); y += dy;
  kvLine(L, y, CW, "SDK:", String(ESP.getSdkVersion())); y += dy;

  y = 28;
  kvLine(R, y, CW, LL("Display:", "Экран:"), sd.dispTime + " (#" + String(renderCount) + ")"); y += dy;
  String wxS = wx.ok ? sd.wxTime : String("--:--");
  wxS += wxFailed ? LL(" Fehler", " ошибка") : (geoOk ? " OK" : LL(" aus", " выкл"));
  kvLine(R, y, CW, LL("Wetter:", "Погода:"), wxS); y += dy;
  kvLine(R, y, CW, LL("Kalender:", "Календарь:"), sd.calTime); y += dy;
  String src = calUrl.length() ? hostOf(calUrl) : String(LL("Datei", "файл"));
  kvLine(R, y, CW, LL("Quelle:", "Источник:"), src); y += dy;
  kvLine(R, y, CW, LL("Termine:", "События:"), String(evShown) + "/" + String(evTotal)); y += dy;
  kvLine(R, y, CW, LL("Typen:", "Типов:"), String(typeCount)); y += dy;
  String nx = (gNext.cnt > 0) ? (fmtShort(gNext.key) + " " + joinNames(gNext)) : String("-");
  kvLine(R, y, CW, LL("Nächster:", "Следующий:"), nx); y += dy;
  kvLine(R, y, CW, LL("Stadt:", "Город:"), cityName.length() ? cityName : String("-")); y += dy;
  String co = geoOk ? (String(geoLat, 2) + ", " + String(geoLon, 2)) : String("-");
  kvLine(R, y, CW, LL("Koord.:", "Коорд.:"), co); y += dy;
  kvLine(R, y, CW, LL("Erledigt:", "Вывезено:"), trashDoneToday ? "ja" : "nein"); y += dy;

  gfx.drawFastHLine(0, 139, W, C_BLACK);
  drawFitC(W / 2, 149, String(LL("Rückkehr zur Hauptanzeige in 2 Min", "Возврат на главный экран через 2 мин")), W - 12, FS_XS, C_BLACK);
}

void ensureLandscape() {
  if (gfx.width() < 296) { rot = DEFAULT_ROT; gfx.setRotation(rot); }
}

void drawScreen(Screen id) {
  ensureLandscape();
  switch (id) {
    case S_MAIN:    scrMain();    break;
    case S_NEUJAHR: scrNeujahr(); break;
    case S_SETUP:   scrSetup();   break;
    case S_SAVED:   scrSaved();   break;
    case S_NOWIFI:  scrNoWifi();  break;
    case S_NOCAL:   scrNoCal();   break;
    case S_SERVICE: scrService(); break;
    default:        scrNoNtp();   break;
  }
}

const char* screenName(Screen id) {
  switch (id) {
    case S_MAIN:    return "main";
    case S_NEUJAHR: return "neujahr";
    case S_SETUP:   return "wlan-setup";
    case S_SAVED:   return "saved-data";
    case S_NOWIFI:  return "no-wifi";
    case S_NOCAL:   return "no-calendar";
    case S_SERVICE: return "service";
    default:        return "wait-ntp";
  }
}

void renderScreen(Screen id) {
  curScreen = id;
  oledMsg("Rendering...", screenName(id));
  pushScreen(id);
  lastRenderMs = millis();
  renderCount++;
  char b[40];
  if (timeValid()) {
    time_t now = nowT();
    struct tm t = *localtime(&now);
    snprintf(b, sizeof(b), "%s %02d:%02d", screenName(id), t.tm_hour, t.tm_min);
  } else {
    snprintf(b, sizeof(b), "%s", screenName(id));
  }
  lastRenderInfo = String(b);
  oledMsg("Done", screenName(id));
}

void cleanCycle() {
#if PANEL_BWRY
  oledMsg("Clean", "not needed (BWRY)");
  return;
#endif
  const uint16_t seq[4] = { C_BLACK, C_WHITE, C_RED, C_WHITE };
  const char* nm[4] = { "Clean: black", "Clean: white", "Clean: red", "Clean: white" };
  for (int i = 0; i < 4; i++) {
    oledMsg(nm[i], "anti-ghosting");
    pushSolid(seq[i]);
  }
}

// =====================================================================
//  Какой экран показывать
// =====================================================================
Screen decideScreen() {
  if (!timeValid()) return S_NONTP;
  if (!LittleFS.exists("/calendar.ics")) return S_NOCAL;

  time_t now = nowT();
  struct tm t = *localtime(&now);
  bool newYear = (t.tm_mon == 11 && t.tm_mday == 31 && t.tm_hour >= 20) ||
                 (t.tm_mon == 0  && t.tm_mday == 1  && t.tm_hour < 12);
  if (newYear) return S_NEUJAHR;
  return S_MAIN;
}

void renderReal() {
  parseCalendar();
  Screen id = decideScreen();
  if (id == S_MAIN) {
    fillMainReal();
  } else {
    fillCommon();
    fillWeather();
    if (id == S_NEUJAHR && timeValid()) {
      time_t now = nowT();
      struct tm t = *localtime(&now);
      sd.year = (t.tm_mon == 11) ? (t.tm_year + 1901) : (t.tm_year + 1900);
    }
    curBanner = B_NONE;
  }

  if (timeValid()) {
    time_t now = nowT();
    struct tm t = *localtime(&now);
    long ck = (long)t.tm_year * 1000L + t.tm_yday;
    if (t.tm_hour == CLEAN_HOUR && ck != lastCleanKey) {
      lastCleanKey = ck;
      cleanCycle();
    }
  }
  renderScreen(id);
}

// =====================================================================
//  Кнопка и диод (заглушки до подключения)
// =====================================================================
void factoryResetNow() {
  oledMsg("RESET!", "restarting...");
  WiFi.disconnect(true);      // стереть сохранённую сеть
  ESP.eraseConfig();
  LittleFS.format();
  ESP.restart();
}

#if ENABLE_BUTTON
unsigned long btnStart = 0;
bool btnHolding = false;
bool btnLightDone = false;

void handleButton() {
  if (millis() < 6000) return;
  if (WiFi.status() != WL_CONNECTED) { btnHolding = false; return; }

  bool down = buttonIsDown();
  if (down) {
    if (!btnHolding) {
      btnHolding = true;
      btnStart = millis();
      btnLightDone = false;
    } else {
      unsigned long held = millis() - btnStart;
      if (held >= LIGHT_HOLD_MS && held < RESET_HOLD_MS && !btnLightDone) {
        btnLightDone = true;
        ledMasterEnabled = !ledMasterEnabled;
        oledMsg(ledMasterEnabled ? "LED an" : "LED aus", "");
      }
      if (held > LIGHT_HOLD_MS && held < RESET_HOLD_MS) {
        oledDraw("FACTORY RESET IN:", String((RESET_HOLD_MS - held) / 1000 + 1), "", "", "");
        oledHoldUntil = millis() + 500;
      }
      if (held >= RESET_HOLD_MS) factoryResetNow();
    }
  } else {
    if (btnHolding) {
      unsigned long dur = millis() - btnStart;
      if (dur > 50 && dur < LIGHT_HOLD_MS && curBanner != B_NONE) {
        trashDoneToday = !trashDoneToday;
        forceRender = true;
      }
    }
    btnHolding = false;
  }
}
#else
void handleButton() {}
#endif

#if ENABLE_LED
void ledSet(bool on) { digitalWrite(LED_PIN, on ? LOW : HIGH); }

void updateLed() {
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink >= 500) { lastBlink = millis(); blinkState = !blinkState; }

  bool night = false;
  if (timeValid()) {
    time_t now = nowT();
    struct tm t = *localtime(&now);
    night = (t.tm_hour >= 23 || t.tm_hour < 6);
  }
  bool achtung = (curBanner != B_NONE) && !trashDoneToday;
  if (!ledMasterEnabled || night || !achtung) ledSet(false);
  else ledSet(blinkState);
}
#else
void updateLed() {}
#endif

// =====================================================================
//  Веб-интерфейс
// =====================================================================
#define WS(de, ru) LL(de, ru)

// ВАЖНО: пустая строка в sendContent() означает КОНЕЦ ответа, поэтому пустые куски не отправляем
void out(const String& s) { if (s.length() > 0) server.sendContent(s); }

String nowString() {
  if (!timeValid()) return String(LL("keine Zeit (NTP...)", "нет времени (NTP...)"));
  time_t now = nowT();
  struct tm t = *localtime(&now);
  char b[40];
  snprintf(b, sizeof(b), "%02d:%02d:%02d  %02d.%02d.%04d", t.tm_hour, t.tm_min, t.tm_sec, t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);
  return String(b);
}

void redirectHome() {
  server.sendHeader("Location", "/");
  server.send(303);
}

void sendPage() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");

  out(F("<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        "<title>MüllBot</title><style>"
        "body{font-family:Arial,sans-serif;background:#121212;color:#fff;text-align:center;padding:12px;margin:0}"
        ".card{background:#1e1e1e;padding:18px;margin:14px auto;max-width:420px;border-radius:12px;box-shadow:0 4px 10px rgba(0,0,0,.5);text-align:left}"
        "h2{color:#00d2ff;margin-top:0;text-align:center}h3{color:#aaa;margin:4px 0;font-size:16px}"
        "input[type=text],input[type=number],input[type=file],select{margin:8px 0;width:96%;padding:10px;background:#2a2a2a;border:1px solid #444;color:#fff;border-radius:6px;box-sizing:border-box}"
        ".btn{background:#008CBA;color:#fff;border:none;padding:12px;border-radius:6px;cursor:pointer;font-size:16px;width:100%;margin-top:6px}"
        ".g{background:#2e7d32}.r{background:#b71c1c}.o{background:#ef6c00}"
        "hr{border:0;height:1px;background:#333;margin:16px 0}.n{background:#263238;border-left:4px solid #00d2ff;padding:8px 10px;margin:8px 0;border-radius:4px}"
        "label{display:block;padding:4px 0}details{margin:6px 0}summary{cursor:pointer;color:#00d2ff}p,li{font-size:14px;line-height:1.45}"
        "</style></head><body>"));

  // ---------- шапка ----------
  out(F("<div class=\"card\"><h2>")); out(esc(deviceName)); out(F("</h2><p>"));
  out(WS("Firmware", "Прошивка")); out(F(": v" FW_VERSION "<br>"));
  out(WS("Gerätezeit", "Время на устройстве")); out(F(": ")); out(nowString()); out(F("<br>WLAN: "));
  out(esc(WiFi.SSID())); out(F("</p>"));
  if (webNotice.length()) { out(F("<div class=\"n\">")); out(esc(webNotice)); out(F("</div>")); webNotice = ""; }
  out(F("</div>"));

  out(F("<div class=\"card\"><form method=\"GET\" action=\"/setup\"><input type=\"submit\" class=\"btn o\" value=\""));
  out(WS("Neu einrichten (Assistent)", "Настроить заново (ассистент)")); out(F("\"></form></div>"));

  // ---------- окно "сохранённые настройки" ----------
  if (bootWindowActive) {
    out(F("<div class=\"card\" style=\"border:2px solid #ef6c00\"><h3>"));
    out(WS("Gespeicherte Einstellungen gefunden", "Найдены сохранённые настройки")); out(F("</h3><p>WLAN: <b>"));
    out(esc(savedSsid)); out(F("</b><br>")); out(WS("Kalender", "Календарь")); out(F(": <b>"));
    out(esc(calUrl.length() ? hostOf(calUrl) : String("-"))); out(F("</b><br>")); out(WS("Stadt", "Город")); out(F(": <b>"));
    out(esc(cityName.length() ? cityName : String("-"))); out(F("</b></p><form method=\"POST\" action=\"/keep\"><input type=\"submit\" class=\"btn g\" value=\""));
    out(WS("Ja, diese Einstellungen verwenden", "Да, использовать эти настройки"));
    out(F("\"></form><form method=\"POST\" action=\"/newdevice\" onsubmit=\"return confirm('?')\"><input type=\"submit\" class=\"btn o\" value=\""));
    out(WS("Als neues Gerät einrichten", "Настроить как новое устройство"));
    out(F("\"></form><p>"));
    out(WS("Ohne Eingabe geht es nach ca. 2 Minuten mit diesen Einstellungen normal weiter.", "Если ничего не нажимать, примерно через 2 минуты устройство продолжит работу с этими настройками."));
    out(F("</p></div>"));
  }

  // ---------- календарь ----------
  parseCalendar();
  out(F("<div class=\"card\"><h3>1. ")); out(WS("Müllkalender (iCal)", "Календарь вывоза (iCal)"));
  out(F("</h3><form method=\"POST\" action=\"/save_url\"><input type=\"text\" name=\"url\" placeholder=\"https://...\" value=\""));
  out(esc(calUrl)); out(F("\"><input type=\"submit\" class=\"btn g\" value=\""));
  out(WS("Speichern und laden", "Сохранить и загрузить")); out(F("\"></form><hr><form method=\"POST\" action=\"/upload\" enctype=\"multipart/form-data\">"
        "<input type=\"file\" name=\"file\" accept=\".ics\"><input type=\"submit\" class=\"btn\" value=\""));
  out(WS(".ics-Datei hochladen", "Загрузить файл .ics")); out(F("\"></form><p>"));
  out(WS("Letzte Aktualisierung", "Последнее обновление")); out(F(": ")); out(esc(sd.calTime.length() ? sd.calTime : String("--")));
  out(F(" &middot; ")); out(WS("Termine", "События")); out(F(": ")); out(String(evShown)); out(F("/")); out(String(evTotal));
  out(F("</p></div>"));

  // ---------- город / погода ----------
  out(F("<div class=\"card\"><h3>2. ")); out(WS("Stadt und Wetter", "Город и погода"));
  out(F("</h3><form method=\"POST\" action=\"/save_city\"><input type=\"text\" name=\"city\" placeholder=\""));
  out(WS("z. B. Köln", "например: Köln")); out(F("\" value=\"")); out(esc(cityName));
  out(F("\"><input type=\"submit\" class=\"btn g\" value=\"")); out(WS("Speichern", "Сохранить")); out(F("\"></form><p>"));
  if (cityName.length()) {
    if (geoOk) {
      out(WS("Gefunden", "Найдено")); out(F(": <b>")); out(esc(geoLabel)); out(F("</b> (")); out(String(geoLat, 2)); out(F(", ")); out(String(geoLon, 2)); out(F(")"));
    } else {
      out(WS("Stadt nicht gefunden - bitte anders schreiben oder größeren Ort wählen.", "Город не найден - напиши иначе или укажи крупный населённый пункт рядом."));
    }
  } else {
    out(WS("Ohne Stadt wird kein Wetter angezeigt.", "Без города погода не показывается."));
  }
  out(F("</p></div>"));

  // ---------- игнор ----------
  out(F("<div class=\"card\"><h3>3. ")); out(WS("Müllarten ignorieren", "Игнорировать виды мусора"));
  out(F("</h3><form method=\"POST\" action=\"/save_ignore\">"));
  if (typeCount == 0) {
    out(F("<p>")); out(WS("Noch keine Kalendereinträge gefunden.", "Событий в календаре пока нет.")); out(F("</p>"));
  } else {
    for (int i = 0; i < typeCount; i++) {
      out(F("<label><input type=\"checkbox\" name=\"ig\" value=\"")); out(esc(typeNames[i])); out(F("\""));
      if (isIgnored(typeNames[i])) out(F(" checked"));
      out(F("> ")); out(esc(typeNames[i])); out(F("</label>"));
    }
    out(F("<p>")); out(WS("Angehakte Arten werden nicht angezeigt.", "Отмеченные виды не показываются.")); out(F("</p>"));
    out(F("<input type=\"submit\" class=\"btn\" value=\"")); out(WS("Speichern", "Сохранить")); out(F("\">"));
  }
  out(F("</form></div>"));

  // ---------- устройство ----------
  out(F("<div class=\"card\"><h3>4. ")); out(WS("Gerät", "Устройство"));
  out(F("</h3><form method=\"POST\" action=\"/save_device\">"));
  outFieldsDevice();
  out(F("<input type=\"submit\" class=\"btn g\" value=\"")); out(WS("Speichern", "Сохранить")); out(F("\"></form></div>"));

  // ---------- экран ----------
  out(F("<div class=\"card\"><h3>5. ")); out(WS("Anzeige", "Экран"));
  out(F("</h3><form method=\"POST\" action=\"/refresh\"><input type=\"submit\" class=\"btn r\" value=\""));
  out(WS("Anzeige jetzt aktualisieren", "Обновить экран сейчас"));
  out(F("\"></form><form method=\"POST\" action=\"/service\"><input type=\"submit\" class=\"btn\" value=\""));
  out(WS("Serviceseite (2 Min)", "Сервисный экран (2 мин)"));
  out(F("\"></form><hr><form method=\"POST\" action=\"/factory\" onsubmit=\"return confirm('?')\"><input type=\"submit\" class=\"btn o\" value=\""));
  out(WS("Werkseinstellungen", "Сброс на заводские настройки"));
  out(F("\"></form></div>"));

  // ---------- прошивка ----------
  out(F("<div class=\"card\"><h3>6. ")); out(WS("Firmware", "Прошивка"));
  out(F("</h3><p>")); out(WS("Installiert: ", "Установлена: ")); out(F("<b>v" FW_VERSION "</b> (" __DATE__ ")</p>"));
  out(F("<p><b>")); out(WS("Aus Datei (.bin)", "Из файла (.bin)")); out(F("</b></p>"));
  out(F("<form method=\"POST\" action=\"/update\" enctype=\"multipart/form-data\"><input type=\"file\" name=\"firmware\" accept=\".bin\">"
        "<input type=\"submit\" class=\"btn o\" value=\"")); out(WS("Datei installieren", "Установить из файла")); out(F("\"></form><hr>"));
  out(F("<p><b>")); out(WS("Von GitHub (neueste Release)", "С GitHub (последний релиз)")); out(F("</b></p><input type=\"text\" id=\"ota_repo\" placeholder=\"user/repository\" value=\""));
  out(esc(ghRepo)); out(F("\"><input type=\"button\" class=\"btn o\" onclick=\"otaGh()\" value=\"")); out(WS("Prüfen und installieren", "Проверить и установить"));
  out(F("\"><hr><p><b>")); out(WS("Von Link (.bin)", "По ссылке (.bin)")); out(F("</b></p><input type=\"text\" id=\"ota_url\" placeholder=\"https://...\">"
        "<input type=\"button\" class=\"btn o\" onclick=\"otaUrl()\" value=\"")); out(WS("Von Link installieren", "Установить по ссылке"));
  out(F("\"><p id=\"ota_st\" style=\"color:#ffcc00\"></p><p style=\"color:#aaa\">"));
  out(WS("Die Datei lädt dein Browser herunter (nicht das Gerät) und schickt sie an das Gerät. Strom nicht trennen!", "Файл скачивает твой браузер (не устройство) и отправляет в устройство. Не отключай питание!"));
  out(F("</p></div>"));
  out(F("<script>var T={"));
  out(F("load:'")); out(WS("Lade ...", "Загрузка ...")); out(F("',"));
  out(F("send:'")); out(WS("Sende an das Gerät", "Отправляю в устройство")); out(F("',"));
  out(F("err:'")); out(WS("Fehler:", "Ошибка:")); out(F("',"));
  out(F("nourl:'")); out(WS("Bitte Link eingeben", "Введи ссылку")); out(F("',"));
  out(F("norepo:'")); out(WS("Bitte user/repository eingeben", "Введи user/repository")); out(F("',"));
  out(F("check:'")); out(WS("Suche neueste Version ...", "Ищу последнюю версию ...")); out(F("',"));
  out(F("nobin:'")); out(WS("Keine .bin-Datei in der Release", "В релизе нет файла .bin")); out(F("',"));
  out(F("install:'")); out(WS("Installieren", "Установить")); out(F("'};"));
  out(F("function st(m){document.getElementById('ota_st').textContent=m;}"
        "async function sendBlob(b){st(T.send+' '+Math.round(b.size/1024)+' KB ...');var fd=new FormData();fd.append('firmware',b,'firmware.bin');"
        "var u=await fetch('/update',{method:'POST',body:fd});st(await u.text());}"
        "async function otaUrl(){var url=document.getElementById('ota_url').value.trim();if(!url){st(T.nourl);return;}"
        "try{st(T.load);var r=await fetch(url);if(!r.ok)throw new Error('HTTP '+r.status);await sendBlob(await r.blob());}catch(e){st(T.err+' '+e.message+' (CORS?)');}}"
        "async function otaGh(){var repo=document.getElementById('ota_repo').value.trim().replace('https://github.com/','').replace(/\\/+$/,'');"
        "if(!repo){st(T.norepo);return;}"
        "try{fetch('/save_gh',{method:'POST',body:new URLSearchParams({repo:repo})});st(T.check);"
        "var a=await fetch('https://api.github.com/repos/'+repo+'/releases/latest');if(!a.ok)throw new Error('GitHub HTTP '+a.status);"
        "var rel=await a.json();var as=null;for(var i=0;i<rel.assets.length;i++){if(/\\.bin$/i.test(rel.assets[i].name)){as=rel.assets[i];break;}}"
        "if(!as)throw new Error(T.nobin);"
        "if(!confirm(T.install+' '+rel.tag_name+' ('+as.name+', '+Math.round(as.size/1024)+' KB)?')){st('');return;}"
        "st(T.load);var r=await fetch(as.url,{headers:{'Accept':'application/octet-stream'}});if(!r.ok)throw new Error('HTTP '+r.status);"
        "await sendBlob(await r.blob());}catch(e){st(T.err+' '+e.message+' (CORS?)');}}"
        "</script>"));

  // ---------- инструкции ----------
  out(F("<div class=\"card\"><h3>")); out(WS("Anleitung", "Инструкция")); out(F("</h3>"));
  out(F("<details><summary>")); out(WS("Wie bekomme ich den Kalender-Link?", "Как получить ссылку на календарь вывоза?")); out(F("</summary><ol><li>"));
  out(WS("Auf der Webseite deines Abfallentsorgers oder deiner Stadt nach „iCal“, „Kalender abonnieren“ oder „Kalender exportieren“ suchen.",
         "На сайте службы вывоза мусора или твоего города найди пункты «iCal», «подписаться на календарь» или «экспорт календаря»."));
  out(F("</li><li>")); out(WS("Den Link kopieren und oben einfügen. Beginnt er mit webcal://, wird er automatisch zu https:// umgewandelt.",
         "Скопируй ссылку и вставь выше. Если она начинается с webcal://, она автоматически станет https://."));
  out(F("</li><li>")); out(WS("Gibt es nur einen Download? Die .ics-Datei speichern und oben hochladen.",
         "Если есть только скачивание, сохрани файл .ics и загрузи его выше."));
  out(F("</li><li>")); out(WS("Nicht benötigte Müllarten unter „Müllarten ignorieren“ abhaken.",
         "Ненужные виды мусора отметь в блоке «Игнорировать виды мусора»."));
  out(F("</li></ol></details><details><summary>")); out(WS("Wie funktioniert das Wetter?", "Как работает погода?")); out(F("</summary><p>"));
  out(WS("Die Daten kommen kostenlos von Open-Meteo, ein Schlüssel oder Link ist nicht nötig. Einfach die Stadt eintragen (z. B. „Köln“), das Gerät findet die Koordinaten selbst. Das Wetter wird einmal pro Stunde aktualisiert.",
         "Данные бесплатно приходят с Open-Meteo, ключ или ссылка не нужны. Просто укажи город (например, «Köln»), координаты устройство найдёт само. Погода обновляется раз в час."));
  out(F("</p></details><details><summary>")); out(WS("WLAN ändern / neu einrichten", "Как поменять WiFi / настроить заново")); out(F("</summary><p>"));
  out(WS("Unten „Werkseinstellungen“ drücken. Danach öffnet das Gerät das WLAN „" AP_NAME "“, dort das Heim-WLAN eintragen.",
         "Нажми «Сброс на заводские настройки». После этого устройство откроет сеть «" AP_NAME "», в ней укажи домашний WiFi."));
  out(F("</p></details><details><summary>")); out(WS("Firmware aktualisieren", "Как обновить прошивку")); out(F("</summary><p>"));
  out(WS("In der Arduino IDE: Sketch - Kompilierte Binärdatei exportieren. Die entstehende .bin-Datei (nicht .elf) hier hochladen oder als Anhang einer GitHub-Release veröffentlichen und mit dem Repository-Namen installieren.",
         "В Arduino IDE: Скетч - Экспорт скомпилированного бинарного файла. Полученный .bin (не .elf) загрузи сюда или опубликуй вложением к релизу на GitHub и установи по имени репозитория."));
  out(F("</p></details></div></body></html>"));

  server.sendContent("");
}

// ---------- общие поля настроек (главная страница и ассистент) ----------
void outFieldsDevice() {
  out(F("<p>")); out(WS("Gerätename", "Название устройства"));
  out(F("</p><input type=\"text\" name=\"name\" maxlength=\"24\" value=\"")); out(esc(deviceName));
  out(F("\"><p>")); out(WS("Sprache", "Язык")); out(F("</p><select name=\"lang\"><option value=\"0\""));
  if (g_lang == 0) out(F(" selected"));
  out(F(">Deutsch</option><option value=\"1\"")); if (g_lang == 1) out(F(" selected"));
  out(F(">Русский</option></select><p>")); out(WS("Zeitzone", "Часовой пояс")); out(F("</p><select name=\"tzsel\">"));
  bool matched = false;
  for (int i = 0; i < TZ_OPT_COUNT; i++) {
    out(F("<option value=\"")); out(esc(String(TZ_OPTS[i].posix))); out(F("\""));
    if (tzString == TZ_OPTS[i].posix) { out(F(" selected")); matched = true; }
    out(F(">")); out(esc(String(TZ_OPTS[i].label))); out(F("</option>"));
  }
  out(F("</select><input type=\"text\" name=\"tzcustom\" placeholder=\""));
  out(WS("oder eigener TZ-String (POSIX), optional", "или своя TZ-строка (POSIX), необязательно"));
  out(F("\" value=\"")); if (!matched) out(esc(tzString));
  out(F("\"><p>")); out(WS("Anzeige aktualisieren alle (Minuten)", "Обновлять экран раз в (минут)"));
  out(F("</p><input type=\"number\" name=\"refresh\" min=\"")); out(String(MIN_REFRESH_MIN));
  out(F("\" max=\"1440\" value=\"")); out(String(refreshMin));
  out(F("\"><p style=\"color:#aaa\">"));
  out(WS("Empfehlung: 60 (stündlich) - schont die Anzeige. Zum Testen: 2.", "Рекомендуем: 60 (раз в час) - бережёт панель. Для теста: 2."));
  out(F("</p>"));
}

void outFieldsIgnore() {
  if (typeCount == 0) {
    out(F("<p>")); out(WS("Noch keine Kalendereinträge gefunden.", "Событий в календаре пока нет.")); out(F("</p>"));
    return;
  }
  for (int i = 0; i < typeCount; i++) {
    out(F("<label><input type=\"checkbox\" name=\"ig\" value=\"")); out(esc(typeNames[i])); out(F("\""));
    if (isIgnored(typeNames[i])) out(F(" checked"));
    out(F("> ")); out(esc(typeNames[i])); out(F("</label>"));
  }
  out(F("<p style=\"color:#aaa\">")); out(WS("Angehakte Arten werden nicht angezeigt.", "Отмеченные виды не показываются.")); out(F("</p>"));
}

void applyDeviceArgs() {
  String nm = server.arg("name");
  nm.trim();
  if (nm.length() == 0) nm = DEFAULT_NAME;
  if (nm.length() > 24) nm = nm.substring(0, 24);
  deviceName = nm;
  writeFile("/name.txt", deviceName);

  g_lang = (server.arg("lang") == "1") ? 1 : 0;
  writeFile("/lang.txt", String(g_lang));

  String tz = server.arg("tzcustom");
  tz.trim();
  if (tz.length() == 0) tz = server.arg("tzsel");
  if (tz.length() >= 3 && tz.length() <= 60) {
    tzString = tz;
    writeFile("/tz.txt", tzString);
    setenv("TZ", tzString.c_str(), 1);
    tzset();
  }

  String rs = server.arg("refresh");
  if (rs.length() > 0) {
    int rm = rs.toInt();
    if (rm < MIN_REFRESH_MIN) rm = MIN_REFRESH_MIN;
    if (rm > 1440) rm = 1440;
    refreshMin = rm;
    writeFile("/interval.txt", String(refreshMin));
  }
  lastHourKey = -2;
  requestRefresh();
}

void applyIgnoreArgs() {
  String list = "\n";
  for (int i = 0; i < server.args(); i++) {
    if (server.argName(i) == "ig") {
      String v = server.arg(i);
      v.trim();
      if (v.length()) list += v + "\n";
    }
  }
  ignoreList = list;
  writeFile("/ignore.txt", ignoreList);
}

void pollPending() {                      // введённый в мастере WiFi успешно подключился: сохранить резерв
  if (pendingSsid.length() > 0 && WiFi.status() == WL_CONNECTED) {
    saveWifiBackup(pendingSsid, pendingPass);
    savedSsid = pendingSsid;
    pendingSsid = "";
    pendingPass = "";
  }
}

void otaUploadChunk() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    otaBusy = true;
    otaOk = false;
    otaErr = "";
    WiFiUDP::stopAll();
    uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    PRF("[ota] start '%s', свободно под прошивку %u байт\n", up.filename.c_str(), maxSketchSpace);
    if (!Update.begin(maxSketchSpace)) {
      otaErr = String("begin() code ") + String((int)Update.getError());
      Update.printError(Serial);
    }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (otaErr.length() == 0) {
      if (Update.write(up.buf, up.currentSize) != up.currentSize) {
        otaErr = String("write() code ") + String((int)Update.getError());
        Update.printError(Serial);
      }
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (otaErr.length() == 0) {
      if (Update.end(true)) {
        otaOk = true;
        PRF("[ota] готово, %u байт\n", (unsigned)up.totalSize);
      } else {
        otaErr = String("end() code ") + String((int)Update.getError()) + " (not a valid ESP8266 .bin?)";
        Update.printError(Serial);
      }
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.end();
    otaErr = "aborted";
  }
}

void uploadChunk() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    LittleFS.remove("/calendar.ics");
    File f = LittleFS.open("/calendar.ics", "w");
    if (f) f.close();
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    File f = LittleFS.open("/calendar.ics", "a");
    if (f) {
      f.write(upload.buf, upload.currentSize);
      f.close();
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    saveCalSyncEpoch();
  }
}

// ---------- ассистент: WLAN -> город -> календарь -> настройки -> одна перезагрузка ----------
void wizRedirect(const char* step) {
  server.sendHeader("Location", String("/setup?step=") + step);
  server.send(303);
}

void wizHead(int stepNo, bool refreshMeta) {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  out(F("<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"));
  if (refreshMeta) out(F("<meta http-equiv=\"refresh\" content=\"2\">"));
  out(F("<title>MüllBot</title><style>"
        "body{font-family:Arial,sans-serif;background:#121212;color:#fff;padding:12px;margin:0}"
        ".card{background:#1e1e1e;padding:18px;margin:14px auto;max-width:420px;border-radius:12px}"
        "h2{color:#00d2ff;margin-top:0;text-align:center}h3{color:#aaa}"
        "input,select{margin:8px 0;width:100%;padding:10px;background:#2a2a2a;border:1px solid #444;color:#fff;border-radius:6px;box-sizing:border-box;font-size:16px}"
        "input[type=radio],input[type=checkbox]{width:auto;margin-right:8px}label{display:block;padding:6px 0}"
        ".btn{background:#2e7d32;color:#fff;border:none;padding:12px;border-radius:6px;font-size:16px;margin-top:8px}"
        ".n{background:#263238;border-left:4px solid #00d2ff;padding:8px 10px;margin:8px 0;border-radius:4px}"
        ".e{border-left-color:#e53935;background:#3a1f1f}a{color:#00d2ff}p{line-height:1.45}"
        "</style></head><body><div class=\"card\"><h2>"));
  out(esc(deviceName)); out(F(" &middot; ")); out(WS("Einrichtung", "Настройка")); out(F("</h2>"));
  if (stepNo > 0) {
    out(F("<p style=\"color:#aaa\">")); out(WS("Schritt ", "Шаг ")); out(String(stepNo)); out(F(" / 4</p>"));
  }
  bool onAp = portalMode && (server.client().localIP() == WiFi.softAPIP());
  out(F("<div class=\"n\">"));
  if (onAp) {
    out(WS("Du bist mit dem WLAN <b>", "Ты подключён к WiFi-сети <b>")); out(AP_NAME);
    out(WS("</b> verbunden (Adresse 192.168.4.1).", "</b> (адрес 192.168.4.1)."));
    if (WiFi.status() == WL_CONNECTED) {
      out(F("<br>")); out(WS("Das Gerät ist mit <b>", "Устройство подключено к <b>")); out(esc(WiFi.SSID())); out(F("</b> ")); out(WS("verbunden.", "."));
    }
  } else {
    out(WS("Du bist im WLAN <b>", "Ты в сети WiFi <b>")); out(esc(WiFi.SSID())); out(F("</b> (")); out(WS("Adresse ", "адрес ")); out(WiFi.localIP().toString()); out(F(")."));
  }
  out(F("</div>"));
  if (wizError.length()) { out(F("<div class=\"n e\">")); out(esc(wizError)); out(F("</div>")); wizError = ""; }
}

void wizFoot() {
  out(F("</div></body></html>"));
  server.sendContent("");
}

void wizStepWifi() {
  if (scanCount == 0) scanNets();
  wizHead(1, false);
  bool conn = (WiFi.status() == WL_CONNECTED);
  out(F("<form method=\"POST\" action=\"/setup/wifi\">"));
  if (conn) {
    out(F("<label><input type=\"radio\" name=\"choice\" value=\"current\" checked>")); out(WS("Aktuelles WLAN verwenden: ", "Использовать текущую сеть: "));
    out(F("<b>")); out(esc(WiFi.SSID())); out(F("</b></label>"));
    out(F("<label><input type=\"radio\" name=\"choice\" value=\"new\">")); out(WS("Anderes WLAN", "Другая сеть")); out(F("</label>"));
  } else {
    out(F("<input type=\"hidden\" name=\"choice\" value=\"new\">"));
  }
  out(F("<p>")); out(WS("Netzwerk wählen", "Выбери сеть")); out(F("</p><select name=\"ssid\">"));
  for (int i = 0; i < scanCount; i++) {
    out(F("<option value=\"")); out(esc(scanSsid[i])); out(F("\">")); out(esc(scanSsid[i]));
    out(F(" (")); out(String(scanRssi[i])); out(F(" dBm)</option>"));
  }
  out(F("</select><p>")); out(WS("oder Namen selbst eingeben", "или введи название вручную"));
  out(F("</p><input type=\"text\" name=\"ssid_manual\"><p>")); out(WS("Passwort", "Пароль"));
  out(F("</p><input type=\"password\" name=\"pass\"><input class=\"btn\" type=\"submit\" value=\""));
  out(WS("Weiter", "Дальше")); out(F("\"></form><p><a href=\"/setup/scan\">"));
  out(WS("Netze neu suchen", "Обновить список сетей")); out(F("</a></p>"));
  wizFoot();
}

void wizStepWait() {
  pollPending();
  if (WiFi.status() == WL_CONNECTED) {
    if (wizConnectedAt == 0) wizConnectedAt = millis();
    PRF("[web] WiFi connected in wizard: %s, ip %s, heap %u\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), ESP.getFreeHeap());
    wizHead(0, false);
    out(F("<h3>")); out(WS("Verbunden!", "Подключено!")); out(F("</h3><p>"));
    out(WS("Das Gerät ist jetzt im WLAN ", "Устройство теперь в сети WiFi ")); out(F("<b>")); out(esc(WiFi.SSID())); out(F("</b>.</p><p>"));
    out(WS("Diese Einrichtungs-WLAN ", "Эта сеть настройки ")); out(AP_NAME); out(WS(" schaltet sich gleich ab. Verbinde dein Handy/deinen PC wieder mit <b>", " сейчас отключится. Подключи телефон или ПК обратно к <b>"));
    out(esc(WiFi.SSID())); out(F("</b> ")); out(WS("und öffne <b>http://", "и открой <b>http://"));
    out(WiFi.localIP().toString()); out(F("</b>. ")); out(WS("Dort stellst du Stadt, Kalender und alles Weitere ein (die Adresse steht auch auf der Anzeige).", "Там укажешь город, календарь и остальные настройки (адрес будет и на экране)."));
    out(F("</p>"));
    wizFoot();
    return;
  }
  if (millis() - wizConnectStart > 25000UL) {
    wizError = String(LL("Verbindung fehlgeschlagen. Passwort prüfen und noch einmal versuchen.", "Не удалось подключиться. Проверь пароль и попробуй ещё раз."));
    pendingSsid = "";
    pendingPass = "";
    wizRedirect("wifi");
    return;
  }
  wizHead(1, true);
  out(F("<p>")); out(WS("Verbinde mit ", "Подключаюсь к ")); out(F("<b>")); out(esc(pendingSsid)); out(F("</b> ...</p>"));
  wizFoot();
}

void wizStepCity() {
  wizHead(2, false);
  out(F("<form method=\"POST\" action=\"/setup/city\"><p>")); out(WS("Stadt (für das Wetter)", "Город (для погоды)"));
  out(F("</p><input type=\"text\" name=\"city\" value=\"")); out(esc(cityName)); out(F("\" placeholder=\""));
  out(WS("z. B. Köln", "например: Köln")); out(F("\"><input class=\"btn\" type=\"submit\" value=\"")); out(WS("Weiter", "Дальше")); out(F("\"></form>"));
  if (geoOk && cityName.length()) {
    out(F("<p>")); out(WS("Gefunden: ", "Найдено: ")); out(F("<b>")); out(esc(geoLabel)); out(F("</b></p>"));
  }
  out(F("<p><a href=\"/setup?step=cal\">")); out(WS("Ohne Stadt weiter", "Пропустить")); out(F("</a></p>"));
  wizFoot();
}

void wizStepCal() {
  wizHead(3, false);
  out(F("<form method=\"POST\" action=\"/setup/cal\"><p>")); out(WS("Link zum Müllkalender (iCal, https:// oder webcal://)", "Ссылка на календарь вывоза (iCal, https:// или webcal://)"));
  out(F("</p><input type=\"text\" name=\"url\" value=\"")); out(esc(calUrl)); out(F("\" placeholder=\"https://...\">"));
  out(F("<input class=\"btn\" type=\"submit\" value=\"")); out(WS("Laden und weiter", "Загрузить и дальше")); out(F("\"></form><hr>"));
  out(F("<form method=\"POST\" action=\"/setup/upload\" enctype=\"multipart/form-data\"><p>")); out(WS("oder .ics-Datei hochladen", "или загрузить файл .ics"));
  out(F("</p><input type=\"file\" name=\"file\" accept=\".ics\"><input class=\"btn\" type=\"submit\" value=\"")); out(WS("Hochladen und weiter", "Загрузить и дальше")); out(F("\"></form>"));
  out(F("<p><a href=\"/setup?step=opts\">")); out(WS("Ohne Kalender weiter", "Пропустить")); out(F("</a></p>"));
  wizFoot();
}

void wizStepOpts() {
  parseCalendar();
  wizHead(4, false);
  out(F("<form method=\"POST\" action=\"/setup/opts\">"));
  outFieldsDevice();
  out(F("<h3>")); out(WS("Müllarten ignorieren", "Игнорировать виды мусора")); out(F("</h3>"));
  outFieldsIgnore();
  out(F("<input class=\"btn\" type=\"submit\" value=\"")); out(WS("Speichern und neu starten", "Сохранить и перезагрузить")); out(F("\"></form>"));
  wizFoot();
}

void wizDone() {
  wizHead(0, false);
  out(F("<h3>")); out(WS("Fertig! Das Gerät startet neu.", "Готово! Устройство перезагружается.")); out(F("</h3><p>"));
  out(WS("Danach verbinde dich wieder mit deinem Heim-WLAN ", "Потом снова подключись к домашнему WiFi "));
  out(F("<b>")); out(esc(WiFi.SSID())); out(F("</b>. "));
  out(WS("Die Einstellungsseite erreichst du dann unter ", "Страница настроек будет по адресу "));
  out(F("<b>http://")); out(WiFi.localIP().toString()); out(F("</b> ")); out(WS("(steht auch auf der Anzeige).", "(он будет и на экране)."));
  out(F("</p>"));
  wizFoot();
  delay(1500);
  ESP.restart();
}

void setupWebRoutes() {
  server.on("/", HTTP_GET, []() {
    if (portalMode) { server.sendHeader("Location", "/setup", true); server.send(302, "text/plain", ""); }
    else sendPage();
  });

  server.on("/setup", HTTP_GET, []() {
    lastWizardActivity = millis();
    String st = server.arg("step");
    PRF("[web] GET /setup step=%s heap=%u\n", st.c_str(), ESP.getFreeHeap());
    if (st == "city") wizStepCity();
    else if (st == "cal") wizStepCal();
    else if (st == "opts") wizStepOpts();
    else if (st == "wait") wizStepWait();
    else wizStepWifi();
  });

  server.on("/setup/scan", HTTP_GET, []() {
    scanNets();
    wizRedirect("wifi");
  });

  server.on("/setup/wifi", HTTP_POST, []() {
    lastWizardActivity = millis();
    PRF("[web] POST /setup/wifi heap=%u\n", ESP.getFreeHeap());
    String choice = server.arg("choice");
    if (choice == "current" && WiFi.status() == WL_CONNECTED) { wizRedirect("city"); return; }
    String ssid = server.arg("ssid_manual");
    ssid.trim();
    if (ssid.length() == 0) ssid = server.arg("ssid");
    String pass = server.arg("pass");
    if (ssid.length() == 0) {
      wizError = String(LL("Bitte ein Netzwerk wählen.", "Выбери сеть."));
      wizRedirect("wifi");
      return;
    }
    pendingSsid = ssid;
    pendingPass = pass;
    wizConnectStart = millis();
    if (!portalMode) {                          // бот в домашней сети: для смены сети поднимаем свою точку доступа
      startPortal();
      wizHead(1, false);
      out(F("<p>")); out(WS("Das Gerät wechselt jetzt das WLAN. Verbinde dein Handy/deinen PC mit dem WLAN <b>", "Устройство сейчас переключает WiFi. Подключи телефон или ПК к сети <b>"));
      out(AP_NAME); out(F("</b> ")); out(WS("und öffne <b>http://192.168.4.1</b>, um weiterzumachen.", "и открой <b>http://192.168.4.1</b>, чтобы продолжить."));
      out(F("</p>"));
      wizFoot();
      WiFi.begin(ssid.c_str(), pass.c_str());
      return;
    }
    WiFi.begin(ssid.c_str(), pass.c_str());
    wizRedirect("wait");
  });

  server.on("/setup/city", HTTP_POST, []() {
    lastWizardActivity = millis();
    PRF("[web] POST /setup/city heap=%u\n", ESP.getFreeHeap());
    String c = server.arg("city");
    c.trim();
    cityName = c;
    writeFile("/city.txt", cityName);
    wx.ok = false;
    if (c.length() == 0) { geoOk = false; LittleFS.remove("/geo.txt"); wizRedirect("cal"); return; }
    if (WiFi.status() != WL_CONNECTED) {
      wizError = String(LL("Keine Internetverbindung, Stadt kann nicht gesucht werden.", "Нет интернета, город не найти."));
      wizRedirect("city");
      return;
    }
    if (geocodeCity(c)) { fetchWeather(); wizRedirect("cal"); }
    else {
      geoOk = false;
      wizError = String(LL("Stadt nicht gefunden - bitte anders schreiben oder einen größeren Ort in der Nähe wählen.", "Город не найден - напиши иначе или выбери крупный населённый пункт рядом."));
      wizRedirect("city");
    }
  });

  server.on("/setup/cal", HTTP_POST, []() {
    lastWizardActivity = millis();
    PRF("[web] POST /setup/cal heap=%u\n", ESP.getFreeHeap());
    String u = server.arg("url");
    u.trim();
    if (u.startsWith("webcal://")) u = "https://" + u.substring(9);
    calUrl = u;
    writeFile("/url.txt", calUrl);
    if (u.length() == 0) { wizRedirect("opts"); return; }
    if (downloadCalendar()) wizRedirect("opts");
    else {
      wizError = String(LL("Download fehlgeschlagen - Link prüfen.", "Не удалось скачать - проверь ссылку."));
      wizRedirect("cal");
    }
  });

  server.on("/setup/upload", HTTP_POST, []() {
    lastWizardActivity = millis();
    parseCalendar();
    requestRefresh();
    wizRedirect("opts");
  }, uploadChunk);

  server.on("/setup/opts", HTTP_POST, []() {
    lastWizardActivity = millis();
    PRF("[web] POST /setup/opts heap=%u\n", ESP.getFreeHeap());
    applyDeviceArgs();
    applyIgnoreArgs();
    wizDone();                                   // единственная перезагрузка: после всех шагов
  });

  server.onNotFound([]() {
    if (portalMode) {                       // captive portal: любой адрес ведёт на страницу настройки
      server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
      server.send(302, "text/plain", "");
    } else {
      server.send(404, "text/plain", "404");
    }
  });

  server.on("/save_url", HTTP_POST, []() {
    String u = server.arg("url");
    u.trim();
    if (u.startsWith("webcal://")) u = "https://" + u.substring(9);
    calUrl = u;
    writeFile("/url.txt", calUrl);
    if (calUrl.length()) {
      bool ok = downloadCalendar();
      webNotice = ok ? String(LL("Kalender geladen.", "Календарь загружен.")) : String(LL("Download fehlgeschlagen - Link prüfen.", "Не удалось скачать - проверь ссылку."));
    }
    requestRefresh();
    redirectHome();
  });

  server.on("/upload", HTTP_POST, []() {
    parseCalendar();                                   // сразу разобрать загруженный файл
    webNotice = String(LL("Datei hochgeladen. Termine: ", "Файл загружен. События: ")) + String(evTotal);
    requestRefresh();
    redirectHome();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      LittleFS.remove("/calendar.ics");
      File f = LittleFS.open("/calendar.ics", "w");
      if (f) f.close();
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      File f = LittleFS.open("/calendar.ics", "a");
      if (f) {
        f.write(upload.buf, upload.currentSize);
        f.close();
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      saveCalSyncEpoch();
    }
  });

  server.on("/save_city", HTTP_POST, []() {
    String c = server.arg("city");
    c.trim();
    cityName = c;
    writeFile("/city.txt", cityName);
    wx.ok = false;
    if (cityName.length()) {
      if (geocodeCity(cityName)) {
        fetchWeather();
        webNotice = String(LL("Stadt gefunden: ", "Город найден: ")) + geoLabel;
      } else {
        geoOk = false;
        webNotice = String(LL("Stadt nicht gefunden.", "Город не найден."));
      }
    } else {
      geoOk = false;
      LittleFS.remove("/geo.txt");
    }
    requestRefresh();
    redirectHome();
  });

  server.on("/save_device", HTTP_POST, []() {
    applyDeviceArgs();
    webNotice = String(LL("Gespeichert.", "Сохранено."));
    redirectHome();
  });

  server.on("/save_ignore", HTTP_POST, []() {
    String list = "\n";
    for (int i = 0; i < server.args(); i++) {
      if (server.argName(i) == "ig") {
        String v = server.arg(i);
        v.trim();
        if (v.length()) list += v + "\n";
      }
    }
    ignoreList = list;
    writeFile("/ignore.txt", ignoreList);
    webNotice = String(LL("Gespeichert.", "Сохранено."));
    requestRefresh();
    redirectHome();
  });

  server.on("/update", HTTP_POST, []() {
    if (otaOk) {
      server.send(200, "text/plain", String(LL("OK: Update installiert, das Gerät startet neu ...", "OK: прошивка установлена, устройство перезагружается ...")));
      writeFile("/ota.txt", "1");
      delay(800);
      ESP.restart();
    } else {
      otaBusy = false;
      server.send(500, "text/plain", String(LL("Fehler beim Update: ", "Ошибка обновления: ")) + otaErr);
    }
  }, otaUploadChunk);

  server.on("/save_gh", HTTP_POST, []() {
    ghRepo = server.arg("repo");
    ghRepo.trim();
    writeFile("/gh.txt", ghRepo);
    server.send(200, "text/plain", "OK");
  });

  server.on("/refresh", HTTP_POST, []() {
    requestRefresh();
    webNotice = String(LL("Anzeige wird aktualisiert.", "Экран обновляется."));
    redirectHome();
  });

  server.on("/service", HTTP_POST, []() {
    redirectHome();
    parseCalendar();
    fillCommon();
    serviceUntil = millis() + SERVICE_MS;
    renderScreen(S_SERVICE);
  });

  server.on("/keep", HTTP_POST, []() {
    bootWindowActive = false;
    redirectHome();
  });

  server.on("/newdevice", HTTP_POST, []() {      // "настроить как новое" = открыть ассистент (без перезагрузки)
    bootWindowActive = false;
    wizRedirect("wifi");
  });

  server.on("/factory", HTTP_POST, []() {
    server.send(200, "text/plain", "OK - reset...");
    delay(500);
    factoryResetNow();
  });

  server.begin();
  webReady = true;
}

// =====================================================================
//  WLAN: старт, портал, "нет соединения", окно сохранённых данных
// =====================================================================
void scanNets() {
  scanCount = 0;
  sd.netCount = 0;
  int n = WiFi.scanNetworks();
  if (n <= 0) return;
  if (n > 32) n = 32;
  bool used[32];
  for (int i = 0; i < 32; i++) used[i] = false;
  for (int k = 0; k < 10; k++) {
    int best = -1;
    for (int i = 0; i < n; i++) {
      if (used[i]) continue;
      String nm = WiFi.SSID(i);
      if (nm.length() == 0) { used[i] = true; continue; }
      bool dup = false;
      for (int j = 0; j < scanCount; j++) if (scanSsid[j] == nm) dup = true;
      if (dup) { used[i] = true; continue; }
      if (best < 0 || WiFi.RSSI(i) > WiFi.RSSI(best)) best = i;
    }
    if (best < 0) break;
    used[best] = true;
    scanSsid[scanCount] = WiFi.SSID(best);
    scanRssi[scanCount] = WiFi.RSSI(best);
    scanCount++;
  }
  for (int i = 0; i < scanCount && i < 4; i++) {
    sd.netName[i] = scanSsid[i];
    sd.netRssi[i] = scanRssi[i];
    sd.netCount++;
  }
  WiFi.scanDelete();
}

// --- собственная точка доступа настройки WiFi (без WiFiManager) ---
void startPortal() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_NAME);
  delay(200);
  dnsServer.start(53, "*", WiFi.softAPIP());   // все адреса -> наша страница (captive portal)
  portalMode = true;
}

void stopPortal() {
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  portalMode = false;
}

// ждём, пока пользователь введёт WiFi или вернётся сохранённая сеть
void portalLoop(bool hadSaved) {
  fillCommon();
  sd.apName = AP_NAME;
  scanNets();
  startPortal();                                   // сеть настройки поднимаем СРАЗУ, до долгого обновления экрана
  if (hadSaved) {
    sd.failedSsid = savedSsid;
    oledMsg("No WiFi", savedSsid);
    renderScreen(S_NOWIFI);
  } else {
    oledMsg("WLAN-Setup", sd.apName);
    renderScreen(S_SETUP);
  }
  unsigned long lastTry = millis();
  lastWizardActivity = millis();
  while (true) {
    dnsServer.processNextRequest();
    server.handleClient();
    handleSerial();
    pollPending();
    bool conn = (WiFi.status() == WL_CONNECTED);
    if (conn && !hadSaved && wizConnectedAt != 0 && millis() - wizConnectedAt > 10000UL) break;   // WiFi введён: точку доступа выключаем, дальше админка в домашней сети
    if (conn && hadSaved) break;                                        // вернулась сохранённая сеть
    if (conn && millis() - lastWizardActivity > 1200000UL) break;       // ассистент заброшен на 20 минут
    if (!conn && hadSaved && millis() - lastTry > 60000UL) {            // роутер мог перезагрузиться
      lastTry = millis();
      WiFi.begin();
    }
    delay(5);
  }
}

// Копия данных WiFi во флэш-файле: на случай, если SDK потеряет сохранённую сеть
bool loadWifiBackup(String& ssid, String& pass) {
  String f = readFileRaw("/wifi.txt");
  int a = f.indexOf('\n');
  if (a <= 0) return false;
  ssid = f.substring(0, a);
  pass = f.substring(a + 1);
  ssid.trim();
  pass.trim();
  return ssid.length() > 0;
}

void saveWifiBackup(const String& ssid, const String& pass) {
  if (ssid.length() > 0) writeFile("/wifi.txt", ssid + "\n" + pass + "\n");
}

void wifiStartup() {
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  savedSsid = WiFi.SSID();
  String bSsid, bPass;
  bool hasBackup = loadWifiBackup(bSsid, bPass);
  PRF("[wifi] SDK saved SSID: '%s', backup file: '%s'\n", savedSsid.c_str(), hasBackup ? bSsid.c_str() : "none");
  setupWebRoutes();                          // веб-сервер нужен и порталу, и обычному режиму

  bool began = false;
  if (savedSsid.length() == 0 && hasBackup) {
    PRF("[wifi] SDK-конфиг пуст, восстанавливаю из резервного файла\n");
    savedSsid = bSsid;
    WiFi.begin(bSsid.c_str(), bPass.c_str());
    began = true;
  } else if (savedSsid.length() > 0) {
    WiFi.begin();
    began = true;
  }
  if (began) {
    oledMsg("WiFi", savedSsid);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000UL) {
      delay(200);
      handleSerial();
      server.handleClient();
    }
    PRF("[wifi] connect %s after %lu ms\n", WiFi.status() == WL_CONNECTED ? "OK" : "FAILED", millis() - t0);
  }
  if (WiFi.status() != WL_CONNECTED) portalLoop(savedSsid.length() > 0);
  if (portalMode) stopPortal();

  if (WiFi.status() == WL_CONNECTED) {                       // сохранить резерв (в т.ч. для уже настроенных устройств)
    if (pendingSsid.length() > 0) { saveWifiBackup(pendingSsid, pendingPass); pendingSsid = ""; pendingPass = ""; }
    else if (!hasBackup) saveWifiBackup(WiFi.SSID(), WiFi.psk());
  }
  delay(300);
}

void bootWindowIfNeeded() {
  bool reflashed = (readFile("/build.txt") != String(BUILD_ID));
  if (LittleFS.exists("/ota.txt")) { LittleFS.remove("/ota.txt"); reflashed = false; }   // обновление через веб: окно не нужно
  bool hasCal = (calUrl.length() > 0 || LittleFS.exists("/calendar.ics"));
  bool allData = (savedSsid.length() > 0 && hasCal && cityName.length() > 0);   // окно только если заполнено ВСЁ

  if (reflashed && allData) {
    fillCommon();
    sd.savedSsid = savedSsid;
    sd.savedCal = calUrl.length() ? hostOf(calUrl) : String("");
    sd.savedCity = cityName;
    renderScreen(S_SAVED);

    bootWindowActive = true;
    unsigned long t0 = millis();
    while (bootWindowActive && millis() - t0 < BOOT_WINDOW_MS) {
      server.handleClient();
      handleSerial();
#if ENABLE_BUTTON
      if (buttonIsDown()) bootWindowActive = false;
#endif
      delay(5);
    }
    bootWindowActive = false;
  }
  writeFile("/build.txt", String(BUILD_ID));
}

// =====================================================================
//  Настройки: загрузка
// =====================================================================
void loadConfig() {
  String s = readFile("/lang.txt");
  g_lang = (s == "1") ? 1 : ((s == "0") ? 0 : DEFAULT_LANG);
  s = readFile("/name.txt");
  if (s.length()) deviceName = s;
  s = readFile("/tz.txt");
  if (s.length() >= 3) tzString = s;
  calUrl = readFile("/url.txt");
  cityName = readFile("/city.txt");
  ignoreList = readFileRaw("/ignore.txt");
  if (ignoreList.length() == 0) ignoreList = "\n";
  s = readFile("/interval.txt");
  if (s.length() > 0) {
    int rm = s.toInt();
    if (rm < MIN_REFRESH_MIN) rm = MIN_REFRESH_MIN;
    if (rm > 1440) rm = 1440;
    refreshMin = rm;
  }
  if (refreshMin < MIN_REFRESH_MIN) refreshMin = MIN_REFRESH_MIN;
  ghRepo = readFile("/gh.txt");
  loadGeo();
  loadCalSyncEpoch();
}

// =====================================================================
//  Serial-команды
// =====================================================================
void printHelp() {
  Serial.println(F("\n================ MÜLLBOT e-ink  v" FW_VERSION " ================"));
  Serial.println(F("Номера = превью экрана с образцами данных (0 = очистить экран):"));
  Serial.println(F("  1   Сервисный экран (реальные данные, возврат через 2 мин)"));
  Serial.println(F("  2   Главный, обычный день"));
  Serial.println(F("  3   Главный + красная полоса «сегодня мусор»"));
  Serial.println(F("  4   Главный + «завтра мусор» (2 вида)"));
  Serial.println(F("  5   Главный + «сегодня и завтра мусор»"));
  Serial.println(F("  6   Главный + вывезено (красный круг с галочкой)"));
  Serial.println(F("  7   Новый год"));
  Serial.println(F("  8   Настройка WiFi (устройство пустое)"));
  Serial.println(F("  9   Настройка + найдены сохранённые данные"));
  Serial.println(F("  10  Нет соединения с WiFi (+ список сетей)"));
  Serial.println(F("  11  Нет календаря"));
  Serial.println(F("  12  Ожидание времени (NTP)"));
  Serial.println(F("  13  Главный без погоды"));
  Serial.println(F("Команды:"));
  Serial.println(F("  now                    обновить экран по реальным данным"));
  Serial.println(F("  sim 2026-10-28 18:30   подмена даты и времени, sim off = вернуть"));
  Serial.println(F("  done                   переключить «вывезено» (как кнопка)"));
  Serial.println(F("  info                   что бот видит в календаре и состоянии"));
  Serial.println(F("  r                      перевернуть экран на 180 (rotation 1 <-> 3)"));
  Serial.println(F("  x / y                  отразить картинку по X / по Y (подгонка ориентации)"));
#if PANEL_BWRY
  Serial.println(F("  o                      4-цветная панель: перечитать и показать OTP"));
#endif
  Serial.println(F("  c                      чистка от ghosting"));
  Serial.println(F("  ?                      эта справка"));
  Serial.println(F("==================================================\n"));
}

void printInfo() {
  PRF("time valid: %s, fakeOffset: %ld s, tz: %s\n", timeValid() ? "yes" : "no", fakeOffset, tzString.c_str());
  if (timeValid()) {
    time_t now = nowT();
    struct tm t = *localtime(&now);
    PRF("now: %02d.%02d.%04d %02d:%02d:%02d\n", t.tm_mday, t.tm_mon + 1, t.tm_year + 1900, t.tm_hour, t.tm_min, t.tm_sec);
  }
  PRF("calendar: %s, url: %s\n", LittleFS.exists("/calendar.ics") ? "yes" : "NO", calUrl.length() ? calUrl.c_str() : "-");
  parseCalendar();
  PRF("events: %d (shown %d, types %d)\n", evTotal, evShown, typeCount);
  PRF("last : %ld  %s\n", gLast.key, gLast.cnt ? joinNames(gLast).c_str() : "-");
  PRF("today: %ld  %s\n", gToday.key, gToday.cnt ? joinNames(gToday).c_str() : "-");
  PRF("next : %ld  %s\n", gNext.key, gNext.cnt ? joinNames(gNext).c_str() : "-");
  PRF("city: %s, geo: %s (%.4f, %.4f), weather: %s\n", cityName.c_str(), geoOk ? "ok" : "no", geoLat, geoLon, wx.ok ? "ok" : "no data");
  PRF("refresh every %d min\n", refreshMin);
  PRF("done today: %s, screen: %s, heap: %u\n", trashDoneToday ? "yes" : "no", screenName(curScreen), ESP.getFreeHeap());
}

void previewScreen(int n) {
  previewUntil = (n == 1) ? 0 : millis() + 600000UL;      // превью висит до команды now (или 10 минут)
  if (n == 0) {
    oledMsg("Clear", "white");
    pushSolid(C_WHITE);
    return;
  }
  if (n == 1) {
    parseCalendar();
    fillCommon();
    serviceUntil = millis() + SERVICE_MS;
    renderScreen(S_SERVICE);
    return;
  }
  if (n >= 2 && n <= 6) { fillSampleMain(n); renderScreen(S_MAIN); return; }
  if (n == 13)          { fillSampleMain(13); renderScreen(S_MAIN); return; }

  fillCommon();
  switch (n) {
    case 7:
      sd.year = 2027;
      renderScreen(S_NEUJAHR);
      break;
    case 8:
      sd.apName = AP_NAME;
      renderScreen(S_SETUP);
      break;
    case 9:
      sd.savedSsid = "FritzBox-7590";
      sd.savedCal = "abfall.example.de";
      sd.savedCity = "Dortmund";
      sd.ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("192.168.1.42");
      renderScreen(S_SAVED);
      break;
    case 10:
      sd.failedSsid = "FritzBox-7590";
      sd.netCount = 4;
      sd.netName[0] = "Nachbar-WLAN";   sd.netRssi[0] = -48;
      sd.netName[1] = "FRITZ!Box 6660"; sd.netRssi[1] = -67;
      sd.netName[2] = "Telekom_FON";    sd.netRssi[2] = -78;
      sd.netName[3] = "Vodafone-A12B";  sd.netRssi[3] = -88;
      renderScreen(S_NOWIFI);
      break;
    case 11:
      sd.ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("192.168.1.42");
      sd.dateShort = LL("So, 25.10.2026", "Вс, 25.10.2026");
      sd.dispTime = "14:00"; sd.calTime = "--"; sd.wxShow = false; sd.city = "";
      renderScreen(S_NOCAL);
      break;
    case 12:
      sd.ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("192.168.1.42");
      renderScreen(S_NONTP);
      break;
    default:
      Serial.println("Нет такого экрана. Доступно 0..13 (? = справка)");
      break;
  }
}

bool isNumber(const String& s) {
  if (s.length() == 0) return false;
  for (unsigned i = 0; i < s.length(); i++) if (!isDigit(s[i])) return false;
  return true;
}

void runCommand(String cmd) {
  cmd.trim();
  String low = cmd;
  low.toLowerCase();
  if (low.length() == 0) return;

  if (isNumber(low)) { previewScreen(low.toInt()); return; }
  if (low == "?" || low == "h" || low == "help") { printHelp(); return; }
  if (low == "now")  { serviceUntil = 0; previewUntil = 0; requestRefresh(); return; }
  if (low == "info") { printInfo(); return; }
  if (low == "done") {
    trashDoneToday = !trashDoneToday;
    requestRefresh();
    Serial.println(trashDoneToday ? "done = yes" : "done = no");
    return;
  }
  if (low == "c") { cleanCycle(); return; }
  if (low == "r") {
    rot = (rot == 1) ? 3 : 1;
    gfx.setRotation(rot);
    PRF("rotation = %d, команда now перерисует экран\n", rot);
    return;
  }
#if PANEL_BWRY
  if (low == "o") { otpOk = false; readOtpAuto(); printOtpDump(); return; }
#endif
  if (low == "x") { flipX = !flipX; PRF("flipX = %d, команда now перерисует экран\n", flipX ? 1 : 0); return; }
  if (low == "y") { flipY = !flipY; PRF("flipY = %d, команда now перерисует экран\n", flipY ? 1 : 0); return; }
  if (low == "sim off") {
    fakeOffset = 0; requestRefresh();
    Serial.println("sim off");
    return;
  }
  if (low.startsWith("sim ")) {
    int Y, M, D, h, m;
    if (sscanf(low.c_str() + 4, "%d-%d-%d %d:%d", &Y, &M, &D, &h, &m) == 5 && timeValid()) {
      struct tm tmv;
      memset(&tmv, 0, sizeof(tmv));
      tmv.tm_year = Y - 1900; tmv.tm_mon = M - 1; tmv.tm_mday = D;
      tmv.tm_hour = h; tmv.tm_min = m; tmv.tm_isdst = -1;
      time_t target = mktime(&tmv);
      fakeOffset = (long)(target - time(nullptr));
      serviceUntil = 0; requestRefresh();
      PRF("sim: %04d-%02d-%02d %02d:%02d\n", Y, M, D, h, m);
    } else {
      Serial.println("формат: sim 2026-10-28 18:30  (время уже должно быть синхронизировано)");
    }
    return;
  }
  Serial.println("Неизвестная команда: " + cmd + "  (? = справка)");
}

void handleSerial() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    String echo = cmd;
    echo.trim();
    if (echo.length() > 0) PRF("[cmd] '%s'\n", echo.c_str());
    runCommand(cmd);
  }
}

// =====================================================================
//  Главный цикл: раз в час + по событиям
// =====================================================================
void checkRender() {
  if (otaBusy) return;
  if (serviceUntil) {
    if (millis() < serviceUntil) return;       // сервисный экран ещё висит
    serviceUntil = 0;
    forceRender = true;
    lastRenderMs = 0;
  }
  if (previewUntil) {
    if (millis() < previewUntil) return;       // превью по номеру висит, пока не придёт now
    previewUntil = 0;
    forceRender = true;
    lastRenderMs = 0;
  }

  long slot = -1;
  if (timeValid()) {
    time_t now = nowT();
    struct tm t = *localtime(&now);
    if (t.tm_yday != lastYday) { lastYday = t.tm_yday; trashDoneToday = false; }
    slot = (long)(now / ((long)refreshMin * 60L));      // слоты привязаны к часам: каждые refreshMin минут
  }

  bool slotChanged = (slot != lastHourKey);
  if (!slotChanged && !forceRender) return;
  if (lastRenderMs != 0 && millis() - lastRenderMs < MIN_RENDER_GAP) return;

  lastHourKey = slot;
  forceRender = false;

  if (slotChanged && geoOk && (lastWeatherMs == 0 || millis() - lastWeatherMs >= 600000UL)) {   // погода не чаще раза в 10 минут
    oledMsg("Weather", "updating...");
    fetchWeather();
  }
  renderReal();
}

void autoUpdateCalendar() {
  if (calUrl.length() > 0 && millis() >= netBlockedUntil && (millis() - lastAutoUpdate >= AUTO_UPDATE_INTERVAL)) {
    lastAutoUpdate = millis();
    oledMsg("Calendar", "downloading...");
    downloadCalendar();
  }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  delay(500);
  Serial.println(F("\n[INFO] MuellBot e-ink v" FW_VERSION " start"));
  PRF("[boot] free heap at start: %u\n", ESP.getFreeHeap());
  printHelp();

  Wire.begin(4, 5);
  oledOk = displayOLED.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledOk) Serial.println(F("OLED failed!"));

  // порядок как в рабочем тесте экрана
  pinMode(BUSY_PIN, INPUT);
  pinMode(RES_PIN, OUTPUT);
  pinMode(DC_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
#if ENABLE_BUTTON && BUTTON_TYPE == 1
  pinMode(FLASH_PIN, INPUT_PULLUP);      // кнопка FLASH на D3: нажата = LOW (не держи её при включении питания!)
#endif
#if PANEL_BWRY
  digitalWrite(DC_PIN, HIGH);
  digitalWrite(RES_PIN, HIGH);
  spiStart();
  PRF("[panel] type: BWRY 4-color (SE2266QS0F9)\n");
  readOtpAuto();                       // OTP читаем ДО запуска WiFi, пока радио молчит (как в тестере)
#else
  PRF("[panel] type: BWR 3-color (VUSION)\n");
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);
#endif

#if ENABLE_LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
#endif

  gfx.setRotation(rot);
  uf.begin(gfx);
  uf.setFontMode(1);
  uf.setFontDirection(0);

  oledMsg("Booting...", "LittleFS");
  if (!LittleFS.begin()) {
    LittleFS.format();
    LittleFS.begin();
  }
  loadConfig();
  crashedOp = readFile("/guard.txt");
  if (crashedOp.length() > 0) {
    skipNet = true;
    netBlockedUntil = millis() + 600000UL;
    LittleFS.remove("/guard.txt");
    PRF("[guard] прошлый запуск упал при обращении к сети (%s): сетевые запросы пропускаю 10 минут\n", crashedOp.c_str());
    webNotice = String("Letzter Netzwerkzugriff (") + crashedOp + ") ist abgestürzt und wurde übersprungen. / Прошлое обращение к сети (" + crashedOp + ") привело к сбою и пропущено.";
  }
  configTime(tzString.c_str(), "pool.ntp.org", "time.nist.gov");

  wifiStartup();
  oledMsg("WiFi OK", WiFi.localIP().toString());

  unsigned long t0 = millis();
  while (!timeValid() && millis() - t0 < 20000UL) {
    server.handleClient();
    handleSerial();
    delay(200);
  }
  oledMsg(timeValid() ? "NTP OK" : "NTP timeout", "");

  bootWindowIfNeeded();

  // сразу после старта: свежий календарь, координаты города (погоду возьмёт первая отрисовка)
  if (!skipNet && calUrl.length() > 0) {
    oledMsg("Calendar", "downloading...");
    downloadCalendar();
  }
  if (!skipNet && cityName.length() > 0 && !geoOk) {
    oledMsg("City", "looking up...");
    geocodeCity(cityName);
  }
  printHelp();
}

void loop() {
  if (portalMode) dnsServer.processNextRequest();
  pollPending();
  server.handleClient();
  handleSerial();
  handleButton();
  updateLed();
  autoUpdateCalendar();
  checkRender();
  oledStatus();
  delay(5);
}
