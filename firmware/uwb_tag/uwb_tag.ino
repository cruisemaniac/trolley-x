/* RYUW122 operator tag. Keep the network and address in sync with the ROS ranging node. Use 3V3, GPIO25/26 for serial and GPIO27 for NRST. */

#include <Arduino.h>

// ---- Config (must match the ROS uwb_ranging node) ----
#define UWB_SERIAL   Serial2
#define UWB_BAUD     115200      // RYUW122 default; try 9600 if it never answers
#define UWB_RX_PIN   25          // ESP32 receives; wire to RYUW122 TXD
#define UWB_TX_PIN   26          // ESP32 transmits; wire to RYUW122 RXD
#define UWB_RST_PIN  27          // RYUW122 NRST - pulsed low->high, then held high

#define TAG_ADDRESS  "TAG00001"
#define NETWORK_ID   "TROLLEYX"
#define CPIN         ""          // 32 hex chars for AES; "" = leave encryption off

#define DBG          Serial      // USB serial monitor
#define DBG_BAUD     115200
#define STATUS_LED   2           // onboard LED on most ESP32 boards; -1 to disable

#define KEEPALIVE_MS 3000UL      // link-check period
#define AT_TIMEOUT_MS 600UL      // per-command reply wait
#define RST_HOLD_MS  150UL       // NRST assert time (matches the Pi reset service)
#define RST_BOOT_MS  400UL       // wait after release before talking

// ---- helpers ----
static String readReply(uint32_t timeout) {
  String acc;
  uint32_t start = millis();
  while (millis() - start < timeout) {
    while (UWB_SERIAL.available()) acc += (char)UWB_SERIAL.read();
    if (acc.indexOf("+OK") >= 0 || acc.indexOf("ERR") >= 0) break;
    delay(2);
  }
  return acc;
}

static bool sendAT(const char* cmd, uint32_t timeout = AT_TIMEOUT_MS) {
  while (UWB_SERIAL.available()) UWB_SERIAL.read();   // clear input
  UWB_SERIAL.print(cmd);
  UWB_SERIAL.print("\r\n");
  DBG.print(">> "); DBG.println(cmd);
  String r = readReply(timeout);
  r.trim();
  if (r.length()) { DBG.print("<< "); DBG.println(r); }
  return r.indexOf("+OK") >= 0;
}

// Pulse NRST low->high and hold high. The module is mute until this edge.
static void resetModule() {
  pinMode(UWB_RST_PIN, OUTPUT);
  digitalWrite(UWB_RST_PIN, LOW);    // assert reset
  delay(RST_HOLD_MS);
  digitalWrite(UWB_RST_PIN, HIGH);   // release and hold high
  delay(RST_BOOT_MS);                // let the module boot
}

static bool configureTag() {
  bool ok = true;
  ok &= sendAT("AT+MODE=0");                 // 0 = TAG
  ok &= sendAT("AT+NETWORKID=" NETWORK_ID);
  ok &= sendAT("AT+ADDRESS=" TAG_ADDRESS);
  if (strlen(CPIN) > 0) ok &= sendAT("AT+CPIN=" CPIN);
  DBG.println(ok ? "[tag] configured OK"
                 : "[tag] CONFIG FAILED - check wiring/baud/pins/NRST");
  return ok;
}

// Reset then configure. Use at boot and on every recovery.
static bool bringUp() {
  resetModule();
  return configureTag();
}

static bool linkAlive() {
  return sendAT("AT");                        // module answers +OK to a bare AT
}

static void setLed(bool on) {
#if STATUS_LED >= 0
  digitalWrite(STATUS_LED, on ? HIGH : LOW);
#endif
}

void setup() {
  DBG.begin(DBG_BAUD);
#if STATUS_LED >= 0
  pinMode(STATUS_LED, OUTPUT);
#endif
  pinMode(UWB_RST_PIN, OUTPUT);
  digitalWrite(UWB_RST_PIN, HIGH);   // idle high until the first reset pulse
  delay(200);
  DBG.println("\n[tag] Trolley-X UWB tag booting");
  UWB_SERIAL.begin(UWB_BAUD, SERIAL_8N1, UWB_RX_PIN, UWB_TX_PIN);
  delay(100);

  uint8_t tries = 0;
  while (!bringUp()) {               // reset + configure, retry until it answers
    setLed(false);
    DBG.println("[tag] retrying (reset+config) in 1 s...");
    delay(1000);
    if (++tries >= 10) {
      DBG.println("[tag] still failing - check NRST wiring (GPIO27) and baud");
      break;
    }
  }
  setLed(true);
}

void loop() {
  static uint32_t lastCheck = 0;
  static bool led = true;

  // pass module output (e.g. +ANCHOR_RCV echoes) to the debug console
  while (UWB_SERIAL.available()) DBG.write(UWB_SERIAL.read());

  if (millis() - lastCheck >= KEEPALIVE_MS) {
    lastCheck = millis();
    if (linkAlive()) {
      led = !led; setLed(led);                // heartbeat: link OK
    } else {
      DBG.println("[tag] link lost - resetting + reconfiguring");
      setLed(false);
      bringUp();                              // re-pulse NRST, then reconfigure
    }
  }
}
