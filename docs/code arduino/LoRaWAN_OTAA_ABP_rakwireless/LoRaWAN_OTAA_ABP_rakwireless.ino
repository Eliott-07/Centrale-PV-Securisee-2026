#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <LoRaWan-RAK4630.h>

// ================= LED =================
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

// ================= GPS =================
TinyGPS gps;

// ================= LORAWAN =================
bool doOTAA = true;

#define LORAWAN_DATERATE DR_0
#define LORAWAN_TX_POWER TX_POWER_5
#define JOINREQ_NBTRIALS 3

DeviceClass_t g_CurrentClass = CLASS_A;
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_EU868;
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;
uint8_t gAppPort = 2;

static lmh_param_t g_lora_param_init =
{
  LORAWAN_ADR_ON,
  LORAWAN_DATERATE,
  LORAWAN_PUBLIC_NETWORK,
  JOINREQ_NBTRIALS,
  LORAWAN_TX_POWER,
  LORAWAN_DUTYCYCLE_OFF
};

// ================= CALLBACKS =================
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);
void tx_lora_periodic_handler(void);

// ================= CALLBACK STRUCT =================
static lmh_callback_t g_lora_callbacks =
{
  BoardGetBatteryLevel,
  BoardGetUniqueId,
  BoardGetRandomSeed,
  lorawan_rx_handler,
  lorawan_has_joined_handler,
  lorawan_confirm_class_handler,
  lorawan_join_failed_handler
};

// ================= OTAA KEYS =================
uint8_t nodeDeviceEUI[8] = {0x70,0xB3,0xD5,0x7E,0xD0,0x07,0x5D,0x2B};
uint8_t nodeAppEUI[8]    = {0xBC,0xD3,0x0A,0xA2,0x49,0x06,0x8F,0x85};
uint8_t nodeAppKey[16]   = {
  0x60,0x79,0x83,0xE9,0x9B,0x3E,0x78,0x76,
  0xE7,0xE8,0x2A,0x82,0x85,0xAC,0x1A,0xE0
};

// ================= PAYLOAD =================
#define LORAWAN_APP_DATA_BUFF_SIZE 64
#define LORAWAN_APP_INTERVAL 60000   // 60 secondes

static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];

static lmh_app_data_t m_lora_app_data =
{
  m_lora_app_data_buffer,
  0,
  0,
  0,
  0
};

// ================= TIMER =================
TimerEvent_t appTimer;


// =================================================
// SETUP
// =================================================
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  delay(3000);

  Serial.println("RAK4630 + RAK1910 GPS + LoRaWAN");

  // ===== GPS POWER =====
  pinMode(WB_IO2, OUTPUT);

  digitalWrite(WB_IO2, LOW);
  delay(1000);

  digitalWrite(WB_IO2, HIGH);
  delay(3000);

  Serial.println("GPS powered");

  Serial1.begin(9600);
  Serial.println("GPS UART started");

  // ===== LORA =====
  lora_rak4630_init();

  TimerInit(&appTimer, tx_lora_periodic_handler);

  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);

  lmh_init(&g_lora_callbacks,
           g_lora_param_init,
           doOTAA,
           g_CurrentClass,
           g_CurrentRegion);

  Serial.println("Joining LoRaWAN...");
  lmh_join();
}


// =================================================
// LOOP
// =================================================
void loop()
{
  while (Serial1.available())
  {
    char c = Serial1.read();

    Serial.write(c);     // affiche trame NMEA
    gps.encode(c);
  }
}


// =================================================
// LORAWAN CALLBACKS
// =================================================
void lorawan_has_joined_handler(void)
{
  Serial.println("Joined LoRaWAN network");

  TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
  TimerStart(&appTimer);
}

void lorawan_join_failed_handler(void)
{
  Serial.println("Join failed");
}

void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("RX RSSI:%d SNR:%d\n",
                app_data->rssi,
                app_data->snr);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("Device Class %c active\n", "ABC"[Class]);
}


// =================================================
// SEND GPS
// =================================================
void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    Serial.println("Not joined yet");
    return;
  }

  float lat, lon;
  unsigned long age;

  gps.f_get_position(&lat, &lon, &age);

  Serial.println("----- GPS DATA -----");

  Serial.print("Latitude: ");
  Serial.println(lat, 6);

  Serial.print("Longitude: ");
  Serial.println(lon, 6);

  Serial.print("Age: ");
  Serial.println(age);

  if (lat == TinyGPS::GPS_INVALID_F_ANGLE)
  {
    Serial.println("No GPS fix yet...");
    return;
  }

  Serial.println("GPS FIX OK -> sending");

  int32_t latitude  = lat * 100000;
  int32_t longitude = lon * 100000;

  uint8_t i = 0;

  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);

  m_lora_app_data.port = gAppPort;

  // payload format
  m_lora_app_data.buffer[i++] = 0x02;

  m_lora_app_data.buffer[i++] = (latitude >> 24) & 0xFF;
  m_lora_app_data.buffer[i++] = (latitude >> 16) & 0xFF;
  m_lora_app_data.buffer[i++] = (latitude >> 8)  & 0xFF;
  m_lora_app_data.buffer[i++] = latitude & 0xFF;

  m_lora_app_data.buffer[i++] = (longitude >> 24) & 0xFF;
  m_lora_app_data.buffer[i++] = (longitude >> 16) & 0xFF;
  m_lora_app_data.buffer[i++] = (longitude >> 8)  & 0xFF;
  m_lora_app_data.buffer[i++] = longitude & 0xFF;

  m_lora_app_data.buffer[i++] = 0x5D;

  m_lora_app_data.buffsize = i;

  if (lmh_send(&m_lora_app_data, g_CurrentConfirm) == LMH_SUCCESS)
  {
    Serial.print("Payload sent: ");

    for (uint8_t j = 0; j < i; j++)
    {
      Serial.printf("%02X ", m_lora_app_data.buffer[j]);
    }

    Serial.println();

    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
  }
  else
  {
    Serial.println("Send failed");
  }
}


// =================================================
// TIMER
// =================================================
void tx_lora_periodic_handler(void)
{
  TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
  TimerStart(&appTimer);

  send_lora_frame();
}