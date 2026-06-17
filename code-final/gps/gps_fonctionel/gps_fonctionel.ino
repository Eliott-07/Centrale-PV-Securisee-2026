#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <LoRaWan-RAK4630.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

TinyGPS gps;

// buffer NMEA
String nmeaSentence = "";

// ================= LORA =================

bool doOTAA = true;

#define LORAWAN_DATERATE DR_0
#define LORAWAN_TX_POWER TX_POWER_5
#define JOINREQ_NBTRIALS 3

DeviceClass_t g_CurrentClass = CLASS_A;
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_EU868;

lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;

uint8_t gAppPort = 2;

#define LORAWAN_APP_INTERVAL 60000   // 60 sec conseillé

static lmh_param_t g_lora_param_init =
{
  LORAWAN_ADR_ON,
  LORAWAN_DATERATE,
  LORAWAN_PUBLIC_NETWORK,
  JOINREQ_NBTRIALS,
  LORAWAN_TX_POWER,
  LORAWAN_DUTYCYCLE_OFF
};

// ================= KEYS =================

uint8_t nodeDeviceEUI[8] =
{ 0x70,0xB3,0xD5,0x7E,0xD0,0x07,0x5D,0x2B };

uint8_t nodeAppEUI[8] =
{ 0xBC,0xD3,0x0A,0xA2,0x49,0x06,0x8F,0x85 };

uint8_t nodeAppKey[16] =
{
  0x60,0x79,0x83,0xE9,0x9B,0x3E,0x78,0x76,
  0xE7,0xE8,0x2A,0x82,0x85,0xAC,0x1A,0xE0
};

// ================= PAYLOAD =================

#define LORAWAN_APP_DATA_BUFF_SIZE 64

static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];

static lmh_app_data_t m_lora_app_data =
{
  m_lora_app_data_buffer,
  0,
  0,
  0,
  0
};

TimerEvent_t appTimer;


// ================= FUNCTIONS =================

void send_lora_frame(void);

void tx_lora_periodic_handler(void)
{
  TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
  TimerStart(&appTimer);

  send_lora_frame();
}


// ================= CALLBACKS =================

void lorawan_has_joined_handler(void)
{
  Serial.println("JOINED");

  TimerStart(&appTimer);
}

void lorawan_join_failed_handler(void)
{
  Serial.println("JOIN FAILED");
}

void lorawan_rx_handler(lmh_app_data_t *app_data) {}

void lorawan_confirm_class_handler(DeviceClass_t Class) {}

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


// ================= SETUP =================

void setup()
{
  Serial.begin(115200);
  delay(3000);

  pinMode(LED_BUILTIN, OUTPUT);

  // power GPS
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, LOW);
  delay(1000);
  digitalWrite(WB_IO2, HIGH);
  delay(3000);

  Serial1.begin(9600);

  // init LoRa
  lora_rak4630_init();

  TimerInit(&appTimer, tx_lora_periodic_handler);
  TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);

  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);

  lmh_init(&g_lora_callbacks,
           g_lora_param_init,
           doOTAA,
           g_CurrentClass,
           g_CurrentRegion);

  Serial.println("JOIN TTN");

  lmh_join();
}


// ================= LOOP =================

void loop()
{
  while (Serial1.available())
  {
    char c = Serial1.read();

    // construire phrase NMEA
    if (c == '\n')
    {
      if (nmeaSentence.startsWith("$GPGGA"))
      {
        Serial.println(nmeaSentence);
      }

      nmeaSentence = "";
    }
    else
    {
      nmeaSentence += c;
    }
  }
}


// ================= SEND =================

void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
    return;

  if (!nmeaSentence.startsWith("$GPGGA"))
  {
    Serial.println("No GPGGA");
    return;
  }

  Serial.println("Sending GPGGA");

  uint8_t len = nmeaSentence.length();

  if (len > 51) len = 51;   // limite LoRa

  memcpy(m_lora_app_data.buffer,
         nmeaSentence.c_str(),
         len);

  m_lora_app_data.buffsize = len;
  m_lora_app_data.port = gAppPort;

  if (lmh_send(&m_lora_app_data,
               g_CurrentConfirm) == LMH_SUCCESS)
  {
    Serial.println("SENT");

    digitalWrite(LED_BUILTIN, HIGH);
    delay(80);
    digitalWrite(LED_BUILTIN, LOW);
  }
}