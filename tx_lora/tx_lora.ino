/* Heltec Automation send + METRICAS (API Radio.Send = void) */
#include "LoRaWan_APP.h"
#include "Arduino.h"

#define RF_FREQUENCY                 915000000UL // Hz
#define TX_OUTPUT_POWER              5           // dBm
#define LORA_BANDWIDTH               0           // 0:125k,1:250k,2:500k
#define LORA_SPREADING_FACTOR        7
#define LORA_CODINGRATE              1           // 1:4/5..4:4/8
#define LORA_PREAMBLE_LENGTH         8
#define LORA_FIX_LENGTH_PAYLOAD_ON   false
#define LORA_IQ_INVERSION_ON         false
#define RX_TIMEOUT_VALUE             1000

#define BUFFER_SIZE                  30          // tamaño del payload
#define TX_INTERVAL_MS               1000        // periodo entre envíos
#define WINDOW_MS                    3000        // ventana de métricas

static RadioEvents_t RadioEvents;

char     txpacket[BUFFER_SIZE];
double   txNumber = 0;
bool     lora_idle = true;

volatile uint32_t win_bytes = 0;
volatile uint32_t win_pkts  = 0;
volatile uint32_t win_airtime_ms = 0;

uint32_t t_last_tx_start = 0;
uint32_t t_win_start     = 0;
size_t   last_len        = 0;

void OnTxDone(void);
void OnTxTimeout(void);

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0,
                    LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  t_win_start = millis();
}

void loop() {
  // Envío periódico cuando la radio está libre
  if (lora_idle) {
    static uint32_t t_last = 0;
    if (millis() - t_last >= TX_INTERVAL_MS) {
      t_last = millis();

      txNumber += 0.01;
      snprintf(txpacket, BUFFER_SIZE, "Hello world number %0.2f", txNumber);
      last_len = strlen(txpacket);

      Serial.printf("\r\n[TX] sending \"%s\" len=%u\r\n", txpacket, (unsigned)last_len);

      // En esta API Radio.Send NO devuelve estado -> solo llama y esperamos callbacks
      Radio.Send((uint8_t*)txpacket, last_len);
      lora_idle = false;                 // envío en curso
      t_last_tx_start = millis();        // para estimar airtime
    }
  }

  // Procesa IRQ (DIO1)
  Radio.IrqProcess();

  // Métricas por ventana
  uint32_t now = millis();
  if (now - t_win_start >= WINDOW_MS) {
    float dt  = (now - t_win_start) / 1000.0f;
    float bps = (win_bytes * 8.0f) / dt;
    float pps = (win_pkts) / dt;
    float avg_len   = (win_pkts ? (float)win_bytes / win_pkts : 0.0f);
    float avg_airms = (win_pkts ? (float)win_airtime_ms / win_pkts : 0.0f);

    Serial.printf("[METRICS] window=%.2fs  bytes=%lu  pkts=%lu  throughput=%.0f bps  pps=%.2f  avg_len=%.1f B  airtime_avg=%.1f ms\r\n",
                  dt, (unsigned long)win_bytes, (unsigned long)win_pkts,
                  bps, pps, avg_len, avg_airms);

    win_bytes = 0;
    win_pkts = 0;
    win_airtime_ms = 0;
    t_win_start = now;
  }
}

void OnTxDone(void) {
  uint32_t air_ms = millis() - t_last_tx_start;  // estimación
  win_airtime_ms += air_ms;
  win_bytes      += last_len;
  win_pkts       += 1;

  Serial.printf("TX done (len=%u, airtime~%ums)\r\n", (unsigned)last_len, air_ms);
  lora_idle = true;
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("TX Timeout...");
  // no sumamos métricas en timeout
  lora_idle = true;
}
