#include "LoRaWan_APP.h"
#include "Arduino.h"

// ---------- Config radio ----------
#define RF_FREQUENCY               915000000UL
#define LORA_BANDWIDTH             0       // 0:125k, 1:250k, 2:500k
#define LORA_SF                    7
#define LORA_CR                    1       // 1:4/5, 2:4/6, 3:4/7, 4:4/8
#define LORA_PREAMBLE_LENGTH       8
#define LORA_SYMBOL_TIMEOUT        0
#define LORA_FIX_LENGTH_PAYLOAD_ON false   // <- DEBE ser false para coincidir con el TX de Heltec
#define LORA_IQ_INVERSION_ON       false
#define RX_CONTINUOUS              true

// ---------- Payload ----------
#define PAYLOAD_SIZE_HINT          220     // sugerencia (no se fuerza porque fixLen=false)

// ---------- Pérdida (opcional) ----------
#define USE_SEQ_HEADER             false    // pon true si el TX manda uint32_t seq al inicio

// ---------- Ventana de métricas ----------
#define WINDOW_MS                  3000

static RadioEvents_t RadioEvents;
volatile bool rxIdle = true;

uint32_t win_bytes = 0, win_pkts = 0, t_win = 0;

// RSSI/SNR stats
float rssi_sum = 0.0f, snr_sum = 0.0f;
float rssi_min =  999.0f, rssi_max = -999.0f;
float snr_min  =  999.0f, snr_max  = -999.0f;

// Pérdida por secuencia (si se usa cabecera)
uint32_t last_seq = 0, lost_in_win = 0;

static void printPacketLine(uint8_t* p, uint16_t size, int16_t rssi, int8_t snr) {
  Serial.printf("[PKT] len=%u  RSSI=%d dBm  SNR=%d dB", (unsigned)size, (int)rssi, (int)snr);
#if USE_SEQ_HEADER
  if (size >= sizeof(uint32_t)) {
    uint32_t seq;
    memcpy(&seq, p, sizeof(uint32_t));
    Serial.printf("  seq=%lu", (unsigned long)seq);
  }
#endif
  Serial.println();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  Radio.Sleep();

  // Contabiliza bytes/paquetes
  win_bytes += size;
  win_pkts  += 1;

  // RSSI/SNR para estadística
  rssi_sum += (float)rssi;
  snr_sum  += (float)snr;
  if (rssi < rssi_min) rssi_min = (float)rssi;
  if (rssi > rssi_max) rssi_max = (float)rssi;
  if (snr  < snr_min ) snr_min  = (float)snr;
  if (snr  > snr_max ) snr_max  = (float)snr;

#if USE_SEQ_HEADER
  // pérdida por secuencia (si TX envía uint32_t seq al inicio)
  if (size >= sizeof(uint32_t)) {
    uint32_t seq; memcpy(&seq, payload, sizeof(uint32_t));
    if (last_seq && seq > last_seq + 1) {
      lost_in_win += (seq - last_seq - 1);
    }
    last_seq = seq;
  }
#endif

  printPacketLine(payload, size, rssi, snr);

  rxIdle = true;   // listo para escuchar otra vez
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.RxDone = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  // SetRxConfig(fixLen=false) para aceptar payload variable (match con TX Heltec example)
  Radio.SetRxConfig(
    MODEM_LORA,
    LORA_BANDWIDTH,
    LORA_SF,
    LORA_CR,
    0,                        // BW AFC (no aplica en LoRa)
    LORA_PREAMBLE_LENGTH,
    LORA_SYMBOL_TIMEOUT,
    LORA_FIX_LENGTH_PAYLOAD_ON,
    LORA_FIX_LENGTH_PAYLOAD_ON ? PAYLOAD_SIZE_HINT : 0, // si fixLen=false, ignora
    true,                     // CRC ON
    false,                    // freqHopOn
    0,                        // hopPeriod
    LORA_IQ_INVERSION_ON,
    RX_CONTINUOUS             // continuous RX
  );

  t_win = millis();
}

void loop() {
  // Arranca escucha si está idle
  if (rxIdle) {
    rxIdle = false;
    Radio.Rx(0);  // 0 = esperar indefinidamente (continuous)
  }

  // Ventana de métricas
  uint32_t now = millis();
  if (now - t_win >= WINDOW_MS) {
    float dt = (now - t_win) / 1000.0f;
    float throughput_bps = (win_bytes * 8.0f) / dt;
    float pps = (win_pkts) / dt;

#if USE_SEQ_HEADER
    uint32_t total = win_pkts + lost_in_win;
    float loss_pct = (total ? (100.0f * lost_in_win / total) : 0.0f);
#else
    float loss_pct = 0.0f;
#endif

    float rssi_avg = (win_pkts ? rssi_sum / win_pkts : 0.0f);
    float snr_avg  = (win_pkts ? snr_sum  / win_pkts : 0.0f);

    Serial.printf(
      "[RX] win=%.2fs  bytes=%lu  pkts=%lu  throughput=%.0f bps  pps=%.2f"
#if USE_SEQ_HEADER
      "  loss=%.2f%%"
#endif
      "  RSSI(avg/min/max)=%.1f/%.1f/%.1f dBm  SNR(avg/min/max)=%.1f/%.1f/%.1f dB\r\n",
      dt,
      (unsigned long)win_bytes,
      (unsigned long)win_pkts,
      throughput_bps,
      pps,
#if USE_SEQ_HEADER
      loss_pct,
#endif
      rssi_avg, rssi_min, rssi_max,
      snr_avg,  snr_min,  snr_max
    );

    // Reset ventana
    win_bytes = 0; win_pkts = 0;
    rssi_sum = snr_sum = 0.0f;
    rssi_min =  999.0f; rssi_max = -999.0f;
    snr_min  =  999.0f; snr_max  = -999.0f;
#if USE_SEQ_HEADER
    lost_in_win = 0;
#endif
    t_win = now;
  }

  // Procesar IRQ de la radio (DIO1)
  Radio.IrqProcess();
}
