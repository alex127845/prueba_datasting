#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "FS.h"
#include "LittleFS.h"

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

// ---------- File transfer constants ----------
#define HEADER_SIZE                6       // packet header size
#define MAX_DATA_SIZE             (PAYLOAD_SIZE_HINT - HEADER_SIZE)
#define RECEIVED_FILE_PATH        "/received_file.pdf"
#define ACK_TIMEOUT_MS            1000     // time to send ACK after receiving packet

// Packet types
#define PKT_FILE_START            0x01
#define PKT_FILE_DATA             0x02 
#define PKT_FILE_END              0x03
#define PKT_ACK                   0x04
#define PKT_NACK                  0x05

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

// File transfer state
enum RxTransferState {
  RX_IDLE,
  RX_RECEIVING_FILE,
  RX_FILE_COMPLETE
};

RxTransferState rx_transfer_state = RX_IDLE;
File received_file;
String received_filename;
uint32_t expected_file_size = 0;
uint32_t bytes_received = 0;
uint16_t expected_total_fragments = 0;
uint16_t next_expected_fragment = 0;
bool* fragment_received;  // bitmap to track received fragments
uint32_t last_rx_time = 0;

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

// File transfer functions
void sendAck(void);
void sendNack(void);
void handleFileStartPacket(uint8_t* payload, uint16_t size);
void handleFileDataPacket(uint8_t* payload, uint16_t size);
void handleFileEndPacket(uint8_t* payload, uint16_t size);
bool initFileReception(const String& filename, uint32_t file_size, uint16_t total_fragments);
void resetFileTransfer(void);
uint32_t calculateCRC32(const uint8_t* data, size_t length);
bool validateFileIntegrity(uint32_t expected_crc);

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

  // Check if this is a file transfer packet
  if (size >= HEADER_SIZE) {
    uint8_t pkt_type = payload[0];
    
    switch (pkt_type) {
      case PKT_FILE_START:
        handleFileStartPacket(payload, size);
        sendAck();
        break;
        
      case PKT_FILE_DATA:
        handleFileDataPacket(payload, size);
        sendAck();
        break;
        
      case PKT_FILE_END:
        handleFileEndPacket(payload, size);
        sendAck();
        break;
        
      default:
        // Regular packet, use original handling
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
        break;
    }
  } else {
    printPacketLine(payload, size, rssi, snr);
  }

  last_rx_time = millis();
  rxIdle = true;   // listo para escuchar otra vez
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed");
    return;
  }
  Serial.println("LittleFS initialized successfully");

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
  
  // Also configure TX for sending ACK/NACK
  Radio.SetTxConfig(MODEM_LORA, 5, 0,  // 5 dBm TX power
                    LORA_BANDWIDTH, LORA_SF, LORA_CR,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  t_win = millis();
  Serial.println("Ready to receive files...");
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
      "  RSSI(avg/min/max)=%.1f/%.1f/%.1f dBm  SNR(avg/min/max)=%.1f/%.1f/%.1f dB",
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
    
    // Add file transfer progress if active
    if (rx_transfer_state == RX_RECEIVING_FILE && expected_total_fragments > 0) {
      uint16_t received_fragments = 0;
      for (uint16_t i = 0; i < expected_total_fragments; i++) {
        if (fragment_received[i]) received_fragments++;
      }
      float progress = (float)received_fragments / expected_total_fragments * 100.0f;
      Serial.printf("  file_progress=%.1f%% (%u/%u)", progress, received_fragments, expected_total_fragments);
    }
    
    Serial.println();

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

void sendAck(void) {
  uint8_t ack_packet[6];
  ack_packet[0] = PKT_ACK;
  ack_packet[1] = 0;  // sequence not needed for ACK
  ack_packet[2] = 0;
  ack_packet[3] = 0;
  ack_packet[4] = 0;
  ack_packet[5] = 0;  // no data
  
  // Switch to TX mode, send ACK, then back to RX
  Radio.SetTxConfig(MODEM_LORA, 5, 0,  // 5 dBm TX power
                    LORA_BANDWIDTH, LORA_SF, LORA_CR,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  
  Radio.Send(ack_packet, 6);
  delay(100);  // Wait for transmission to complete
  
  // Switch back to RX
  Radio.SetRxConfig(
    MODEM_LORA, LORA_BANDWIDTH, LORA_SF, LORA_CR,
    0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,
    LORA_FIX_LENGTH_PAYLOAD_ON,
    LORA_FIX_LENGTH_PAYLOAD_ON ? PAYLOAD_SIZE_HINT : 0,
    true, false, 0, LORA_IQ_INVERSION_ON, RX_CONTINUOUS
  );
}

void sendNack(void) {
  uint8_t nack_packet[6];
  nack_packet[0] = PKT_NACK;
  nack_packet[1] = 0;
  nack_packet[2] = 0;
  nack_packet[3] = 0;
  nack_packet[4] = 0;
  nack_packet[5] = 0;
  
  // Switch to TX mode, send NACK, then back to RX
  Radio.SetTxConfig(MODEM_LORA, 5, 0,
                    LORA_BANDWIDTH, LORA_SF, LORA_CR,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  
  Radio.Send(nack_packet, 6);
  delay(100);
  
  // Switch back to RX
  Radio.SetRxConfig(
    MODEM_LORA, LORA_BANDWIDTH, LORA_SF, LORA_CR,
    0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,
    LORA_FIX_LENGTH_PAYLOAD_ON,
    LORA_FIX_LENGTH_PAYLOAD_ON ? PAYLOAD_SIZE_HINT : 0,
    true, false, 0, LORA_IQ_INVERSION_ON, RX_CONTINUOUS
  );
}

void handleFileStartPacket(uint8_t* payload, uint16_t size) {
  uint16_t total_fragments = (payload[3] << 8) | payload[4];
  uint8_t data_len = payload[5];
  
  if (data_len < 4 || size < HEADER_SIZE + data_len) {
    Serial.println("Invalid FILE_START packet");
    sendNack();
    return;
  }
  
  // Extract filename (null-terminated)
  uint8_t filename_len = data_len - 4;
  char filename[32];
  memcpy(filename, &payload[6], min(filename_len, (uint8_t)31));
  filename[min(filename_len, (uint8_t)31)] = '\0';
  
  // Extract file size
  uint32_t file_size;
  memcpy(&file_size, &payload[6 + filename_len], sizeof(uint32_t));
  
  Serial.printf("Receiving file: %s (%lu bytes, %u fragments)\r\n", 
                filename, (unsigned long)file_size, (unsigned)total_fragments);
  
  if (initFileReception(String(filename), file_size, total_fragments)) {
    rx_transfer_state = RX_RECEIVING_FILE;
  } else {
    sendNack();
  }
}

void handleFileDataPacket(uint8_t* payload, uint16_t size) {
  if (rx_transfer_state != RX_RECEIVING_FILE) {
    sendNack();
    return;
  }
  
  uint16_t fragment_num = (payload[1] << 8) | payload[2];
  uint8_t data_len = payload[5];
  
  if (size < HEADER_SIZE + data_len || fragment_num >= expected_total_fragments) {
    Serial.println("Invalid FILE_DATA packet");
    sendNack();
    return;
  }
  
  // Check if we already received this fragment
  if (fragment_received[fragment_num]) {
    Serial.printf("Duplicate fragment %u, ignoring\r\n", (unsigned)fragment_num);
    return;  // Still send ACK
  }
  
  // Write data to file at correct position
  size_t offset = fragment_num * MAX_DATA_SIZE;
  received_file.seek(offset);
  size_t written = received_file.write(&payload[6], data_len);
  
  if (written == data_len) {
    fragment_received[fragment_num] = true;
    bytes_received += data_len;
    Serial.printf("Received fragment %u/%u (%u bytes)\r\n", 
                  (unsigned)fragment_num, (unsigned)expected_total_fragments, (unsigned)data_len);
  } else {
    Serial.printf("Write error for fragment %u\r\n", (unsigned)fragment_num);
    sendNack();
    return;
  }
}

void handleFileEndPacket(uint8_t* payload, uint16_t size) {
  if (rx_transfer_state != RX_RECEIVING_FILE) {
    // Might be duplicate END packet, send ACK anyway if we completed transfer
    if (rx_transfer_state == RX_FILE_COMPLETE) {
      sendAck();
    } else {
      sendNack();
    }
    return;
  }
  
  if (size < HEADER_SIZE + 4) {
    Serial.println("Invalid FILE_END packet");
    sendNack();
    return;
  }
  
  uint32_t expected_crc;
  memcpy(&expected_crc, &payload[6], sizeof(uint32_t));
  
  // Check if all fragments received
  uint16_t received_fragments = 0;
  for (uint16_t i = 0; i < expected_total_fragments; i++) {
    if (fragment_received[i]) received_fragments++;
  }
  
  if (received_fragments != expected_total_fragments) {
    Serial.printf("Missing fragments: %u/%u received\r\n", 
                  received_fragments, expected_total_fragments);
    sendNack();
    return;
  }
  
  // Validate file integrity
  if (validateFileIntegrity(expected_crc)) {
    Serial.printf("File transfer completed successfully! CRC32: 0x%08X\r\n", (unsigned)expected_crc);
    rx_transfer_state = RX_FILE_COMPLETE;
  } else {
    Serial.println("File integrity check failed!");
    sendNack();
    resetFileTransfer();
    return;
  }
  
  // Close file and cleanup
  received_file.close();
  free(fragment_received);
  fragment_received = nullptr;
}

bool initFileReception(const String& filename, uint32_t file_size, uint16_t total_fragments) {
  // Close any existing file
  if (received_file) {
    received_file.close();
  }
  
  // Free existing fragment bitmap
  if (fragment_received) {
    free(fragment_received);
  }
  
  // Open file for writing
  received_file = LittleFS.open(RECEIVED_FILE_PATH, "w");
  if (!received_file) {
    Serial.println("Failed to create received file");
    return false;
  }
  
  // Allocate fragment tracking bitmap
  fragment_received = (bool*)calloc(total_fragments, sizeof(bool));
  if (!fragment_received) {
    Serial.println("Failed to allocate fragment bitmap");
    received_file.close();
    return false;
  }
  
  received_filename = filename;
  expected_file_size = file_size;
  expected_total_fragments = total_fragments;
  next_expected_fragment = 0;
  bytes_received = 0;
  
  Serial.printf("Initialized file reception: %s\r\n", filename.c_str());
  return true;
}

void resetFileTransfer(void) {
  if (received_file) {
    received_file.close();
  }
  
  if (fragment_received) {
    free(fragment_received);
    fragment_received = nullptr;
  }
  
  rx_transfer_state = RX_IDLE;
  bytes_received = 0;
  
  // Remove incomplete file
  if (LittleFS.exists(RECEIVED_FILE_PATH)) {
    LittleFS.remove(RECEIVED_FILE_PATH);
  }
  
  Serial.println("File transfer reset");
}

uint32_t calculateCRC32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320;
      } else {
        crc >>= 1;
      }
    }
  }
  
  return ~crc;  // Final inversion
}

bool validateFileIntegrity(uint32_t expected_crc) {
  received_file.seek(0);
  uint32_t calculated_crc = 0xFFFFFFFF;
  uint8_t buffer[64];
  
  while (received_file.available()) {
    size_t bytes_read = received_file.read(buffer, sizeof(buffer));
    for (size_t i = 0; i < bytes_read; i++) {
      calculated_crc ^= buffer[i];
      for (int j = 0; j < 8; j++) {
        if (calculated_crc & 1) {
          calculated_crc = (calculated_crc >> 1) ^ 0xEDB88320;
        } else {
          calculated_crc >>= 1;
        }
      }
    }
  }
  calculated_crc = ~calculated_crc;  // Final inversion
  
  Serial.printf("Expected CRC: 0x%08X, Calculated CRC: 0x%08X\r\n", 
                (unsigned)expected_crc, (unsigned)calculated_crc);
  
  return calculated_crc == expected_crc;
}
