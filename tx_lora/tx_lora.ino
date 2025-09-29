/* Heltec Automation send + METRICAS + PDF File Transfer */
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "FS.h"
#include "LittleFS.h"

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

// File transfer constants
#define HEADER_SIZE                  6           // packet header size
#define MAX_DATA_SIZE               (BUFFER_SIZE - HEADER_SIZE)  // 24 bytes data per packet
#define FILE_PATH                   "/archivo_prueba.pdf"
#define PACKET_TIMEOUT_MS           5000        // timeout for packet acknowledgment
#define MAX_RETRIES                 3           // max retransmission attempts

// Packet types
#define PKT_FILE_START              0x01
#define PKT_FILE_DATA               0x02 
#define PKT_FILE_END                0x03
#define PKT_ACK                     0x04
#define PKT_NACK                    0x05

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

// File transfer state
enum TransferState {
  IDLE,
  SENDING_START,
  SENDING_DATA,
  SENDING_END,
  WAITING_ACK
};

TransferState transfer_state = IDLE;
File pdfFile;
uint16_t current_fragment = 0;
uint16_t total_fragments = 0;
uint32_t file_size = 0;
uint32_t bytes_sent = 0;
uint8_t retry_count = 0;
uint32_t last_tx_time = 0;
bool file_transfer_mode = false;

void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

// File transfer functions
bool initFileTransfer(void);
void sendNextPacket(void);
void sendFileStartPacket(void);
void sendFileDataPacket(void);
void sendFileEndPacket(void);
void handleTimeout(void);
uint32_t calculateCRC32(const uint8_t* data, size_t length);

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed");
    return;
  }
  Serial.println("LittleFS initialized successfully");

  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone    = OnRxDone;  // Add RX callback for ACK/NACK

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0,
                    LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  
  // Also configure RX for receiving ACK/NACK
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    0, LORA_PREAMBLE_LENGTH, 0, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, false);

  t_win_start = millis();
  
  // Check if file exists and initialize transfer
  if (LittleFS.exists(FILE_PATH)) {
    Serial.println("PDF file found, ready for transmission");
    file_transfer_mode = true;
  } else {
    Serial.println("PDF file not found, using normal mode");
    file_transfer_mode = false;
  }
}

void loop() {
  if (file_transfer_mode) {
    // File transfer mode
    if (lora_idle) {
      if (transfer_state == IDLE) {
        // Start file transfer
        if (initFileTransfer()) {
          transfer_state = SENDING_START;
          sendNextPacket();
        }
      } else if (transfer_state != WAITING_ACK) {
        // Continue sending next packet
        sendNextPacket();
      } else {
        // Check for timeout while waiting for ACK
        if (millis() - last_tx_time > PACKET_TIMEOUT_MS) {
          handleTimeout();
        }
      }
    }
  } else {
    // Original periodic message mode
    if (lora_idle) {
      static uint32_t t_last = 0;
      if (millis() - t_last >= TX_INTERVAL_MS) {
        t_last = millis();

        txNumber += 0.01;
        snprintf(txpacket, BUFFER_SIZE, "Hello world number %0.2f", txNumber);
        last_len = strlen(txpacket);

        Serial.printf("\r\n[TX] sending \"%s\" len=%u\r\n", txpacket, (unsigned)last_len);

        Radio.Send((uint8_t*)txpacket, last_len);
        lora_idle = false;
        t_last_tx_start = millis();
      }
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

    if (file_transfer_mode) {
      float progress = total_fragments > 0 ? (float)current_fragment / total_fragments * 100.0f : 0.0f;
      Serial.printf("[METRICS] window=%.2fs  bytes=%lu  pkts=%lu  throughput=%.0f bps  pps=%.2f  avg_len=%.1f B  airtime_avg=%.1f ms  progress=%.1f%%\r\n",
                    dt, (unsigned long)win_bytes, (unsigned long)win_pkts,
                    bps, pps, avg_len, avg_airms, progress);
    } else {
      Serial.printf("[METRICS] window=%.2fs  bytes=%lu  pkts=%lu  throughput=%.0f bps  pps=%.2f  avg_len=%.1f B  airtime_avg=%.1f ms\r\n",
                    dt, (unsigned long)win_bytes, (unsigned long)win_pkts,
                    bps, pps, avg_len, avg_airms);
    }

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
  
  if (file_transfer_mode) {
    transfer_state = WAITING_ACK;
    last_tx_time = millis();
    // Start listening for ACK/NACK
    Radio.Rx(PACKET_TIMEOUT_MS);
  } else {
    lora_idle = true;
  }
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("TX Timeout...");
  if (file_transfer_mode) {
    handleTimeout();
  } else {
    lora_idle = true;
  }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  Radio.Sleep();
  
  if (file_transfer_mode && size >= HEADER_SIZE) {
    uint8_t pkt_type = payload[0];
    
    if (pkt_type == PKT_ACK) {
      Serial.println("Received ACK");
      retry_count = 0;
      
      if (transfer_state == SENDING_START) {
        transfer_state = SENDING_DATA;
      } else if (transfer_state == SENDING_DATA) {
        current_fragment++;
        if (current_fragment >= total_fragments) {
          transfer_state = SENDING_END;
        }
      } else if (transfer_state == SENDING_END) {
        Serial.println("File transfer completed successfully!");
        transfer_state = IDLE;
        if (pdfFile) {
          pdfFile.close();
        }
      }
      lora_idle = true;
      
    } else if (pkt_type == PKT_NACK) {
      Serial.println("Received NACK, retransmitting...");
      retry_count = 0;  // Reset retry count on NACK
      lora_idle = true;
    }
  }
}

bool initFileTransfer(void) {
  pdfFile = LittleFS.open(FILE_PATH, "r");
  if (!pdfFile) {
    Serial.println("Failed to open PDF file");
    return false;
  }
  
  file_size = pdfFile.size();
  total_fragments = (file_size + MAX_DATA_SIZE - 1) / MAX_DATA_SIZE;  // ceiling division
  current_fragment = 0;
  bytes_sent = 0;
  retry_count = 0;
  
  Serial.printf("Starting file transfer: %s (%lu bytes, %u fragments)\r\n", 
                FILE_PATH, (unsigned long)file_size, (unsigned)total_fragments);
  
  return true;
}

void sendNextPacket(void) {
  if (transfer_state == SENDING_START) {
    sendFileStartPacket();
  } else if (transfer_state == SENDING_DATA) {
    sendFileDataPacket();
  } else if (transfer_state == SENDING_END) {
    sendFileEndPacket();
  }
}

void sendFileStartPacket(void) {
  memset(txpacket, 0, BUFFER_SIZE);
  
  // Header
  txpacket[0] = PKT_FILE_START;
  txpacket[1] = (current_fragment >> 8) & 0xFF;
  txpacket[2] = current_fragment & 0xFF;
  txpacket[3] = (total_fragments >> 8) & 0xFF;
  txpacket[4] = total_fragments & 0xFF;
  
  // Data: filename and file size
  String filename = "archivo_prueba.pdf";
  uint8_t filename_len = min(filename.length(), (size_t)(MAX_DATA_SIZE - 4));
  txpacket[5] = filename_len + 4;  // data length includes filename + file size
  
  memcpy(&txpacket[6], filename.c_str(), filename_len);
  memcpy(&txpacket[6 + filename_len], &file_size, sizeof(uint32_t));
  
  last_len = HEADER_SIZE + filename_len + 4;
  
  Serial.printf("Sending FILE_START: %s (%lu bytes)\r\n", filename.c_str(), (unsigned long)file_size);
  
  Radio.Send((uint8_t*)txpacket, last_len);
  lora_idle = false;
  t_last_tx_start = millis();
}

void sendFileDataPacket(void) {
  if (current_fragment >= total_fragments) {
    return;
  }
  
  memset(txpacket, 0, BUFFER_SIZE);
  
  // Header
  txpacket[0] = PKT_FILE_DATA;
  txpacket[1] = (current_fragment >> 8) & 0xFF;
  txpacket[2] = current_fragment & 0xFF;
  txpacket[3] = (total_fragments >> 8) & 0xFF;
  txpacket[4] = total_fragments & 0xFF;
  
  // Read data from file
  size_t bytes_to_read = min((size_t)MAX_DATA_SIZE, file_size - bytes_sent);
  size_t bytes_read = pdfFile.read((uint8_t*)&txpacket[6], bytes_to_read);
  
  txpacket[5] = bytes_read;  // data length
  last_len = HEADER_SIZE + bytes_read;
  bytes_sent += bytes_read;
  
  Serial.printf("Sending DATA fragment %u/%u (%u bytes)\r\n", 
                (unsigned)current_fragment, (unsigned)total_fragments, (unsigned)bytes_read);
  
  Radio.Send((uint8_t*)txpacket, last_len);
  lora_idle = false;
  t_last_tx_start = millis();
}

void sendFileEndPacket(void) {
  // Calculate CRC32 of entire file
  pdfFile.seek(0);
  uint32_t crc = 0;
  uint8_t buffer[64];
  while (pdfFile.available()) {
    size_t bytes_read = pdfFile.read(buffer, sizeof(buffer));
    crc = calculateCRC32(buffer, bytes_read);  // Simple CRC implementation
  }
  
  memset(txpacket, 0, BUFFER_SIZE);
  
  // Header
  txpacket[0] = PKT_FILE_END;
  txpacket[1] = 0;  // no sequence for end packet
  txpacket[2] = 0;
  txpacket[3] = (total_fragments >> 8) & 0xFF;
  txpacket[4] = total_fragments & 0xFF;
  txpacket[5] = 4;  // CRC32 size
  
  // Data: CRC32
  memcpy(&txpacket[6], &crc, sizeof(uint32_t));
  
  last_len = HEADER_SIZE + 4;
  
  Serial.printf("Sending FILE_END (CRC32: 0x%08X)\r\n", (unsigned)crc);
  
  Radio.Send((uint8_t*)txpacket, last_len);
  lora_idle = false;
  t_last_tx_start = millis();
}

void handleTimeout(void) {
  retry_count++;
  
  if (retry_count >= MAX_RETRIES) {
    Serial.println("Max retries reached, aborting transfer");
    transfer_state = IDLE;
    if (pdfFile) {
      pdfFile.close();
    }
    lora_idle = true;
    return;
  }
  
  Serial.printf("Timeout, retrying (%u/%u)\r\n", (unsigned)retry_count, (unsigned)MAX_RETRIES);
  lora_idle = true;  // Allow retransmission
}

// Simple CRC32 implementation
uint32_t calculateCRC32(const uint8_t* data, size_t length) {
  static uint32_t crc = 0xFFFFFFFF;
  
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
  
  return crc;
}
