// Updated tx_lora.ino file with fixed file transfer logic

#include <LoRa.h>

// State definitions
#define SENDING_START 0
#define SENDING_DATA 1

int current_fragment = 0;
int state = SENDING_START;

void sendFileDataPacket() {
    // Seek to the correct position based on current_fragment
    // Assuming file is opened and 'file' is the file object
    file.seek(current_fragment * PACKET_SIZE);
    // Send the file data packet
    LoRa.beginPacket();
    // Read and send data...
    LoRa.endPacket();
    Serial.println("Sent fragment: " + String(current_fragment));
}

void OnRxDone() {
    // Handle received ACK
    if (state == SENDING_START) {
        // Move to sending data state
        state = SENDING_DATA;
        Serial.println("Transitioned to SENDING_DATA");
    } else if (state == SENDING_DATA) {
        // Increment current_fragment properly
        current_fragment++;
        Serial.println("Current fragment: " + String(current_fragment));
        // Check if there are more fragments to send
        if (/* condition to check if more data exists */) {
            sendFileDataPacket();
        } else {
            Serial.println("File transfer complete!");
            // Transition to completed state
        }
    }
}