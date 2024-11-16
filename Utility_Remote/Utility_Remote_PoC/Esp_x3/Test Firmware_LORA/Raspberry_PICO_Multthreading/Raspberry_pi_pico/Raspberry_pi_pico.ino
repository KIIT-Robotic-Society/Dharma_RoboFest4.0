#include <SPI.h>

// Pin definitions for SPI0 on Raspberry Pi Pico
#define CS_PIN 10  // Chip Select (CS)
#define MISO_PIN 16 // MISO pin
#define MOSI_PIN 19 // MOSI pin
#define SCK_PIN 18  // SCK pin

// Buffer for sending and receiving data
char buffer[6]; // Increased size to hold "world" and null terminator

void setup() {
    Serial.begin(115200);
    
    // Set up SPI as slave on SPI0
    SPI.begin();  // Initialize the SPI interface
    
    // Configure MISO pin as output (it must be an output for the slave)
    pinMode(MISO_PIN, OUTPUT);
    
    // Configure CS pin as input with pull-up
    pinMode(CS_PIN, INPUT_PULLUP); // Use pull-up to avoid floating state
    
    Serial.println("SPI Slave Initialized");
}

void loop() {
    // Check for incoming SPI data and respond
    Serial.println("HI");
    receiveMessage();
}

// Function to receive data from the SPI master
void receiveMessage() {
    if (digitalRead(CS_PIN) == LOW) {  // Check if CS is low, indicating the master is communicating
        Serial.println("Master is communicating...");
        
        // Clear buffer before reading
        memset(buffer, 0, sizeof(buffer));
        
        // This will read the incoming message from the master
        int bytesRead = 0;
        while (digitalRead(CS_PIN) == LOW && bytesRead < sizeof(buffer) - 1) {
            buffer[bytesRead] = SPI.transfer(0); // Sending 0 to receive data
            bytesRead++;
            delay(10); // Add a slight delay to give time for data transfer
        }
        
        // Ensure the buffer is null-terminated
        buffer[bytesRead] = '\0'; 
        
        // Print the received message
        if (bytesRead > 0) {
            Serial.print("Received from Master: ");
            Serial.println(buffer);
        } else {
            Serial.println("No data received from Master");
        }
        
        // Send a response back to the master
        sendMessage("world");
    }
}

// Function to send data back to the master
void sendMessage(const char* response) {
    // Fill buffer with response message
    strncpy(buffer, response, sizeof(buffer) - 1); // Avoid buffer overflow
    buffer[sizeof(buffer) - 1] = '\0'; // Ensure null termination
    
    // Send response to master over SPI
    for (int i = 0; i < strlen(buffer); i++) {
        SPI.transfer(buffer[i]); // Sending each byte of the response
    }
    
    // Print confirmation that data was sent
    Serial.print("Data sent to Master: ");
    Serial.println(buffer);
}
