/**
 * Communication Module
 * 
 * Handles serial communication protocol
 */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>

#define COMMAND_BUFFER_SIZE 128

class Communication {
public:
    Communication();
    
    // Initialize communication
    void init();
    
    // Check if a complete command is available
    bool hasCommand();
    
    // Get the next command
    String getCommand();
    
    // Send a response
    void sendResponse(String type, String message);
    
private:
    char commandBuffer[COMMAND_BUFFER_SIZE];
    int bufferIndex;
    bool commandReady;
    
    // Process incoming serial data
    void processSerial();
};

#endif // COMMUNICATION_H
