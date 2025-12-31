/**
 * Communication Module Implementation
 */

#include "communication.h"

Communication::Communication() 
    : bufferIndex(0),
      commandReady(false) {
    memset(commandBuffer, 0, COMMAND_BUFFER_SIZE);
}

void Communication::init() {
    // Nothing to initialize, Serial is already initialized in main
}

bool Communication::hasCommand() {
    processSerial();
    return commandReady;
}

String Communication::getCommand() {
    if (!commandReady) {
        return "";
    }
    
    String cmd = String(commandBuffer);
    
    // Reset buffer for next command
    memset(commandBuffer, 0, COMMAND_BUFFER_SIZE);
    bufferIndex = 0;
    commandReady = false;
    
    return cmd;
}

void Communication::sendResponse(String type, String message) {
    Serial.print(type);
    Serial.print(":");
    Serial.println(message);
}

void Communication::processSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        // Check for newline (command terminator)
        if (c == '\n' || c == '\r') {
            if (bufferIndex > 0) {
                commandBuffer[bufferIndex] = '\0';
                commandReady = true;
            }
        }
        // Check for buffer overflow
        else if (bufferIndex >= COMMAND_BUFFER_SIZE - 1) {
            // Buffer overflow - reset
            memset(commandBuffer, 0, COMMAND_BUFFER_SIZE);
            bufferIndex = 0;
            Serial.println("ERROR:Command too long");
        }
        // Add character to buffer
        else {
            commandBuffer[bufferIndex++] = c;
        }
    }
}
