#include "logger.h"
#include <SoftwareSerial.h>

SoftwareSerial softSerial(10, 11);
const uint8_t NODE_ID = 1;
Logger logger(Serial, NODE_ID);       // HardwareSerial
// Logger logger(softSerial, NODE_ID); // SoftwareSerial

void setup()
{
    Serial.begin(115200);
    g_logger = &logger;
    LOG_INFO("Logger online");
}