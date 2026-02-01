// Canbus transmitter and self test

#include <aim_network.h>
#include <aim_can_driver.h>
#include <SoftwareSerial.h>
#include <cstdint>
#include <cstring>
#define CAN_RX_PIN PB8
#define CAN_TX_PIN PB9
#define USB_RX_PIN PA10 // for GPS board
#define USB_TX_PIN PA9 // for GPS board
#define DB_LED_PIN PA15
#define INTERVAL 1000
#define USB_BAUD 9600
#define RX_SIZE_16 16
SoftwareSerial usb(USB_RX_PIN, USB_TX_PIN); 

AimCanDriver canHw(AIM_ORG_GPS, 62500);
AimNetwork aimn(&canHw, AIM_ORG_GPS);

// Reading from network
dataPkt recv_data;
uint8_t recv_origin;
uint8_t recv_type;

dataPkt recv_datas[RX_SIZE_16];
uint8_t recv_types[RX_SIZE_16];

// Writing to network
const uint8_t num_pkts = 3;
uint8_t aim_dests[num_pkts] = {AIM_DEST_COMMS, AIM_DEST_COMMS, AIM_DEST_BROADCAST};
uint8_t aim_types[num_pkts] = {AIM_TYP_GPS_LAT, AIM_TYP_GPS_LONG, AIM_TYP_TIME};
uint8_t active_num_pkts = num_pkts;


unsigned long current_time;
unsigned long previous_time;
unsigned int current_tx_index=0;

void setup() {
  // setup AIM network
  aimn.begin();

  // setup usb serial
  usb.begin(USB_BAUD);

  pinMode(DB_LED_PIN, OUTPUT);
}

void loop() {
  uint32_t num_recv = 0;
  while(aimn.readPkt(recv_data, recv_origin, recv_type)) {
    usb.print("Received Packet #");
    usb.print(num_recv);
    usb.print(": origin=0x");
    usb.print(recv_origin, HEX);
    usb.print(", type=0x");
    usb.print(recv_type, HEX);
    usb.print(", data=0x");
    usb.print(recv_data.data, HEX);
    usb.print(", =");
    usb.print(recv_data.dayMilis);
    usb.println("ms");

    switch(recv_type){
      // add flash logging
      case AIM_TYP_TIME:
        // Logic: Sync system clock with received timestamp
        break;
      case AIM_TYP_PT1:
        // Logic: Handle Pressure Transducer 1 data
        break;
      case AIM_TYP_PT2:
        // Logic: Handle Pressure Transducer 2 data
        break;
      case AIM_TYP_TC: 
        // Logic: Handle Thermocouple temperature
        break;
      case AIM_TYP_VAL1: 
        // Logic: Handle Value 1
        break;
      case AIM_TYP_VAL2:
        // Logic: Handle Value 2
        break;
      case AIM_TYP_GPS_LAT:
        // Logic: Update Latitude coordinate
        break;
      case AIM_TYP_GPS_LONG:
        // Logic: Update Longitude coordinate
        break;
      case AIM_TYP_ALT:
        // Logic: Update Altitude
        break;
      case AIM_TYP_LOWPW:
        // Logic: Turn on low power mode (e.g., reduce TX rate, disable non-critical sensors)
        // EX: only send Long/Lat
        active_num_pkts = 2; 
        break;
      case AIM_TYP_NODATA:
        // Logic: nothing to do
        break;
      case AIM_TYP_UNDEFINED:
        // Logic: Error - someone sending garbage data or uninitialized type
        break;
      default:
        // Logic: Unexpected type received that isn't explicitly defined
        break;
    }
    num_recv++;
  }
  current_time = millis();
  //old: for(uint8_t i = 0; i < active_num_pkts; i++) {
  if((current_time-previous_time)>INTERVAL){
    previous_time=current_time;

    dataPkt aim_data = {current_tx_index, current_time };
    aimn.sendPkt(aim_data, aim_dests[current_tx_index], aim_types[current_tx_index]);
    usb.print("Sent Packet #");
    usb.print(current_tx_index);
    usb.print(": dest=0x");
    usb.print(aim_dests[current_tx_index], HEX);
    usb.print(", type=0x");
    usb.print(aim_types[current_tx_index], HEX);
    usb.print(", data=0x");
    usb.print(aim_data.data, HEX);
    usb.print(", =");
    usb.print(aim_data.dayMilis);
    usb.println("ms");
    digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
    current_tx_index = ++current_tx_index % 3;
    
  }
}
