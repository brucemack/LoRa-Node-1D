// BASE STATION

#include <SPI.h>
#include <RH_RF95.h>
#include <RHMesh.h>
//#include<avr/wdt.h> /* Header for watchdog timers in AVR */
#include "WiFi.h"
#include <esp_task_wdt.h>

//define the pins used by the transceiver module
// ESP32
#define ss 5
#define rst 14
#define dio0 2

// Arduino Pro Mini
//#define ss 10
//#define rst 9
//#define dio0 2

#define MY_NODE_ADDR 2
static const char CALL[9] = "KC1FSZ  ";

RH_RF95 rf95(ss, dio0);
RHMesh mesh_manager(rf95, MY_NODE_ADDR);

// Channel configurations
//float frequency = 434;
float frequency = 915;
// Transmit power in dBm
int txPower = 20;
// Higher spreading factor for longer distance
int spreadingFactor = 12;
// Setup BandWidth, option: 7800,10400,15600,20800,31250,41700,62500,125000,250000,500000
// Lower BandWidth for longer distance.
long signalBandwidth = 125000;
// Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8) 
int codingRate = 5;
// Address
int address = 0xf3;

void configRadio(RH_RF95& radio) {
  radio.setFrequency(frequency);
  radio.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096);
  radio.setTxPower(txPower);
  // Adjust over-current protection
  radio.spiWrite(RH_RF95_REG_0B_OCP, 0x31);
  //radio.setSpreadingFactor(spreadingFactor);
  //radio.setSignalBandwidth(signalBandwidth);
  //radio.setCodingRate4(codingRate);
  radio.setThisAddress(address);
}

int counter = 0;

void setup() {

  //wdt_disable();  
  
  /* Done so that the Arduino doesn't keep resetting infinitely in 
     case of a wrong configuration */
  delay(1000); 
  // Turn on the watch dog 
  //wdt_enable(WDTO_2S);

  // Initialize serial monitor
  Serial.begin(115200);
  //delay(1000); 
  Serial.println("Node 1");

  // Reset the radio 
  pinMode(rst, OUTPUT);
  digitalWrite(rst, HIGH);
  digitalWrite(rst, LOW);
  digitalWrite(rst, HIGH);
  // Put the pin in hi-Z mode!
  pinMode(rst, INPUT);
  // Per datasheet, wait for 5ms!
  delay(5);

  if (!rf95.init()) {
    Serial.println("driver init failed");
  } else {
    configRadio(rf95);  
    if (!mesh_manager.init()) {
      Serial.println("manager init failed");
    } else {
      Serial.println("LoRa Initializing OK!");
    }
  }
  
  // Turn off WIFI and BlueTooth to reduce power 
  WiFi.mode(WIFI_OFF);
  btStop();
}

long lastSend = 0;
long sendInterval = 10000;

void send_1() {

    uint16_t num = 7;
    char *msg = "Hello World!";

    // Build the paket
    uint8_t data[64];
    // VERSION
    data[0] = 1;
    // COMMAND
    data[1] = 3;
    // Callsign
    for (int i = 0; i < 8; i++) 
       data[2 + i] = CALL[i];
    // Num
    data[10] = num >> 8;
    data[11] = num & 0xff;
    // Payload
    for (int i = 0; i < strlen(msg); i++)
      data[12 + i] = msg[i];

    uint8_t data_len = 12 + strlen(msg);    
    uint8_t rc = mesh_manager.sendtoWait(data, data_len, 3);
    if (rc == RH_ROUTER_ERROR_NONE) {    
      Serial.println("OK");  
    } else if (rc == RH_ROUTER_ERROR_NO_ROUTE) {
      Serial.println("NR");
    } else if (rc == RH_ROUTER_ERROR_UNABLE_TO_DELIVER) {
      Serial.println("UTD");
    }
}

void loop() {

  //wdt_reset();  /* Reset the watchdog */

  if ((millis() - lastSend) > sendInterval) {

    lastSend = millis();
    //int batteryLevel = analogRead(A0);
    
    Serial.print("Sending: ");
    Serial.println(counter);
  
    uint8_t data[32];
    // VERSION
    data[0] = 1;
    // COMMAND
    data[1] = 1;
    // Callsign
    for (int i = 0; i < 8; i++) 
       data[2 + i] = CALL[i];
    // Payload
    for (int i = 0; i < 16; i++)
      data[10 + i] = 0;
    sprintf((char*)(data + 10),"KC1FSZ %d", counter);
    
    uint8_t rc = mesh_manager.sendtoWait(data, 26, 3);
    if (rc == RH_ROUTER_ERROR_NONE) {    
      Serial.println("OK");  
    } else if (rc == RH_ROUTER_ERROR_NO_ROUTE) {
      Serial.println("NR");
    } else if (rc == RH_ROUTER_ERROR_UNABLE_TO_DELIVER) {
      Serial.println("UTD");
    }

    // GET THE RESPONSE
    uint8_t rec_data[32];
    uint8_t rec_len = 32;
    uint8_t rec_source = 0;
    uint16_t rec_timeout = 4000;
    
    if (mesh_manager.recvfromAckTimeout(rec_data, &rec_len, rec_timeout, &rec_source)) {

      int16_t last_rssi = (int)rf95.lastRssi();
      
      if (rec_data[1] == 2) {
        if (rec_len >= 32) {

          int16_t counter = rec_data[10] << 8;
          counter |= rec_data[11];
          int16_t rssi = rec_data[12] << 8;
          rssi |= rec_data[13];
          uint16_t battery = rec_data[14] << 8;
          battery |= rec_data[15];
          uint32_t uptime_seconds = rec_data[16] << 24;
          uptime_seconds |= rec_data[17] << 16;
          uptime_seconds |= rec_data[18] << 8;
          uptime_seconds |= rec_data[19];
             
          Serial.print("Count ");
          Serial.print(counter, DEC);
          Serial.print(", Local RSSI ");
          Serial.print(last_rssi, DEC);
          Serial.print(", Remote RSSI ");
          Serial.print(rssi, DEC);
          Serial.print(", ");
          Serial.print(battery, DEC);
          Serial.print(", ");
          Serial.print(uptime_seconds, DEC);
          Serial.print(", ");
          Serial.println((const char*)&(rec_data[20]));
        } else {
          Serial.println(F("Length error"));
        }
      } else {
        Serial.println(F("Unrecognized command"));
      }
    }

    counter++;
  }
}
