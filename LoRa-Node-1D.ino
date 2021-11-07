// BASE STATION

#include <SPI.h>
#include <RH_RF95.h>
#include <RHMesh.h>
//#include<avr/wdt.h> /* Header for watchdog timers in AVR */
#include "WiFi.h"
#include <esp_task_wdt.h>

//define the pins used by the transceiver module
// ESP32 Pins
#define ss 5
#define rst 14
// This pin has an on-board LED
//#define dio0 2
#define dio0 4

// Arduino Pro Mini
//#define ss 10
//#define rst 9
//#define dio0 2

#define MY_NODE_ADDR 2
static const char CALL[9] = "KC1FSZ  ";

// Watchdog timeout in seconds (NOTE: I think this time might be off because
// we are changing the CPU clock frequency)
#define WDT_TIMEOUT 5

RH_RF95 rf95(ss, dio0);
RHMesh mesh_manager(rf95, MY_NODE_ADDR);

// Channel configurations
//float frequency = 434;
float frequency = 916;
// Transmit power in dBm
int txPower = 20;
// Higher spreading factor for longer distance
int spreadingFactor = 9;
// Setup BandWidth, option: 7800,10400,15600,20800,31250,41700,62500,125000,250000,500000
// Lower BandWidth for longer distance.
//long signalBandwidth = 125000;
long signalBandwidth = 31250;
// Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8) 
int codingRate = 5;
// Address
int address = 0xf3;

void configRadio(RH_RF95& radio) {
  //radio.setModemConfig(RH_RF95::ModemConfigChoice::Bw125Cr48Sf4096);
  radio.setFrequency(frequency);
  radio.setTxPower(txPower);
  radio.setSpreadingFactor(spreadingFactor);
  // Adjust over-current protection
  radio.spiWrite(RH_RF95_REG_0B_OCP, 0x31);
  //radio.setSignalBandwidth(signalBandwidth);
  //radio.setCodingRate4(codingRate);
  //radio.setThisAddress(address);
}

int counter = 0;

void setup() {

  //wdt_disable();  
  
  /* Done so that the Arduino doesn't keep resetting infinitely in 
     case of a wrong configuration */
  delay(3000); 

  // Slow down ESP32 to 10 MHz in order to reduce battery consumption
  setCpuFrequencyMhz(10);

  // Initialize serial monitor
  Serial.begin(115200);
  //delay(1000); 
  Serial.println("Node 1");

  // LED
  pinMode(2, OUTPUT);
  pinMode(21, OUTPUT);
  digitalWrite(2, LOW);
  digitalWrite(21, LOW);

  // Reset the radio 
  pinMode(rst, OUTPUT);
  digitalWrite(rst, HIGH);
  digitalWrite(rst, LOW);
  digitalWrite(rst, HIGH);
  // Put the pin in hi-Z mode!
  pinMode(rst, INPUT);
  // Per datasheet, wait for 5ms!
  delay(5);

  if (!mesh_manager.init()) {
    Serial.println("LoRa init failed");
  } else {
    configRadio(rf95);
    Serial.println("LoRa Initializing OK!");
    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);
    delay(200);
    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);
  }
  
  // Turn off WIFI and BlueTooth to reduce power 
  WiFi.mode(WIFI_OFF);
  btStop();

  // Enable the watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  esp_task_wdt_reset();
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

  // Keep the watchdog fed
  esp_task_wdt_reset();

  if ((millis() - lastSend) > sendInterval) {

    // Make sure the radio is responsive
    Serial.println("Checking version");
    uint8_t radioVersion = rf95.getDeviceVersion();
    Serial.print("Vesion ");
    Serial.println(radioVersion);
    

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

    unsigned long startMs = millis();
    uint8_t rc = mesh_manager.sendtoWait(data, 26, 3);
    unsigned long stopMs = millis();
    if (rc == RH_ROUTER_ERROR_NONE) {    
      Serial.print("OK ");  
    } else if (rc == RH_ROUTER_ERROR_NO_ROUTE) {
      Serial.print("NR ");
    } else if (rc == RH_ROUTER_ERROR_UNABLE_TO_DELIVER) {
      Serial.print("UTD ");
    } else {
      Serial.print("OTHER ");
    }
    Serial.println((stopMs - startMs));

    // GET THE RESPONSE
    uint8_t rec_data[40];
    uint8_t rec_len = 40;
    uint8_t rec_source = 0;
    uint16_t rec_timeout = 4000;

    startMs = millis();
    if (mesh_manager.recvfromAckTimeout(rec_data, &rec_len, rec_timeout, &rec_source)) {
      stopMs = millis();

      int16_t last_rssi = (int)rf95.lastRssi();
      
      if (rec_data[1] == 2) {
        if (rec_len >= 40) {

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
          uint16_t bootCount = rec_data[20] << 8;
          bootCount |= rec_data[21];
          uint16_t sleepCount = rec_data[22] << 8;
          sleepCount |= rec_data[23];
             
          Serial.print("Count ");
          Serial.print(counter, DEC);
          Serial.print(", Local RSSI ");
          Serial.print(last_rssi, DEC);
          Serial.print(", Remote RSSI ");
          Serial.print(rssi, DEC);
          Serial.print(", battery ");
          Serial.print(battery, DEC);
          Serial.print(", uptime ");
          Serial.print(uptime_seconds, DEC);
          Serial.print(", ms ");
          Serial.print((stopMs - startMs), DEC);
          Serial.print(", boots ");
          Serial.print(bootCount, DEC);
          Serial.print(", sleeps ");
          Serial.print(sleepCount, DEC);
          Serial.print(", ");
          Serial.println((const char*)&(rec_data[24]));

          digitalWrite(21, HIGH);
          delay(300);
          digitalWrite(21, LOW);
        } else {
          Serial.println(F("Length error"));
          Serial.println(rec_len);
        }
      } else {
        Serial.println(F("Unrecognized command"));
      }
    }

    counter++;
  }
  
}
