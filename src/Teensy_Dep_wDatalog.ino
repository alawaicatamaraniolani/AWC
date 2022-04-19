/* Communication over AVR to TTN using LoRaWAN for the Teensy 3.5 w/ RFM9x breakout
 * by: Caleb and Dillion
   Robotics 4
    
 * PINOUT:
 * 
 * HARDWARE SPI PINS FOR THE TEENSY 3.5 ARE:
 * MOSI: 11
 * MISO: 12
 * CLK/SLK: 27
 * CS: 10
 * OTHER CONNECTIONS TO RFM9x BREAKOUT:
 * RST: 21
 * G0: 22
 * G1: 23
 *
   Additional Libraries required:
   "MCCI_LoRaWAN_LMIC_library"
   NOTE:
   #define DISABLE_JOIN
   MUST BE TYPED OUT in arduino-lmic/project_config/lmic_project_config.h for
   this to work (this disables OTAA), otherwise you must add these lines :

   void os_getArtEui (u1_t* buf) { }
   void os_getDevEui (u1_t* buf) { }
   void os_getDevKey (u1_t* buf) { }
*/
#include <NMEAGPS.h>
#include <GPSport.h>
#include <Streamers.h>
#include <ping1d.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Watchdog.h>
#include <MS5837.h>

//----------Pressure-Vars-----------
MS5837 pressure;
//----------------------------------

//----------Watchdog-Vars-----------
Watchdog watchdog;
boolean reseted = false;
//----------------------------------

//----------Neo-GPS-Vars------------
static NMEAGPS  gps;
static gps_fix  fix;
long int latit,longi = 0;
float date = 0;
//----------------------------------

//--------Atlas-Sensor-Vars---------
char sensordata[30];                  // A 30 byte character array to hold incoming data from the sensors
byte sensor_bytes_received = 0;       // We need to know how many characters bytes have been received
byte in_char = 0;                     // used as a 1 byte buffer to store in bound bytes from the I2C Circuit.

#define TOTAL_CIRCUITS 4 // set how many I2C circuits are attached to the Tentacle shield(s): 1-8

int channel_ids[] = {97, 99, 100, 102}; // A list of I2C ids that you set your circuits to.
char *channel_names[] = {"DO", "PH", "EC", "TP"}; // A list of channel names (must be the same order as in channel_ids[])

//variables for holding the data from the atlas sensors
float temp = 0;
float pH = 0;
long int cond = 0;
float oxy = 0;
//----------------------------------

//-------Encapsulation-Vars---------
#define numberOfValues 9
#define bitSum 17+15+18+17+28+29+15+15+10  //must manualy enter all values due to the way that C++ accesses memory
#define packetLen (int)((float)(bitSum-1)/8)+1
// the order of sensors: temp, pH, cond, oxy, lat, lon, depth, confidence
int bitLen[] = {17,15,18,17,28,29,15,15,10};
long int bitVal[numberOfValues];
uint8_t packets[packetLen]; 
int oldbit;
int bitCounter;
byte state;
//----------------------------------

//--------LoRaWAN/LMIC-Vars---------
//define the frequency the SPI bus will communcate on
#define LMIC_SPI_FREQ 1000000

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = { 0x47, 0x17, 0xF0, 0x7D, 0x0D, 0xEB, 0x4C, 0xBE,
0x8A, 0xF6, 0xA2, 0x29, 0xF1, 0xDB, 0xA5, 0x3D };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = { 0xC3, 0xCC, 0xA7, 0x0D, 0x23, 0xA5, 0xAF, 0x20,
0x1A, 0xD7, 0x15, 0xBB, 0x84, 0x38, 0x73, 0x48 };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed,
// so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260C1BE6 ; // <-- Change this address for every node!

static osjob_t sendjob;

// Pin mapping for Teensy
const lmic_pinmap lmic_pins = {
  .nss = 10, // chip select on CS (must be 5)
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 21, // reset pin (can be connected to any I/O)
  .dio = {22, 23, LMIC_UNUSED_PIN}, 
  // DIO Pins (can also be connected to any I/O pin, should not be connected to 12-15)
};

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 3;
//----------------------------------

//------------Ping-Vars-------------
static Ping1D ping { Serial4 }; //defines the serial communication line for ping

int dist = 0;
int conf = 0;
//----------------------------------

//-------------SD-Vars--------------
//SD card
#define fileDir "sensors"
#define fileName "loggingdata"
char fileNameChar[sizeof(fileName) - 1 + sizeof(fileDir) -1 + 8];
int fileNum = 0;
bool newname = true;
bool logging = false;
//----------------------------------

//===================SETUP=======================
void setup()
{
  pinMode(24,INPUT);
  //Begin communications
  Serial.println("Starting...");
  Serial.begin(115200);
  Serial4.begin(115200); //start ping communication
  if(!ping.initialize())
  {
    Serial.println("Ping sonar not properly connected");
  }
  Wire.setSDA(34);
  Wire.setSCL(33);
  Wire.begin();
  // if(!pressure.init())
  // {
  //   Serial.println("Pressure sensor not properly connected");
  // }
  SPI.setMOSI(11);
  SPI.setMISO(12);
  SPI.setSCK(27);
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present");
  }
  else
  {
    Serial.println("card initialized.");
    logging = true;
    //intialize SD Card name
    String FILENAME = String(fileDir) + "/" + String(fileName) + "000.csv";
    for(int i = 0; i < FILENAME.length();i++)
    {
      fileNameChar[i] = FILENAME[i];
    }
    SDNew(); //Set a new directory (if neccessary)
    
  }
  // pressure.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  if(watchdog.tripped())
  {
    reseted = true;
  }
  watchdog.enable(Watchdog::TIMEOUT_8S);

  gpsPort.begin( 9600 );

  os_init(); // LMIC init
  LMIC_reset(); // Reset the MAC state. Session and pending data transfers will be discarded.

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are to be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

  LMIC_selectSubBand(1); //sets the region
  LMIC_setDrTxpow(DR_SF7, 14); // Set data rate and transmit power for uplink

  do_send(&sendjob); // Start job
}
//==================END=SETUP====================

//====================LOOP=======================
void loop() {
  for (int Channel = 0; Channel < TOTAL_CIRCUITS; Channel++) {       // loop through all the sensors
    if(reseted)
    {
      watchdog.reset();
    }
    Wire.beginTransmission(channel_ids[Channel]);     // call the circuit by its ID number.
    Wire.write('r');                              // request a reading by sending 'r'
    Wire.endTransmission();                            // end the I2C data transmission.
    
    for (unsigned long starting = millis(); millis() - starting < 1000;)
    {
      
      while (gps.available(gpsPort))
      {
        char c = Serial1.read();
        if (gps.available(gpsPort)) // Did a new valid sentence come in?
        {
          fix = gps.read();
          doSomeWork();
        }
      }

      if(digitalRead(24)==0)
      {
        watchdog.reset();
      }
    } 

    sensor_bytes_received = 0;                        // reset data counter
    memset(sensordata, 0, sizeof(sensordata));        // clear sensordata array;
    Wire.requestFrom(channel_ids[Channel], 48, 1);    // call the circuit and request 48 bytes 

    while (Wire.available()) {          // are there bytes to receive?
      in_char = Wire.read();            // receive a byte.
      if (in_char == 0) {               // null character indicates end of command
        Wire.endTransmission();         // end the I2C data transmission.
        break;                          // exit the while loop, we're done here
      }
      else {
        sensordata[sensor_bytes_received] = in_char;      // append this byte to the sensor data array.
        sensor_bytes_received++;
      }
    }
    Serial.print(channel_names[Channel]);
    Serial.print(":");
    
    for (int i = 0; i < 29; i++){
      sensordata[i] = sensordata[i+1];
    }
    //assign sensor data to appropriate variables
    switch (Channel) {
      case 0:
        cond = atof(sensordata);
        bitVal[3] = (long int)(100 * atoi(sensordata));
        Serial.println(cond,2);
        break;
      case 1:
        pH = atof(sensordata);
        bitVal[1] = (int)(atof(sensordata) * 1000);
        Serial.println(pH,3);
        break;
      case 2:
        oxy = atoi(sensordata);
        bitVal[2] = (long int)atoi(sensordata);
        Serial.println(oxy);
        break;
      case 3:
        temp = atof(sensordata);
        bitVal[0] = (int)(atof(sensordata) * 1000);
        Serial.println(temp,3);
        break;
    }
  } 
  //should be called at least once in a while
  os_runloop_once();
}
//===================END=LOOP====================

//--------LoRa/MCCI-function--------
void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}
//----------------------------------

//----------Lora-Transmit-----------
//function for sending actual data over LoRa
void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, encapsulate() , packetLen , 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}
//----------------------------------

//--------Data-Encapsulate----------
//Ecapsulates data to be sent over Lora
uint8_t* encapsulate() {
  int runningBitLen = bitLen[0];
  int startBit = 0;
  int packetNum = 0;
  for(int i = 0; i < packetLen; i++)
  {
    state = 0;
    do
    {
      int shift = (runningBitLen%8 + startBit)+ 8*((runningBitLen/8)-1);
      if(runningBitLen > 8)
      {
        state = state | (((abs(bitVal[packetNum]) & ((int)pow(2,(runningBitLen))-1))>> shift) & 255);
      }
      else
      {
        state = state | (((abs(bitVal[packetNum]) & ((int)pow(2,(runningBitLen))-1))<< (8-(startBit + runningBitLen%8))) & 255);
      }
      
      if(runningBitLen + startBit <= 8)
      {
        packetNum++;
        startBit = runningBitLen;
        if(packetNum >= numberOfValues) 
        {
          break;
        }
        runningBitLen = bitLen[packetNum];
      } 
      else
      {
        runningBitLen = runningBitLen - (8- startBit);
        startBit = 0;
        break;
      }
    } while (true);
    packets[i] = state;
    // Serial.print("packet: ");
    // Serial.println(i);
    // Serial.print("hasvalue: ");
    // for(int i2 = 0; i2<8;i2++)
    // {
    //   Serial.print(bitRead(packets[i],i2));
    // }
    // Serial.println();
  }
  return (uint8_t*)&packets;
}
//----------------------------------

//-------------SD-Save--------------
//prints current data readings to SD card
void SDSave()
{
  //log current data

  String dataString = "";
  dataString += String(millis());
  dataString += ",";
  dataString += String(fix.dateTime.day);
  dataString += "-";
  dataString += String(fix.dateTime.month);
  dataString += "-";
  dataString += String(fix.dateTime.year);
  dataString += " ";
  dataString += String(fix.dateTime.hours);
  dataString += ":";
  dataString += String(fix.dateTime.minutes);
  dataString += ":";
  dataString += String(fix.dateTime.seconds);
  dataString += ".";
  dataString += String(fix.dateTime_ms());
  dataString += ",";
  dataString += String(fix.location.lat());
  dataString += ",";
  dataString += String(fix.location.lon());
  dataString += ",";
  dataString += String(temp,3);
  dataString += ",";
  dataString += String(pH,3);
  dataString += ",";
  dataString += String(cond);
  dataString += ",";
  dataString += String(oxy,2);
  dataString += ",";
  dataString += String(dist);
  dataString += ",";
  dataString += String(conf);
  
  File dataFile = SD.open(fileNameChar,FILE_WRITE);
  
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  }
  else //if the file isn't open, pop up an error:
  {
    Serial.println("error opening .csv");
  }
}
void SDNew()
{
  //create a directory to hold the data
  char FILEDirChar[sizeof(fileDir)-1];
  for (int i = 0; i < sizeof(fileDir); i++)
  {
    FILEDirChar[i] = fileDir[i];
  }
  if(!SD.exists(FILEDirChar))
  {
    SD.mkdir(FILEDirChar);
  }
  
  //find the next avilable fileName for use
  while(fileNum < 999)
  {
    for(int i = 0; i < 3;i++)
    {
      fileNameChar[sizeof(fileNameChar) - 7 + (2-i)] = (char)((int)('0') + ((fileNum/(int)pow(10,i))%10));
    }
    if(!SD.exists(fileNameChar))
      {
        Serial.print("logging as:");
        Serial.println(fileNameChar);
        break;
      }
    fileNum++;
  }
  File dataFile = SD.open(fileNameChar,FILE_WRITE);
  
  // if the file is available, write to it:
  String nameString = "";
  nameString += "Millis (ms)";
  nameString += ",";
  nameString += "Time";
  nameString += ",";
  nameString += "Latitutde";
  nameString += ",";
  nameString += "Longitude";
  nameString += ",";
  nameString += "Temperature (C)";
  nameString += ",";
  nameString += "pH";
  nameString += ",";
  nameString += "Electric Conductivity";
  nameString += ",";
  nameString += "Dissolved Oxygen (uS)";
  nameString += ",";
  nameString += "Distance (mm)";
  nameString += ",";
  nameString += "Confidence (%)";
  while(!dataFile);
  dataFile.println(nameString);
  dataFile.close();
 }
//----------------------------------

//----------Work-For-GPS------------
static void doSomeWork()
{
  // Print all the things!
  trace_all( DEBUG_PORT, gps, fix );
  // watchdog.reset(); 
  Serial.print("Laitutde:");
  latit = fix.latitude()*1E7;
  longi = fix.longitude()*1E7;
  date = fix.dateTime_ms();
  bitVal[4] =(int)(latit < 0 ? abs(latit) + pow(2,27) : abs(latit));
  bitVal[5] =(int)(longi < 0 ? abs(longi) + pow(2,28) : abs(longi));
  Serial.print("Time+Date: ");
  Serial.print(date);
  Serial.print(". latitude: ");
  Serial.print(latit);
  Serial.print("Second: ");
  Serial.println(latit < 0 ? abs(latit) + pow(2,27) : abs(latit));
  
  if(latit==0)
  {
    watchdog.reset();
  }

  if((ping.update()))
  {
    dist = ping.distance();
    conf = ping.confidence();
    
    Serial.print("depth:");
    Serial.print(dist);
    Serial.print(" Confidence: ");
    Serial.println(conf);

    bitVal[6] = dist;
    bitVal[7] = conf;
  }
  
  if(logging)
  {
    SDSave();
  }
  
  os_runloop_once();

} 
//----------------------------------