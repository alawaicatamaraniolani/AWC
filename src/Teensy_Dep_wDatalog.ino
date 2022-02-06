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

#include <TinyGPS.h>
#include <ping1d.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

//---------Tiny GPS vars-------------
static void smartdelay(unsigned long ms);
TinyGPS gps;
unsigned long fix_age2, timeGPS2;
long lat, lon;
//-----------------------------------

//==============Atlas Sensor Vars===============
char sensordata[30];                  // A 30 byte character array to hold incoming data from the sensors
byte sensor_bytes_received = 0;       // We need to know how many characters bytes have been received
byte in_char = 0;                     // used as a 1 byte buffer to store in bound bytes from the I2C Circuit.

#define TOTAL_CIRCUITS 4 // set how many I2C circuits are attached to the Tentacle shield(s): 1-8

int channel_ids[] = {97, 99, 100, 102}; // A list of I2C ids that you set your circuits to.
char *channel_names[] = {"DO", "PH", "EC", "TP"}; // A list of channel names (must be the same order as in channel_ids[])

//------------encapsulation definitions-----------------------
#define numberOfValues 8
#define bitSum 17+15+18+17+28+29+15+7  //must manualy enter all values due to the way that C++ accesses memory
#define packetLen (int)(ceil((float)(bitSum)/8))
// the order of sensors: temp, pH, cond, oxy, lat, lon, depth, confidence
int bitLen[] = {17,15,18,17,28,29,15,7};
long int bitVal[numberOfValues];
uint8_t packets[packetLen]; 
int oldbit;
int bitCounter;
byte state;

//------------------------------------------------------------
//variables for holding the data from the atlas sensors
float temp = 0;
float pH = 0;
long int cond = 0;
float oxy = 0;

//-----------------------------------------

//------------LoRaWAN/LMIC vars---------------
//define the frequency the SPI bus will communcate on
#define LMIC_SPI_FREQ 10000000

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
//---------------------------------------

//-------------teensy Pinout---------------

// Pin mapping for Teenst 
const lmic_pinmap lmic_pins = {
  .nss = 10, // chip select on CS (must be 5)
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 21, // reset pin (can be connected to any I/O)
  .dio = {22, 23, LMIC_UNUSED_PIN}, 
  // DIO Pins (can also be connected to any I/O pin, should not be connected to 12-15)
};
//----------------------------------------------


//-----------------ping variables-----------------
static Ping1D ping { Serial4 }; //defines the serial communication line for ping

int dist = 0;
int conf = 0;
//------------------------------------------------
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 3;

//-------------------SD Card---------------------
//SD card
#define fileDir "sensors"
#define fileName "loggingdata"
char fileNameChar[sizeof(fileName) - 1 + sizeof(fileDir) -1 + 8];
int fileNum = 0;
bool newname = true;
bool logging = false;
//----------------------------------------------

//---------------LoRa/MCCI function----------------
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

void GPSData(bool newData)
{
  unsigned long chars;
  unsigned short sentences, failed;
  if(newData)
  {
    unsigned long age, fix_age, timeGPS, date, speedGPS, courseGPS;
    
    //pull data
    gps.get_position(&lat, &lon, &age);
    gps.get_datetime(&date, &timeGPS, &fix_age);
    speedGPS = gps.speed();
    uint32_t satel = gps.satellites();
    int32_t hDOP = gps.hdop();

    Serial.print("satel:");
    Serial.print(satel);
    Serial.print(" Real:");
    Serial.println(gps.satellites());
    Serial.print("hdop:");
    Serial.print(hDOP);
    Serial.print(" Real:");
    Serial.println(gps.hdop());
    
    //format data
    bitVal[4]= 0;
    bitVal[5] = 0;
    if(lat < 0)
    {
      bitVal[4] = pow(2,27);
    }
    if(lon <0)
    {
      bitVal[5] = pow(2,28);
    }
    bitVal[4] += abs(lat);
    bitVal[5] += abs(lon);
    fix_age2 = fix_age;
    timeGPS2 = timeGPS;
    if(lat != 999999999)
    {
      Serial.print("Lat:");
      Serial.print(lat);
      Serial.print("  Lon:");
      Serial.println(lon);
    }
  }
  gps.stats(&chars, &sentences, &failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS **");
}

void setup()
{
  //Begin communications
  Serial.println("Starting...");
  Serial.begin(115200);
  Serial4.begin(115200); //start ping communication
  if(!ping.initialize())
  {
    Serial.println("Ping sonar not properly connected");
  }
  Serial1.begin(9600);
  delay(1000);
  Wire.setSDA(34);
  Wire.setSCL(33);
  Wire.begin();
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

void loop() {
  bool newData = true;
  for (int Channel = 0; Channel < TOTAL_CIRCUITS; Channel++) {       // loop through all the sensors
    Wire.beginTransmission(channel_ids[Channel]);     // call the circuit by its ID number.
    Wire.write('r');                              // request a reading by sending 'r'
    Wire.endTransmission();                            // end the I2C data transmission.
    
    bool osToggle = true;
    for (unsigned long starting = millis(); millis() - starting < 1000;)
    {
      
      while (Serial1.available())
      {
        char c = Serial1.read();
        if (gps.encode(c)) // Did a new valid sentence come in?
        {
          newData = true;
        }
      }
      if(millis()%150 == 0) 
      {
        if((ping.update()))
        {
        dist = ping.distance();
        conf = ping.confidence();
        bitVal[6] = dist;
        bitVal[7] = conf;
        }
        os_runloop_once();
        if(logging)
        {
          SDSave();
        }
      }
    }
    GPSData(newData); 

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
        oxy = atof(sensordata);
        bitVal[3] = (long int)(100 * atoi(sensordata));
        Serial.println(oxy,2);
        break;
      case 1:
        pH = atof(sensordata);
        bitVal[1] = (int)(atof(sensordata) * 1000);
        Serial.println(pH,3);
        break;
      case 2:
        cond = atoi(sensordata);
        bitVal[2] = (long int)atoi(sensordata);
        Serial.println(cond);
        break;
      case 3:
        temp = atof(sensordata);
        bitVal[0] = (int)(atof(sensordata) * 1000);
        Serial.println(temp,3);
        break;
    }
  } // for loop
  os_runloop_once();
  
  /*
  //should be called at least once every so often to ensure that LoRaWAN
  //still communicates correctly
  */
}


//function that will translate data from all sensors to bits
uint8_t* encapsulate()
{
  int runSum = 0;
  int startNum;
  int stopNum;
  bitCounter =0;
  for(int i = 0; i < numberOfValues;i++)
  {
    startNum = runSum;
    stopNum = runSum - 1 + bitLen[i];
    oldbit = bitLen[i];
    int startPos = 0;
    int binLength = 8 - startNum%8;
    for(int bytes = 0; bytes < stopNum/8-startNum/8+1;bytes++)
    {
      if(oldbit < 9)
      {
        binLength = oldbit;
        startPos = 8-oldbit;
      }
      byteTransfer(bitVal[i],binLength,startPos);
      packets[startNum/8 + bytes] = state;
      startPos = 0;
      binLength = 8;
      if(bitCounter == 8)
      {
        for(int i2 = 0; i2< 8;i2++)
        {
          bitWrite(state,i2,0);
        }
        bitCounter = 0;
      }
    }
    runSum += bitLen[i];
  }
  return (uint8_t*)&packets;
}

//function to transfer bits into bytes
void byteTransfer(int reading, int i2, int buff)
{
  oldbit -= i2;
  for (int i = 0; i < i2; i++)
  {
    bitWrite(state, i + buff, bitRead(reading, oldbit));
    oldbit++;
    bitCounter++;
  }
  oldbit -= i2;
}

//-------------------SD Save-------------------
//prints current data readings to SD card
void SDSave()
{
  //log current data

  String dataString = "";
  dataString += String(timeGPS2);
  dataString += ",";
  dataString += String(lat);
  dataString += ",";
  dataString += String(lon);
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
//   if the file isn't open, pop up an error:
  else 
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
