/*
  Arduino LSM6DSOX - Read FIFO

  This example demonstrates high frequency reading of accelerometer, 
  gyroscope, temperature and timestamp values from an LSM6DSOX FIFO,
  and sending the data through BLE.

  The circuit:
  - Arduino Nano RP2040 Connect

  created 23 February 2023
  by Rene van Ee

  This example code is in the public domain.
*/

#include <Arduino_LSM6DSOX.h>
#include <ArduinoBLE.h>

#include <math.h> // isnan
#include <map>

#define ARDUINO_IMU_VERSION   1
#define BLE_BLOCKS_PER_PACKET 2
#define BLE_SAMPLES_PER_BLOCK 8

#define BLE_UUID_IMU_SERVICE "9A48ECBA-2E92-082F-C079-9E75AAE428B1"
#define BLE_UUID_SENSOR_DATA "6f18bfca-8787-4516-89a7-cb272e36c653"

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} XYZData;

typedef struct {
  XYZData gyr;
  XYZData acc;
} SampleData;

typedef struct {
  double timestamp;
  uint32_t counter;
  uint16_t G_fullScale;
  uint8_t XL_fullScale;
  uint8_t samples;
  SampleData sample[BLE_SAMPLES_PER_BLOCK];
} SampleBlock;

typedef struct {
  uint8_t version;
  uint8_t blocks;
  uint8_t temperature_index;
  uint8_t reserved;
  float temperature;
  SampleBlock block[BLE_BLOCKS_PER_PACKET];
} SensorData;

SensorData sensordata = { ARDUINO_IMU_VERSION, 0, 0, 0, 0.0f, };

BLEService imuService(BLE_UUID_IMU_SERVICE);
BLECharacteristic sensorCharacteristic(BLE_UUID_SENSOR_DATA, BLERead | BLENotify, sizeof(SensorData), (1 == 1));

uint8_t current_sample;
uint8_t current_block;

std::map<SampleStatus, String> mapSampleStatusToString = {
  {SampleStatus::OK, "OK"},
  {SampleStatus::BUFFER_UNDERRUN, "BUFFER_UNDERRUN"},
  {SampleStatus::BUFFER_OVERRUN, "BUFFER_OVERRUN"},
  {SampleStatus::TAG_NOT_IMPLEMENTED, "TAG_NOT_IMPLEMENTED"},
  {SampleStatus::UNKNOWN_TAG, "UNKNOWN_TAG"},
  {SampleStatus::PARITY_ERROR, "PARITY_ERROR"},
  {SampleStatus::COMMUNICATION_ERROR, "COMMUNICATION_ERROR"}
};

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Starting device...");

  Serial.println("SensorData size="+String(sizeof(SensorData)));

  IMU.settings.sampleRate = 104;
  IMU.settings.accelRange = 2;
  IMU.settings.gyroRange = 500;
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  // Begin BLE initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  /* Set a local name for the Bluetooth® Low Energy device
     This name will appear in advertising packets
     and can be used by remote devices to identify this Bluetooth® Low Energy device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("Arduino IMU");
  BLE.setAdvertisedService(imuService); // add the service UUID
  imuService.addCharacteristic(sensorCharacteristic);
  BLE.addService(imuService);
  
  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Initial value
  sensorCharacteristic.writeValue((const uint8_t*)&sensordata, sizeof(SensorData));

  /* Start advertising Bluetooth® Low Energy.  It will start continuously transmitting Bluetooth® Low Energy
     advertising packets and will be visible to remote Bluetooth® Low Energy central devices
     until it receives a new connection */
  BLE.setConnectionInterval(0x0006, 0x000A);
  BLE.advertise();
  Serial.println("Bluetooth® device active (rssi="+String(BLE.rssi())+"), waiting for connections...");

  // Initialize
  current_sample = 0;
  current_block = 0;
}

void loop() {
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    if (central.connected()) {
      Sample sample;
      SampleStatus sampleResult = IMU.fifo.getSample(sample);
      if(sampleResult == SampleStatus::OK) {
        // Test
        uint32_t blocknr = sample.counter / 100;
        if(sample.counter == (blocknr*100)) {        
          switch(blocknr) {
            case 3:
              IMU.setPosSelfTestG();
              IMU.setPosSelfTestXL();
              break;
            case 6:
              IMU.setNegSelfTestG();
              IMU.setNegSelfTestXL();
              break;
            default:
              IMU.resetSelfTestG();
              IMU.resetSelfTestXL();
          }
        }

        // Create new block if a timestamp is found
        if((current_block < BLE_BLOCKS_PER_PACKET) && !isnan(sample.timestamp)) {
          current_block++;
          if(current_block >= BLE_BLOCKS_PER_PACKET) {
            sensordata.blocks = current_block;
//unsigned long currentTime = micros();
            // Send BLE packet
            sensorCharacteristic.writeValue((const uint8_t*)&sensordata, sizeof(SensorData), false);
//unsigned long deltaTime = micros() - currentTime;
//Serial.println("Sending BLE packet took "+String(deltaTime)+"us");

            // New packet, reset block
            current_block = 0;
          }

          sensordata.block[current_block].timestamp = sample.timestamp;
          sensordata.block[current_block].counter = sample.counter;
          sensordata.block[current_block].XL_fullScale = sample.XL_fullScale;
          sensordata.block[current_block].G_fullScale = sample.G_fullScale;
          sensordata.block[current_block].samples = 0;
          current_sample = 0;
        }
        
        // Store temperature if set. This will generally not happen more than
        // once in a block, except for low sample rates. In that case, only the
        // last temperature recorded within the block is kept.
        if(!isnan(sample.temperature)) {
          sensordata.temperature_index = current_block*BLE_SAMPLES_PER_BLOCK + current_sample;
          sensordata.temperature = sample.temperature;
        }
        /*  
      Serial.println("sample ["+String(current_block)+":"+String(current_sample)+"] retrieved: counter="+String(sample.counter)+" t="+String(sample.timestamp)+" T="+String(sample.temperature)+
        " XL="+String(sample.XL_X)+","+String(sample.XL_Y)+","+String(sample.XL_Z)+"(FS "+String(sample.XL_fullScale)+")"+
        " G="+String(sample.G_X)+","+String(sample.G_Y)+","+String(sample.G_Z)+"(FS "+String(sample.G_fullScale)+")");
        */
        
        // Store sample data
        if(current_sample < BLE_SAMPLES_PER_BLOCK) {
          sensordata.block[current_block].sample[current_sample].acc.x = sample.XL_X;
          sensordata.block[current_block].sample[current_sample].acc.y = sample.XL_Y;
          sensordata.block[current_block].sample[current_sample].acc.z = sample.XL_Z;
          sensordata.block[current_block].sample[current_sample].gyr.x = sample.G_X;
          sensordata.block[current_block].sample[current_sample].gyr.y = sample.G_Y;
          sensordata.block[current_block].sample[current_sample].gyr.z = sample.G_Z;
          current_sample++;
          sensordata.block[current_block].samples = current_sample;
        }
      } else if(sampleResult != SampleStatus::BUFFER_UNDERRUN) {
        Serial.println("Error reading from IMU: "+mapSampleStatusToString[sampleResult]+" @ counter="+String(sample.counter));
      }
    }
  } // END if (central.connected())
      
  // poll for Bluetooth® Low Energy events
  BLE.poll();
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  
  // Reset timestamp
  IMU.resetTimestamp();

  //Start FIFO
  IMU.fifo.settings.compression = true;
  IMU.fifo.settings.timestamp_decimation = 8;
  IMU.fifo.settings.temperature_frequency = 2;
  IMU.fifo.begin();
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  IMU.fifo.end();
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}