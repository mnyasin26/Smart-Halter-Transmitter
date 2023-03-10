#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <JY901.h> // IMU Lib
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <axp20x.h>
#include <TinyGPS++.h>

// HR Sensor
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

#define SS 18   // LoRa radio chip select
#define RST 14  // LoRa radio reset
#define DIO0 26 // change for your board; must be a hardware interrupt pin
#define SCK 5
#define MISO 19
#define MOSI 27
#define LEDINDIKATOR 4
// #define BUTTONYELLOW 14
// #define BUTTONGREEN 25

const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;
const uint8_t slave_address = AXP192_SLAVE_ADDRESS;

// const uint8_t slave_address = AXP192_SLAVE_ADDRESS;

int ledTime = 100;

String outgoing; // outgoing message
String ID = "SHMKR100601";
String ROMUSAGE = "244";
String PayLoad, Latitude, Longitude, Altitude, SOG, COG, ABAT, VBAT, STAT;
String AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ, roll, pitch, yaw;
String BPM, SpO2, Suhu = "0";
String RSI = "NAN";
String SNR = "NAN";

char t = '1';

byte msgCount = 0;       // count of outgoing messages
byte NextAddress = 0x13; // address of this device

long lastSendTime = 0; // last send time

int localAddress = 1; // address of this device
int interval = 50;    // interval between sends
int count;
int destination = 0; // destination to send to
int BTNG, BTNY;
int timeLED = 10;
bool state, state2;

TinyGPSPlus gps;
AXP20X_Class axp;
HardwareSerial GPS2(1);
Adafruit_SSD1306 display(128, 64, &Wire);

// Deklarasi
void sendMessage(String outgoing);
void sendMessageNext(String outgoing);
void onReceive(int packetSize);
void LED();

MAX30105 particleSensor;

const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

#define MAX_BRIGHTNESS 255
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
// Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
// To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100];  // infrared LED sensor data
uint16_t redBuffer[100]; // red LED sensor data
#else
uint32_t irBuffer[100];  // infrared LED sensor data
uint32_t redBuffer[100]; // red LED sensor data
#endif

int32_t bufferLength;  // data length
int32_t spo2;          // SPO2 value
int8_t validSPO2;      // indicator to show if the SPO2 calculation is valid
int32_t heartRate;     // heart rate value
int8_t validHeartRate; // indicator to show if the heart rate calculation is valid
byte pulseLED = 11;    // Must be on PWM pin
byte readLED = 13;     // Blinks with each data read

void setup()
{
  Serial.begin(9600);
  Serial.println("Initializing...");

  // Belum kepake
  // pinMode(pulseLED, OUTPUT);
  // pinMode(readLED, OUTPUT);

  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(915E6))
  { // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
  }

  GPS2.begin(9600, SERIAL_8N1, 34, 12);

  Wire.begin(i2c_sda, i2c_scl);

  JY901.startIIC();

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1)
      ;
  }

  int ret = axp.begin(Wire, slave_address);
  if (ret)
  {
    Serial.println("Ooops, AXP202/AXP192 power chip detected ... Check your wiring!");
  }
  axp.adc1Enable(AXP202_VBUS_VOL_ADC1 |
                     AXP202_VBUS_CUR_ADC1 |
                     AXP202_BATT_CUR_ADC1 |
                     AXP202_BATT_VOL_ADC1,
                 true);

  Serial.println("Place your index finger on the sensor with steady pressure.");
  // particleSensor.setup();                    // Configure sensor with default settings
  byte ledBrightness = 60;                                                                       // Options: 0=Off to 255=50mA
  byte sampleAverage = 4;                                                                        // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;                                                                              // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100;                                                                         // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;                                                                          // Options: 69, 118, 215, 411
  int adcRange = 4096;                                                                           // Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure sensor with these settings
  particleSensor.setPulseAmplitudeRed(0x0A);                                                     // Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);                                                      // Turn off Green LED

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed 1"));
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D))
    {
      Serial.println(F("SSD1306 allocation failed 2"));
    }
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  int i = 0;
  while (i < 3)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("\n   START");
    display.println(String() + "     " + (i + 1));
    Serial.println(i);
    display.display();
    delay(1000);
    i++;
  }

  pinMode(LEDINDIKATOR, OUTPUT);
  // put your setup code here, to run once:
  bufferLength = 100; // buffer length of 100 stores 4 seconds of samples running at 25sps
  // read the first 100 samples, and determine the signal range
  for (byte i = 0; i < bufferLength; i++)
  {
    while (particleSensor.available() == false) // do we have new data?
      particleSensor.check();                   // Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  // calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}

void loop()
{
  // // dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  // for (byte i = 25; i < 100; i++)
  // {
  //   redBuffer[i - 25] = redBuffer[i];
  //   irBuffer[i - 25] = irBuffer[i];
  // }

  // // take 25 sets of samples before calculating the heart rate.
  // for (byte i = 75; i < 100; i++)
  // {
  //   while (particleSensor.available() == false) // do we have new data?
  //     particleSensor.check();                   // Check the sensor for new data

  //   // digitalWrite(readLED, !digitalRead(readLED)); // Blink onboard LED with every data read

  //   redBuffer[i] = particleSensor.getRed();
  //   irBuffer[i] = particleSensor.getIR();
  //   particleSensor.nextSample(); // We're finished with this sample so move to next sample

  //   // send samples and calculation result to terminal program through UART
  //   // Serial.print(F("red="));
  //   // Serial.print(redBuffer[i], DEC);
  //   // Serial.print(F(", ir="));
  //   // Serial.print(irBuffer[i], DEC);

  //   // Serial.print(F(", HR="));
  //   // Serial.print(heartRate, DEC);

  //   // Serial.print(F(", HR Avg="));
  //   // Serial.print(beatAvg, DEC);

  //   // Serial.print(F(", HRvalid="));
  //   // Serial.print(validHeartRate, DEC);

  //   // Serial.print(F(", SPO2="));
  //   // Serial.print(spo2, DEC);

  //   // Serial.print(F(", SPO2Valid="));
  //   // Serial.println(validSPO2, DEC);
  // }

  // // After gathering 25 new samples recalculate HR and SP02
  // maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // long irValue = particleSensor.getIR();
  // if (checkForBeat(irValue) == true)
  // {
  //   // We sensed a beat!
  //   long delta = millis() - lastBeat;
  //   lastBeat = millis();

  //   beatsPerMinute = 60 / (delta / 1000.0);

  //   if (beatsPerMinute < 255 && beatsPerMinute > 20)
  //   {
  //     rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
  //     rateSpot %= RATE_SIZE;                    // Wrap variable

  //     // Take average of readings
  //     beatAvg = 0;
  //     for (byte x = 0; x < RATE_SIZE; x++)
  //       beatAvg += rates[x];
  //     beatAvg /= RATE_SIZE;
  //   }
  // }

  if (millis() - lastSendTime > 100)
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    if (axp.isBatteryConnect())
    {
      VBAT = axp.getBattVoltage();
      ABAT = axp.getAcinCurrent();
      state2 = false;
    }
    else
    {
      ABAT = "NAN";
      VBAT = "NAN";
      state2 = true;
    }

    if (!gps.location.isValid())
    {
      Latitude = String(gps.location.lat(), 8);
      Longitude = String(gps.location.lng(), 8);
      SOG = String(gps.speed.kmph());
      COG = String(gps.course.deg());
    }
    else
    {
      Latitude = "NAN";
      Longitude = "NAN";
      SOG = "NAN";
      COG = "NAN";
    }

    Altitude = JY901.getAltitude() / 100.0;
    AccX = JY901.getAccX();
    AccY = JY901.getAccY();
    AccZ = JY901.getAccZ();
    GyroX = JY901.getGyroX();
    GyroY = JY901.getGyroY();
    GyroZ = JY901.getGyroZ();
    MagX = JY901.getMagX();
    MagY = JY901.getMagY();
    MagZ = JY901.getMagZ();
    roll = JY901.getRoll();
    pitch = JY901.getPitch();
    yaw = JY901.getYaw();

    PayLoad = String() + "02" + "," + ID + "," + Latitude + "," + Longitude + "," + Altitude + "," + SOG + "," + COG + "," + AccX + "," + AccY + "," + AccZ + "," + GyroX + "," + GyroY + "," + GyroZ + "," + MagX + "," + MagY + "," + MagZ + "," + roll + "," + yaw + "," + pitch + "," + ABAT + "," + VBAT + "," + heartRate + "," + spo2 + "," + Suhu + "#";
    // display.println(String() + "   " + ID);
    display.println(String() + "Lat/Long/Alt : " + Latitude + "," + Longitude + "," + Altitude);
    display.println(String() + "Acc : " + AccX + "," + AccY + "," + AccZ);
    display.println(String() + "Gyro : " + GyroX + "," + GyroY + "," + GyroZ);
    display.println(String() + "AngleDir : " + roll + "," + yaw + "," + pitch);
    display.println(String() + "ABAT/VBAT : " + ABAT + "," + VBAT);
    display.println(String() + "HR/SpO2/Suhu : " + heartRate + "," + spo2 + "," + Suhu);

    
    // display.println(String() + "IR Value : " + irValue);
    // if (irValue < 50000)
    // {
    //   display.println("BPM Avg : No finger");
    //   beatAvg = 0;
    // }
    // else
    // {
    //   display.println(String() + "BPM Avg : " + beatAvg);
    // }
    Serial.println(AccX + "," + AccY + "," + AccZ + ",");
    // if (state == false)
    // {
    //   display.println(String() + "STAT: NOT CONNECT");
    //   display.println(String() + "RSSI: ");
    // }
    // else
    // {
    //   display.println(String() + "STAT: " + STAT);
    //   display.println(String() + "RSSI: " + RSI);
    // }
    // LED();
    sendMessage(PayLoad);
    //    Serial.println("Sending " + PayLoad);
    //    Serial.println(String()+ "FREE RAM : " + ESP.getFreeHeap() + "\tRAM SIZE : " +  ESP.getHeapSize() + "\t USED RAM : " + (ESP.getMaxAllocHeap()/1000));
    // display.println(String() + "   USED       USED");
    // display.println(String() + "RAM:" + (ESP.getMaxAllocHeap() / 1000) + "kb  ROM:" + ROMUSAGE + "kb");
    display.display();
    lastSendTime = millis(); // timestamp the message
    interval = 200;
  }
  else{
    while (GPS2.available())
    {
      gps.encode(GPS2.read());
    }
  }
  if (millis() - lastSendTime > 100)
  {
    count++;
    if (count >= 5)
    {
      state = false;
    }
    lastSendTime = millis(); // timestamp the message
  }
  onReceive(LoRa.parsePacket());
  // put your main code here, to run repeatedly:
}

unsigned long lastLed = 0;
bool stateLed = true;

void LED()
{
  stateLed = !stateLed;
  if (stateLed == true)
  {
    digitalWrite(LEDINDIKATOR, HIGH);
  }
  else
  {
    digitalWrite(LEDINDIKATOR, LOW);
  }
}

// definsi
void sendMessage(String outgoing)
{
  LoRa.beginPacket();            // start packet
  LoRa.write(destination);       // add destination address
  LoRa.write(localAddress);      // add sender address
  LoRa.write(msgCount);          // add message ID
  LoRa.write(outgoing.length()); // add payload length
  LoRa.print(outgoing);          // add payload
  LoRa.endPacket();              // finish packet and send it
  msgCount++;
  LED();
}
void sendMessageNext(String outgoing)
{
  LoRa.beginPacket();            // start packet
  LoRa.write(NextAddress);       // add destination address
  LoRa.write(localAddress);      // add sender address
  LoRa.write(msgCount);          // add message ID
  LoRa.write(outgoing.length()); // add payload length
  LoRa.print(outgoing);          // add payload
  LoRa.endPacket();              // finish packet and send it
  msgCount++;                    // increment message ID
}

void onReceive(int packetSize)
{
  if (packetSize == 0)
    return; // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();       // recipient address
  byte sender = LoRa.read();         // sender address
  byte incomingMsgId = LoRa.read();  // incoming msg ID
  byte incomingLength = LoRa.read(); // incoming msg length

  String incoming = "";

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0x10)
  {
    return; // skip rest of function
  }
  else if (recipient == localAddress)
  {
    STAT = String() + "CONNECTED";
    sendMessage(PayLoad);
    count = 0;
    state = true;
  }
  while (LoRa.available())
  {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length())
  { // check length for error
    Serial.println("error: message length does not match length");
    return; // skip rest of function
  }

  //  Serial.println(incoming);
  RSI = String(LoRa.packetRssi());
  SNR = String(LoRa.packetSnr());
}