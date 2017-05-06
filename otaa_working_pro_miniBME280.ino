/*******************************************************************************
  REAL ONE!!!!
  Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example will send Temperature and Humidity
   using frequency and encryption settings matching those of
   the The Things Network. Application will 'sleep' 7x8 seconds (56 seconds)

   This uses OTAA (Over-the-air activation), where where a DevEUI and
   application key is configured, which are used in an over-the-air
   activation procedure where a DevAddr and session keys are
   assigned/generated for use with all further communication.

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate an AppEUI, DevEUI and AppKey.
   Multiple devices can use the same AppEUI, but each device has its own
   DevEUI and AppKey.

   Do not forget to define the radio type correctly in config.h.

 *******************************************************************************/

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
#include "SparkFunBME280.h"
#include <CayenneLPP.h>


#define MAX_SIZE 200 // depends on spreading factor and frequency used
#define DEBUG
CayenneLPP Payload(MAX_SIZE);


//#define BME
#ifdef BME
BME280 bme; // I2C
#endif

#include <Arduino.h>

int sleepcycles = 1;//600/8;  // every sleepcycle will last 8 secs, total sleeptime will be sleepcycles * 8 sec
bool joined = false;
bool sleeping = false;
#define LedPin 6     // pin 13 LED is not used, because it is connected to the SPI port

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.

static const u1_t DEVEUI[8]  = { 0x34, 0x12, 0x85, 0x84, 0x57, 0x00, 0x40, 0x88 };
static const u1_t APPEUI[8] = { 0xF4, 0x3E, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70 };

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.

static const u1_t APPKEY[16] = { 0x8E, 0xF7, 0x73, 0xE9, 0x66, 0x32, 0x6C, 0x7E, 0x9E, 0xBB, 0x22, 0x56, 0x19, 0x18, 0x0C, 0x59 };

void os_getArtEui (u1_t* buf) {
  memcpy(buf, APPEUI, 8);
}

// provide DEVEUI (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
  memcpy(buf, DEVEUI, 8);
}

// provide APPKEY key (16 bytes)
void os_getDevKey (u1_t* buf) {
  memcpy(buf, APPKEY, 16);
}

static osjob_t sendjob;
static osjob_t initjob;

// Pin mapping is hardware specific.
// Pin mapping Doug Larue PCB
// Pin mapping
/*const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,
  .dio = {2, 3, 4},
  };
*/
/*
  const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = 0, //LMIC_UNUSED_PIN,
  .rst = 0,
  .dio = {4, 5, 7},
  };
  const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
  };*/

const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,
  .dio = {2, 3, 4},
};

void onEvent (ev_t ev) {
  int i, j;
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      // Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      // Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      //Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      digitalWrite(LedPin, HIGH);
      // after Joining a job with the values will be sent.
      joined = true;
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      // Re-init
      os_setCallback(&initjob, initfunc);
      break;
    case EV_TXCOMPLETE:
      sleeping = true;
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        // if any data received, a LED will blink
        // this number of times, with a maximum of 10
        Serial.print(F("Data Received: "));
        Serial.println(LMIC.frame[LMIC.dataBeg], HEX);
        i = (LMIC.frame[LMIC.dataBeg]);
        // i (0..255) can be used as data for any other application
        // like controlling a relay, showing a display message etc.
        if (i > 10) {
          i = 10;   // maximum number of BLINKs
        }
        for (j = 0; j < i; j++)
        {
          digitalWrite(LedPin, HIGH);
          delay(200);
          digitalWrite(LedPin, LOW);
          delay(400);
        }
      }
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      delay(50);  // delay to complete Serial Output before Sleeping

      digitalWrite(LedPin, HIGH);
      delay(500);
      digitalWrite(LedPin, LOW);
      delay(500);
      // Schedule next transmission
      // next transmission will take place after next wake-up cycle in main loop
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
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

// initial job
static void initfunc (osjob_t* j) {
  // reset MAC state
  LMIC_reset();
  // start joining
  LMIC_startJoining();
  // init done - onEvent() callback will be invoked...
}

void do_send(osjob_t* j)
{
  // Check if there is not a current TX/RX job running
  /* if (LMIC.opmode & OP_TXRXPEND) {
       Serial.println(F("OP_TXRXPEND, not sending"));
    } else */{
    // Prepare upstream data transmission at the next possible time.

    float temp;
    float pressure;
    float alt;
    float hum;
    float volt;

#ifdef BME
    bmeForceRead();
    Payload.reset();
    // Get temperature event and print its value.
    temp = bme.readTempC();
    pressure = bme.readFloatPressure() / 100.0F;
    alt = bme.readFloatAltitudeMeters();
    hum = bme.readFloatHumidity();

    volt = 3.3 * analogRead(A0) / 1024;
    Serial.println(volt);

    //Serial.println (String(temp) );
    //+ " Pressure:=" + String(pressure) + " Humidity:=" + hum + "% Altitude:= " + String(alt));

    Payload.addTemperature(0, temp);
    Payload.addBarometricPressure (1, pressure);
    Payload.addRelativeHumidity (2, hum);
    Payload.addBarometricPressure(3, alt);
    Payload.addAnalogInput(4, volt);

    LMIC_setTxData2(2, Payload.getBuffer(), Payload.getSize(), 0);
    Serial.println(F("Packet queued"));
    // Next TX is scheduled after TX_COMPLETE event.
#else
    Payload.reset();
    // Get temperature event and print its value.
    temp = 77;
    pressure = 100 / 100.0F;
    alt = 567;
    hum = 99;

    // Serial.println ("Temp:= " + String(temp) + " Pressure:=" + String(pressure) + " Humidity:=" + hum + "% Altitude:= " + String(alt));

    Payload.addTemperature(0, temp);
    Payload.addBarometricPressure (1, pressure);
    Payload.addRelativeHumidity (2, hum);
    Payload.addBarometricPressure(3, alt);

    LMIC_setTxData2(2, Payload.getBuffer(), Payload.getSize(), 0);
    Serial.println(F("Packet queued"));
    // Next TX is scheduled after TX_COMPLETE event.
#endif
  }
}

#ifdef BME
void bmeForceRead()
{

  // We set the sensor in "forced mode" to force a reading.
  // After the reading the sensor will go back to sleep mode.
  uint8_t value = bme.readRegister(BME280_CTRL_MEAS_REG);
  value = (value & 0xFC) + 0x01;
  bme.writeRegister(BME280_CTRL_MEAS_REG, value);

  // Measurement Time (as per BME280 datasheet section 9.1)
  // T_max(ms) = 1.25
  //  + (2.3 * T_oversampling)
  //  + (2.3 * P_oversampling + 0.575)
  //  + (2.4 * H_oversampling + 0.575)
  //  ~ 9.3ms, for current settings multiplied by 5 (sample rate
  delay(10 * 5 * 5);
}
#endif

void setup()
{
  Serial.begin(115200);
  Serial.println(F("Enter setup"));

  //***Driver settings********************************//
  //commInterface can be I2C_MODE or SPI_MODE
  //specify chipSelectPin using arduino pin names
  //specify I2C address.  Can be 0x77(default) or 0x76

  //For I2C, enable the following and disable the SPI section
#ifdef BME
  bme.settings.commInterface = I2C_MODE;
  bme.settings.I2CAddress = 0x77;

  //For SPI enable the following and dissable the I2C section
  //bme.settings.commInterface = SPI_MODE;
  //bme.settings.chipSelectPin = 10;


  //***Operation settings*****************************//

  //runMode can be:
  //  0, Sleep mode
  //  1 or 2, Forced mode
  //  3, Normal mode
  bme.settings.runMode = 2; //Forced

  //tStandby can be:
  //  0, 0.5ms
  //  1, 62.5ms
  //  2, 125ms
  //  3, 250ms
  //  4, 500ms
  //  5, 1000ms
  //  6, 10ms
  //  7, 20ms
  bme.settings.tStandby = 0;

  //filter can be off or number of FIR coefficients to use:
  //  0, filter off
  //  1, coefficients = 2
  //  2, coefficients = 4
  //  3, coefficients = 8
  //  4, coefficients = 16
  bme.settings.filter = 4; //Lots of HW filter

  //tempOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  bme.settings.tempOverSample = 5;

  //pressOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  bme.settings.pressOverSample = 5;

  //humidOverSample can be:
  //  0, skipped
  //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  bme.settings.humidOverSample = 5;

  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  else
  {
    Serial.println("BME found");
  }
#endif
  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.


  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  /*LMIC_setupChannel(0, 903900000, DR_RANGE_MAP(DR_SF10, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(1, 904100000, DR_RANGE_MAP(DR_SF10, DR_SF7), BAND_CENTI);      // g-band
     LMIC_setupChannel(2, 904300000, DR_RANGE_MAP(DR_SF10, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(3, 904500000, DR_RANGE_MAP(DR_SF10, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(4, 904700000, DR_RANGE_MAP(DR_SF10, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(5, 904900000, DR_RANGE_MAP(DR_SF10, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(6, 905100000, DR_RANGE_MAP(DR_SF10, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(7, 905300000, DR_RANGE_MAP(DR_SF10, DR_SF7),  BAND_CENTI);      // g-band
     LMIC_setupChannel(8, 904600000, DR_RANGE_MAP(DR_SF8,  DR_SF8),  BAND_CENTI);      // g2-band
     // TTN defines an additional channel at 869.525Mhz using SF9 for class B
     // devices' ping slots. LMIC does not have an easy way to define set this
     // frequency and support for class B is spotty and untested, so this
     // frequency is not configured here.
  */

  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setClockError(MAX_CLOCK_ERROR * 1.0 / 100);


  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  do_send(&sendjob);

#ifdef DEBUG
  Serial.println(F("Leave setup"));
#endif
}

unsigned long time;
void loop()
{

  // start OTAA JOIN
  if (joined == false)
  {

    os_runloop_once();

  }
  else
  {
    do_send(&sendjob);    // Sent sensor values
    while (sleeping == false)
    {
      os_runloop_once();
    }
    sleeping = false;
    for (int i = 0; i < sleepcycles; i++)
    {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds
    }
  }

  digitalWrite(LedPin, ((millis() / 100) % 2) && (joined == false)); // only blinking when joining and not sleeping
}

