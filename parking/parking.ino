#include <XBee.h>
#include <SoftwareSerial.h>
#include <Wire.h> //I2C Arduino Library
#include <avr/sleep.h>
#include <avr/power.h>

// Define Xbee Objects
XBee xbee = XBee();
uint8_t payload[] = { 0, 100, 0, 0, 0, 0, 0, 0, 0 };
XBeeAddress64 addr64 = XBeeAddress64(0x00000000, 0x00000000);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

// Define Xbee TX/RX pins
SoftwareSerial nss(8, 9);

// Define Magenetic Sensor pins
#define address 0x1E

int statusLed = 13;
int tempIn = 3;
int sleepIn = 2;
int sleepIRQ = 0;

void setup() {
  // Setup led
  pinMode(statusLed, OUTPUT);

  // Setup Magnetic Sensor
  Wire.begin();
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x01);
  Wire.write(0x60);
  Wire.endTransmission();

  // Setup Temp Sensor
  pinMode(tempIn, INPUT);

  // Setup Sleep Pin
  pinMode(sleepIn, INPUT);

  // Setup Xbee
  nss.begin(9600);
  xbee.setSerial(nss);

  Serial.begin(9600);
  
  delay(1000);

  // Habilito las interrupciones para dormir
  attachInterrupt(0, sleepHandler, LOW);
}

void loop() {  
  delay(1000); 

  readMagneticSensor();
  readTempSensor();

  Serial.print("Enviando... ");

  xbee.send(zbTx);

  // flash TX indicator
  flashLed(1, 100);

  // read xbee response
  if (xbee.readPacket(500)) {
    // should be a znet tx status              
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);

      // get the delivery status, the fifth byte
      if (txStatus.getDeliveryStatus() == SUCCESS) {
        // success.  time to celebrate
        flashLed(3, 50);
        
        Serial.println("OK");
      } else {
        // the remote XBee did not receive our packet. is it powered on?
        flashLed(2, 300);
        
        Serial.println("No Remote Response");
      }
    }
  } else if (xbee.getResponse().isError()) {
    flashLed(3, 300);
    
    Serial.print("Error reading packet.  Error code: ");  
    Serial.println(xbee.getResponse().getErrorCode());
  } else {
    // local XBee did not provide a timely TX Status Response -- should not happen
    flashLed(4, 300);

    Serial.println("Error");
  }
  
  if(sleepIRQ == 1)
  {
    Serial.println("Entering sleep");
    delay(200);
    sleepIRQ = 0;
    doSleep();
  }
}

void readMagneticSensor() {
  Serial.print("Magnetic... ");
    
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x02); //select mode register
  Wire.write(0x01); //single measurement mode
  
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    payload[2] = Wire.read(); //X msb
    payload[3] = Wire.read(); //X lsb
    payload[4] = Wire.read(); //Z msb
    payload[5] = Wire.read(); //Z lsb
    payload[6] = Wire.read(); //Y msb
    payload[7] = Wire.read(); //Y lsb
    
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
  }
}
void readTempSensor() {
  payload[8] = analogRead(tempIn);
  
  Serial.print("Temp... ");
  Serial.println(payload[6]);
}

// Sleep Handling
void wakeupHandler(void)
{
  detachInterrupt(0);
}
void sleepHandler(void)
{
  detachInterrupt(0);
  sleepIRQ = 1;
}
void doSleep(void)
{
  attachInterrupt(0, wakeupHandler, HIGH);
  delay(100);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  sleep_disable(); 

  attachInterrupt(0, sleepHandler, LOW);
}


// Status led
void flashLed(int times, int wait) {
  for (int i = 0; i < times; i++) {
    digitalWrite(statusLed, HIGH);
    delay(wait);
    digitalWrite(statusLed, LOW);

    if (i + 1 < times) {
      delay(wait);
    }
  }
}
