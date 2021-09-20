#include <WISOL.h>
#include <Tsensors.h>
#include <Wire.h>
#include <math.h>
#include <SimpleTimer.h>
#include <avr/wdt.h>
#include <millisDelay.h>
#include <SPI.h>
#include <DHT.h>

// definimos los pines que vamos a utilizar 
#define windDirectionPin A5
#define windSensorPin A4     

#define UVOUT A2
#define REF_3V3 A3
#define DHTPIN 4  // Definimos el pin digital donde se conecta el sensor

#define DHTTYPE DHT11 // Dependiendo del tipo de sensor
DHT dht(DHTPIN, DHTTYPE); // Inicializamos el sensor DHT11

millisDelay timerDelay;
millisDelay Delay;
//variables estacion meteorologica 
int pulses = 0;
int windTest1 = 1;
int windTest2 = 0;
int windTimer = 1;
int result = 1;
int startTest = 1;
int timerStart = 1;
int windCounter = 0;

  // velocidad del viento 
int windPulse = 0; //pulsos por giro
float windSpeed = 0; //velocidad en

  // direccion de viento

int windDir =10;

//

Isigfox *Isigfox = new WISOL();
Tsensors *tSensors = new Tsensors();
SimpleTimer timer;
int watchdogCounter;
uint8_t buttonCounter;
uint8_t PublicModeSF;
uint8_t stateLED;
uint8_t ledCounter;
const uint8_t buttonPin = A1;
const int redLED = 6;

typedef union{
    float number;
    uint8_t bytes[4];
} FLOATUNION_t;

typedef union{
    uint16_t number;
    uint8_t bytes[2];
} UINT16_t;

typedef union{
    int16_t number;
    uint8_t bytes[2];
} INT16_t;

void setup() {
  int flagInit;
  
  Wire.begin();
  Wire.setClock(100000);

 

  // Init watchdog timer
  watchdogSetup();
  watchdogCounter = 0;
  
  // WISOL test
  flagInit = -1;
  while (flagInit == -1) {
  
  delay(1000);
  PublicModeSF = 0;
  flagInit = Isigfox->initSigfox();
  Isigfox->testComms();
  GetDeviceID();
  //Isigfox->setPublicKey(); // set public key for usage with SNEK
  }
  // iniciar sensores
  pinMode(windSensorPin, INPUT);
  pinMode(windDirectionPin, INPUT); 
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
  dht.begin();   // iniciamos el sensor DHT
  
  // Init sensors on Thinxtra Module
  //tSensors->initSensors();
  //tSensors->setReed(reedIR);
  buttonCounter = 0;
  tSensors->setButton(buttonIR);

  // Init LED
  stateLED = 0;
  ledCounter = 0;
//  pinMode(redLED, INPUT);

  // Init timer to send a Sigfox message every 10 minutes
  unsigned long sendInterval = 600000;
  timer.setInterval(sendInterval, timeIR);

  
  delay(1000);
}

void loop() {
  timer.run();
  wdt_reset();
  watchdogCounter = 0;

    // lectura de la estacion meteorologica  

  if (analogRead(windSensorPin) < 1000 && startTest == 1 ){
    windTest1 = 1;
    windTest2 = 1;
    windTimer = 1;
    result = 1;
    startTest = 0;
  }
  // lectura del viento
  WindSpeed();  
  windDelay();

//  getDirection(analogRead(windDirectionPin));
 // Serial.println(windDir);
}

void Send_Sensors(){
  UINT16_t tempt, photo, hum,uv,windVel,wind;
  INT16_t x_g, y_g, z_g;
  acceleration_xyz *xyz_g;
  FLOATUNION_t a_g;

   
  
  // Sending a float requires at least 4 bytes
  // In this demo, the measure values (temperature, pressure, sensor) are scaled to ranged from 0-65535.
  // Thus they can be stored in 2 bytes
  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  float outputVoltage = 3.3 / refLevel * uvLevel; // voltaje de referencia 3V3
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //conversion de voltaje del sensor a radiacion UV dependiendo de la referencia
  
  uv.number = (uint16_t) (uvIntensity * 100);
 // Serial.print("UV: "); Serial.println((float)uv.number/100);
  
  // lectura de la temperatura y humedad del ambiente
  float h = dht.readHumidity(); //leer humedad
  float t = dht.readTemperature(); // leer temperatira en °F 
  tempt.number = (uint16_t) (t * 100);
  //Serial.print("Temp: "); Serial.println((float)tempt.number/100);
  
  hum.number =(uint16_t) (h*100);
 // Serial.print("Humidity: "); Serial.println((float)hum.number/100);

  windVel.number = (uint16_t) (windSpeed * 100);
  //Serial.print("Velocidad: "); Serial.println((float)windVel.number/1000);

  getDirection(analogRead(windDirectionPin));  //obtener la direccion del viento
//  Serial.print("Direccion: ");Serial.println(windDir);
  wind.number=(uint16_t) (windDir);
  
  //xyz_g = (acceleration_xyz *)malloc(sizeof(acceleration_xyz));
  //tSensors->getAccXYZ(xyz_g);
  x_g.number = (int16_t) (12.5 * 250);
  y_g.number = (int16_t) (34.3 * 250);
  z_g.number = (int16_t) (45.2 * 250);
 // Serial.print("Acc X: "); Serial.println((float)x_g.number/250);
  //Serial.print("Acc Y: "); Serial.println((float)y_g.number/250);
  //Serial.print("Acc Z: "); Serial.println((float)z_g.number/250);
  //Serial.print("\0");
  free(xyz_g);

  const uint8_t payloadSize = 12; //in bytes
//  byte* buf_str = (byte*) malloc (payloadSize);
  uint8_t buf_str[payloadSize];

  buf_str[0] = tempt.bytes[0];
  buf_str[1] = tempt.bytes[1];
  buf_str[2] = hum.bytes[0];
  buf_str[3] = hum.bytes[1];
  buf_str[4] = uv.bytes[0];
  buf_str[5] = uv.bytes[1];
  buf_str[6] = windVel.bytes[0];
  buf_str[7] = windVel.bytes[1];
  buf_str[8] = wind.bytes[0];
  buf_str[9] = y_g.bytes[1];
  buf_str[10] = z_g.bytes[0];
  buf_str[11] = z_g.bytes[1];

  Send_Pload(buf_str, payloadSize);
//  free(buf_str);
}

void reedIR(){
  Serial.println("Reed");
  timer.setTimeout(50, Send_Sensors); // send a Sigfox message after get out IRS
}

void buttonIR(){
  if (buttonCounter==0) {
    timer.setTimeout(500, checkLongPress); // check long click after 0.5s
  }
}

void checkLongPress() {
  buttonCounter++;
  if ((buttonCounter < 4)) {
    if (digitalRead(buttonPin) == 1) {
      //Serial.println("Short Press");
      Send_Sensors();
      buttonCounter = 0;
    } else {
      timer.setTimeout(500, checkLongPress); // check long click after 0.5s
    }
  } else {
    Serial.println("Long Press");
    BlinkLED();
    pinMode(redLED, OUTPUT);
    if (PublicModeSF == 0) {
     // Serial.println("Set public key");
      Isigfox->setPublicKey();
      PublicModeSF = 1;
  
    } else {
     // Serial.println("Set private key");
      Isigfox->setPrivateKey();
      PublicModeSF = 0;
    }
    buttonCounter = 0;
  }
}


void BlinkLED() {
  ledCounter++;
  if (ledCounter<=6) {
    if (stateLED == 0){
      digitalWrite(redLED, HIGH);
      stateLED = 1;
      timer.setTimeout(200, BlinkLED);
    } else {
      digitalWrite(redLED, LOW);
      stateLED = 0;
      timer.setTimeout(200, BlinkLED);
    }
  } else {
    pinMode(redLED, INPUT);
    ledCounter = 0;
  }
  
  
}

void timeIR(){
  Serial.println("Time");
  Send_Sensors();
}

void getDLMsg(){
  recvMsg *RecvMsg;
  int result;

  RecvMsg = (recvMsg *)malloc(sizeof(recvMsg));
  result = Isigfox->getdownlinkMsg(RecvMsg);
  for (int i=0; i<RecvMsg->len; i++){
    //Serial.print(RecvMsg->inData[i]);
  }
  //Serial.println("");
  free(RecvMsg);
}


void Send_Pload(uint8_t *sendData, const uint8_t len){
  // No downlink message require
  recvMsg *RecvMsg;

  RecvMsg = (recvMsg *)malloc(sizeof(recvMsg));
  Isigfox->sendPayload(sendData, len, 0, RecvMsg);
  for (int i = 0; i < RecvMsg->len; i++) {
  //  Serial.print(RecvMsg->inData[i]);
  }
  //Serial.println("");
  free(RecvMsg);


  // If want to get blocking downlink message, use the folling block instead
  /*
  recvMsg *RecvMsg;
  RecvMsg = (recvMsg *)malloc(sizeof(recvMsg));
  Isigfox->sendPayload(sendData, len, 1, RecvMsg);
  for (int i=0; i<RecvMsg->len; i++){
    Serial.print(RecvMsg->inData[i]);
  }
  Serial.println("");
  free(RecvMsg);
  */

  // If want to get non-blocking downlink message, use the folling block instead
  /*
  Isigfox->sendPayload(sendData, len, 1);
  timer.setTimeout(46000, getDLMsg);
  */
}


void GetDeviceID(){
  recvMsg *RecvMsg;
  const char msg[] = "AT$I=10";

  RecvMsg = (recvMsg *)malloc(sizeof(recvMsg));
  Isigfox->sendMessage(msg, 7, RecvMsg);

 // Serial.print("Device ID: ");
  for (int i=0; i<RecvMsg->len; i++){
    Serial.print(RecvMsg->inData[i]);
  }
  //Serial.println("");
  free(RecvMsg);
}


void watchdogSetup(void) { // Enable watchdog timer
  cli();  // disable all interrupts
  wdt_reset(); // reset the WDT timer
  /*
   WDTCSR configuration:
   WDIE = 1: Interrupt Enable
   WDE = 1 :Reset Enable
   WDP3 = 1 :For 8000ms Time-out
   WDP2 = 1 :For 8000ms Time-out
   WDP1 = 1 :For 8000ms Time-out
   WDP0 = 1 :For 8000ms Time-out
  */
  // Enter Watchdog Configuration mode:
  // IF | IE | P3 | CE | E | P2 | P1 | P0
  WDTCSR |= B00011000;
  WDTCSR = B01110001;
//  WDTCSR |= (1<<WDCE) | (1<<WDE);
//  // Set Watchdog settings:
//   WDTCSR = (1<<WDIE) | (1<<WDE) | (1<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
  sei();
}


void watchdog_disable() { // Disable watchdog timer
  cli();  // disable all interrupts
  WDTCSR |= B00011000;
  WDTCSR = B00110001;
  sei();
}


ISR(WDT_vect) // Watchdog timer interrupt.
{
// Include your code here - be careful not to use functions they may cause the interrupt to hang and
// prevent a reset.
 // Serial.print("WD reset: ");
  Serial.println(watchdogCounter);
  watchdogCounter++;
  if (watchdogCounter == 20) { // reset CPU after about 180 s
      // Reset the CPU next time
      // Enable WD reset
      cli();  // disable all interrupts
      WDTCSR |= B00011000;
      WDTCSR = B01111001;
      sei();
      wdt_reset();
  } else if (watchdogCounter < 8) {
    wdt_reset();
  }
}




//funciones sensores

void windDelay(){
   if (analogRead(windSensorPin) < 1000 && timerStart == 1){
    timerDelay.start(3000);
    result = 1;
    timerStart = 0;
  }
  
  if (timerDelay.justFinished() && result == 1){
    windSpeed = (1.25 * (pulses/3))*(1.492/2.4);  
    pulses = 0;
    //windSpeed = 0;
    startTest = 1;
    timerStart = 1;
    
  }
}
void WindSpeed(){
  if (analogRead(windSensorPin) < 1000){
    windCounter++;
  }
  if (windCounter > 0 && analogRead(windSensorPin) > 1010 && windTest1 == 1) {
    windPulse += 1;
    windCounter = 0; 
  }
 
  if (windPulse == 3 && windTest2 == 1){
    pulses += 2;
    windPulse = 0;
  }
}

void getDirection(int direction){
  if (direction <=937 && direction >930){
      //N
      windDir=10;
  }else if(direction <=898 && direction >887){ 
     //N
      windDir=10;
  }else if(direction <=694 && direction >684){ 
     //NE
      windDir=20;
  }else if(direction <=742 && direction >732){ 
    //NE
      windDir=20;
  }else if(direction <=284 && direction >274){ 
    //E
      windDir=30;
  }else if(direction <=305 && direction >295){ 
    //E
      windDir=30;
  }else if(direction <=239 && direction >229){ 
    //SE
      windDir=40;
  }else if(direction <=460 && direction >450){ 
     //SE
      windDir=40;
  }else if(direction <=574 && direction >564){ 
      //S
      windDir=50;
  }else if(direction <=531 && direction >521){ 
     //S
      windDir=50;
  }else if(direction <=858 && direction >848){ 
     //SW
      windDir=60;
  }else if(direction <=840 && direction >830){ 
    //SW
      windDir=60;
  }else if(direction <=956 && direction >946){ 
    //W
      windDir=70;
  }else if(direction <=1002 && direction >992){ 
       //W
      windDir=70;
  }else if(direction <=980 && direction >970){ 
      //NW
      windDir=80;
  }else{
      //NA
      windDir=90;
  }

  
}


int averageAnalogRead(int pinToRead){
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 
  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;
  return(runningValue);
}
 
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
