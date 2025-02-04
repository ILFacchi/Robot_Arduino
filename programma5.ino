#include "Adafruit_VL53L0X.h"
#include <QTRSensors.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>



//QTR
QTRSensors qtr;
const uint8_t SensorCount = 13;
uint16_t sensorValues[SensorCount];

int massimi[13]={2264, 1376, 1304, 1216, 1376, 1136, 1296, 1296, 1296, 1372, 1300, 1376, 1784};
int minimi[13]={660, 508, 508, 508, 508, 436, 508, 508, 508, 508, 508, 576, 584};

//unsigned long t1, dt; //non credo serva

//PID
float Kp = 0.2; //proportional 0.61
float Ki = 0.001; //integral 0.002
float Kd = 0.3; //derivative 0.9
int P;
int I;
int D;

int lastError = 0;

const uint8_t maxspeeddx = 180;
const uint8_t maxspeedsx = 180;
const uint8_t basespeeddx = 120;
const uint8_t basespeedsx = 120;

//motori
#define dirSX 7 //m1dir
#define dirDX 8 //m2dir
#define velSX 9 //m1pwm
#define velDX 10 //m2pwm
#define velo 160
int ledPin = 13;

//porte SCL e SDA
#define TCAADDR 0x70 //multiplexer
#define sens_sx 7
#define sens_dx 2
#define tof 1
#define giroscopio 0

MPU6050 mpu6050(Wire);

// inizializzazione sensori
Adafruit_TCS34725 LeftColorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_16X);
Adafruit_TCS34725 RightColorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_16X);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//funzioni per muoversi
void tcaselect(uint8_t addr) {
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << addr);
  Wire.endTransmission();
}

void sinistra(int vel) {//
  digitalWrite(dirSX, HIGH);
  digitalWrite(dirDX, LOW);
  analogWrite(velSX, vel);
  analogWrite(velDX, vel);
  //Serial.println("Sinistra");
}

void destra(int vel) {//
  digitalWrite(dirSX, LOW);
  digitalWrite(dirDX, HIGH);
  analogWrite(velSX, vel);
  analogWrite(velDX, vel);
  //Serial.println("Destra");
}

void avanti(int vel) {//
  digitalWrite(dirSX, LOW);
  digitalWrite(dirDX, LOW);
  analogWrite(velSX, vel);
  analogWrite(velDX, vel);
  //Serial.println("Avanti");
}

void indietro(int vel) {//
  digitalWrite(dirSX, HIGH);
  digitalWrite(dirDX, HIGH);
  analogWrite(velSX, vel);
  analogWrite(velDX, vel);
  //Serial.println("Indietro");
}

void girodx(int gradi)
{
  Serial.print("Turn right: ");Serial.println(gradi);
  tcaselect(giroscopio);
  mpu6050.update();
  float x = mpu6050.getAngleZ();
  while ((x - mpu6050.getAngleZ()) * 1.03 < gradi) {
    //Serial.println(mpu6050.getAngleZ() - x);
    tcaselect(giroscopio);
    mpu6050.update();
    destra(140);
  }
  ferma();
}

void girosx(int gradi)
{
  Serial.print("Turn left: ");Serial.println(gradi);
  tcaselect(giroscopio);
  mpu6050.update();
  float x = mpu6050.getAngleZ();
  while ((mpu6050.getAngleZ() - x) * 1.03 < gradi) {
    //Serial.println(mpu6050.getAngleZ() - x);
    tcaselect(giroscopio);
    mpu6050.update();
    sinistra(140);
  }
  ferma();
}

void ferma() {
  digitalWrite(dirSX, LOW);
  digitalWrite(dirDX, HIGH);
  analogWrite(velSX, 0);
  analogWrite(velDX, 0);
  //Serial.println("Ferma");
}

void avanticm(float cm)
{
  avanti(140);
  delay(1000 / 9 * cm);
}

void indietrocm(float cm)
{
  indietro(140);
  delay(1000 / 9 * cm);
}

void avanticmostacolo(float cm) {
  int i = 0;
  while (somma_sensori_dx() < 1000 && i < (100 / 9 * cm)) {
    avanti(140);
    delay(10);
    i += 1;
  }
}

int somma_sensori()
{
  int s_sensori = 0;
  uint16_t position = qtr.readLineBlack(sensorValues);
  for (int i = 0; i < SensorCount; i++) {
    //Serial.print(sensorValues[i]);Serial.print("  ");
    s_sensori += sensorValues[i];
  }
  //Serial.println("");
  //Serial.print("somma sensori: "); Serial.println(s_sensori);
  return s_sensori;
}

int somma_sensori_dx() {
  int s_sensori_dx = 0;
  uint16_t position = qtr.readLineBlack(sensorValues);
  for (int i = 10; i < SensorCount; i++) {
    s_sensori_dx += sensorValues[i];
  }
  return s_sensori_dx;
}

void qtrsingolo() {
  int s_sensori_dx = 0;
  int s_sensori_cx = 0;
  int s_sensori_sx = 0;
  
  uint16_t position = qtr.readLineBlack(sensorValues);
  for (int i = 0; i < 4; i++) {
    s_sensori_sx += sensorValues[i];
  }
  for (int i = 5; i < 8; i++) {
    s_sensori_cx += sensorValues[i];
  }
  for (int i = 9; i < 13; i++) {
    s_sensori_dx += sensorValues[i];
  }
  
  //Serial.print("Differenza s_sx e s_dx: ");Serial.print(s_sensori_cx - s_sensori_sx);Serial.print("        ");Serial.println(s_sensori_cx - s_sensori_dx);
  //Serial.print("s_sensori sx, cx, dx: ")Serial.print(s_sensori_sx);Serial.print("   ");Serial.print(s_sensori_cx);Serial.print("   ");Serial.print(s_sensori_dx);Serial.println("   ");
  //Serial.print("Somma sensori: ")Serial.println(somma_sensori());
  
  if ((s_sensori_cx - s_sensori_sx < 0) && (s_sensori_cx - s_sensori_dx < 0) && (somma_sensori() < 8000) && (somma_sensori() > 5500)) {
    
    int qtrbassi = 0;
    for (int i = 2; i < SensorCount - 2; i++) {
      Serial.print(sensorValues[i]);Serial.print("   ");
      if (sensorValues[i] < 100) {
        qtrbassi += 1;
      }
    }
    Serial.println("");
    
    if (qtrbassi >= 2) {
      Serial.println("Crossroads");
      ferma();
      delay(100);
      indietrocm(2);
      ferma();
      contrVerde();
    }
  }
}

bool leggiVerde(Adafruit_TCS34725 &TCSensor, int sens)
{
  delay(100);
  tcaselect(sens);
  delay(600);  // takes 50ms to read
  float red, green, blue;
  TCSensor.getRGB(&red, &green, &blue);
  Serial.print("Green"); Serial.print(sens); Serial.print(": R "); Serial.print(red); Serial.print("  G "); Serial.print(green); Serial.print("  B "); Serial.println(blue);
  if (green - red > 20 && green - blue > 20)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool leggiRosso(Adafruit_TCS34725 &TCSensor, int sens)
{
  delay(100);
  tcaselect(sens);
  delay(600);  // takes 50ms to read
  float red, green, blue;
  TCSensor.getRGB(&red, &green, &blue);
  Serial.print("Red"); Serial.print(sens);Serial.print("  R:"); Serial.print(red); Serial.print("  G:"); Serial.print(green); Serial.println("  B:"); Serial.println(blue);
  if (red - green > 80 or red - blue > 80)
  {
    return true;
  }
  else
  {
    return false;

  }
}

float red, green, blue;

int leggiRGB(Adafruit_TCS34725 &TCSensor, int sens)
{
  tcaselect(sens);
  delay(600);  // takes 50ms to read
  float red, green, blue;
  TCSensor.getRGB(&red, &green, &blue);
  int somma = int(uint8_t(red) + uint8_t(green) + uint8_t(blue));
  //Serial.print("somma RGB: "); Serial.println(somma);
  return somma;
}

bool leggiBianco(Adafruit_TCS34725 &TCSensor, int sens)
{
  delay(100);
  tcaselect(sens);
  float red, green, blue;
  delay(600);  // takes 50ms to read
  Serial.print("White"); Serial.print(sens); Serial.print(": R "); Serial.print(red); Serial.print("  G "); Serial.print(green); Serial.print("  B "); Serial.println(blue);
  if (red > 240 and green > 240 and blue > 240)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void contrVerde()// DA MIGLIORARE#############################################################################à
{
  Serial.println("Green Check");
  delay(500);
  if (leggiVerde(RightColorSensor, sens_dx)) //se vede verde a destra
  {
    if (leggiVerde(LeftColorSensor, sens_sx)) //se vede verde anche a sinistra
    {
      Serial.println("Double greeen");
      girodx(150); //torna indietro
      ferma();
      delay(100);
    }
    else //verde solo a destra
    {
      avanticm(3);
      ferma();
      delay(100);
      girodx(50);
      ferma();
      delay(100);
    }
  }
  else if (leggiVerde(LeftColorSensor, sens_sx)) //altrimenti, se il verde è a sinistra
  {
    avanticm(3);
    ferma();
    delay(100);
    girosx(50);
    ferma();
    delay(100);
  }
  else
  {
    Serial.println("No green");
    delay(3000);
    avanticm(2);
    ferma();
    delay(500);
    return;
  }

}

long dist_tof(int sens)
{
  VL53L0X_RangingMeasurementData_t measure;
  tcaselect(sens);
  lox.rangingTest(&measure, false);
  int dist = 8100;
  if (measure.RangeStatus != 4)
  { dist = measure.RangeMilliMeter;
  }
  return dist;
}

void stanza() { //DA FARE ###########################################################################
  Serial.println("======================================== EVACUATION ZONE");
  digitalWrite(ledPin,HIGH);
  ferma();
  delay(100000);
}

void bianco()
{
  ferma();
  Serial.println("GAP");
  while (somma_sensori() < 1000) {
    avanti(basespeeddx);
  }
  Serial.println("GAP end");

}

void girostacolo() {
  Serial.println("Obstacle");
  indietrocm(3);
  ferma();
  delay(100);
  girosx(90);
  while (somma_sensori_dx() < 1000) {
    digitalWrite(dirSX, LOW);
    digitalWrite(dirDX, LOW);
    analogWrite(velSX, 180);
    analogWrite(velDX, 40);
  }
  avanticm(6);
  girosx(60);
  ferma();
  Serial.println("Obstacle overcome");
  return;
}


void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("");Serial.println("======================================== SETUP START");
  pinMode(47, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(dirDX, OUTPUT);
  pinMode(dirSX, OUTPUT);
  pinMode(velDX, OUTPUT);
  pinMode(velSX, OUTPUT);
  delay(100);
  //pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(ledPin, HIGH);

  
  Serial.println("QTR calibration");
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    23, 22, 25, 24, 27, 26, 29, 28, 31, 30, 33, 32, 35
  }, SensorCount);
  qtr.calibrate();
  for (int i = 0; i < SensorCount; i++)
  {
    qtr.calibrationOn.minimum[i] = minimi[i];
    qtr.calibrationOn.maximum[i] = massimi[i];
  }

  Serial.println("RGB calibration");
  leggiVerde(RightColorSensor, sens_dx);   //forse si possono togliere
  leggiVerde(LeftColorSensor, sens_sx);

  Serial.println("TOF calibration");
  tcaselect(tof);
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
  }

  Serial.println("GYRO calibration");
  tcaselect(giroscopio);
  mpu6050.begin();
  delay(100);
  mpu6050.calcGyroOffsets(true);
  //mpu6050.setGyroOffsets(-1.25, 0.35, -0.45);
  int gradX=0;
  for (int i=0; i<30;i++){
    tcaselect(giroscopio);
    mpu6050.update();
    gradX = mpu6050.getAngleX();
  }
  
  digitalWrite(ledPin, LOW);
  Serial.println("\nSETUP END\n======================================== LOOP START");
}

int pwrSX = 1;
int pwrDX = 1;
int i = 0;
int old_position = 0;
void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 6000 - position;

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int varsomma = somma_sensori();
  
  //Serial.print("Position: ");Serial.print(position);Serial.print("    somma_sensori: ");Serial.println(varsomma);

  qtrsingolo();
  if (!digitalRead(47)){
    ferma();
    delay(100);
    if (!digitalRead(47)){
      stanza();
    }
  }
  else if (varsomma < 750 && old_position >= 5000 && old_position <= 7000)
  {
    bianco();
  }
  else if (varsomma > 12000) //vedere se variare con old_position
  {
    ferma();
    delay(100);
    Serial.println("Black");
    contrVerde();
  }
  else
  {
    //int motorspeed = Kp*(P+(1/Ki)*I + Kd*D);
    int motorspeed = P * Kp + I * Ki + D * Kd - 100 ; //calculate the correction
    int motorspeedsx = basespeedsx - motorspeed;
    int motorspeeddx = basespeeddx + motorspeed;
    if (position < 6150 and position > 5850)
    {
      motorspeedsx = 140;
      motorspeeddx = 140;
    }

    if (motorspeedsx >= 0)
    {
      pwrSX = 0;
    }
    else
    {
      pwrSX = 1;
    }
    if (motorspeeddx >= 0)
    {
      pwrDX = 0;
    }
    else
    {
      pwrDX = 1;
    }
    if (motorspeedsx > maxspeedsx || -motorspeedsx > maxspeedsx) {
      motorspeedsx = maxspeedsx;
    }
    if (motorspeeddx > maxspeeddx || -motorspeeddx > maxspeeddx) {
      motorspeeddx = maxspeeddx;
    }
    if (motorspeedsx < 0) { //inizialmente < 0
      motorspeedsx = -motorspeedsx;
    }
    if (motorspeeddx < 0) { //inizialmente < 0
      motorspeeddx = -motorspeeddx;
    }

    digitalWrite(dirSX, pwrSX);
    digitalWrite(dirDX, pwrDX);
    analogWrite(velSX, motorspeedsx);
    analogWrite(velDX, motorspeeddx);
  }
  old_position = position;
}

void PID_discesa() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  Serial.print("Position: ");Serial.println(position);
  int error = 6000 - position;
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int varsomma = somma_sensori();

  //int motorspeed = Kp*(P+(1/Ki)*I + Kd*D);
  int motorspeed = P * 0.004; //calculate the correction
  int motorspeedsx = basespeedsx - motorspeed;
  int motorspeeddx = basespeeddx + motorspeed;
  if (position < 6150 and position > 5850)
  {
    motorspeedsx = 140;
    motorspeeddx = 140;
  }

  if (motorspeedsx >= 0)
  {
    pwrSX = 0;
  }
  else
  {
    pwrSX = 1;
  }
  if (motorspeeddx >= 0)
  {
    pwrDX = 0;
  }
  else
  {
    pwrDX = 1;
  }
  if (motorspeedsx > maxspeedsx || -motorspeedsx > maxspeedsx) {
    motorspeedsx = maxspeedsx;
  }
  if (motorspeeddx > maxspeeddx || -motorspeeddx > maxspeeddx) {
    motorspeeddx = maxspeeddx;
  }
  if (motorspeedsx < 0) { //inizialmente < 0
    motorspeedsx = -motorspeedsx;
  }
  if (motorspeeddx < 0) { //inizialmente < 0
    motorspeeddx = -motorspeeddx;
  }

  digitalWrite(dirSX, pwrSX);
  digitalWrite(dirDX, pwrDX);
  analogWrite(velSX, motorspeedsx);
  analogWrite(velDX, motorspeeddx);
}

void fine_discesa() {
  Serial.println("End descent");
  avanticm(4);
  ferma();
  delay(100);
  ferma();
  uint16_t position = qtr.readLineBlack(sensorValues);
  int posizione = position;
  Serial.println(position);
  if (position > 7000) {
    indietrocm(15);
    girodx(25);
    avanticm(13);
    ferma();
  }
  else if (position < 5000) {
    indietrocm(15);
    girosx(25);
    avanticm(13);
    ferma();
  }
}

const long tempo = 100;
unsigned long MillisPrecedenti = 0;

bool presenza_discesa = true;
int gradX = 0;
int gradY = 0;
int gradXvecchio = 0;

void loop() {
  unsigned long MillisAdesso = millis();
  PID_control();
  if (MillisAdesso - MillisPrecedenti >= tempo)
  {
    MillisPrecedenti = MillisAdesso;
    if (dist_tof(tof) < 40) {
      ferma();
      delay(100);
      if (dist_tof(tof) < 45) {
        girostacolo();
      }
    }
  }

  tcaselect(giroscopio);
  mpu6050.update();
  gradX = mpu6050.getAngleX();
  //gradY = mpu6050.getAngleY();

  //Serial.print("GradX: ");Serial.println(gradX); Serial.print("     GradY: "); Serial.println(gradY);
  //Serial.print("Bottone: ")Serial.println(digitalRead(47));
  if (!presenza_discesa && gradX < 3) {
    presenza_discesa = true;
  }
  else if (gradX >= 8) {
    Serial.println("Start descent");
    while (gradX >= 8) {
      tcaselect(giroscopio);
      mpu6050.update();
      gradX = mpu6050.getAngleX();
      PID_discesa();
      if ((gradX == 15 && gradXvecchio == 16) && presenza_discesa) {
        fine_discesa();
        presenza_discesa = false;
      }
      gradXvecchio = gradX;
    }
  }
  else if (!digitalRead(47)){
    ferma();
    delay(100);
    if (!digitalRead(47)){
      stanza();
    }
  }
  
  gradXvecchio = gradX;
}
