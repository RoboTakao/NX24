#include "M5Capsule.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> //別途「Adafruit BusIO」ライブラリ必要

#include <FastLED.h>

#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// OLED設定
#define SCREEN_WIDTH 128  //OLED 幅指定
#define SCREEN_HEIGHT 64  //OLED 高さ指定
#define OLED_RESET -1     //リセット端子（未使用-1）

#define SERVICE_UUID "1010"
#define CHRX_UUID "1012"
#define CHTX_UUID "1011"

byte joyLX=100, joyLY=100, joyRX=100, joyRY=100, joyLSW, joyRSW, joyLDistance, joyRDistance;

BLEServer* pServer = NULL;
BLECharacteristic* pCharTx = NULL;
BLECharacteristic* pCharRx = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

// eye
int Sw = SCREEN_WIDTH;
int Sh = SCREEN_HEIGHT;
int Pe = 60;
int We = 20;
int He = 26;
int Hec = 8;
int ReX = Sw/2 - Pe/2 - We/2;
int LeX = Sw/2 + Pe/2 - We/2;
int eY = Sh/2 - He/2;
int ecY = Sh/2 - Hec/2;
int Re = 6;
int eC = 0;

// I2Cに接続されたSSD1306用「display」の宣言
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const uint8_t Srv0 = 1; //GPIO Right Front
const uint8_t Srv1 = 3; //GPIO Right Back
const uint8_t Srv2 = 5; //GPIO Left Front
const uint8_t Srv3 = 7; //GPIO Left Back

const uint8_t srv_CH0 = 0, srv_CH1 = 1, srv_CH2 = 2, srv_CH3 = 3; //チャンネル
const double PWM_Hz = 50;   //PWM周波数
const uint8_t PWM_level = 14; //PWM 14bit(0～16384)

int pulseMIN = 410;  //0deg 500μsec 50Hz 14bit : PWM周波数(Hz) x 2^14(bit) x PWM時間(μs) / 10^6
int pulseMAX = 2048;  //180deg 2500μsec 50Hz 14bit : PWM周波数(Hz) x 2^14(bit) x PWM時間(μs) / 10^6

int cont_min = 0;
int cont_max = 180;

int angZero[] = {87,85,90,95}; //Trimming
int ang0[4];
int ang1[4];
int ang_b[4];
char ang_c[4];
float ts=160;  //100msごとに次のステップに移る
float td=10;   //20回で分割

int position_status = 0;

int walk_mode = 1;

// Forward Step
int f_s[4][4]={
  { 30,  0,  0,-30},
  {  0,-30, 30,  0},
  {  0, 30,-30,  0},
  {-30,  0,  0, 30}};
  
// Back Step
int b_s[4][4]={
  {-30,  0,  0, 30},
  {  0, 30,-30,  0},
  {  0,-30, 30,  0},
  { 30,  0,  0,-30}};

// Left Step
int l_s[4][4]={
  {  0,-30, 30,  0},
  {-30,  0,  0, 30},
  { 30,  0,  0,-30},
  {  0, 30,-30,  0}};

// Right Step
int r_s[4][4]={
  {  0, 30,-30,  0},
  { 30,  0,  0,-30},
  {-30,  0,  0, 30},
  {  0,-30, 30,  0}};

  
// Left Turn_Step
int l_t_s[4][4]={
  { 30,  0,  0, 30},
  {  0,-30,-30,  0},
  {  0, 30, 30,  0},
  {-30,  0,  0,-30}};

// Right Turn Step
int r_t_s[4][4]={
  {-30,  0,  0,-30},
  {  0, 30, 30,  0},
  {  0,-30,-30,  0},
  { 30,  0,  0, 30}};

// Home position
int h_p[4]={0,0,0,0};
  
// Right Hand
int r_h_s[4]={90,10,-10,0};

// Left Hand
int l_h_s[4]={10,0,-90,-10};

// Up
int u_s[4]={0,-70,0,70};

// Down
int d_s[4]={70,0,-70,0};

void Initial_Value(){  //initial servo angle
  int cn = 50;
  for (int j=0; j <=3; j++){
    Srv_drive(j, angZero[j]);
    ang0[j] = angZero[j];
    ang1[j] = angZero[j];
    delay(cn);
  }
}

void face_clear(){
  display.clearDisplay();     //表示クリア
}

void face_center_eye(void *pvParameters){
  while(1)
  {
    face();
    delay(1500);
    face_clear();
    display.fillRect(ReX + eC, ecY, We, Hec,WHITE);      //Rectangle（Filled）
    display.fillRect(LeX + eC, ecY, We, Hec,WHITE);      //Rectangle（Filled）
    display.display();  //表示実行
    delay(200);
  }
}

void face(){
  face_clear();
  display.fillRoundRect(ReX + eC, eY, We, He, Re,WHITE);      //Rounded Rectangle（Filled）
  display.fillRoundRect(LeX + eC, eY, We, He, Re,WHITE);      //Rounded Rectangle（Filled）
  display.display();  //表示実行
}

void Srv_drive(int srv_CH,int SrvAng){
  SrvAng = map(SrvAng, cont_min, cont_max, pulseMIN, pulseMAX);
  ledcWrite(srv_CH, SrvAng);
}


void servo_set(){
  int a[4],b[4];
  
  for (int j=0; j <=3 ; j++){
      a[j] = ang1[j] - ang0[j];
      b[j] = ang0[j];
      ang0[j] = ang1[j];
  }

  for (int k=0; k <=td ; k++){

      Srv_drive(srv_CH0, a[0]*float(k)/td+b[0]);
      Srv_drive(srv_CH1, a[1]*float(k)/td+b[1]);
      Srv_drive(srv_CH2, a[2]*float(k)/td+b[2]);
      Srv_drive(srv_CH3, a[3]*float(k)/td+b[3]);

      delay(ts/td);
  }
}

void forward_step()
{
  eC = 0;
  face();
  for (int i=0; i <=3 ; i++){
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + f_s[i][j];
    }
  servo_set();
  }
}

void back_step()
{
  eC = 0;
  face();
  for (int i=0; i <=3 ; i++){
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + b_s[i][j];
    }
  servo_set();
  }
}

void right_step()
{
  eC = -20;
  face();
  for (int i=0; i <=3 ; i++){
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + r_s[i][j];
    }
  servo_set();
  }
}

void left_step()
{
  eC = 20;
  face();
  for (int i=0; i <=3 ; i++){
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + l_s[i][j];
    }
  servo_set();
  }
}

void right_turn_step()
{
  eC = -20;
  face();
  for (int i=0; i <=3 ; i++){
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + r_t_s[i][j];
    }
  servo_set();
  }
}

void left_turn_step()
{
  eC = 20;
  face();
  for (int i=0; i <=3 ; i++){
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + l_t_s[i][j];
    }
  servo_set();
  }
}

void right_hand_step()
{
  eC = -20;
  face();
  for (int j=0; j <=3 ; j++){
    ang1[j] = angZero[j] + r_h_s[j];
  }
  servo_set();
}

void left_hand_step()
{
  eC = 20;
  face();
  for (int j=0; j <=3 ; j++){
    ang1[j] = angZero[j] + l_h_s[j];
  }
  servo_set();
}

void up_step()
{
  eC = 0;
  face();
  for (int j=0; j <=3 ; j++){
    ang1[j] = angZero[j] + u_s[j];
  }
  servo_set();
}


void down_step()
{
  eC = 0;
  face();
  for (int j=0; j <=3 ; j++){
    ang1[j] = angZero[j] + d_s[j];
  }
  servo_set();
}

void home_position()
{
  for (int j=0; j <=3 ; j++){
    ang1[j] = angZero[j] + h_p[j];
  }
  servo_set();
}

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string value = pChar->getValue();
    if (value.length()>0) {
      joyLX=value[0];
      joyLY=value[1];
      joyRX=value[2];
      joyRY=value[3];
      joyLSW=value[4];
      joyRSW=value[5];
    }
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setupBLE() {
  BLEDevice::init("NX23_M5CoreS3");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharTx = pService->createCharacteristic(CHTX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharRx = pService->createCharacteristic(CHRX_UUID, BLECharacteristic::PROPERTY_WRITE_NR);
  pCharRx ->setCallbacks(new MyCallbacks());
  pCharTx->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void checkBLE() {
  // notify changed value
  if (deviceConnected) {
      pCharTx->setValue((uint8_t*)&value, 6);
      pCharTx->notify();
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
}

void setup() {
  Serial.begin(151200);
  auto cfg = M5.config();
  M5Capsule.begin(cfg);

  pinMode(GPIO_NUM_46, OUTPUT);

  setupBLE();
  
  Wire.begin(13, 15);   //Grove端子をI2C設定(SDA,SCL)

  FastLED.addLeds<NEOPIXEL, GPIO_NUM_21>(leds, NUM_LEDS);
  leds[0] = 0xFF44DD;
  FastLED.show();

  // OLED初期設定
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306:0 allocation failed"));
    for (;;); //エラーなら無限ループ
  }
  // OLED表示設定
  display.setTextColor(SSD1306_WHITE);  //文字色
  
  pinMode(Srv0, OUTPUT);
  pinMode(Srv1, OUTPUT);
  pinMode(Srv2, OUTPUT);
  pinMode(Srv3, OUTPUT);
  
  //モータのPWMのチャンネル、周波数の設定
  ledcSetup(srv_CH0, PWM_Hz, PWM_level);
  ledcSetup(srv_CH1, PWM_Hz, PWM_level);
  ledcSetup(srv_CH2, PWM_Hz, PWM_level);
  ledcSetup(srv_CH3, PWM_Hz, PWM_level);

  //モータのピンとチャンネルの設定
  ledcAttachPin(Srv0, srv_CH0);
  ledcAttachPin(Srv1, srv_CH1);
  ledcAttachPin(Srv2, srv_CH2);
  ledcAttachPin(Srv3, srv_CH3);

  face();

  Initial_Value();

  xTaskCreatePinnedToCore(face_center_eye, "face_center_eye", 4096, NULL, 1, NULL, 1);
}

void loop() {
  M5.update();

  checkBLE();
  
  if ( M5.BtnA.wasReleased() ) {
    digitalWrite(GPIO_NUM_46, LOW);
  }

  if (joyRSW == 1)
  {
    walk_mode = -1*walk_mode;
    delay(100);
  }

  if(walk_mode == -1)
  {
    leds[0] = CRGB::Green;
    FastLED.show();
    
    if (joyRY > 150){
      forward_step();
    }

    if (joyRY < 50){
      back_step();
    }

    if (joyRX > 150){
      left_step();
    }

    if (joyRX < 50){
      right_step();
    }

    if (joyLX > 150){
      left_turn_step();
    }

    if (joyLX < 50){
      right_turn_step();
    }

    if (((joyRY <= 150) && (joyRY >= 50)) && ((joyRX <= 150) && (joyRX >= 50)))
    {
      eC = 0;
      home_position();
      }
  }

  if(walk_mode == 1)
  {
      leds[0] = CRGB::Blue;
      FastLED.show();
      
      if (joyLX > 150){
        left_hand_step();
      }

      else if (joyLX < 50){
        right_hand_step();
      }

      else if (joyLY < 50){
        down_step();
      }

      else if (joyLY > 150){
        up_step();
      }
      
      else if (((joyRY <= 150) && (joyRY >= 50)) && ((joyRX <= 150) && (joyRX >= 50)))
      {
        eC = 0;
        home_position();
      }
  }
}
