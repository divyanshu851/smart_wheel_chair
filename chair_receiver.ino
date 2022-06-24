#include <esp_now.h>
#include <WiFi.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <NewPing.h>

#define TRIGGER_PIN1 5 
#define ECHO_PIN1 35  
#define MAX_DISTANCE1 200 

#define TRIGGER_PIN2  33 
#define ECHO_PIN2    32  
#define MAX_DISTANCE2 200 

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE1);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE2);

uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xB2, 0x99, 0xD4};
String success;
MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

typedef struct struct_message_t {
  uint16_t pot1;
  uint16_t pot2;
  bool button1;
} struct_message_t;

struct_message_t incoming_value;

typedef struct struct_message_r {
float beatsPerMinute;
uint8_t beatAvg;
long irValue;
bool is_object;
} struct_message_r;

struct_message_r new_send_value;

uint16_t pot1;
uint16_t pot2;
bool button1;

#define r_en 2
#define l_en 15
#define r_pwm 12
#define l_pwm 13
#define r_pwm2 14
#define l_pwm2 27

const int frequency = 1000;
const int pwmChannel = 0;
const int pwmChannel2 = 1;
const int pwmChannel3 = 2;
const int pwmChannel4 = 3;
const int pwmResolution = 8;
const int max_duty_cycle = 255;

uint16_t duty;
uint16_t forward;
uint16_t backward;
uint16_t left;
uint16_t right;
uint8_t control_s;
uint8_t control_l;

uint8_t sonar_val1;
uint8_t sonar_val2;
bool is_obj;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming_value, incomingData, sizeof(incoming_value));
//  Serial.print("Bytes received: ");
//  Serial.println(len);
  pot1 = incoming_value.pot1;
  pot2 = incoming_value.pot2;
  button1 = incoming_value.button1;
}


void setup() {
   // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

 if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

   // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  //motor setup
  pinMode(r_en, OUTPUT);
  digitalWrite(r_en, HIGH);
  pinMode(l_en, OUTPUT);
  digitalWrite(l_en, HIGH);
  ledcSetup(pwmChannel, frequency, pwmResolution);
  ledcAttachPin(r_pwm, pwmChannel);
  ledcSetup(pwmChannel2, frequency, pwmResolution);
  ledcAttachPin(l_pwm, pwmChannel2);
  ledcSetup(pwmChannel3, frequency, pwmResolution);
  ledcAttachPin(r_pwm2, pwmChannel3);
  ledcSetup(pwmChannel4, frequency, pwmResolution);
  ledcAttachPin(l_pwm2, pwmChannel4);
}

void loop() {
  
long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

 sonar_val1 = sonar1.ping_cm();
 sonar_val2 = sonar2.ping_cm(); 
 
 if((sonar_val1 > 1 and sonar_val1 < 20) or (sonar_val2 > 1 and sonar_val2 < 20)){
    ledcWrite(pwmChannel, 0);
    ledcWrite(pwmChannel2, 0);
    ledcWrite(pwmChannel3, 0);
    ledcWrite(pwmChannel4, 0);
    is_obj = true;
 }else
  if(pot1 < 1800){
  forward = map(pot1,1850, 0, 0, 255);
//  Serial.print("Forward : ");
//  Serial.println(forward);
    ledcWrite(pwmChannel, forward);
    ledcWrite(pwmChannel2, 0);
    ledcWrite(pwmChannel3, forward);
    ledcWrite(pwmChannel4, 0);
    is_obj = false;    
}else if(pot1 > 1900){
  backward = map(pot1, 1901, 4095, 0, 255);
//  Serial.print("backward : ");
//  Serial.println(backward);
    ledcWrite(pwmChannel, 0);
    ledcWrite(pwmChannel2, backward);
    ledcWrite(pwmChannel3, 0);
    ledcWrite(pwmChannel4, backward);
    is_obj = false;
  }else if(pot2 < 1800){
  left = map(pot2,1850, 0, 0, 255);
//  Serial.print("left : ");
//  Serial.println(left);
    ledcWrite(pwmChannel, left);
    ledcWrite(pwmChannel2, 0);
    ledcWrite(pwmChannel3, 0);
    ledcWrite(pwmChannel4, 0);
    is_obj = false;
}else if(pot2 > 1900){
  right = map(pot2, 1901, 4095, 0, 255);
//  Serial.print("right : ");
//  Serial.println(right);
    ledcWrite(pwmChannel, 0);
    ledcWrite(pwmChannel2, 0);
    ledcWrite(pwmChannel3, right);
    ledcWrite(pwmChannel4, 0);
    is_obj = false;
  }else{
    ledcWrite(pwmChannel, 0);
    ledcWrite(pwmChannel2, 0);
    ledcWrite(pwmChannel3, 0);
    ledcWrite(pwmChannel4, 0);
    is_obj = false;
  }

new_send_value.beatsPerMinute = beatsPerMinute;
new_send_value.beatAvg = beatAvg;
new_send_value.irValue = irValue;
new_send_value.is_object = is_obj;

// Send message via ESP-NOW
esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &new_send_value, sizeof(new_send_value));

 if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  


//
// Serial.print("IR Value :");
// Serial.print(irValue);
// Serial.print( " | Beats Per Min:");
// Serial.print(beatsPerMinute);
// Serial.print(" | Aveage Beats : ");
// Serial.println(beatAvg);

  Serial.print("Ping: ");
  Serial.print(sonar1.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print("cm");
  Serial.print(" | Ping: ");
  Serial.print(sonar2.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");

Serial.print(is_obj);

delay(50);
}
