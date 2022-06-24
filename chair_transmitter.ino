#include <esp_now.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>


uint8_t broadcastAddress[] = {0x3C, 0x61, 0x05, 0x12, 0x64, 0xA8};

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);


String success;

typedef struct struct_message_t {
  uint16_t pot1;
  uint16_t pot2;
  bool button1;
} struct_message_t;

struct_message_t sending_values;

typedef struct struct_message_r {
float beatsPerMinute;
uint8_t beatAvg;
long irValue;
bool is_object;
} struct_message_r;

struct_message_r new_send_value;

#define pot_1 32
#define pot_2 33
#define button_1 23
#define button_2 5
#define button_3 18

 uint16_t pot_val;
 uint16_t pot_val2;
 bool but_value1;
 bool but_value2;
 bool but_value3;
 float beats;
 int avgBeat;
 long irValues;
 bool is_obj;


esp_now_peer_info_t peerInfo;

// Callback when data is sent
//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  Serial.print("\r\nLast Packet Send Status:\t");
//  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//  if (status ==0){
//    success = "Delivery Success :)";
//  }
//  else{
//    success = "Delivery Fail :(";
//  }
//}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&new_send_value, incomingData, sizeof(new_send_value));
  Serial.print("Bytes received: ");
  Serial.println(len);
  beats = new_send_value.beatsPerMinute;
  avgBeat = new_send_value.beatAvg;
  irValues = new_send_value.irValue;
  is_obj = new_send_value.is_object;
}

void setup() {
   // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // adding button and pots
  pinMode(pot_1, INPUT);
  pinMode(pot_2, INPUT);
  pinMode(button_1, INPUT);
  pinMode(button_2, INPUT);

// initialize the LCD
  lcd.begin();
  
// Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
//  esp_now_register_send_cb(OnDataSent);

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


  
}

void loop() {
 pot_val = analogRead(pot_1);
 pot_val2 = analogRead(pot_2);
 but_value1 = digitalRead(button_1);
 but_value2 = digitalRead(button_2);
  
  sending_values.pot1 = pot_val;
  sending_values.pot2 = pot_val2;
  sending_values.button1 = but_value1;
  
// Serial.print("IR Value :");
// Serial.print(irValues);
// Serial.print( " | Beats Per Min:");
// Serial.print(beats);
// Serial.print(" | Aveage Beats : ");
// Serial.println(avgBeat);

if(but_value2){
lcd.print("IR Value = ");
lcd.print(irValues);
lcd.setCursor(0,1);
lcd.print("Beats = ");
lcd.print(beats);
}
else if(is_obj)
  lcd.print("Can't Move");
else
  lcd.print("Smart Chair");

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sending_values, sizeof(sending_values));
 delay(100);
 lcd.clear();
}
