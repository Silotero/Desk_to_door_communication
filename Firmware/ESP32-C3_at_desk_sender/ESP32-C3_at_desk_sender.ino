#include <esp_now.h> // Library for espnow
#include <WiFi.h> //Library for WiFi
#include <Wire.h> //library for I2C
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> //Libraries for OLED displays
#include "driver/rtc_io.h" //library for rtc gpio functions
#include <time.h> // library for time functions, here used for getting and storing time from the internet
#include "esp_sleep.h"

#define DEBUG 1 //change to 0 to disable debugging, all the debbuging won't be compiled

#if DEBUG
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif

TwoWire I2C_default = TwoWire(0); //create new i2c lines

const uint8_t SMALL_OLED_SCREEN_WIDTH = 128; // OLED display width, in pixels
const uint8_t SMALL_OLED_SCREEN_HEIGHT = 32; // OLED display height, in pixels
// Declaration for SSD1306 display connected using I2C
const int SMALL_OLED_RESET = -1; // Reset pin # (or -1 if sharing Arduino reset pin or not having a reset pin)
const uint8_t SMALL_OLED_SCREEN_ADDRESS = 0x3C; //OLED screen I2C address
//create and object SMALL_OLED
Adafruit_SSD1306 SMALL_OLED(SMALL_OLED_SCREEN_WIDTH, SMALL_OLED_SCREEN_HEIGHT, &I2C_default, SMALL_OLED_RESET);
bool SMALL_display_nodisplay = true; //boolean for checking if new message should be displayed

const uint8_t BIG_OLED_SCREEN_WIDTH = 128; // OLED display width, in pixels
const uint8_t BIG_OLED_SCREEN_HEIGHT = 64; // OLED display height, in pixels
// Declaration for SSD1306 display connected using I2C
const int BIG_OLED_RESET = -1; // Reset pin # (or -1 if sharing Arduino reset pin or not having a reset pin)
const uint8_t BIG_OLED_SCREEN_ADDRESS = 0x3D; //OLED screen I2C address
//create and object BIG_OLED
Adafruit_SSD1306 BIG_OLED(BIG_OLED_SCREEN_WIDTH, BIG_OLED_SCREEN_HEIGHT, &I2C_default, BIG_OLED_RESET);
bool BIG_display_nodisplay = true; //boolean for checking if new message should be displayed
//pins for rows and columns as well as getting the sizes of the arrays

const char* SMALLoledMessages[] = {
  "",                 // 0 - unused
  "",          // 1
  "",// 2
  "WAZNE",            // 3
  "MAMA",             // 4
  "TATA",             // 5
  "DANIELA",          // 6
  "MACIEK",           // 7
};

portMUX_TYPE key_mux = portMUX_INITIALIZER_UNLOCKED; // a mux to block the same variables from being accessed at the same time, used beacuse of RTOS
portMUX_TYPE message_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE sleep_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE repeat_message_mux = portMUX_INITIALIZER_UNLOCKED;

//task handles for RTOS tasks, needed to check their memory usage
TaskHandle_t buttonWatcherHandle = NULL;
TaskHandle_t SMALLOLEDWatcherHandle = NULL;
TaskHandle_t BIGOLEDWatcherHandle = NULL;
TaskHandle_t KEYPADWatcherHandle = NULL;

SemaphoreHandle_t i2cMutex;

const gpio_num_t ROWS[] = {GPIO_NUM_5,GPIO_NUM_6,GPIO_NUM_7,GPIO_NUM_8};
const gpio_num_t COLS[] = {GPIO_NUM_9,GPIO_NUM_10,GPIO_NUM_20,GPIO_NUM_21};
const uint8_t NUM_ROWS = sizeof(ROWS)/sizeof(gpio_num_t);
const uint8_t NUM_COLS = sizeof(COLS)/sizeof(gpio_num_t);

//create array of the keypad
const char KEYS[NUM_ROWS][NUM_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

// ===========================
// Enter your WiFi credentials
// ===========================
const char* ssid = "Orange_Swiatlowod_7310";
const char* password = "dcCv5oqVPdfbcGQ9U9";

//variables for buttons
gpio_num_t wakeup_sleep_pin = GPIO_NUM_2; //gpio for wakeup;
gpio_num_t buzzer_pin = GPIO_NUM_0;
gpio_num_t Displays_power_pin = GPIO_NUM_1; //pin that powers a mosfet for powering the displays
gpio_num_t SDA_PIN = GPIO_NUM_4; //new i2c sda pin, being the same as the default
gpio_num_t SCL_PIN = GPIO_NUM_3; //new i2c scl pin, being the same as the default
const uint8_t debounce_delay = 130;

bool should_sleep = true;

uint8_t broadcastAddress[] = {0x74,0x4D,0xBD,0x98,0x92,0x18};

const unsigned long uS_TO_S_FACTOR = 1000000;  // conversion factor for seconds to microseconds
unsigned long last_wakeup_time;
const unsigned int keep_awake_for = 10000;

bool message_to_be_sent = false;

String success;

typedef struct struct_message {
  char keypad_char;
} struct_message;
struct_message dataToSend;

typedef struct struct_message2 {
  int message_char;
} struct_message2;
struct_message2 incomingData;

int message_data = 3; //char for receiveing message char

const uint8_t maxRetries = 6; //retries for connecting wifi
int Retries = 0;

RTC_DATA_ATTR char key = '1'; // char for holding a char read from the keypad
char last_key = '1'; // char for holding last key char from the keypad

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  DEBUG_PRINT("\r\nLast Packet Send Status:\t");
  DEBUG_PRINTLN(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0){
    success = "Delivery Success :)";
    portENTER_CRITICAL(&repeat_message_mux)
    message_to_be_sent = false;
    portEXIT_CRITICAL(&repeat_message_mux)
  }
  else{
    success = "Delivery Fail :(";
    portENTER_CRITICAL(&repeat_message_mux)
    message_to_be_sent = true;
    portEXIT_CRITICAL(&repeat_message_mux)
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingBytes, int len) {
  memcpy(&incomingData, incomingBytes, sizeof(incomingData));
  DEBUG_PRINT("Bytes received: ");
  DEBUG_PRINTLN(len);
  DEBUG_PRINT("Value received: ");
  portENTER_CRITICAL(&message_mux);
  message_data = incomingData.message_char;
  DEBUG_PRINTLN(message_data);
  SMALL_display_nodisplay = true;
  last_wakeup_time = millis();
  portEXIT_CRITICAL(&message_mux);
  digitalWrite(buzzer_pin, HIGH);
  delay(100);
  digitalWrite(buzzer_pin, LOW);
}


void synchronizeTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  // Set Poland time zone with DST (daylight saving time, basically the time switch beetween winter and summer) support
  setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
  tzset();
  struct tm timeinfo; // special struct from time.h library, represents broken-down time — meaning it stores the components of a calendar time (like year, month, day, hour, minute, second) in a human-readable form.
  while (!getLocalTime(&timeinfo)) {
    delay(1000);
    DEBUG_PRINTLN("Waiting for NTP time sync...");
  }
}

void prep_for_deep_sleep () {
  struct tm timeheld;
  getLocalTime(&timeheld);
  int hour = timeheld.tm_hour;
  int min = timeheld.tm_min;
  int sec = timeheld.tm_sec;
  DEBUG_PRINT("Godzina ");
  DEBUG_PRINTLN(hour);
  DEBUG_PRINT("Minuta ");
  DEBUG_PRINTLN(min);
  DEBUG_PRINT("Ile będzie spać ");
  DEBUG_PRINTLN(14 - min%15);
  if (hour >= 7 && hour <= 23 ) esp_sleep_enable_timer_wakeup(((14 - min%15) * 60 + (60 - sec))* uS_TO_S_FACTOR); // enable timer based wakeup if inside hours, wakeup at 15 parts of an hour
    esp_deep_sleep_enable_gpio_wakeup(1ULL << wakeup_sleep_pin, ESP_GPIO_WAKEUP_GPIO_LOW);
    delay(100);
    esp_deep_sleep_start();// deep sleep start, no code after it will be performed if triggered
}

//Debounce function for buttons
bool debounce(int button_index) {
  static unsigned long last_debounce_time[2] = {0,0};
  unsigned long now = millis();
  if(now - last_debounce_time[button_index] >= debounce_delay) {
    last_debounce_time[button_index] = now;
    return true;
  }
  return false;
}

void printKeypadLayout() {
  for(int x = 0; x < NUM_ROWS; x++) {
    for(int y = 0; y < NUM_COLS; y++) {
      DEBUG_PRINT(KEYS[x][y]);
      DEBUG_PRINT("\t");
    }
    DEBUG_PRINT("\n");
  }
  DEBUG_PRINT("\n");
}

void showOLEDMessage(uint8_t index) {
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
    SMALL_OLED.clearDisplay();
    SMALL_OLED.setTextColor(WHITE);
    SMALL_OLED.setCursor(0,0);
    if (index != 6) {
      SMALL_OLED.setTextSize(3);  // Default large text
      SMALL_OLED.print(SMALLoledMessages[index]);
    }
    else {
      SMALL_OLED.setTextSize(2);
      SMALL_OLED.print(SMALLoledMessages[index]);
    }
    SMALL_OLED.display();
    xSemaphoreGive(i2cMutex);
  }
}

//functions for FreeRTOS - functions that run in parrallel, basically run in the background without interrupting and waiting for each other

void TaskButtonWatcher(void *pvParameters) {
  for(;;) {
    if (digitalRead(wakeup_sleep_pin) == LOW && debounce(1)) {
      prep_for_deep_sleep();
    }
    vTaskDelay(40 / portTICK_PERIOD_MS);  // Poll every 40 ms
  }
}
//checking if the small oled display should be updated
void TaskSmallOledUpdater(void *pvParameters) {
  for (;;) {
    bool should_update = false;
    int msg_val;
    uint16_t delay_k = 400;
    portENTER_CRITICAL(&message_mux);
    should_update = SMALL_display_nodisplay;
    SMALL_display_nodisplay = false;
    msg_val = message_data;
    message_data = 0;
    portEXIT_CRITICAL(&message_mux);
    
    if (should_update) {
      showOLEDMessage(msg_val);
    }
  }
}
//checking if the bid display should be updated
void TaskBIGOledUpdater(void *pvParameters) {
  for (;;) {
    bool should_update2 = false;
    char key_value;
    uint16_t delay_k = 400;
    portENTER_CRITICAL(&key_mux);
    key_value = key;
    should_update2 = BIG_display_nodisplay;
    BIG_display_nodisplay = false;
    DEBUG_PRINTLN(key_value);
    if (key != 'H' && key != '#') {
      last_key = key;
      dataToSend.keypad_char = key;
    }
    if (key == '#') {
      key = 'H';
      BIG_display_nodisplay = true;
    }
    else if (key == 'H' || key =='C') {
      key = last_key;
      BIG_display_nodisplay = true;;
    }
    portEXIT_CRITICAL(&key_mux);
    if (should_update2) {

      if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        switch (key_value) {
          case '0':
            break;
          case 'H':
            BIG_OLED.clearDisplay();
            BIG_OLED.setTextSize(2);
            BIG_OLED.setTextColor(WHITE);
            BIG_OLED.setCursor(22,22);
            BIG_OLED.print("WYSLANO");
            BIG_OLED.display();
            delay_k = 1000;
            break;
          case '1':
            BIG_OLED.clearDisplay();
            BIG_OLED.setTextSize(3);
            BIG_OLED.setTextColor(WHITE);
            BIG_OLED.setCursor(12,0);
            BIG_OLED.print("MOZNA");
            BIG_OLED.setCursor(12,31);
            BIG_OLED.print("WEJSC");
            break;
          case '2':
            BIG_OLED.clearDisplay();
            BIG_OLED.setTextSize(2);
            BIG_OLED.setTextColor(WHITE);
            BIG_OLED.setCursor(43,10);
            BIG_OLED.print("MAM");
            BIG_OLED.setCursor(15,40);
            BIG_OLED.print("EGZAMIN");
            break;
          case '3':
            BIG_OLED.clearDisplay();
            BIG_OLED.setTextSize(2);
            BIG_OLED.setTextColor(WHITE);
            BIG_OLED.setCursor(36,10);
            BIG_OLED.print("UCZE");
            BIG_OLED.setCursor(43,40);
            BIG_OLED.print("SIE");
            break;
          case '4':
            BIG_OLED.clearDisplay();
            BIG_OLED.setTextSize(2);
            BIG_OLED.setTextColor(WHITE);
            BIG_OLED.setCursor(43,10);
            BIG_OLED.print("NIE");
            BIG_OLED.setCursor(8,40);
            BIG_OLED.print("WCHODZIC");
            break;
          case '5':
            BIG_OLED.clearDisplay();
            BIG_OLED.setTextSize(2);
            BIG_OLED.setTextColor(WHITE);
            BIG_OLED.setCursor(22,10);
            BIG_OLED.print("GRAM Z");
            BIG_OLED.setCursor(36,40);
            BIG_OLED.print("KIMS");
            break;
          case '6':
            BIG_OLED.clearDisplay();
            BIG_OLED.setTextSize(2);
            BIG_OLED.setTextColor(WHITE);
            BIG_OLED.setCursor(1,10);
            BIG_OLED.print("ROZMAWIAM");
            BIG_OLED.setCursor(22,40);
            BIG_OLED.print("Z KIMS");
            break;
          case '7':
            BIG_OLED.clearDisplay();
            BIG_OLED.setTextSize(2);
            BIG_OLED.setTextColor(WHITE);
            BIG_OLED.setCursor(8,10);
            BIG_OLED.print("NAJPIERW");
            BIG_OLED.setCursor(15,40);
            BIG_OLED.print("ZAPUKAJ");
            break;
          case '8':
            BIG_OLED.clearDisplay();
            BIG_OLED.setTextSize(2);
            BIG_OLED.setTextColor(WHITE);
            BIG_OLED.setCursor(15,10);
            BIG_OLED.print("CZY COS");
            BIG_OLED.setCursor(15,40);
            BIG_OLED.print("WAZNEGO?");
            break;
          case '9':
            BIG_OLED.clearDisplay();
            BIG_OLED.setTextSize(2);
            BIG_OLED.setTextColor(WHITE);
            BIG_OLED.setCursor(1,22);
            BIG_OLED.print("ZAMKNIETE");
            break;
          case '#': {
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataToSend , sizeof(dataToSend));
            break;
          }
        }
        BIG_OLED.display();
      }
    }
    xSemaphoreGive(i2cMutex);
    vTaskDelay(delay_k / portTICK_PERIOD_MS);
  }
}

void TaskreadKey(void *pvParameters) {
  for(;;) {
    for(int x = 0; x < NUM_ROWS; x++) {
      for(int row = 0; row < NUM_ROWS; row++) digitalWrite(ROWS[row], x != row);
      for(int y = 0; y < NUM_COLS; y++) {
        if(!digitalRead(COLS[y])){
          while(!digitalRead(COLS[y])){}
          vTaskDelay(150 / portTICK_PERIOD_MS);
          portENTER_CRITICAL(&key_mux);
          key = KEYS[x][y];
          BIG_display_nodisplay = true;
          portEXIT_CRITICAL(&key_mux);
        }
      }
    }
    vTaskDelay(70 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // put your setup code here, to run once:
  //Initialize serial communication
  #if DEBUG
    Serial.begin(9600);   // Start the Serial Monitor
    delay(100);             // Give it time to connect (especially for USB serial)
    DEBUG_PRINTLN("Debugging started...");
  #endif
  pinMode(buzzer_pin, OUTPUT);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Retries += 1;
    DEBUG_PRINT(".");
    if (Retries == maxRetries)
    {
      WiFi.disconnect(true);
      delay(700); 
      WiFi.begin(ssid, password);
      DEBUG_PRINTLN("Retrying");
      Retries = 0;
    }
  }
  DEBUG_PRINTLN("WiFi connected");
  DEBUG_PRINT(WiFi.localIP());
  DEBUG_PRINTLN("' to connect");

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    DEBUG_PRINTLN("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    DEBUG_PRINTLN("Failed to add peer");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

    // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  pinMode(wakeup_sleep_pin, INPUT_PULLUP);
  synchronizeTime();
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();//checking what caused the wake up and taking appropriate actions after finding out
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    DEBUG_PRINTLN("Woke up from timer");
    delay(20000);
    portENTER_CRITICAL(&message_mux);
    last_wakeup_time = millis();
    portEXIT_CRITICAL(&message_mux);
    prep_for_deep_sleep();
  } 
  else if (wakeup_reason ==  ESP_SLEEP_WAKEUP_GPIO) {
    DEBUG_PRINTLN("Woke up from external GPIO");
    portENTER_CRITICAL(&message_mux);
    last_wakeup_time = millis();
    portEXIT_CRITICAL(&message_mux);
    portENTER_CRITICAL(&sleep_mux);
    should_sleep = false;
    portEXIT_CRITICAL(&sleep_mux);
  } 
  else {
    DEBUG_PRINTLN("Fresh boot or other wake-up reason");
  }
  //set button pins modes
  pinMode(Displays_power_pin, OUTPUT);
  digitalWrite(Displays_power_pin, HIGH);
  delay(40);

  I2C_default.begin(SDA_PIN,SCL_PIN, 100000);
  delay(10);

  i2cMutex = xSemaphoreCreateMutex();

  if (i2cMutex == NULL) {
    DEBUG_PRINTLN("Failed to create I2C mutex!");
  }
  delay(10);

  //setup keypad buttons modes
  for(int x = 0; x < NUM_ROWS; x++) {
    pinMode(ROWS[x], OUTPUT);
    digitalWrite(ROWS[x], LOW);
  }

  for(int x = 0; x < NUM_COLS; x++) {
    pinMode(COLS[x], INPUT_PULLUP);
  }
  
  DEBUG_PRINTLN("Keypad initialized");
  printKeypadLayout();

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
    //initial setup for SMALL_OLED
    if(!SMALL_OLED.begin(SSD1306_SWITCHCAPVCC, SMALL_OLED_SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
    }
    xSemaphoreGive(i2cMutex);
  }
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
    //initial setup for BIG_OLED
    if(!BIG_OLED.begin(SSD1306_SWITCHCAPVCC, BIG_OLED_SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
    }
    xSemaphoreGive(i2cMutex);
  }

// initialize the tasks that use FreeRTOS
  xTaskCreatePinnedToCore(
  TaskreadKey,     // Task function
  "KEYPAD Watcher",      // Name of task
  1024,                // Stack size in words (2048 = 8 KB)
  NULL,                // Task input parameter
  1,                   // Priority (1 is default)
  &KEYPADWatcherHandle,                // Task handle (not needed here)
  0                    // Core to run on (0 or 1)
);

  xTaskCreatePinnedToCore(
  TaskButtonWatcher,     // Task function
  "Button Watcher",      // Name of task
  768,                // Stack size in words (2048 = 8 KB)
  NULL,                // Task input parameter
  1,                   // Priority (1 is default)
  &buttonWatcherHandle,                // Task handle (not needed here)
  0                    // Core to run on (0 or 1)
);

xTaskCreatePinnedToCore(
  TaskBIGOledUpdater,
  "BIG OLED Updater",
  2048,
  NULL,
  1,
  &BIGOLEDWatcherHandle,
  0
);

xTaskCreatePinnedToCore(
  TaskSmallOledUpdater,
  "SMALL OLED Updater",
  2048,
  NULL,
  1,
  &SMALLOLEDWatcherHandle,
  0
);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long now = millis();
  unsigned long last_wakeup_time_copy;
  portENTER_CRITICAL(&message_mux);
  last_wakeup_time_copy = last_wakeup_time;
  portEXIT_CRITICAL(&message_mux);
  if (now - last_wakeup_time_copy > keep_awake_for && should_sleep) {
    DEBUG_PRINTLN("Idzie spac bo dlugo nic");
    prep_for_deep_sleep();
  }
  portENTER_CRITICAL(&repeat_message_mux)
  bool message_send = message_to_be_sent;
  portEXIT_CRITICAL(&repeat_message_mux)
  if (message_send) {
    struct tm timeheld;
    getLocalTime(&timeheld);
    int hour = timeheld.tm_hour;
    int min = timeheld.tm_min;
    int sec = timeheld.tm_sec;
    if (hour >= 7 && hour <= 23 && min%15 == 0 && sec >= 10 ) {
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataToSend , sizeof(dataToSend));
    }
  }
  #if DEBUG
    static unsigned long last_debug = 0;
    if (now - last_debug > 1000) {
      DEBUG_PRINT("ButtonWatcher stack: ");
      DEBUG_PRINTLN(uxTaskGetStackHighWaterMark(buttonWatcherHandle));

      DEBUG_PRINT("KEYPADWatcher stack: ");
      DEBUG_PRINTLN(uxTaskGetStackHighWaterMark(KEYPADWatcherHandle));

      DEBUG_PRINT("BIGOLEDWatcher stack: ");
      DEBUG_PRINTLN(uxTaskGetStackHighWaterMark(BIGOLEDWatcherHandle));

      DEBUG_PRINT("SMALLOLEDWatcher stack: ");
      DEBUG_PRINTLN(uxTaskGetStackHighWaterMark(SMALLOLEDWatcherHandle));

      last_debug = now;
    }
  #endif
  delay(500);
}
