#include <esp_now.h> // Library for espnow
#include "esp_camera.h"
#include <WiFi.h> //Library for WiFi
#include <LCD_I2C.h>//library for lcd that works with custom i2c pins
#include <Wire.h> //library for I2C
#include <SPI.h> // Library for SPI
#include <MFRC522.h> //rfid reader library
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> //Libraries for OLED displays
#include "driver/rtc_io.h" //library for rtc gpio functions
#include <time.h> // library for time functions, here used for getting and storing time from the internet
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM

#include "camera_pins.h"

#define PCF_ADDRESS 0x26 // I2C address of the PCF expander

//debbugging area
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

const uint8_t OLED_SCREEN_WIDTH = 128; // OLED display width, in pixels
const uint8_t OLED_SCREEN_HEIGHT = 32; // OLED display height, in pixels
// Declaration for SSD1306 display connected using I2C
const int OLED_RESET = -1; // Reset pin # (or -1 if sharing Arduino reset pin or not having a reset pin)
const uint8_t OLED_SCREEN_ADDRESS = 0x3C; //OLED screen I2C address

gpio_num_t button_choice_pin = GPIO_NUM_1; //button for chosing a message to be sent
gpio_num_t button_send_pin = GPIO_NUM_2; //button for sending the chosen message
gpio_num_t motion_wakeup_pin = GPIO_NUM_4; //gpio for motion sensor wakeup;
gpio_num_t Displays_power_pin = GPIO_NUM_3; //pin that powers a mosfet for powering the displays
gpio_num_t SDA_PIN = GPIO_NUM_5; //new i2c sda pin, being the same as the default
gpio_num_t SCL_PIN = GPIO_NUM_6; //new i2c scl pin, being the same as the default
gpio_num_t SS_PIN = GPIO_NUM_44;
gpio_num_t RST_PIN = GPIO_NUM_43;
gpio_num_t SCK_PIN = GPIO_NUM_7;
gpio_num_t MOSI_PIN = GPIO_NUM_9;
gpio_num_t MISO_PIN = GPIO_NUM_8; //ALL 4 SPI pins  and a reset pin for the rfid reader

MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
String Card = "4369BC1C";
String Brelok = "C3B8DF0D";
String tagID = "";

TwoWire I2C_default = TwoWire(0);
//create and object OLED
Adafruit_SSD1306 OLED(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &I2C_default, OLED_RESET);

const uint8_t LCD_SCREEN_WIDTH = 16; // LCD display width, in pixels
const uint8_t LCD_SCREEN_HEIGHT = 2; // LCD display height, in pixels
const uint8_t LCD_SCREEN_ADDRESS = 0x27; // LCD screen I2C address
//create an object lcd
LCD_I2C lcd(I2C_default, LCD_SCREEN_ADDRESS, LCD_SCREEN_WIDTH, LCD_SCREEN_HEIGHT); // I2C address 0x27, 16 column and 2 rows

//CUSTOM CHARS FOR LCD
byte Lock[8] = {
0b01110,
0b10001,
0b10001,
0b11111,
0b11011,
0b11011,
0b11111,
0b00000
};

byte Skull[8] = {
0b00000,
0b01110,
0b10101,
0b11011,
0b01110,
0b01110,
0b00000,
0b00000
};

byte Bell[8] = {
0b00100,
0b01110,
0b01110,
0b01110,
0b11111,
0b00000,
0b00100,
0b00000
};

byte Check[8] = {
0b00000,
0b00001,
0b00011,
0b10110,
0b11100,
0b01000,
0b00000,
0b00000
};

byte Speaker[8] = {
0b00001,
0b00011,
0b01111,
0b01111,
0b01111,
0b00011,
0b00001,
0b00000
};

const char* oledMessages[] = {
  "",                 // 0 - unused
  "WYSLANO",          // 1
  "WYBIERZ WIADOMOSC",// 2
  "WAZNE",            // 3
  "MAMA",             // 4
  "TATA",             // 5
  "DANIELA",          // 6
  "MACIEK",           // 7
  "",                 // 8 - unused
  "MASTER",           // 9
  "DAMIAN"            // 10
};

String success;
int message_value = 2; //value for keeping which message is to be displayed or sent
bool display_nodisplay = true; //boolean for checking if new message should be displayed

//Structs for sending and receiveing data, likely to change
typedef struct struct_message {
  char keypad_char;
} struct_message;
struct_message incomingData;

typedef struct struct_message2 {
  int message_char;
} struct_message2;
struct_message2 dataToSend;

char keypad_data = '0';//char for receiveing keypad chars
RTC_DATA_ATTR char last_keypad_data = '0';

const uint8_t debounce_delay = 130; //debounce delay time

//mac address of the arduino_nano_esp32-s3
uint8_t broadcastAddress[] = {0x50,0x78,0x7D,0x41,0x56,0x48}; //this is for arduino nano esp32 mac address, the one on the left is one of the c3 super mini {0x34,0x85,0x18,0x7B,0xD8,0x78};
esp_now_peer_info_t peerInfo;
//sleep variables
const unsigned long uS_TO_S_FACTOR = 1000000;  // conversion factor for seconds to microseconds
unsigned long last_wakeup_time;
unsigned long long keep_awake_for = 10000;

const uint8_t maxRetries = 6; //retries for connecting wifi
int Retries = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // a mux to block the same variables from being accessed at the same time, used beacuse of RTOS

//task handles for RTOS tasks, needed to check their memory usage
TaskHandle_t buttonWatcherHandle = NULL;
TaskHandle_t RFIDWatcherHandle = NULL;
TaskHandle_t OLEDWatcherHandle = NULL;
TaskHandle_t LCDWatcherHandle = NULL;

SemaphoreHandle_t i2cMutex;

// Callback when data is sent through espnow
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  DEBUG_PRINT("\r\nLast Packet Send Status:\t");
  DEBUG_PRINTLN(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received through espnow
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingBytes, int len) {
  memcpy(&incomingData, incomingBytes, sizeof(incomingData));
  // DEBUG_PRINT("Bytes received: ");
  // DEBUG_PRINTLN(len);
  // DEBUG_PRINT("Value received: ");
  portENTER_CRITICAL(&mux);
  keypad_data = incomingData.keypad_char;
  //DEBUG_PRINTLN(keypad_data);
  last_keypad_data = keypad_data;
  last_wakeup_time = millis();
  if (keypad_data == 'C')  {
    keep_awake_for = 60000 * 10;
  }
  else keep_awake_for = 10000;
  portEXIT_CRITICAL(&mux);
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

// Sets all the bits to 1 of the PCF expander
void resetPlayer() {
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
    I2C_default.beginTransmission(PCF_ADDRESS);
    I2C_default.write(0xFF); // 1111 1111 (All High)
    I2C_default.endTransmission();
    xSemaphoreGive(i2cMutex);
  }
}

// Plays the song of choice
void playSong(byte songNumber) {
  if (songNumber == 0) return; // Song 0 doesn't exist

  // STEP 1: Calculate the command
  // The PCF8574 needs '0' to trigger a pin, and '1' to leave it alone.
  // So we invert the song number bits using the ~ operator.
  byte command = ~songNumber; 

  DEBUG_PRINT("Playing Song: ");
  DEBUG_PRINTLN(songNumber);
  DEBUG_PRINT(" | Sending Binary: ");
  DEBUG_PRINTLN(command);

  // STEP 2: Send the Trigger (Pull pins LOW)
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
    I2C_default.beginTransmission(PCF_ADDRESS);
    I2C_default.write(command);
    I2C_default.endTransmission();
    xSemaphoreGive(i2cMutex);
  }

  // STEP 3: Wait for DY-HV20T to register the signal
  // 20ms is usually safe for Mode 0
  delay(20); 

  // STEP 4: Reset pins to HIGH (Idle)
  // If we don't do this, the buttons stay "pressed" and song won't finish/restart correctly
  resetPlayer();
}

//Read new RFID tag if available
bool getID() {
  // Getting ready for Reading PICCs
  if (!mfrc522.PICC_IsNewCardPresent()) return false;
  if (!mfrc522.PICC_ReadCardSerial()) return false;

  tagID = ""; //clear tagID
  char buffer[3];//buffer for storing bytes, need 3 beacuse the 0xAB is needed for string, so it is stored as a char, do in ASCII so, A needs a full byte and B also a full byte and then null terminator to mark it as a string for sprintf
  for (uint8_t i = 0; i < 4; i++) {
    sprintf(buffer, "%02X", mfrc522.uid.uidByte[i]);  // always 2 digits
    tagID.concat(buffer);
  }
  //DEBUG_PRINTLN(tagID);

  mfrc522.PICC_HaltA();
  return true;
}

//Getting time from the internet and synchronizing the internal clock
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

void showOLEDMessage(uint8_t index) {
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
    OLED.clearDisplay();
    OLED.setTextColor(WHITE);
    OLED.setCursor(0,0);
    if (index == 2) {
      OLED.setTextSize(1);  // For long prompt
      OLED.print("WYBIERZ WIADOMOSC");
      OLED.setCursor(0, 11);
      OLED.print("JAKA CHCESZ WYSLAC");
      OLED.setCursor(0, 22);
      OLED.print("UZYJ PRZYCISKOW");
    } 
    else if (index == 1 || index == 6) {
      OLED.setTextSize(2);
      OLED.print(oledMessages[index]);
    }
    else {
      OLED.setTextSize(3);  // Default large text
      OLED.print(oledMessages[index]);
    }
    OLED.display();
    xSemaphoreGive(i2cMutex);
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
    rtc_gpio_init(motion_wakeup_pin); 
    rtc_gpio_pulldown_en(motion_wakeup_pin); // give a pulldown for the gpio so that it triggers on HIGH
    rtc_gpio_pullup_dis(motion_wakeup_pin); //disable any pullups for the gpio just in case
    rtc_gpio_set_direction(motion_wakeup_pin, RTC_GPIO_MODE_INPUT_ONLY);
    esp_sleep_enable_ext1_wakeup(1ULL << motion_wakeup_pin, ESP_EXT1_WAKEUP_ANY_HIGH); // enable external wakeup 
    delay(100);
    esp_deep_sleep_start();// deep sleep start, no code after it will be performed if triggered
}

bool is_motion_detected() {
  if (digitalRead(motion_wakeup_pin) == HIGH) {
    return true;
  }
  return false;
}

//functions for FreeRTOS - functions that run in parrallel, basically run in the background without interrupting and waiting for each other
//checking if there is an RFID card to be read
void TaskRFIDWatcher(void *pvParameters) {
  for(;;) { //loop forever
    if (getID()) {
      // Lock the critical section
      portENTER_CRITICAL(&mux);

      // --- Begin critical section (no one else can access these now)
      if (tagID == Card) {
        message_value = 9;
        playSong(random(1,4));
      } 
      else if (tagID == Brelok) {
        message_value = 10;
        playSong(random(4,8));
      }

      keypad_data = 'G';
      last_wakeup_time = millis();
      display_nodisplay = true;
      // --- End critical section

      portEXIT_CRITICAL(&mux);  // Unlock it

      DEBUG_PRINT("RFID Scanned: ");
      DEBUG_PRINTLN(tagID);
    }
    vTaskDelay(300 / portTICK_PERIOD_MS); // Sleep 200ms before next check
  }
}

void TaskButtonWatcher(void *pvParameters) {
  for(;;) {
    if (digitalRead(button_choice_pin) == LOW && debounce(0)) {
      portENTER_CRITICAL(&mux);
      message_value++;
      display_nodisplay = true;
      last_wakeup_time = millis();
      portEXIT_CRITICAL(&mux);
    }

    if (digitalRead(button_send_pin) == LOW && debounce(1)) {
      portENTER_CRITICAL(&mux);
      if (message_value != 2) {
        dataToSend.message_char = message_value;
        message_value = 1;
        display_nodisplay = true;
        last_wakeup_time = millis();
        portEXIT_CRITICAL(&mux);
        esp_now_send(broadcastAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));
      }
      else {
        portEXIT_CRITICAL(&mux);
      }
    }
    vTaskDelay(40 / portTICK_PERIOD_MS);  // Poll every 40 ms
  }
}

void TaskOLEDUpdater(void *pvParameters) {
  for (;;) {
    bool should_update = false;
    int msg_val;
    uint16_t delay_k = 400;

    portENTER_CRITICAL(&mux);
    should_update = display_nodisplay;
    if (message_value == 8) message_value = 3;
    msg_val = message_value;
    portEXIT_CRITICAL(&mux);

    if (should_update) {

      if (msg_val < sizeof(oledMessages) / sizeof(oledMessages[0])) {
        showOLEDMessage(msg_val);

        if (msg_val == 9 || msg_val == 10 || msg_val == 1) {
          portENTER_CRITICAL(&mux);
          message_value = 2;
          display_nodisplay = !display_nodisplay;
          portEXIT_CRITICAL(&mux);
          delay_k = 1000;
        }
        if (msg_val == 2) {
          delay_k = 1000;
        }
        portENTER_CRITICAL(&mux);
        display_nodisplay = !display_nodisplay;
        portEXIT_CRITICAL(&mux);
      }
    }
    vTaskDelay(delay_k / portTICK_PERIOD_MS);
  }
}

void TaskLCDUpdater(void *pvParameters) {
  for (;;) {
    char key;
    uint16_t delay_k = 400;
    portENTER_CRITICAL(&mux);
    key = keypad_data;
    if (keypad_data == 'G') {
      keypad_data = last_keypad_data;
    }
    else keypad_data = '0';
    portEXIT_CRITICAL(&mux);
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      if (key != '0') {
        lcd.clear();
        switch (key) {
          case '1':
            lcd.setCursor(3, 0); lcd.write(3); lcd.setCursor(5, 0);
            lcd.print("MOZNA"); lcd.setCursor(12, 0); lcd.write(3);
            lcd.setCursor(4, 1); lcd.print("WEJSC");
            break;
          case '2':
            lcd.setCursor(6,0); lcd.print("MAM");
            lcd.setCursor(4,1); lcd.print("EGZAMIN");
            break;
          case '3':
            lcd.setCursor(6,0); lcd.print("UCZE"); lcd.setCursor(6,1);
            lcd.print("SIE");
            break;
          case '4':
            lcd.setCursor(4,0); lcd.write(1); lcd.setCursor(6,0);
            lcd.print("NIE"); lcd.setCursor(11,0); lcd.write(1);
            lcd.setCursor(4,1); lcd.print("WCHODZIC");
            break;
          case '5':
            lcd.setCursor(5,0); lcd.print("GRAM Z"); lcd.setCursor(6,1);
            lcd.print("KIMS");
            break;
          case '6':
            lcd.setCursor(1,0); lcd.write(4); lcd.setCursor(3,0);
            lcd.print("ROZMAWIAM"); lcd.setCursor(14,0); lcd.write(4);
            lcd.setCursor(5,1); lcd.print("Z KIMS");
            break;
          case '7':
            lcd.setCursor(2,0); lcd.write(2); lcd.setCursor(4,0);
            lcd.print("NAJPIERW"); lcd.setCursor(14,0); lcd.write(2);
            lcd.setCursor(4,1); lcd.print("ZAPUKAJ");
            break;
          case '8':
            lcd.setCursor(3,0); lcd.print("CZY TO COS"); lcd.setCursor(4,1);
            lcd.print("WAZNEGO?");
            break;
          case '9':
            lcd.setCursor(4,0); lcd.print("ZAMKNIETE"); lcd.setCursor(1,1);
            lcd.write(0); lcd.setCursor(3,1); lcd.write(0);
            lcd.setCursor(5,1); lcd.write(0); lcd.setCursor(7,1);
            lcd.write(0); lcd.setCursor(9,1); lcd.write(0);
            lcd.setCursor(11,1); lcd.write(0); lcd.setCursor(13,1);
            lcd.write(0); lcd.setCursor(15,1); lcd.write(0);
            break;
          // Add rest of the cases...
          case 'G':
            lcd.setCursor(5,0); lcd.print("WELCOME");
            delay_k = 1000;
            break;
          default:
          DEBUG_PRINT(".");
        }
      }
      xSemaphoreGive(i2cMutex);
      vTaskDelay(delay_k / portTICK_PERIOD_MS);
    }
  }
}

// ===========================
// Enter your WiFi credentials
// ===========================
const char* ssid = "Orange_Swiatlowod_7310";
const char* password = "dcCv5oqVPdfbcGQ9U9";

void startCameraServer();
void setupLedFlash(int pin);

void setup() {
  #if DEBUG
    Serial.begin(115200);   // Start the Serial Monitor
    delay(100);             // Give it time to connect (especially for USB serial)
    DEBUG_PRINTLN("Debugging started...");
  #endif
  // setup that initializes only a part of the project, if it doesnt detect movement it goes back to deep sleep, if it detects movement it goes on to initialize the rest of the project
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
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("WiFi connected");
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    DEBUG_PRINTLN("Error initializing ESP-NOW");
    return;
  }
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    DEBUG_PRINTLN("Failed to add peer");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  pinMode(motion_wakeup_pin, INPUT_PULLDOWN);
  synchronizeTime();
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();//checking what caused the wake up and taking appropriate actions after finding out
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    DEBUG_PRINTLN("Woke up from timer");
    delay(30000);
    portENTER_CRITICAL(&mux);
    last_wakeup_time = millis();
    portEXIT_CRITICAL(&mux);
    if (!is_motion_detected()) {
      DEBUG_PRINTLN("Poszło spać po wybudzeniu z timera, nie wykryto ruchu");
      prep_for_deep_sleep();
    }
  } 
  else if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
    DEBUG_PRINTLN("Woke up from external GPIO");
    portENTER_CRITICAL(&mux);
    last_wakeup_time = millis();
    portEXIT_CRITICAL(&mux);
  } 
  else {
    DEBUG_PRINTLN("Fresh boot or other wake-up reason");
  }
  //setup that initializes the rest of the project if movement was detected

  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  portENTER_CRITICAL(&mux);
  keypad_data = last_keypad_data;
  portEXIT_CRITICAL(&mux);

  Serial.setDebugOutput(true);
  DEBUG_PRINTLN();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 40000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    DEBUG_PRINTF("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif
//////////////////////////////////////////////////////////////////////////////////////////
//here starts the stuff after creating a setup for the web server
//////////////////////////////////////////////////////////////////////////////////////////
  pinMode(button_choice_pin, INPUT_PULLUP);
  pinMode(button_send_pin, INPUT_PULLUP);

  pinMode(Displays_power_pin, OUTPUT);
  digitalWrite(Displays_power_pin, HIGH);
  delay(40);
  I2C_default.begin(SDA_PIN,SCL_PIN, 200000);
  SPI.begin(SCK_PIN,MISO_PIN,MOSI_PIN,SS_PIN);			// Init SPI bus
  delay(10);
  mfrc522.PCD_Init();  // MFRC522

  i2cMutex = xSemaphoreCreateMutex();

  if (i2cMutex == NULL) {
    DEBUG_PRINTLN("Failed to create I2C mutex!");
  }
  delay(10);
  //initial setup for OLED
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    if(!OLED.begin(SSD1306_SWITCHCAPVCC, OLED_SCREEN_ADDRESS)) {
      DEBUG_PRINTLN(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
    }
    xSemaphoreGive(i2cMutex);
  }
  //initial setup for LCD
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    lcd.begin();
    lcd.clear();         
    lcd.backlight(); 
    lcd.createChar(0, Lock);//initialize the custom char for lcd
    lcd.createChar(1, Skull);//initialize the custom char for lcd
    lcd.createChar(2, Bell);//initialize the custom char for lcd
    lcd.createChar(3, Check);//initialize the custom char for lcd
    lcd.createChar(4, Speaker);//initialize the custom char for lcd
    xSemaphoreGive(i2cMutex);
  }
  // Set device as a Wi-Fi Station

  startCameraServer();//function that actually turns on the camera server

  DEBUG_PRINT("Camera Ready! Use 'http://");
  DEBUG_PRINT(WiFi.localIP());
  DEBUG_PRINTLN("' to connect");
  
  // initialize the tasks that use FreeRTOS
  xTaskCreatePinnedToCore(
  TaskRFIDWatcher,     // Task function
  "RFID Watcher",      // Name of task
  2048,                // Stack size in words (2048 = 8 KB)
  NULL,                // Task input parameter
  1,                   // Priority (1 is default)
  &RFIDWatcherHandle,                // Task handle (not needed here)
  0                    // Core to run on (0 or 1) //   SPRAWDZ CZY TO MOŻNA WRZUCIĆ NA RDZEŃ 1
);

  xTaskCreatePinnedToCore(
  TaskButtonWatcher,     // Task function
  "Button Watcher",      // Name of task
  2048,                // Stack size in words (2048 = 8 KB)
  NULL,                // Task input parameter
  1,                   // Priority (1 is default)
  &buttonWatcherHandle,                // Task handle (not needed here)
  1                    // Core to run on (0 or 1)
);

xTaskCreatePinnedToCore(
  TaskOLEDUpdater,
  "OLED Updater",
  2048,
  NULL,
  1,
  &OLEDWatcherHandle,
  0
);

xTaskCreatePinnedToCore(
  TaskLCDUpdater,
  "LCD Updater",
  2048,
  NULL,
  1,
  &LCDWatcherHandle,
  0
);
}

void loop() {
  // Do nothing. Everything is done in another task by the web server
  // put your main code here, to run repeatedly:
  //checking if the motion sensor is still picking someone up
  if (is_motion_detected()) {
    portENTER_CRITICAL(&mux);
    last_wakeup_time = millis();
    portEXIT_CRITICAL(&mux);
    DEBUG_PRINTLN("Wykryto ruch");
  }
  DEBUG_PRINTLN(millis() - last_wakeup_time);
  // if noone detected for a long period put to deep sleep
  unsigned long now = millis();
  unsigned long last_wakeup_time_copy;
  DEBUG_PRINTLN(keep_awake_for);
  portENTER_CRITICAL(&mux);
  last_wakeup_time_copy = last_wakeup_time;
  portEXIT_CRITICAL(&mux);
  if (now - last_wakeup_time_copy > keep_awake_for) {
    DEBUG_PRINTLN("Idzie spac bo dlugo nic");
    prep_for_deep_sleep();
  }
  #if DEBUG
    static unsigned long last_debug = 0;
    if (now - last_debug > 1000) {
      DEBUG_PRINT("ButtonWatcher stack: ");
      DEBUG_PRINTLN(uxTaskGetStackHighWaterMark(buttonWatcherHandle));

      DEBUG_PRINT("RFIDWatcher stack: ");
      DEBUG_PRINTLN(uxTaskGetStackHighWaterMark(RFIDWatcherHandle));

      DEBUG_PRINT("LCDWatcher stack: ");
      DEBUG_PRINTLN(uxTaskGetStackHighWaterMark(LCDWatcherHandle));

      DEBUG_PRINT("OLEDWatcher stack: ");
      DEBUG_PRINTLN(uxTaskGetStackHighWaterMark(OLEDWatcherHandle));

      last_debug = now;
    }
  #endif
  delay(500);
}
