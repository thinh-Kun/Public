// import library
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include <TimeLib.h>
#include "DHT.h"

// define Pin
const int buttonDown = 34;
const int buttonBack = 35;
const int buttonEnterUp = 13;  // Here is button increase value don't nagvi
#define Relay1 15
#define Relay2 16
#define DHTPIN 27
#define BUZZER_PIN 17
#define DHTTYPE DHT11

// config name wifi and mqtt brooker
const char* ssid = "Hoai Nam 2.4";
const char* password = "10101999";
// Add your MQTT Broker IP address, example:
const char* mqtt_server = "thinhtv482002.hopto.org";
const int port_id = 18883;
const char* ssid_server = "home_broker";
const char* pass_server = "thinhtv48";

// Initialize functions
StaticJsonDocument<256> JSONbuffer;
JsonObject JSONencoder = JSONbuffer.to<JsonObject>();
WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHTPIN, DHTTYPE);

// varible time data
unsigned long previousMillis_buzzer_one_time = 0; // buzzer time
unsigned long previousMillis_alarm = 0;           // time each Alarm
unsigned long lastMsg = 0;
unsigned long time_send_data = 0;                 // time each sends data to server


// State enable modes
bool enable_time_server = 0;
bool enable_buzzer = 0;
bool enable_alarm = 0;
bool enable_alarm_delay = 0;
int layer = 0;  // lớp hiện tại của menu
int menu_main = 0;  // menu man hinh chinh
bool menu_display = 0;
bool menu_setting = 0;
bool menu_setting_pos = 0;
bool enable_config_temp = 0;
bool config_day = 0;
bool stateRelay1 = 1;   // 1 is off
bool stateRelay2 = 1;

// State of the push button

int buttonStateDown = 0;
int buttonStateBack = 0;
int buttonStateEnter_Up = 0;

// create varible meter
int temp_config = 35;
int day_hatch = 20;
int day_hatch_remain = 0;
float humidity = 0;
float temperature = 0;
char temperature_str[10];
void display_sign();
void menu();
void display_menu();
void update_display_menu();
void setting_menu();
void display_sign(int posX, int posY) // hàm cập nhật dấu con trỏ
{
  lcd.setCursor(0, 0);
  lcd.print("  ");
  lcd.setCursor(0, 1);
  lcd.print("  ");
  lcd.setCursor(posX, posY);
  lcd.print("> ");
}
void menu() {                         // menu main
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("> ");
  lcd.setCursor(1, 0);
  lcd.print(" Display");
  lcd.setCursor(1, 1);
  lcd.print(" Setting");
}
void display_menu() {                 // menu of display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  lcd.setCursor(10, 0);
  lcd.print("Hum:");
  lcd.setCursor(0, 1);
  lcd.print("Remaning Days:");

}
void update_display_menu()            //Update just necessary variable
{
  lcd.setCursor(5, 0);                // update only parement
  lcd.print("    ");
  lcd.setCursor(14, 0);
  lcd.print("  ");
  lcd.setCursor(14, 1);
  lcd.print("  ");

  lcd.setCursor(5, 0);
  lcd.print(temperature);
  lcd.setCursor(14, 0);
  lcd.print(humidity);
  lcd.setCursor(14, 1);
  lcd.print(day_hatch_remain);
}
void setting_menu() {               //menu of setting
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("> ");
  lcd.setCursor(2, 0);
  lcd.print("Temperature");
  lcd.setCursor(2, 1);
  lcd.print("Incubation day");
  enable_config_temp = 1;
  while (digitalRead(buttonBack) == 0) {
    Serial.println("Pending!");
    Serial.print("enable_config_temp:"); Serial.println(enable_config_temp);
    Serial.print("config_day:"); Serial.println(config_day);
    delay(1000);
    if (digitalRead(buttonDown) == 1 && menu_setting_pos == 0)
    {
      menu_setting_pos = 1;
      display_sign(0, 1);
      Serial.println("Pending 1 !");
      enable_config_temp = 0;
      config_day = 1;
    }
    else if (digitalRead(buttonDown) == 1 && menu_setting_pos == 1)
    {
      menu_setting_pos = 0;
      display_sign(0, 0);
      Serial.println("Pending 2 !");
      enable_config_temp = 1;
      config_day = 0;
    }
    if (digitalRead(buttonEnterUp) == 1 && enable_config_temp == 1 && config_day == 0)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Setting Temp:");
      while (digitalRead(buttonBack) == 0)
      {
        lcd.setCursor(13, 0);
        lcd.print("   ");
        if (digitalRead(buttonEnterUp) == 1) temp_config ++;
        else if (digitalRead(buttonDown) == 1) temp_config --;
        lcd.setCursor(13, 0);
        lcd.print(temp_config);
        delay(1000);
      }
      lcd.clear();
      enable_config_temp = 0;
      config_day = 0;
    }
    if (digitalRead(buttonEnterUp) == 1 && config_day == 1 && enable_config_temp == 0)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("day_hatch:");
      while (digitalRead(buttonBack) == 0)
      {
        lcd.setCursor(13, 0);
        lcd.print("   ");
        if (digitalRead(buttonEnterUp) == 1) day_hatch ++;
        else if (digitalRead(buttonDown) == 1) day_hatch --;
        lcd.setCursor(13, 0);
        lcd.print(day_hatch);
        delay(1000);
      }
      char day_hatch_char[5];
      sprintf(day_hatch_char, "%d", day_hatch);
      client.publish("esp32/day_hatching", day_hatch_char);
      lcd.clear();
      enable_config_temp = 0;
      config_day = 0;
    }




  }

}
void setup() {
  // put your setup code here, to run once:
  delay(1000);
  Serial.begin(115200);
  delay(500);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(buttonDown , INPUT);
  pinMode(buttonBack , INPUT);
  pinMode(buttonEnterUp , INPUT);
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  digitalWrite(Relay1, 1); // turn off relay2
  digitalWrite(Relay2, 1); // turn off relay2
  dht.begin();
  lcd.init();
  lcd.backlight();
  lcd.print("Id:");
  lcd.setCursor(4, 0);
  lcd.print(ssid);
  lcd.setCursor(0, 1);
  lcd.print("Pass:");
  lcd.setCursor(6, 1);
  lcd.print(password);
  lcd.setCursor(0, 2);
  lcd.print("Connecting...");
  delay(1000);
  setup_wifi();
  client.setServer(mqtt_server, port_id);
  client.setCallback(callback);

  xTaskCreatePinnedToCore(      // RTOS Read and send data to server
    Read_And_Send
    ,  "Read_And_Send"   // A name just for humans
    ,  4000   // This stack size can be checked & adjusted by reading the Stack Highwater 1kb - 1024, 2kb 2048, 3kb- 3072
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL
    ,  1);
  lcd.clear();
  menu();
  delay(500);
}
long time_restart = 0;  // TIME TO AUTO RESTART WHEN CAN'T CONNECT AFTER A PERIOD OF TIME (10000MS)
void setup_wifi() {   // CONNECT TO WIFI
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  delay(100);
  int m = 13, n = 2;
  WiFi.begin(ssid, password);
  delay(1000);
  time_restart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    lcd.setCursor(m, n);
    lcd.print(".");
    m++;
    if (m  == 20) {
      m = 0;
      n++;
      if (n == 4) n = 0;
    }
    if (millis() - time_restart > 10000)

      ESP.restart();
    time_restart = millis();
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(2, 1);
  lcd.clear();
  lcd.print("Connected!");
  delay(3000);
  lcd.clear();

}

void callback(char* topic, byte * message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  if (messageTemp.length() > 10)  //COMMAND OF SERVER
  {
    enable_time_server = 1;     
    DynamicJsonDocument doc(1024);  //handle Json convert to value
    deserializeJson(doc, messageTemp);
    JsonObject obj = doc.as<JsonObject>();
    temp_config = int(obj["temperature_config"]);
    day_hatch_remain = int(obj["day_remain"]);
    delay(2000);
  }
  if (messageTemp.length() < 5 && enable_time_server == 1 )
  {
    Serial.print("Data Type Payload:"); Serial.println(messageTemp);
    delay(1000);
  }
}

// reconnect wifi when disconnect 
long time_wifi = 0;   // time wait reconnect wifi
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client_Charge_Dchg_1", ssid_server, pass_server)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
      client.publish("esp32/status", "Connect Success");
      delay(2000);
      client.publish("esp32/status", "Connect Failed");
    } else {
      long now = millis();
      if (now - time_wifi > 15000)
      {
        setup_wifi();
        time_wifi = now;
        delay(5000);
      }

      Serial.print("failed, err_code:");
      Serial.println(client.state());
      delay(100);
      Serial.println(" try again in 5 seconds");
      client.disconnect();
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  lcd.clear();
  delay(100);
}
void buzzer()
{
  enable_alarm_delay = 1;
  enable_buzzer = 1;
  unsigned long currentMillis = millis();
  if (currentMillis  - previousMillis_buzzer_one_time >= 10000)
  {
    previousMillis_buzzer_one_time = currentMillis;
    enable_buzzer = 0;
    Serial.println("THinh test delay");
    client.publish("esp32/alarm", "Alarm_Hatching");
  }
  if (enable_buzzer == 1) digitalWrite(BUZZER_PIN, 1);
  else
  {
    digitalWrite(BUZZER_PIN, 0);
    enable_alarm_delay = 0;
    Serial.println("THinh test");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (float(temp_config + 0.3) >= temperature)
  {
    digitalWrite(Relay1, 0);  // duy trì đèn
    digitalWrite(Relay2, 1);  // tắt quạt
  }
  if (temperature > float(temp_config + 0.3) && temperature < float(temp_config + 1))
  {
    digitalWrite(Relay1, 1);  // tắt đèn
  }
  if (temperature > float(temp_config + 1))
  {
    digitalWrite(Relay1, 1);  // tắt đèn
    digitalWrite(Relay2, 0);  // bật quạt
  }
  if (day_hatch_remain <= 0)
  {
    enable_alarm = 1;
    enable_alarm_delay = 1;
    while (enable_alarm == 1 && digitalRead(buttonBack) == 0 && menu_display == 1)
    {
      unsigned long currentMillis = millis();
      if (currentMillis  - previousMillis_alarm >= 30000)
      {
        previousMillis_alarm = currentMillis;
        previousMillis_buzzer_one_time = millis();
        Serial.println("OUTSIDE ON");
        enable_alarm = 1;
        enable_alarm_delay = 1;
        Serial.print("enable_alarm_delay: "); Serial.println(enable_alarm_delay);
        delay(1000);
      }
      if (digitalRead(buttonBack) == 1)
      {
        enable_buzzer = 0;
        break;
      }
      Serial.print("enable_alarm_delay: "); Serial.println(enable_alarm_delay);
      delay(1000);
      if (enable_alarm_delay == 1)
      {
        enable_buzzer = 1;

        buzzer();

        Serial.print("OK: "); Serial.println("OKE");
      }
    }
    Serial.println("OUTSIDE IF");
    Serial.print("enable_buzzer: "); Serial.println(enable_buzzer);
    Serial.print("digitalRead(buttonBack): "); Serial.println(digitalRead(buttonBack));
    delay(1000);
    //    enable_buzzer = 0;
    //    buzzer();
  }
  else {
    enable_buzzer = 0;
    buzzer();
  }
  buttonStateDown = digitalRead(buttonDown);
  buttonStateBack = digitalRead(buttonBack);
  buttonStateEnter_Up = digitalRead(buttonEnterUp);
  if (layer == 0 && buttonStateDown == 1)
  {
    if (menu_main == 0) {
      menu_main = 1;
      Serial.println("OK 1!!!");
    }
    else if (menu_main == 1)
    {
      menu_main = 0;
      Serial.println("OK 0!!!");
    }
  }
  if (menu_main == 0) {
    menu();
    if (buttonStateEnter_Up == 1)
    {
      if (menu_display == 0)
      {
        menu_display = 1;
        menu_setting = 0;
        menu_main = 3;
        Serial.println("In Menu Display");
        display_menu();
      }
      else
      {
        menu_display = 0;
        menu_setting = 0;
        menu_main = 0;
        Serial.println("In Menu Main");
      }
    }
  }
  else if (menu_main == 1) {
    if (buttonStateEnter_Up == 1)
    {

      if (menu_setting == 0)
      {
        menu_setting = 1;
        menu_display = 0;
        menu_main = 3;
        Serial.println("In Menu Setting");
        setting_menu();
        menu_setting = 0;
        menu_display = 0;
        menu_main = 0;
      }
      else
      {
        menu_setting = 0;
        menu_display = 0;
        menu_main = 0;
        Serial.println("In Menu Main");
      }
    }
  }
  else if (menu_main == 3) {
    if (buttonStateBack == 1)
    {
      lcd.clear();
      menu();
      if (menu_display == 1)
      {
        menu_setting = 0;
        menu_display = 0;
        menu_main = 0;
        Serial.println("In Menu Main");
      }
      else if (menu_setting == 1)
      {
        menu_setting = 0;
        menu_display = 0;
        menu_main = 1;
        Serial.println("In Menu Main");
      }
    }
  }

  if (menu_main <= 1)
  {
    display_sign(0, int(menu_main));

  }
  else if (menu_main == 3) {
    if (menu_display == 1) update_display_menu();
  }
  delay(1000);
}

void Read_And_Send(void *pvParameters)
{
  (void) pvParameters;
  for (;;) // A Task shall never return or exit.
  {
    // READ sensor
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    humidity = dht.readHumidity();
    float temp = dht.readTemperature();
    temperature = round(temp * 100) / 100.0;
    sprintf(temperature_str, "%.2f", temperature);
    long now = millis();
    if (now - lastMsg > 3000) {
      lastMsg = now;
      client.subscribe("esp32/Ap_trung/control");
      delay(10);
      if (now - time_send_data > 1000)
      {
        StaticJsonDocument<256> doc1;
        doc1["humidity"] = humidity;
        doc1["temperature"] = temperature_str;
        String Jdata;
        serializeJson(doc1, Jdata);
        //gửi json đến topic esp32/json
        client.publish("esp32/data", Jdata.c_str());
        //Serial.println(state_str);
        time_send_data = now;
      }
    }
    vTaskDelay(500);
  }
}
