#include <WiFi.h>
#include <PubSubClient.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

// h-bridge output/pwm pins
#define EN_B GPIO_NUM_23
#define IN_4 GPIO_NUM_19
#define IN_3 GPIO_NUM_5
#define IN_2 GPIO_NUM_18
#define IN_1 GPIO_NUM_21
#define EN_A GPIO_NUM_22

// pwm frequency
#define LEDC_FREQ 2000

// WiFi parameters
const char *ssid = "VIVOFIBRA-C9A8"; // Enter your WiFi name
const char *password = "83D24D124F";  // Enter WiFi password

// MQTT Broker parameters
const char *mqtt_broker = "192.168.15.120";
const char *topic = "bri/command";
const char *debug_topic = "bri/debug";
const bool debug = false;
const char *mqtt_username = "subscriber";
const char *mqtt_password = "subscriber";
const int mqtt_port = 1883;

// Setting PWM properties
ledc_channel_config_t channel_EN_A = {
  .gpio_num   =   EN_A,             // Seleciona o pino para atuar o PWM
  .speed_mode =   LEDC_LOW_SPEED_MODE,   // Modo de Velocidade -> LOW
  .channel    =   LEDC_CHANNEL_0,           
  .timer_sel  =   LEDC_TIMER_0,
  .duty       =   755,              // duty slightly bigger than EN_B in order to compesate uneven load
  .hpoint     =   0

};

ledc_channel_config_t channel_EN_B = {
  .gpio_num   =   EN_B,             // Seleciona o pino para atuar o PWM
  .speed_mode =   LEDC_LOW_SPEED_MODE,   // Modo de Velocidade -> LOW
  .channel    =   LEDC_CHANNEL_1,           
  .timer_sel  =   LEDC_TIMER_0,
  .duty       =   750,
  .hpoint     =   0

};

//  Estrutura de dados para receber  as váriaveis  de configuração de frequência (1Hz, 100 Hz e outros), modo (HIGH ou LOW) e selecionar o TIMER (0,1,2)  

ledc_timer_config_t timer = {                   // Configuração do timer 

    .speed_mode      = LEDC_LOW_SPEED_MODE,     // Modo de Velocidade -> LOW
    .duty_resolution = LEDC_TIMER_10_BIT,       // Resolução do do ciclo de trabalho (2^10 = 1024 valores)
    .timer_num       = LEDC_TIMER_0,            // Utilizado o TIMER 0
    .freq_hz         = LEDC_FREQ,               // Frequência de operação do sinal PWM
    .clk_cfg         = LEDC_AUTO_CLK            // Seleção automatica da fonte geradora do clock (interna ou externa)
  
};

uint32_t dutyCycleL = 755;
uint32_t dutyCycleR = 750;

WiFiClient espClient;
PubSubClient client(espClient);

typedef enum {
  foward,
  rotate_clockwise,
  rotate_anticlockwise,
  stop,
  continue_movement
} movement_t;

void switch_movement(movement_t mode) {
  switch(mode) {
    case 0:
      // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyCycleL, 0);
      // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyCycleR, 0);
      gpio_set_level(IN_1, 1);
      gpio_set_level(IN_2, 0);
      gpio_set_level(IN_3, 1);
      gpio_set_level(IN_4, 0);
      break;
    case 1:
      // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyCycleL, 0);
      // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0, 0);
      gpio_set_level(IN_1, 1);
      gpio_set_level(IN_2, 0);
      gpio_set_level(IN_3, 0); 
      gpio_set_level(IN_4, 0);
      break;
    case 2:        
      // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
      // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyCycleR, 0);  
      gpio_set_level(IN_1, 0);
      gpio_set_level(IN_2, 0);
      gpio_set_level(IN_3, 1);
      gpio_set_level(IN_4, 0);
      break;
    case 3:        
      // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
      // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0, 0);  
      gpio_set_level(IN_1, 0);
      gpio_set_level(IN_2, 0);
      gpio_set_level(IN_3, 0);
      gpio_set_level(IN_4, 0);
      break;
    default:
      // do nothing
      break;
  }
}

void set_pins() {
  ledc_timer_config(&timer); // Envia o endereço  da estrutura timer para a função de configuração do canal PWM 

  ledc_channel_config(&channel_EN_A);
  ledc_channel_config(&channel_EN_B);

  ledc_fade_func_install(0);

  // Configure the GPIO ports as outputs  
  gpio_pad_select_gpio(IN_1);
  gpio_set_direction(IN_1, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(IN_2);
  gpio_set_direction(IN_2, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(IN_3);
  gpio_set_direction(IN_3, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(IN_4);
  gpio_set_direction(IN_4, GPIO_MODE_OUTPUT);
}

void connect_mqtt() {
  // set mqtt broker parameters
  client.setServer(mqtt_broker, mqtt_port);
  
  // register callback for recieving data
  client.setCallback(callback);

  // loops until successfull connection with mqtt broker
  while (!client.connected()) {
      String client_id = "esp32-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
        if(debug) { client.publish(debug_topic, "Public MQTT broker connected"); }
      } else {
          Serial.print("failed with state ");
          Serial.print(client.state());
          delay(2000);
      }
  }
  // Subscribe to topic
  client.subscribe(topic);
  if(debug) { client.publish(debug_topic, "Subscibed to topic:"); client.publish(debug_topic, topic); }
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  if(debug) { client.publish(debug_topic, "Disconnected from WiFi access point"); }
  Serial.print("WiFi lost connection. Reason: ");
  if(debug) { client.publish(debug_topic, (char*) &info.wifi_sta_disconnected.reason); }
  if(debug) { client.publish(debug_topic, "Trying to Reconnect"); }
  // reconnects to WiFi
  WiFi.begin(ssid, password);
  // Reconnects to mqtt broker and subscribes to topic
  connect_mqtt();
}

void read_mqtt_message(char *topic, byte *payload, uint32_t length, char *message) {
  if(debug) {
    client.publish(debug_topic, "Message arrived in topic: ");
    client.publish(debug_topic, topic);
    Serial.println("Message arrived in topic: "); 
    Serial.println(topic);
  }

  Serial.printf("Message length %d\n", length);
  Serial.print("Message:");

  // store payload into buffer
  for (int i = 0; i < length; i++) {
      message[i] = (char) payload[i];
  }
  // transforms buffer into string
  message[length] = '\0';
  Serial.println(message);
  if(debug) { client.publish(debug_topic, "Recieved message: "); client.publish(debug_topic, message); }
  
}

void setup() {
  // set pins EN_A and EN_B as pwm channels and the remaining as outputs
  set_pins();
  
  // Set software serial baud to 115200;
  Serial.begin(115200);

  // delete old config
  WiFi.disconnect(true);
  // configure callback for wifi disconnection
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  // Connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
  }
  //connecting to a mqtt broker
  connect_mqtt();
}

void callback(char *topic, byte *payload, unsigned int length) {
  // turns payload in a string in order to use strcmp function
  char message[length + 1];
  read_mqtt_message(topic, payload, length, message);
  
  movement_t move = stop;

  // Verifiy which command was received
  if(!strcmp(message, "pull")) { move = stop; }
  else if(!strcmp(message, "push")) { move = foward; }
  else if(!strcmp(message, "lift")) { move = rotate_clockwise; }
  else if(!strcmp(message, "drop")) { move = rotate_anticlockwise; }
  else { move = stop; }
  
  // Update pin output accordingly to the received command
  switch_movement(move);
  if(debug) { client.publish(debug_topic, "-----------------------"); }
}

void loop() {
  if (!client.connected()) { // if mqtt client was disconnected
    if (WiFi.status() == WL_CONNECTED) { // connects to mqtt client if WiFi is connected
      connect_mqtt();
    } else { // or waits until WiFi reconnection
      // TO DO: parar o robô em caso de desconexão
      delay(2000);
    }
  } else {
    // Client connected
    client.loop();
  }
}
