#include <WiFi.h>
#include <PubSubClient.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define EN_B GPIO_NUM_32
#define IN_4 GPIO_NUM_33
#define IN_3 GPIO_NUM_26
#define IN_2 GPIO_NUM_27
#define IN_1 GPIO_NUM_14
#define EN_A GPIO_NUM_13

#define LEDC_FREQ 2000

// WiFi
const char *ssid = "VIVOFIBRA-C9A8"; // Enter your WiFi name
const char *password = "83D24D124F";  // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "192.168.15.120";
const char *topic = "bri/command";
const char *mqtt_username = "subscriber";
const char *mqtt_password = "subscriber";
const int mqtt_port = 1883;

// Setting PWM properties
ledc_channel_config_t channel_EN_A = {
  .gpio_num   =   EN_A,             // Seleciona o pino para atuar o PWM
  .speed_mode =   LEDC_LOW_SPEED_MODE,   // Modo de Velocidade -> LOW
  .channel    =   LEDC_CHANNEL_0,           
  .timer_sel  =   LEDC_TIMER_0,
  .duty       =   755,
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

void setup() {
    // pinMode(EN_A, OUTPUT);
    // gpio_pad_select_gpio(EN_A);
    // gpio_set_direction(EN_A, GPIO_MODE_OUTPUT);
    

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
  // gpio_pad_select_gpio(EN_B);
  // gpio_set_direction(EN_B, GPIO_MODE_OUTPUT);
    
  // Set software serial baud to 115200;
  //Serial.begin(115200);
  // Connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      //Serial.println("Connecting to WiFi..");
  }
  //Serial.println("Connected to the Wi-Fi network");
  //connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
      String client_id = "esp32-client-";
      client_id += String(WiFi.macAddress());
      //Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
        //Serial.println("Public MQTT broker connected");
      } else {
          //Serial.print("failed with state ");
          //Serial.print(client.state());
          delay(2000);
      }
  }
  // Publish and subscribe
  client.subscribe(topic);
}

void callback(char *topic, byte *payload, unsigned int length) {
  //Serial.print("Message arrived in topic: ");
  //Serial.println(topic);
  char message[length + 1];
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
      message[i] = (char) payload[i];
  }
  message[length] = '\0';
  if(message[0] == 'p'){ // command is push or pull
      if(message[2] == 'l'){ // command is pull
        //Serial.println("Stopping");
        // gpio_set_level(EN_A, 0);
        // gpio_set_level(EN_B, 0);
          
        // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
        // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0, 0);

        gpio_set_level(IN_1, 0);
        gpio_set_level(IN_2, 0);
        gpio_set_level(IN_3, 0);
        gpio_set_level(IN_4, 0);
  
      } else { // command is push
        //Serial.println("Moving Foward");
        // gpio_set_level(EN_A, 1);
        // gpio_set_level(EN_B, 1);

        // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyCycleL, 0);
        // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyCycleR, 0);
          
        gpio_set_level(IN_1, 1);
        gpio_set_level(IN_2, 0);
        gpio_set_level(IN_3, 1);
        gpio_set_level(IN_4, 0);
      }
  } else if(message[0] == 'l') { // command is lift or left
    if(message[1] == 'i'){ // command is lift
      //Serial.println("Rotating Clockwise");
        // gpio_set_level(EN_A, 1);
        // gpio_set_level(EN_B, 1);

        // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyCycleL, 0);
        // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0, 0);
        
        gpio_set_level(IN_1, 1);
        gpio_set_level(IN_2, 0);
        gpio_set_level(IN_3, 0);
        gpio_set_level(IN_4, 0);
    }
  } else if(message[0] == 'd'){ // command is drop or disapear
    if(message[1] == 'r'){ // command is drop
      //Serial.println("Rotating Anti-Clockwise");
        // gpio_set_level(EN_A, 1);
        // gpio_set_level(EN_B, 1);

        // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
        // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyCycleR, 0);
          
        gpio_set_level(IN_1, 0);
        gpio_set_level(IN_2, 0);
        gpio_set_level(IN_3, 1);
        gpio_set_level(IN_4, 0);
    }
  } else { // command is neutral or isn't supported
    if(message[0] == 'n') { // command is neutral
      //Serial.println("Continuing Movement");
    } else{ // command isn't supported
    //  Serial.println("Stoppping");
      // gpio_set_level(EN_A, 0);
      // gpio_set_level(EN_B, 0);

      // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
      // ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0, 0);

      gpio_set_level(IN_1, 0);
      gpio_set_level(IN_2, 0);
      gpio_set_level(IN_3, 0);
      gpio_set_level(IN_4, 0);

    }
  }
  //Serial.println("-----------------------");
}

void loop() {
    client.loop();
}
