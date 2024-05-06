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
#define PWM_FREQ 2000

// WiFi parameters
const char *ssid = "VIVOFIBRA-C9A8"; // Enter WiFi name
const char *password = "83D24D124F";  // Enter WiFi password

// MQTT Broker parameters
const char *mqtt_broker = "192.168.15.120"; // MQTT Broker is in localhost
const char *topic = "bri/command";          // Sets MQTT topic to subscribe and receive mental commands
const char *debug_topic = "bri/debug";      // Sets debug topic
const bool debug = false;                   // Flag that controls debug logic
const char *mqtt_username = "subscriber";   // Sets MQTT client username
const char *mqtt_password = "subscriber";   // Sets MQTT client password
const int mqtt_port = 1883;                 // Sets MQTT connection port

// Setting PWM properties

// PWM duty cycle
uint32_t dutyCycleL = 755;              // Duty Cycle for left motor. It is slightly bigger right motor duty cycle in order to compesate uneven load
uint32_t dutyCycleR = 750;              // Duty Cycle for right motor

ledc_channel_config_t channel_EN_A = {   // PWM channel config
  .gpio_num   =   EN_A,                  // Selects PWM pin
  .speed_mode =   LEDC_LOW_SPEED_MODE,   // Sets Speed Mode to LOW
  .channel    =   LEDC_CHANNEL_0,        // Sets PWM channel
  .timer_sel  =   LEDC_TIMER_0,          // Selects TIMER0 as the PWM timer
  .duty       =   dutyCycleL,            // Sets the duty cylce
  .hpoint     =   0                      // Sets the H point
};

ledc_channel_config_t channel_EN_B = {   // PWM channel config
  .gpio_num   =   EN_B,                  // Selects PWM pin
  .speed_mode =   LEDC_LOW_SPEED_MODE,   // Sets Speed Mode to LOW
  .channel    =   LEDC_CHANNEL_1,        // Sets PWM channel
  .timer_sel  =   LEDC_TIMER_0,          // Selects TIMER0 as the PWM timer
  .duty       =   dutyCycleR,            // Sets the duty cylce
  .hpoint     =   0                      // Sets the H point
};

ledc_timer_config_t timer = {                   // Timer Config
    .speed_mode      = LEDC_LOW_SPEED_MODE,     // Sets Speed Mode to LOW
    .duty_resolution = LEDC_TIMER_10_BIT,       // Sets Duty Cycle resolution to 10 bits, i.e. ranging from 0 to 1023 
    .timer_num       = LEDC_TIMER_0,            // Selects TIMER0 as the timer
    .freq_hz         = PWM_FREQ,                // Sets PWM frequency
    .clk_cfg         = LEDC_AUTO_CLK            // Automatic selection of clock generation source
};

WiFiClient espClient;           // WiFI client helper
PubSubClient client(espClient); // MQTT client helper

typedef enum { // Enum type for improved readability
  foward,
  rotate_clockwise,
  rotate_counterclockwise,
  stop,
  continue_movement
} command_t;

// Enum type variable to track commands
command_t command;

/*
 * Parameters: command_t command: Command indicating the UGV how to move in the next cycle
 * This function switchs the received command and sets pin outputs accordingly
 * Returns: None
*/
void switch_movement(command_t command) {
  switch(command) {
    // Command = foward
    case 0:
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyCycleL, 0);
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyCycleR, 0);
      gpio_set_level(IN_1, 1);
      gpio_set_level(IN_2, 0);
      gpio_set_level(IN_3, 1);
      gpio_set_level(IN_4, 0);
      break;
    // Command = rotate_clockwise
    case 1:
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyCycleL, 0);
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0, 0);
      gpio_set_level(IN_1, 1);
      gpio_set_level(IN_2, 0);
      gpio_set_level(IN_3, 0); 
      gpio_set_level(IN_4, 0);
      break;
    // Command = rotate_counterclockwise
    case 2:
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyCycleR, 0);  
      gpio_set_level(IN_1, 0);
      gpio_set_level(IN_2, 0);
      gpio_set_level(IN_3, 1);
      gpio_set_level(IN_4, 0);
      break;
    // Command = stop
    case 3:
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0, 0);  
      gpio_set_level(IN_1, 0);
      gpio_set_level(IN_2, 0);
      gpio_set_level(IN_3, 0);
      gpio_set_level(IN_4, 0);
      break;
    // Command is continue movement or a value that is not supported
    default:
      // do nothing
      break;
  }
}

/*
 * Parameters: None
 * This function configures output pins and PWM channels 
 * Returns: None
*/
void set_pins() {
  ledc_timer_config(&timer); // Sends timer to PWM channel configuration

  // Configure PWM channels
  ledc_channel_config(&channel_EN_A);
  ledc_channel_config(&channel_EN_B);

  // Starts PWM generation
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

/*
 * Parameters: None
 * This function handles MQTT connection, connecting the UGV client to the designated broker and ~]]~; 
 * Returns: None
*/
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
  
  // Default movement is to stop, in order to stop the UGV in case of receiving an usuppoted command 
  command = stop;

  // Verifiy which command was received
  if(!strcmp(message, "pull")) { command = stop; }
  else if(!strcmp(message, "push")) { command = foward; }
  else if(!strcmp(message, "lift")) { command = rotate_clockwise; }
  else if(!strcmp(message, "drop")) { command = rotate_counterclockwise; }
  else if(!strcmp(message, "neutral")) { command = continue_movement; }
  // else { command = stop; }
  
  // Update pin output accordingly to the received command
  switch_movement(command);
  if(debug) { client.publish(debug_topic, "-----------------------"); }
}

void loop() {
  if (!client.connected()) { // if mqtt client was disconnected
    command = stop;
    switch_movement(command);
    if (WiFi.status() == WL_CONNECTED) { // connects to mqtt client if WiFi is connected
      connect_mqtt();
    } else { // waits for WiFi reconnection
      delay(200);
    }
  } else {
    // Client connected
    client.loop();
  }
}
