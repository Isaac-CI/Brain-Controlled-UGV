#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

// h-bridge output/pwm pins
#define EN_B GPIO_NUM_5
#define IN_4 GPIO_NUM_18
#define IN_3 GPIO_NUM_19
#define IN_2 GPIO_NUM_21
#define IN_1 GPIO_NUM_22
#define EN_A GPIO_NUM_23

// pwm frequency
#define PWM_FREQ 2000

// WiFi parameters
const char *ssid = "VIVOFIBRA-C9A8"; // Enter WiFi name
const char *password = "83D24D124F";  // Enter WiFi password

// MQTT Broker parameters
const char *mqtt_broker = "9462fe64c36446ae9a704e0a61ce45b2.s1.eu.hivemq.cloud";   // MQTT Broker is in localhost
const char *topic = "bri/command";          // Sets MQTT topic to subscribe and receive mental commands
const char *debug_topic = "bri/debug";      // Sets debug topic
const bool debug = true;                   // Flag that controls debug logic
const char *mqtt_username = "subscriber";   // Sets MQTT client username
const char *mqtt_password = "#1Subscriber";   // Sets MQTT client password
const int mqtt_port = 8883;                 // Sets MQTT connection port


static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

// Setting PWM properties

// PWM duty cycle
uint32_t dutyCycleL = 750;              // Duty Cycle for left motor. It is slightly bigger right motor duty cycle in order to compesate uneven load
uint32_t dutyCycleR = 770;              // Duty Cycle for right motor

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

WiFiClientSecure espClient;     // WiFI client helper
PubSubClient client(espClient); // MQTT client helper

typedef enum { // Enum type for improved readability
  forward,
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
    // Command = forward
    case 0:
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyCycleL, 0);
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyCycleR, 0);
      gpio_set_level(IN_1, 1);
      gpio_set_level(IN_2, 0);
      gpio_set_level(IN_3, 1);
      gpio_set_level(IN_4, 0);
      Serial.println("Forward");
      break;

    // Command = rotate_clockwise
    case 1:
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutyCycleL, 0);
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyCycleR, 0);
      gpio_set_level(IN_1, 1);
      gpio_set_level(IN_2, 0);
      gpio_set_level(IN_3, 0); 
      gpio_set_level(IN_4, 1);
      Serial.println("Rotate Clockwise");
      break;
    // Command = rotate_counterclockwise
    case 2:
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dutyCycleR, 0);  
      gpio_set_level(IN_1, 0);
      gpio_set_level(IN_2, 1);
      gpio_set_level(IN_3, 1);
      gpio_set_level(IN_4, 0);
      Serial.println("Rotate Counter Clockwise");
      break;
    // Command = stop
    case 3:
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
      ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0, 0);  
      gpio_set_level(IN_1, 0);
      gpio_set_level(IN_2, 0);
      gpio_set_level(IN_3, 0);
      gpio_set_level(IN_4, 0);
      Serial.println("Stop");
      break;
    // Command is continue movement or a value that is not supported
    default:
      // do nothing
      Serial.println("Doing nothing");
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
  espClient.setCACert(root_ca);
  client.setServer(mqtt_broker, mqtt_port);
  
  // register callback for recieving data
  client.setCallback(callback);

  // loops until successfull connection with mqtt broker
  while (!client.connected()) {
      String client_id = "esp32-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {Serial.println("Public MQTT broker connected"); }
      else {
          Serial.print("failed with state ");
          Serial.println(client.state());
          delay(2000);
      }
  }
  // Subscribe to topic
  client.subscribe(topic);
  if(debug) { Serial.println("Subscibed to topic:"); Serial.println(topic); }
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  command = stop;
  switch_movement(command);
  if(debug) { Serial.println("Disconnected from WiFi access point"); }
  Serial.print("WiFi lost connection. Reason: ");
  if(debug) { Serial.println((char*) &info.wifi_sta_disconnected.reason); }
  if(debug) { Serial.println("Trying to Reconnect"); }
  // reconnects to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Reconnecting to WiFi");
  }
  // Reconnects to mqtt broker and subscribes to topic
  Serial.println("Reconnected to WiFi");
  connect_mqtt();
}

void read_mqtt_message(char *topic, byte *payload, uint32_t length, char *message) {
  if(debug) {
    Serial.println("Message arrived in topic: ");
    Serial.println(topic);
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
  if(debug) { Serial.println("Recieved message: "); Serial.println(message); }
  
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
  command = stop;
  switch_movement(command);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
  }
  Serial.println("Connected to wifi");
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
  if(!strcmp(message, "pull")) { command = forward; }                   // If the received command is pull, the ugv moves forward
  else if(!strcmp(message, "push")) { command = rotate_clockwise; }     // If the received command is push, the ugv is rotated clockwise
  else if(!strcmp(message, "neutral")) { command = continue_movement; }  // if the received command is neutral, the ugv continues it's movement
  else if(!strcmp(message, "frown")) { command = stop; }                     // if the received command is smile, the ugv stops. Stopping command has priority, so even if the mental command says otherwise. Should the user be perceived smilling, the ugv stops
  
  // Update pin output accordingly to the received command
  switch_movement(command);
  Serial.println("message");
  if(debug) { Serial.println("-----------------------"); }
}

void loop() {
  if (!client.connected()) { // if mqtt client was disconnected
    command = stop;
    switch_movement(command);
    if (WiFi.status() == WL_CONNECTED) { // connects to mqtt client if WiFi is connected
      connect_mqtt();
    } else { // waits for WiFi reconnection
      Serial.println("Waiting for WiFi connection");
      delay(200);
    }
  } else {
    // Client connected
    client.loop();
  }
}