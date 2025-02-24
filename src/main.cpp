// /**
//  * ESP32 position motion control example with magnetic sensor
//  */
#include <Arduino.h>
//  #include <ESPAsyncWebServer.h>
#include <SimpleFOC.h>
#include <WiFi.h>
#include <secrets.h>
#define PWM_PIN 14  // GPIO36 (ADC1_CH0) pour lire le signal PWM

// const int VELOCITY_LIMIT = 40;

MagneticSensorPWM sensor = MagneticSensorPWM(PWM_PIN, 4, 904);

// Motor instance
    BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 15);

// angle set point variable
float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

// // Serveur WebSocket
// // AsyncWebServer server(80);
// // AsyncWebSocket ws("/ws");

// // Fonction pour analyser la commande reçue
// // struct Command {
// //   char action;
// //   int axis;
// //   int value;
// //   int interval;
// //   bool valid;
// // };

// // Command parseCommand(const String& input) {
// //   // Expression régulière pour extraire les données
// //   // Format attendu : [Action][Axis][Value][Type][Interval]
// //   if (input.length() >= 7) {                    // La commande doit être d'une longueur suffisante
// //     Command cmd;
// //     cmd.action = input.charAt(0);               // 'L'
// //     cmd.axis = input.charAt(1) - '0';           // '0' -> 0
// //     cmd.value = input.substring(2, 4).toInt();  // '50' -> 50
// //     cmd.interval = input.substring(5).toInt();  // '5000' -> 5000
// //     cmd.valid = true;
// //     return cmd;
// //   } else {
// //     Command cmd = {'\0', -1, -1, -1, false};  // Valeurs par défaut pour une commande invalide
// //     return cmd;
// //   }
// // }

// // Fonction pour gérer les messages reçus via WebSocket
// // void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
// //   if (type == WS_EVT_DATA) {
// //     // Convertir les données en chaîne de caractères
// //     String input = String((char*)data);
// //     Serial.println("Command received: " + input);

// //     // Parser la commande
// //     Command cmd = parseCommand(input);
// //     if (cmd.action == 'L') {
// //       // Angle cible
// //       float target_position = cmd.value;  // C'est la valeur que tu veux atteindre
// //       // Angle actuel
// //       float current_position = sensor.getAngle(); // Valeur actuelle de l'angle
// //       // Calcul de la différence d'angle
// //       float delta_angle = target_position - current_position;
// //       // Calcul de la vitesse nécessaire (en degrés par seconde)
// //       float velocity = delta_angle / (cmd.interval / 1000.0); // intervalle en secondes

// //       // S'assurer que la vitesse ne dépasse pas la limite de vitesse maximale
// //       if (velocity > VELOCITY_LIMIT) {
// //         velocity = VELOCITY_LIMIT;
// //       }

// //       // Définir la vitesse et l'angle cible
// //       target_angle = target_position;
// //       motor.velocity_limit = velocity;  // Utilise la vitesse calculée
// //     }
// //   } else if (type == WS_EVT_CONNECT) {
// //     Serial.println("WebSocket client connected");
// //   } else if (type == WS_EVT_DISCONNECT) {
// //     Serial.println("WebSocket client disconnected");
// //   }
// // }

void doPWM() {
  sensor.handlePWM();
}


void setup() {

  Serial.begin(115200);
  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi...");
  // }
  // Serial.println("Connected to WiFi!");

  // Initialisation du serveur WebSocket
  // ws.onEvent(onWsEvent);
  // server.addHandler(&ws);

  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // initialise magnetic sensor hardware
  sensor.init();
  sensor.enableInterrupt(doPWM);
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // maximal voltage to be set to the motor
  motor.voltage_limit = 6;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P = 20;
  // maximal velocity of the position control
  motor.velocity_limit = 20;
  
  // comment out if not needed
  motor.useMonitoring(Serial);


  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target angle");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
}


void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_angle);


  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();
}

