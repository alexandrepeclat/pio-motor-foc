// /**
//  * ESP32 position motion control example with magnetic sensor
//  */
#include <Arduino.h>
#include <SimpleFOC.h>
#include <WebSocketsClient.h>
#include <WiFi.h>
#include <secrets.h>
#define PWM_PIN 14

// const int VELOCITY_LIMIT = 40;

MagneticSensorPWM sensor = MagneticSensorPWM(PWM_PIN, 4, 904);

// Motor instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 15);

// WebSocket client instance
WebSocketsClient ws;

// angle set point variable
float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_angle, cmd);
}

// Fonction pour analyser la commande reçue
struct Command {
  char action;
  int axis;
  int value;
  int interval;
  bool valid;
};

Command parseCommand(const String& input) {
  // Expression régulière pour extraire les données
  // Format attendu : [Action][Axis][Value][Type][Interval]
  if (input.length() >= 7) {  // La commande doit être d'une longueur suffisante
    Command cmd;
    cmd.action = input.charAt(0);               // 'L'
    cmd.axis = input.charAt(1) - '0';           // '0' -> 0
    cmd.value = input.substring(2, 4).toInt();  // '50' -> 50
    cmd.interval = input.substring(5).toInt();  // '5000' -> 5000
    cmd.valid = true;
    return cmd;
  } else {
    Command cmd = {'\0', -1, -1, -1, false};  // Valeurs par défaut pour une commande invalide
    return cmd;
  }
}

const float ANGLE_MIN = 0;
const float ANGLE_MAX = 2 * PI;
const int CMD_MIN = 0;
const int CMD_MAX = 100;

float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void executeCommand(Command cmd) {
  if (cmd.action == 'L') {
    Serial.println("CMD: A: " + String(cmd.axis) + " V: " + String(cmd.value) + " I: " + String(cmd.interval));
    float target_position = mapFloat(cmd.value, CMD_MIN, CMD_MAX, ANGLE_MIN, ANGLE_MAX);
    float current_position = sensor.getAngle();
    float delta_angle = target_position - current_position;
    float velocity = abs(delta_angle / (cmd.interval / 1000.0));
    Serial.println("T: " + String(target_position) + " C: " + String(current_position) + " D: " + String(delta_angle) + " V: " + String(velocity));
    Serial.print(target_position, 10);
    // S'assurer que la vitesse ne dépasse pas la limite de vitesse maximale
    if (velocity > 20) {
      velocity = 20;
    }

    // Définir la vitesse et l'angle cible
    float roundedValue = round(target_position * 100) / 100.0;
    Serial.println("Rounded value: " + String(roundedValue));
    noInterrupts();
    target_angle = roundedValue;
    interrupts();
    // motor.velocity_limit = velocity;
  }
}

// Fonction pour gérer les messages reçus via WebSocket
void onWsEvent(WStype_t type, uint8_t* payload, size_t length) {
  Serial.println();
  if (type == WStype_TEXT) {
    String input = String((char*)payload);
    Serial.println("Command received: " + input);
    Command cmd = parseCommand(input);
    executeCommand(cmd);
  } else if (type == WStype_BIN) {
    // Traitement des données binaires (Blob)
    Serial.print("Received Binary Data:");
    for (size_t i = 0; i < length; i++) {
      Serial.print(payload[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    String input = String((char*)payload);
    Serial.println("Command received: " + input);
    Command cmd = parseCommand(input);
    executeCommand(cmd);
  } else if (type == WStype_CONNECTED) {
    Serial.println("WebSocket connected");
    ws.sendTXT("Hello Server");  // Envoie un message initial
  } else if (type == WStype_DISCONNECTED) {
    Serial.println("WebSocket disconnected");
  } else if (type == WStype_ERROR) {
    Serial.println("WebSocket error");
  }
}

void doPWM() {
  sensor.handlePWM();
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");

  // Initialisation du serveur WebSocket
  ws.begin("192.168.0.173", 1234, "/");
 ws.onEvent(onWsEvent);
   ws.setReconnectInterval(5000);

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
  motor.LPF_velocity.Tf = 0.05;  // Augmente le filtre (par défaut 0.01)


  // angle P controller https://docs.simplefoc.com/angle_loop
  motor.P_angle.P = 20;
  motor.P_angle.P = 10;  // Commence par diminuer à la moitié (valeur actuelle = 20)
  // maximal velocity of the position control
  motor.velocity_limit = 20;

  // comment out if not needed
  // motor.useMonitoring(Serial);

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
  ws.loop();
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
  //  motor.monitor();

  // user communication
  command.run();
}
