// /**
//  * ESP32 position motion control example with magnetic sensor
//  */
#include <Arduino.h>
#include <SimpleFOC.h>
#include <WebSocketsClient.h>
#include <WiFi.h>
#include <secrets.h>
#include <limits>
#include <ESPAsyncWebServer.h>
#include <SerialCommandHandler.h>
#include <RestCommandHandler.h>

enum AppState {
  CALIBRATING_MANUALLY,
  RUNNING,
  STOPPED
};

WebSocketsClient ws;
AsyncWebServer server(80);
AsyncCorsMiddleware cors;
SerialCommandHandler serialCommandHandler;
RestCommandHandler restCommandHandler(server);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 5);
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 15);  // 3 pwm pins + enable pin

float minAngle = std::numeric_limits<float>::max();
float maxAngle = std::numeric_limits<float>::min();
AppState appState = AppState::STOPPED;
float target_angle = 0;

const int MAX_VELOCITY = 50;
const int CMD_MIN = 0;
const int CMD_MAX = 100;

float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
  if (inMin == inMax)
    return outMin;                         // Évite la division par zéro
  value = constrain(value, inMin, inMax);  // Empêche l'extrapolation
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

bool isCalibrated() {
  return minAngle != std::numeric_limits<float>::max() && maxAngle != std::numeric_limits<float>::min();
}

String doTargetRaw(float angle) {
  appState = AppState::RUNNING;
  target_angle = angle;
  return "Target raw: " + String(target_angle);
}

String doTargetNormalized(int angleNormalized) {
  if (!isCalibrated()) {
    return "Motor not calibrated yet";
  }
  appState = AppState::RUNNING;
  target_angle = mapFloat(angleNormalized, 0, 100, minAngle, maxAngle);
  return "Target normalized: " + String(angleNormalized) + " Target raw: " + String(target_angle);
}

String doGetRestRoutes() {
  return restCommandHandler.getRoutesList();
}

String doGetSerialCommands() {
  return serialCommandHandler.getCommandsList();
}

String doGetDebug() {
  return "Min angle: " + String(minAngle) + " Max angle: " + String(maxAngle) + " Current angle: " + String(sensor.getAngle()) + " Target angle: " + String(target_angle) + " App state: " + String(appState);
}

String doStop() {
  appState = AppState::STOPPED;
  return "STOPPED";
}

String doRun() {
  appState = AppState::RUNNING;
  return "RUNNING";
}

String doCalibrate() {
  appState = AppState::CALIBRATING_MANUALLY;
  minAngle = std::numeric_limits<float>::max();  // Valeur initiale très haute
  maxAngle = std::numeric_limits<float>::min();  // Valeur initiale très basse
  return "Calibration started " + String(motor.target);
}

// Fonction pour analyser la commande reçue
struct Command {
  char action;
  int axis;
  int value;
  int interval;
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
    return cmd;
  } else {
    Command cmd = {'\0', -1, -1, -1};  // Valeurs par défaut pour une commande invalide
    return cmd;
  }
}

void executeCommand(Command cmd) {
  if (!isCalibrated()) {
    Serial.println("Command ignored: Motor not calibrated yet");
    return;
  }
  if (appState != AppState::RUNNING) {
    Serial.println("Command ignored: Motor not running");
    return;
  }
  if (cmd.action == 'L' && cmd.axis == 0) {
    float target_position = mapFloat(cmd.value, CMD_MIN, CMD_MAX, minAngle, maxAngle);
    float current_position = sensor.getAngle();
    float delta_angle = target_position - current_position;
    float velocity = abs(delta_angle / (cmd.interval / 1000.0));
    float target_angle_rounded = round(target_position * 100) / 100.0;
    Serial.println("CMD: A: " + String(cmd.axis) + " V: " + String(cmd.value) + " I: " + String(cmd.interval) + " // T(r): " + String(target_angle_rounded) + " C(s): " + String(current_position) + " C(m): " + String(motor.shaft_angle) + " D: " + String(delta_angle) + " V: " + String(velocity));

    // S'assurer que la vitesse ne dépasse pas la limite de vitesse maximale
    if (velocity > MAX_VELOCITY) {
      velocity = MAX_VELOCITY;
    }

    // Définir la vitesse et l'angle cible
    noInterrupts();
    target_angle = target_angle_rounded;
    interrupts();
    motor.velocity_limit = velocity;
  }
}

// Fonction pour gérer les messages reçus via WebSocket
void onWsEvent(WStype_t type, uint8_t* payload, size_t length) {
  Serial.println();
  if (type == WStype_TEXT) {
    String input = String((char*)payload);
    // Serial.println("Command received: " + input);
    Command cmd = parseCommand(input);
    executeCommand(cmd);
  } else if (type == WStype_BIN) {
    // Traitement des données binaires (Blob)
    // Serial.print("Received Binary Data:");
    // for (size_t i = 0; i < length; i++) {
    //   Serial.print(payload[i], HEX);
    //   Serial.print(" ");
    // }
    // Serial.println();
    String input = String((char*)payload);
    // Serial.println("Command received: " + input);
    Command cmd = parseCommand(input);
    executeCommand(cmd);
  } else if (type == WStype_CONNECTED) {
    Serial.println("WebSocket connected");
    ws.sendTXT("{ \"identifier\": \"myesp\", \"address\": \"00000000\", \"version\": 0 }");  // Envoie un message initial
  } else if (type == WStype_DISCONNECTED) {
    Serial.println("WebSocket disconnected");
  } else if (type == WStype_ERROR) {
    Serial.println("WebSocket error");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");
  delay(1000);

  // Initialisation du serveur WebSocket
  ws.begin("192.168.0.173", 1234, "/");  // PC
  //ws.begin("192.168.0.204", 54817, "/"); //HyperV
  ws.onEvent(onWsEvent);
  ws.setReconnectInterval(5000);

  // SimpleFOCDebug::enable(&Serial);   // enable more verbose output for debugging
  // motor.useMonitoring(Serial);

  sensor.init();

  driver.voltage_power_supply = 12;  // power supply voltage [V]
  driver.init();

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;  // choose FOC modulation (optional)
  motor.controller = MotionControlType::angle;               // set motion control loop to be used
  motor.PID_velocity.P = 0.2f;                               // velocity PI controller parameters
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.01;  // velocity low pass filtering time constant  - the lower the less filtered
  motor.P_angle.P = 20;          // angle P controller https://docs.simplefoc.com/angle_loop

  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);
  motor.init();     // initialize motor
  motor.initFOC();  // align sensor and start FOC // TODO deplacer ailleurs pour le lancer que sur demande à la calibration

  // Register Serial commands
  serialCommandHandler.registerCommand("s", doStop);
  serialCommandHandler.registerCommand("stop", doStop);
  serialCommandHandler.registerCommand("c", doCalibrate);
  serialCommandHandler.registerCommand("calibrate", doCalibrate);
  serialCommandHandler.registerCommand("r", doRun);
  serialCommandHandler.registerCommand("run", doRun);
  serialCommandHandler.registerCommand<int>("target", {"target"}, doTargetNormalized);
  serialCommandHandler.registerCommand("debug", doGetDebug);
  serialCommandHandler.registerCommand("help", doGetSerialCommands);

  // Register REST API routes
  restCommandHandler.registerCommand("stop", HTTP_GET, doStop);
  restCommandHandler.registerCommand("calibrate", HTTP_GET, doCalibrate);
  restCommandHandler.registerCommand("run", HTTP_GET, doRun);
  restCommandHandler.registerCommand<int>("target", HTTP_GET, {"target"}, doTargetNormalized);
  restCommandHandler.registerCommand("debug", HTTP_GET, doGetDebug);
  restCommandHandler.registerCommand("help", HTTP_GET, doGetRestRoutes);
}

void loop() {
  
  switch (appState) {
    case CALIBRATING_MANUALLY: {
      if (motor.enabled) {
        motor.disable();
      }
      motor.voltage_limit = 0;
      motor.velocity_limit = 0;
      float currentAngle = sensor.getAngle();
      target_angle = currentAngle;  // Set target angle as the current angle, for not "jumping" when starting
      if (currentAngle < minAngle) {
        minAngle = currentAngle;
        Serial.println("Min-max angles: " + String(minAngle) + " " + String(maxAngle));
      }
      if (currentAngle > maxAngle) {
        maxAngle = currentAngle;
        Serial.println("Min-max angles: " + String(minAngle) + " " + String(maxAngle));
      }
      break;
    }
    case RUNNING: {
      if (!motor.enabled) {
        motor.enable();
      }
      motor.voltage_limit = 6;    // maximal voltage to be set to the motor
      motor.velocity_limit = MAX_VELOCITY;  // maximal velocity of the position control
      break;
    }
    case STOPPED: {
      if (motor.enabled) {
        motor.disable();
      }
      float currentAngle = sensor.getAngle();
      target_angle = currentAngle;  // Set target angle as the current angle, for not "jumping" when starting
      motor.voltage_limit = 0;
      motor.velocity_limit = 0;
      break;
    }
  }

  // main FOC algorithm function the faster you run this function the better //TODO à voir si loopFoc doit être fait avant les trucs dans le switch
  // Even if motor is stopped, For updating sensors
  if (motor.enabled) {
    motor.loopFOC();
    motor.move(target_angle);  // Motion control function velocity, position or voltage (defined in motor.controller) this function can be run at much lower frequency than loopFOC() function
  } else {
    sensor.update();
  }

  ws.loop(); //TODO pendant la reconnexion ça fout la merde.....
  serialCommandHandler.handleSerial();
  restCommandHandler.handleClient();
}
