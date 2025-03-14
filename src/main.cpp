/**
 * ESP32 position motion control with magnetic sensor
 */
#include <Arduino.h>
#include <ArduinoJson.h>
#include <DebugBuilder.h>
#include <ESPAsyncWebServer.h>
#include <RestCommandHandler.h>
#include <SerialCommandHandler.h>
#include <SimpleFOC.h>
#include <WebSocketsClient.h>
#include <WiFi.h>
#include <encoders/as5048a/MagneticSensorAS5048A.h>
#include <secrets.h>
#include <limits>

const int PIN_SENSOR_CS = 5;
const int PIN_MOTOR_PWM1 = 25;
const int PIN_MOTOR_PWM2 = 26;
const int PIN_MOTOR_PWM3 = 27;
const int PIN_MOTOR_ENABLE = 15;

enum AppState {
  CALIBRATING_MANUALLY,
  RUNNING,
  STOPPED
};
const int DEFAULT_MOTOR_VOLTAGE = 12;
const int DEFAULT_DRIVER_VOLTAGE = 12;
const int DEFAULT_MAX_VELOCITY = 80;
const int CMD_MIN = 0;
const int CMD_MAX = 100;

// const String WS_CLIENT_IP = "192.168.0.173";  // PC
// const int WS_CLIENT_PORT = 1234;
const String WS_CLIENT_IP = "192.168.0.204";  // HyperV
const int WS_CLIENT_PORT = 54817;
// const String WS_CLIENT_IP = "192.168.0.100";  // VmWare
// const int WS_CLIENT_PORT = 54817;

WebSocketsClient wsclient;
bool wsClientEnabled = false;
AsyncWebServer server(80);
AsyncCorsMiddleware cors;
AsyncWebSocket wsserver("/position");
SerialCommandHandler serialCommandHandler;
RestCommandHandler restCommandHandler(server);
MagneticSensorAS5048A sensor(PIN_SENSOR_CS, true);

// TODO loop foc dans tâches prioritaire

BLDCMotor motor = BLDCMotor(11);  // GM4108H-120T has 22 poles, so 11 pole pairs
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_MOTOR_PWM1, PIN_MOTOR_PWM2, PIN_MOTOR_PWM3, PIN_MOTOR_ENABLE);

float minAngle = std::numeric_limits<float>::max();
float maxAngle = std::numeric_limits<float>::min();
AppState appState = AppState::STOPPED;
volatile float targetAngle = 0;
volatile float targetVelocity = 0;

float roundFloat2dec(float value) {
  return round(value * 100.0) / 100.0;
}

float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
  if (inMin == inMax)
    return outMin;                         // Évite la division par zéro
  value = constrain(value, inMin, inMax);  // Empêche l'extrapolation
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void setPositionNormalized(int positionNormalized, int velocityNormalized) {
  float cmdAngle = mapFloat(positionNormalized, 0, 100, minAngle, maxAngle);  // cmd value [0-100] -> calibrated angles [minAngle-maxAngle]
  float cmdVelocity = mapFloat(velocityNormalized, 0, 100, 0, DEFAULT_MAX_VELOCITY);
  cmdAngle = roundFloat2dec(cmdAngle);
  cmdVelocity = roundFloat2dec(cmdVelocity);
  noInterrupts();
  targetAngle = cmdAngle;
  targetVelocity = cmdVelocity;
  interrupts();
}

bool isCalibrated() {
  return minAngle != std::numeric_limits<float>::max() && maxAngle != std::numeric_limits<float>::min();
}

/**
 * Set position with normalized values (0-100)
 */
String doSetPosition(int positionNormalized, int velocityNormalized) {
  if (!isCalibrated()) {
    return "Motor not calibrated yet";
  }
  appState = AppState::RUNNING;
  setPositionNormalized(positionNormalized, velocityNormalized);
  return "Target normalized: " + String(positionNormalized) + " Target raw: " + String(targetAngle) + " velocity normalized: " + String(velocityNormalized) + " velocity raw: " + String(targetVelocity);
}

String doGetRestRoutes() {
  return restCommandHandler.getRoutesList();
}

String doGetSerialCommands() {
  return serialCommandHandler.getCommandsList();
}

String doGetDebug() {
  return "Min angle: " + String(minAngle) + " Max angle: " + String(maxAngle) + " Current angle: " + String(sensor.getAngle()) + " Target angle: " + String(targetAngle) + " App state: " + String(appState);
}

String doStop() {
  appState = AppState::STOPPED;
  return "STOPPED";
}

String doRun() {
  appState = AppState::RUNNING;
  return "RUNNING | Min angle: " + String(minAngle) + " Max angle: " + String(maxAngle) + " Current angle: " + String(sensor.getAngle()) + " Target angle: " + String(targetAngle);
}

String doCalibrate() {
  appState = AppState::CALIBRATING_MANUALLY;
  minAngle = std::numeric_limits<float>::max();  // Valeur initiale très haute
  maxAngle = std::numeric_limits<float>::min();  // Valeur initiale très basse
  return "CALIBRATING_MANUALLY | Target angle:" + String(targetAngle) + " Sensor angle:" + String(sensor.getAngle());
}

String doStartWsClient() {
  if (wsClientEnabled) {
    return "WebSocket client already connected";
  }
  wsClientEnabled = true;
  wsclient.begin(WS_CLIENT_IP, WS_CLIENT_PORT, "/");
  return "WebSocket client connecting to " + WS_CLIENT_IP + ":" + String(WS_CLIENT_PORT);
}

String doStopWsClient() {
  if (!wsClientEnabled) {
    return "WebSocket client already disconnected";
  }
  wsClientEnabled = false;
  wsclient.disconnect();
  return "WebSocket client disconnected";
}

// Fonction pour analyser la commande reçue
struct Command {
  String rawCommand;
  char action;
  int axis;
  int value;
  int interval;
};

const float NOTIFICATTION_CHANGE_PERCENTAGE = 0.005f;  // 1% de changement d'angle pour envoyer une notification

void notifyAngleChange() {
  static unsigned long lastNotification = 0;
  if (millis() - lastNotification < 100) {  // Limiter les notifications à 10 Hz
    return;
  }
  lastNotification = millis();
  static float lastAngle = -1;
  float currentAngle = sensor.getAngle();
  float threshold = abs(minAngle - maxAngle) * NOTIFICATTION_CHANGE_PERCENTAGE;
  if (abs(currentAngle - lastAngle) > threshold) {  // Seulement envoyer si l'angle a changé de plus de 0.01 degré
    int angleNormalized = mapFloat(currentAngle, minAngle, maxAngle, CMD_MIN, CMD_MAX);
    int angleNormalizedRounded = round(angleNormalized);
    wsserver.textAll(String(angleNormalizedRounded));
    lastAngle = currentAngle;
  }
}

Command parseCommand(const String& input) {
  // Expression régulière pour extraire les données
  // Format attendu : [Action][Axis][Value][Type][Interval]
  if (input.length() >= 7) {  // La commande doit être d'une longueur suffisante
    Command cmd;
    cmd.rawCommand = input;
    cmd.action = input.charAt(0);               // 'L'
    cmd.axis = input.charAt(1) - '0';           // '0' -> 0
    cmd.value = input.substring(2, 4).toInt();  // '50' -> 50
    cmd.interval = input.substring(5).toInt();  // '5000' -> 5000
    return cmd;
  } else {
    Command cmd = {input, '\0', -1, -1, -1};  // Valeurs par défaut pour une commande invalide
    return cmd;
  }
}

void executeCommand(Command cmd) {  // TODO fonction doMove() qui prend le minimum en paramètre et est appelée à la place de doTargetNormalized() ?
  if (!isCalibrated()) {
    Serial.println("CMD: " + cmd.rawCommand + " ignored: Motor not calibrated yet");
    return;
  }
  if (appState != AppState::RUNNING) {
    Serial.println("CMD: " + cmd.rawCommand + " ignored: Motor not running");
    return;
  }
  if (cmd.action == 'L' && cmd.axis == 0) {
    sensor.update();
    float cmdAngle = mapFloat(cmd.value, CMD_MIN, CMD_MAX, minAngle, maxAngle);  // cmd value [0-100] -> calibrated angles [minAngle-maxAngle]
    float currentAngle = sensor.getAngle();
    float deltaPosition = cmdAngle - currentAngle;
    float cmdVelocity = abs(deltaPosition / (cmd.interval / 1000.0));  // cmd interval (ms) -> velocity (rad/s)
    cmdVelocity = roundFloat2dec(cmdVelocity);
    cmdAngle = roundFloat2dec(cmdAngle);
    Serial.println("CMD: " + cmd.rawCommand + " A: " + String(cmd.axis) + " V: " + String(cmd.value) + " I: " + String(cmd.interval) + " // T: " + String(cmdAngle) + " C: " + String(currentAngle) + " Shaft: " + String(motor.shaft_angle) + " D: " + String(deltaPosition) + " V: " + String(cmdVelocity));

    // S'assurer que la vitesse ne dépasse pas la limite de vitesse maximale
    if (cmdVelocity > DEFAULT_MAX_VELOCITY) {
      cmdVelocity = DEFAULT_MAX_VELOCITY;
    }

    // Définir la vitesse et l'angle cible
    noInterrupts();
    targetAngle = cmdAngle;
    targetVelocity = cmdVelocity;
    interrupts();
  }
}

void onWsServerEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("WebSocket client disconnected");
  } else if (type == WS_EVT_ERROR) {
    Serial.println("WebSocket error");
  } else if (type == WS_EVT_PONG) {
    Serial.println("WebSocket pong");
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      String input = String((char*)data);

      if (!isCalibrated()) {
        Serial.println("CMD: " + input + " ignored: Motor not calibrated yet");
        return;
      }
      if (appState != AppState::RUNNING) {
        Serial.println("CMD: " + input + " ignored: Motor not running");
        return;
      }

      // Parse JSON command
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, input);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }
      if (doc["position"].is<int>() && doc["velocity"].is<int>()) {
        int cmdPosition = doc["position"];
        int cmdVelocity = doc["velocity"];
        setPositionNormalized(cmdPosition, cmdVelocity);
      } else {
        Serial.println("Invalid command received: " + input);
      }
    }
  }
}

// Fonction pour gérer les messages reçus via WebSocket
void onWsEvent(WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    String input = String((char*)payload);
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
    Command cmd = parseCommand(input);
    executeCommand(cmd);
  } else if (type == WStype_CONNECTED) {
    Serial.println("WebSocket connected");
    wsclient.sendTXT("{ \"identifier\": \"myesp\", \"address\": \"00000000\", \"version\": 0 }");  // Envoie un message initial
  } else if (type == WStype_DISCONNECTED) {
    Serial.println("WebSocket disconnected");
  } else if (type == WStype_ERROR) {
    Serial.println("WebSocket error");
  }
}

std::vector<DebugField> debugFields = {
    {"angle", true, [] { return sensor.getAngle(); }},
    {"targetAngle", true, [] { return targetAngle; }},
    {"state", true, [] { return appState; }},
    {"minAngle", true, [] { return minAngle; }},
    {"maxAngle", true, [] { return maxAngle; }},
    {"shaft_velocity", false, [] { return motor.shaft_velocity; }},
    {"voltageQ", false, [] { return motor.voltage.q; }},
    // {"voltageD", false, [] { return motor.voltage.d; }},
    // {"currentQ", false, [] { return motor.current.q; }},
    // {"currentD", false, [] { return motor.current.d; }},
    {"voltage_limit", true, [] { return motor.voltage_limit; }},
    {"velocity_limit", true, [] { return motor.velocity_limit; }},
    {"targetVelocity", true, [] { return targetVelocity; }},
    {"calibrated", true, [] { return isCalibrated(); }},
    {"wsClients", true, [] { return wsserver.count(); }},
    // {"motorVoltageLimit", true, [] { return motorVoltageLimit; }},
    // {"driverVoltage", true, [] { return driver.voltage_power_supply; }},
    // {"motorEnabled", true, [] { return motor.enabled; }},
    // {"motorShaftAngle", false, [] { return motor.shaft_angle; }},
    // {"motorShaftVelocity", false, [] { return motor.shaft_velocity; }},
    // {"motorTargetAngle", false, [] { return motor.target; }},
    // {"motorCurrentLimit", false, [] { return motor.current_limit; }},
    // {"motorPhaseResistance", false, [] { return motor.phase_resistance; }},
    // {"motorPhaseInductance", false, [] { return motor.phase_inductance; }},
    // {"motorKV", false, [] { return motor.KV_rating; }},
    // {"motorFOCModulation", false, [] { return motor.foc_modulation; }},
    // {"motorLPFVelocityTf", true, [] { return motor.LPF_velocity.Tf; }},
    // {"motorPIDVelocityP", true, [] { return motor.PID_velocity.P; }},
    // {"motorPIDVelocityI", true, [] { return motor.PID_velocity.I; }},
    // {"motorPIDVelocityD", true, [] { return motor.PID_velocity.D; }},
    // {"motorPAngleP", true, [] { return motor.P_angle.P; }},
    // {"motorPAngleI", true, [] { return motor.P_angle.I; }},
    // {"motorPAngleD", true, [] { return motor.P_angle.D; }}
};
DebugBuilder debugBuilder(debugFields);

TaskHandle_t WebSocketTaskHandle;
TaskHandle_t SecondaryTaskHandle;

void SecondaryTask(void* parameter) {
  while (1) {
    serialCommandHandler.handleSerial();
    restCommandHandler.handleClient();
    notifyAngleChange();
    if (debugBuilder.hasChanged()) {  // 110 us
      Serial.println(debugBuilder.buildJson());
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // 1000ms
  }
}

void WebSocketTask(void* parameter) {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi! " + WiFi.localIP().toString());
  delay(1000);

  cors.setOrigin("*");
  server.addMiddleware(&cors);
  server.addHandler(&wsserver);
  wsserver.onEvent(onWsServerEvent);
  server.begin();
  Serial.println("✅ HTTP server started: http://" + WiFi.localIP().toString() + ":" + "PORT");  // TODO charger port depuis settings ? virer du constructeur
  wsclient.onEvent(onWsEvent);
  wsclient.setReconnectInterval(5000);

  while (1) {
    if (wsClientEnabled) {
      wsclient.loop();
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);  // 10ms
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2);

  // Initialisation du serveur WebSocket
  xTaskCreatePinnedToCore(
      WebSocketTask,         // Fonction de la tâche
      "WebSocketTask",       // Nom de la tâche
      10000,                 // Taille de la pile (10k)
      NULL,                  // Paramètre (aucun ici)
      1,                     // Priorité (1 = normale)
      &WebSocketTaskHandle,  // Handle pour la gestion de la tâche
      0                      // Boucle principale tourne sur core 1
  );

  xTaskCreatePinnedToCore(
      SecondaryTask,         // Fonction de la tâche
      "SecondaryTask",       // Nom de la tâche
      10000,                 // Taille de la pile (10k)
      NULL,                  // Paramètre (aucun ici)
      1,                     // Priorité (1 = normale)
      &SecondaryTaskHandle,  // Handle pour la gestion de la tâche
      0                      // Boucle principale tourne sur core 1
  );

  // SimpleFOCDebug::enable(&Serial);   // enable more verbose output for debugging
  // motor.useMonitoring(Serial);

  sensor.init();

  driver.voltage_power_supply = DEFAULT_DRIVER_VOLTAGE;  // power supply voltage [V]
  driver.init();

  // control loop type and torque mode
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;  // choose FOC modulation (optional)
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle;
  // motor.motion_downsample = 0.0;

  // velocity loop PID
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20.0;
  motor.PID_velocity.D = 0.0;
  // motor.PID_velocity.output_ramp = 1000.0;
  // motor.PID_velocity.limit = 12.0;
  // Low pass filtering time constant // default 0.01 - the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;
  // angle loop PID
  motor.P_angle.P = 20.0;  // angle P controller https://docs.simplefoc.com/angle_loop
  // motor.P_angle.I = 0.0;
  // motor.P_angle.D = 0.0;
  // motor.P_angle.output_ramp = 0.0;
  // motor.P_angle.limit = 20.0;
  // Low pass filtering time constant
  // motor.LPF_angle.Tf = 0.0;
  // current q loop PID
  // motor.PID_current_q.P = 3.0;
  // motor.PID_current_q.I = 300.0;
  // motor.PID_current_q.D = 0.0;
  // motor.PID_current_q.output_ramp = 0.0;
  // motor.PID_current_q.limit = 12.0;
  // Low pass filtering time constant
  // motor.LPF_current_q.Tf = 0.005;
  // current d loop PID
  // motor.PID_current_d.P = 3.0;
  // motor.PID_current_d.I = 300.0;
  // motor.PID_current_d.D = 0.0;
  // motor.PID_current_d.output_ramp = 0.0;
  // motor.PID_current_d.limit = 12.0;
  // Low pass filtering time constant
  // motor.LPF_current_d.Tf = 0.005;
  // Limits
  motor.velocity_limit = DEFAULT_MAX_VELOCITY;  // default 20
  motor.voltage_limit = DEFAULT_MOTOR_VOLTAGE;
  motor.current_limit = 2.0;
  // sensor zero offset - home position
  // motor.sensor_offset = 0.0;
  // sensor zero electrical angle
  // this parameter enables skipping a part of initFOC
  // general settings
  // motor phase resistance
  // motor.phase_resistance = 0.189;
  // pwm modulation settings
  // motor.foc_modulation = FOCModulationType::SinePWM;
  // motor.modulation_centered = 1.0;

  // Devrait atteindre 567rpm et 60rad/s en théorie
  // 20V et 1,5A max d'après specs
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
  serialCommandHandler.registerCommand<int, int>("position", {"position", "velocity"}, doSetPosition);
  serialCommandHandler.registerCommand("debug", doGetDebug);
  serialCommandHandler.registerCommand("help", doGetSerialCommands);
  serialCommandHandler.registerCommand("fake", [] { minAngle = -3*PI; maxAngle = 3*PI; return "debug angles set to -3pi +3pi"; });

  // Register REST API routes
  restCommandHandler.registerCommand("stop", HTTP_GET, doStop);
  restCommandHandler.registerCommand("calibrate", HTTP_GET, doCalibrate);
  restCommandHandler.registerCommand("run", HTTP_GET, doRun);
  restCommandHandler.registerCommand<int, int>("position", HTTP_POST, {"position", "velocity"}, doSetPosition);
  restCommandHandler.registerCommand("debug", HTTP_GET, doGetDebug);
  restCommandHandler.registerCommand("help", HTTP_GET, doGetRestRoutes);
  restCommandHandler.registerCommand("startWsClient", HTTP_GET, doStartWsClient);
  restCommandHandler.registerCommand("stopWsClient", HTTP_GET, doStopWsClient);
}

void loop() {
  switch (appState) {
    case CALIBRATING_MANUALLY: {
      if (motor.enabled) {
        motor.disable();
      }
      float currentAngle = sensor.getAngle();
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

      break;
    }
    case STOPPED: {
      if (motor.enabled) {
        motor.disable();
      }
      break;
    }
  }

  // main FOC algorithm function the faster you run this function the better //TODO à voir si loopFoc doit être fait avant les trucs dans le switch
  // Even if motor is stopped, For updating sensors
  if (motor.enabled) {
    motor.voltage_limit = DEFAULT_MOTOR_VOLTAGE;  // maximal voltage to be set to the motor
    motor.velocity_limit = targetVelocity;        // maximal velocity of the position control
    motor.loopFOC();
    motor.move(targetAngle);  // Motion control function velocity, position or voltage (defined in motor.controller) this function can be run at much lower frequency than loopFOC() function
  } else {
    motor.voltage_limit = 0;
    motor.velocity_limit = 0;
    sensor.update();
    targetAngle = sensor.getAngle();  // Set target angle as the current angle, for not "jumping" when starting
  }
}
