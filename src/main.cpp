// /**
//  * ESP32 position motion control example with magnetic sensor
//  */
#include <Arduino.h>
#include <SimpleFOC.h>
#include <WebSocketsClient.h>
#include <WiFi.h>
#include <secrets.h>

enum AppState {
  CALIBRATING_MIN,
  CALIBRATING_MAX,
  RUNNING,
  STOPPED
};

AppState appState = AppState::STOPPED;

// const int VELOCITY_LIMIT = 40;

// #define PWM_PIN
// MagneticSensorPWM sensor = MagneticSensorPWM(PWM_PIN, 4, 904);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 5);

// Motor instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 15);

// WebSocket client instance
WebSocketsClient ws;

// Variables pour stocker les limites min et max
float minAngle = 0;
float maxAngle = 0;

// angle set point variable
float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  appState = AppState::RUNNING;
  command.scalar(&target_angle, cmd);
}

void doStop(char* cmd) {
  appState = AppState::STOPPED;
}

void doCalibration(char* cmd) {
  appState = AppState::CALIBRATING_MIN;
  target_angle = sensor.getAngle() - (4 * PI);  // Déplacement vers la butée min
  Serial.println("Calibration started " + String(motor.target));
}

void doSetMinAngle(char* cmd) {
  minAngle = sensor.getAngle();
  Serial.println("Min angle set to: " + String(minAngle));
}

void doSetMaxAngle(char* cmd) {
  maxAngle = sensor.getAngle();
  Serial.println("Max angle set to: " + String(maxAngle));
}

void doPrintDebug(char* cmd) {
  Serial.println("Min angle: " + String(minAngle) + " Max angle: " + String(maxAngle) + " Current angle: " + String(sensor.getAngle()) + " Target angle: " + String(target_angle) + " App state: " + String(appState));
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

const int CMD_MIN = 0;
const int CMD_MAX = 100;

float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void executeCommand(Command cmd) {
  if (cmd.action == 'L') {
    Serial.println("CMD: A: " + String(cmd.axis) + " V: " + String(cmd.value) + " I: " + String(cmd.interval));
    float target_position = mapFloat(cmd.value, CMD_MIN, CMD_MAX, minAngle, maxAngle);
    float current_position = sensor.getAngle();
    float current_shaft_position = motor.shaft_angle;  // semble identique à sensor.getAngle()
    float delta_angle = target_position - current_position;
    float velocity = abs(delta_angle / (cmd.interval / 1000.0));
    Serial.println("T: " + String(target_position) + " C(s): " + String(current_position) + " C(m): " + current_shaft_position + " D: " + String(delta_angle) + " V: " + String(velocity));
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
    motor.velocity_limit = velocity;
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
    ws.sendTXT("{ \"identifier\": \"myesp\", \"address\": \"00000000\", \"version\": 0 }");  // Envoie un message initial
  } else if (type == WStype_DISCONNECTED) {
    Serial.println("WebSocket disconnected");
  } else if (type == WStype_ERROR) {
    Serial.println("WebSocket error");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");

  // Serial.print("MOSI: ");
  // Serial.println(MOSI);
  // Serial.print("MISO: ");
  // Serial.println(MISO);
  // Serial.print("SCK: ");
  // Serial.println(SCK);
  // Serial.print("SS: ");
  // Serial.println(SS);

  // Initialisation du serveur WebSocket
  ws.begin("192.168.0.173", 1234, "/");  // PC
  // ws.begin("192.168.0.204", 54817, "/"); //HyperV
  ws.onEvent(onWsEvent);
  ws.setReconnectInterval(5000);

  // enable more verbose output for debugging
  // comment out if not needed
  // SimpleFOCDebug::enable(&Serial);

  // initialise magnetic sensor hardware
  sensor.init();
  // sensor.enableInterrupt(doPWM);
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

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;  // Augmente le filtre (par défaut 0.01)

  // angle P controller https://docs.simplefoc.com/angle_loop
  motor.P_angle.P = 20;  // Commence par diminuer à la moitié (valeur actuelle = 20)

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target angle");
  command.add('S', doStop, "stop");
  command.add('C', doCalibration, "calibrate");
  command.add('M', doSetMinAngle, "set min angle");
  command.add('X', doSetMaxAngle, "set max angle");
  command.add('D', doPrintDebug, "print debug");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
}

// Fonction pour calibrer les limites du moteur
void calibrateLimits() {
  static float lastAngle = sensor.getAngle();
  static unsigned long lastTime = millis();         // Temps du dernier calcul de vitesse
  static unsigned long stallStartTime = 0;          // Temps depuis le dernier "stall" détecté
  const float velocityThreshold = 0.1;              // Seuil de vitesse pour détecter un mouvement insuffisant
  const unsigned long velocityCheckInterval = 200;  // Vérifier la vitesse toutes les 200 ms

  unsigned long currentTime = millis();
  float currentAngle = sensor.getAngle();

  // Si l'intervalle de temps est dépassé, on calcule la vitesse
  if (currentTime - lastTime >= velocityCheckInterval) {
    // Calcul de la vitesse en fonction de l'angle
    float deltaAngle = fabs(currentAngle - lastAngle);
    float deltaTime = (currentTime - lastTime) / 1000.0;  // Temps écoulé en secondes
    float velocity = deltaAngle / deltaTime;              // Vitesse en degrés par seconde
    Serial.println("Velocity: " + String(velocity) + " DeltaTime: " + String(deltaTime) + " DeltaAngle: " + String(deltaAngle));

    // Si la vitesse est inférieure au seuil, on détecte un "stall"
    if (velocity < velocityThreshold) {
      if (stallStartTime == 0) {
        stallStartTime = currentTime;  // On commence à compter le temps du "stall"
      }
      // Si le moteur reste bloqué pendant l'intervalle de vérification, on détecte le "stall"
      if (currentTime - stallStartTime >= velocityCheckInterval) {
        if (appState == CALIBRATING_MIN) {
          minAngle = currentAngle;  // Enregistrer la position minimale
          appState = CALIBRATING_MAX;
          Serial.println("Min angle detected: " + String(minAngle));
          target_angle = currentAngle + 4 * PI;  // Déplacer vers l'autre extrémité pour le calibrage max
          stallStartTime = 0;                    // Réinitialiser le temps de blocage
        } else if (appState == CALIBRATING_MAX) {
          maxAngle = currentAngle;  // Enregistrer la position maximale
          appState = STOPPED;       // Terminer la calibration et mettre en STOPPED
          Serial.println("Max angle detected: " + String(maxAngle));
          target_angle = currentAngle;  // Arrêter le moteur à la position actuelle
          stallStartTime = 0;           // Réinitialiser le temps de blocage
        }
      }
    } else {
      stallStartTime = 0;  // Réinitialiser si le moteur bouge (vitesse suffisante)
    }

    lastTime = currentTime;  // Mettre à jour le temps du dernier calcul de vitesse
  }

  lastAngle = currentAngle;  // Mettre à jour l'angle précédent
}

void loop() {
   ws.loop();
  switch (appState) {
    case CALIBRATING_MIN:
    case CALIBRATING_MAX:
    if (!motor.enabled) {
      motor.enable();
    }
      motor.voltage_limit = 3;     // Réduire la tension pour éviter une force excessive
      motor.velocity_limit = 0.5;  // Vitesse lente pour la calibration
      motor.loopFOC();
      motor.move(target_angle);
      // Serial.println("Moving to target: " + String(target_angle) + " Current angle: " + String(sensor.getAngle()));
      calibrateLimits();  // Vérifier si le moteur est bloqué
      break;
    case RUNNING:
    if (!motor.enabled) {
      motor.enable();
    }
      motor.voltage_limit = 6;    // maximal voltage to be set to the motor
      motor.velocity_limit = 20;  // maximal velocity of the position control
      motor.loopFOC();            // main FOC algorithm function the faster you run this function the better
      motor.move(target_angle);   // Motion control function velocity, position or voltage (defined in motor.controller) this function can be run at much lower frequency than loopFOC() function
      break;
    case STOPPED:
      if (motor.enabled) {
        motor.disable();
      }
      target_angle = sensor.getAngle();
      motor.voltage_limit = 0;
      motor.velocity_limit = 0;
      motor.loopFOC();     // Mainly for updating sensors
      motor.move(target_angle);
      break;
  }

  // user communication
  command.run();
}
