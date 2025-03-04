#include <SimpleFOC.h>
#include <encoders/as5048a/MagneticSensorAS5048A.h>
const int PIN_SENSOR_CS = 5;
MagneticSensorAS5048A sensor(PIN_SENSOR_CS, true);
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 15);  // 3 pwm pins + enable pin

// include commander interface
Commander command = Commander(Serial);
void doMotor(char* cmd) {
  command.motor(&motor, cmd);
}

void setup() {
  Serial.begin(115200);

  ////////////////
  sensor.init();

  driver.voltage_power_supply = 20;  // power supply voltage [V]
  driver.init();

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;  // choose FOC modulation (optional)
  motor.controller = MotionControlType::angle;               // set motion control loop to be used
  motor.PID_velocity.P = 0.2f;                               // velocity PI controller parameters
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.03;  // default 0.01 velocity low pass filtering time constant  - the lower the less filtered
  motor.P_angle.P = 20;          // angle P controller https://docs.simplefoc.com/angle_loop

  // motor.current_limit = 2; // Amps - default 0.2Amps
  // Devrait atteindre 567rpm et 60rad/s en théorie
  //  20V et 1,5A max d'après specs
  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);
  motor.init();     // initialize motor
  motor.initFOC();  // align sensor and start FOC // TODO deplacer ailleurs pour le lancer que sur demande à la calibration

  ////////////////
  // add the motor to the commander interface
  // The letter id (here 'M') of the motor
  char motor_id = 'M';
  command.add(motor_id, doMotor, "motor");
  // tell the motor to use the monitoring
  motor.useMonitoring(Serial);
  // configuring the monitoring to be well parsed by the webcontroller

  // command.verbose = VerboseMode::machine_readable;  // can be set using the webcontroller - optional
}
void loop() {
  motor.loopFOC();
  motor.move();  // Par exemple 2 rad/s, selon ton mode de contrôle

  // real-time monitoring calls
  motor.monitor();
  // real-time commander calls
  command.run();
}