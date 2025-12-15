#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_NeoPixel.h>
#include <utility/imumaths.h>
#include <AlfredoCRSF.h>
#include <AdvancedPID.h>

#define IN1_1 3
#define IN2_1 4
#define IN3_1 5
#define IN1_2 10
#define IN2_2 9
#define IN3_2 8
#define SENSOR1_CS 11
#define SENSOR2_CS 13
#define SENSOR1_CLK 14
#define SENSOR1_RX 12

#define RX_INPUT 15
#define TX_INPUT 26
#define CH_TH 2
#define CH_DIR 1

#define LED 16
#define SCL_PIN 1
#define SDA_PIN 0

#define PITCH_OFFSET 0
#define ANGLEMAX 60 * DEG_TO_RAD
#define MAX_SPEED 50
#define MAX_ROTATION_SPEED 30
#define V_DEADZONE 0.0005

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

Adafruit_NeoPixel led(1, LED, NEO_GRB + NEO_KHZ800);

BLDCMotor motor1 = BLDCMotor(7, 22.7, 100);
BLDCMotor motor2 = BLDCMotor(7, 22.7, 100);

MagneticSensorMT6701SSI sensor1(SENSOR1_CS);
MagneticSensorMT6701SSI sensor2(SENSOR2_CS);

BLDCDriver3PWM driver1 = BLDCDriver3PWM(IN1_1, IN2_1, IN3_1);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(IN1_2, IN2_2, IN3_2);

AdvancedPID pidStab{0.42, 0.5, 0.035, 0.0};
AdvancedPID pidVel{0.004, 0.004, 0.00001, 0.0};
AdvancedPID pidYaw{0.02, 0.0005, 0.0, 0.0};

LowPassFilter lpf_speed{0.02};
LowPassFilter lpf_yaw{0.1};

SerialPIO elrsSerial(TX_INPUT, RX_INPUT);
AlfredoCRSF elrs;

volatile bool motors_ready = false;
volatile double sh_target_voltage_1 = 0.0;
volatile double sh_target_voltage_2 = 0.0;
volatile double sh_velocity_1 = 0.0;
volatile double sh_velocity_2 = 0.0;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
void mapChannels();

// SETUP CORE 1 MOTORS CONTROL
void setup1()
{
  SPI1.setSCK(SENSOR1_CLK);
  SPI1.setRX(SENSOR1_RX);

  sensor1.init(&SPI1);
  sensor2.init(&SPI1);
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  driver1.voltage_power_supply = 12.5;
  driver2.voltage_power_supply = 12.5;
  // link driver
  motor1.linkDriver(&driver1);
  if (!driver1.init())
  {
    Serial.println("Driver1 init failed!");
    return;
  }
  motor2.linkDriver(&driver2);
  if (!driver2.init())
  {
    Serial.println("Driver2 init failed!");
    return;
  }

  // aligning voltage [V]
  motor1.voltage_sensor_align = 1;
  motor2.voltage_sensor_align = 1;

  motor1.torque_controller = TorqueControlType::voltage;
  motor2.torque_controller = TorqueControlType::voltage;

  if (!motor1.init())
  {
    Serial.println("Motor1 init failed!");
    return;
  }
  if (!motor2.init())
  {
    Serial.println("Motor2 init failed!");
    return;
  }

  // motor1.zero_electric_angle = 4.63;        // rad
  // motor1.sensor_direction = Direction::CCW; // CW or CCW
  // motor2.zero_electric_angle = 7.69;        // rad
  // motor2.sensor_direction = Direction::CW;  // CW or CCW

  motor1.initFOC();
  motor2.initFOC();

  motors_ready = true;
}

// LOOP CORE 1 MOTORS CONTROL
void loop1()
{
  motor1.loopFOC();
  motor2.loopFOC();

  motor1.move(sh_target_voltage_1);
  motor2.move(sh_target_voltage_2);

  sh_velocity_1 = motor1.shaft_velocity;
  sh_velocity_2 = motor2.shaft_velocity;
}

// SETUP CORE 0 ROBOT CONTROL
void setup()
{
  Serial.begin(115200);

  pidStab.setOutputLimits(-2, 2);
  pidVel.setOutputLimits(-0.2, 0.2);
  pidYaw.setOutputLimits(-2, 2);

  pidVel.setDerivativeFilter(0.7);
  // pidStab.setIntegralZone(0.3);

  led.begin();
  led.setBrightness(100);
  led.setPixelColor(0, led.Color(0, 0, 150));
  led.show();

  // elrsSerial.begin(CRSF_BAUDRATE, SERIAL_8N1);
  // if (!elrsSerial)
  //   while (1)
  //     Serial.println("Invalid crsfSerial configuration");
  // elrs.begin(elrsSerial);

  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);
  Wire.setClock(400000);

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (10)
      ;
  }
  bno.setExtCrystalUse(true);
  Serial.println("Core 0 Ready. Waiting for Core 1 Motors...");
  while (!motors_ready)
  {
    delay(10);
  }
  led.setPixelColor(0, led.Color(150, 150, 0));
  led.show();
  Serial.println("Motors Ready. Starting Control Loop.");
  delay(2000);
  pidStab.reset();
  pidVel.reset();
  pidYaw.reset();
}

unsigned long lastTimeLoop = 0;
sensors_event_t absolute, gyro;
float speedTarget = 0;
float yawTarget = 0;

// LOOP CORE 0 ROBOT CONTROL
void loop()
{
  if (millis() > lastTimeLoop + 5)
  {
    // time update for cycle
    lastTimeLoop = millis();

    // update the radio commands
    // elrs.update();
    // mapChannels();

    // update the sensor status
    bno.getEvent(&absolute, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
    double pitch = ((absolute.orientation.y) - PITCH_OFFSET) * DEG_TO_RAD;
    double pitch_speed = -gyro.gyro.y;

    double calcVoltage = 0;
    double calcVoltageDiff = 0;
    double pitchTarget = 0;

    if (abs(pitch) < ANGLEMAX)
    {
      // first PID
      double linear_speed = lpf_speed((motor1.shaft_velocity + motor2.shaft_velocity) / 2);
      pitchTarget = -pidVel.run(linear_speed, speedTarget);
      // pitchTarget = 0;

      // second PID
      float gravity_ff = sin(-pitch) * 0.1;
      calcVoltage = pidStab.run(pitch, pitchTarget, 0.0, gyro.gyro.y);
      Serial.print(pitchTarget - pitch);
      Serial.print(" ");
      Serial.println(linear_speed);

      calcVoltageDiff = lpf_yaw(pidYaw.run((motor1.shaft_velocity - motor2.shaft_velocity), yawTarget));
    }
    else
    {
      // reset PIDs and motor speed if not up
      sh_target_voltage_1 = 0;
      sh_target_voltage_2 = 0;
      led.setPixelColor(0, led.Color(150, 150, 0));
      led.show();
      delay(2000);
      pidStab.reset();
      pidVel.reset();
      pidYaw.reset();
    }

    // update variable for core 1
    sh_target_voltage_1 = calcVoltage + calcVoltageDiff;
    sh_target_voltage_2 = calcVoltage - calcVoltageDiff;

    if (abs(pitchTarget - pitch) < 0.02)
    {
      led.setPixelColor(0, led.Color(0, 150, 0));
      led.show();
    }
    else
    {
      led.setPixelColor(0, led.Color(150, 0, 0));
      led.show();
    }
  }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void mapChannels()
{
  if (elrs.isLinkUp())
  {
    speedTarget = mapFloat(elrs.getChannel(CH_TH), 1000, 2000, MAX_SPEED, -MAX_SPEED);
    yawTarget = mapFloat(elrs.getChannel(CH_DIR), 1000, 2000, MAX_ROTATION_SPEED, -MAX_ROTATION_SPEED);
    if (abs(speedTarget) < 1)
      speedTarget = 0;
    if (abs(yawTarget) < 1)
      yawTarget = 0;
  }
  else
  {
    speedTarget = 0;
    yawTarget = 0;
  }
}