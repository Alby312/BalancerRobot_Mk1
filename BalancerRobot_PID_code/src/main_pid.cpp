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

#define TX_INPUT 27
#define RX_INPUT 26
#define CH_TH 2
#define CH_DIR 1

#define LED 16
#define SCL_PIN 1
#define SDA_PIN 0

#define TIMESTEP 2
#define TIMESTEPRADIO 10
#define PITCH_OFFSET 0
#define ANGLEMAX 60 * DEG_TO_RAD
#define MAX_SPEED 1.0
#define MAX_ROTATION_SPEED 5.0
#define WHEEL_RADIUS 0.02365
#define BATTERY_VOLTAGE 12.0
#define WHEELBASE 0.13 // meters
#define ACC 2
#define SPEEDLIMIT 1.0 // m/s

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

Adafruit_NeoPixel led(1, LED, NEO_GRB + NEO_KHZ800);

BLDCMotor motor1 = BLDCMotor(7, 22.7, 100);
BLDCMotor motor2 = BLDCMotor(7, 22.7, 100);

MagneticSensorMT6701SSI sensor1(SENSOR1_CS);
MagneticSensorMT6701SSI sensor2(SENSOR2_CS);

BLDCDriver3PWM driver1 = BLDCDriver3PWM(IN1_1, IN2_1, IN3_1);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(IN1_2, IN2_2, IN3_2);

AdvancedPID pidStab{0.40, 0.2, 0.030, 0.0};
AdvancedPID pidVel{0.33, 0.3, 0.001, 0.2};
AdvancedPID pidYaw{0.1, 0.01, 0.0, 0.0};

LowPassFilter lpf_speed{0.06};
LowPassFilter lpf_yaw{0.05};
LowPassFilter lpf_cmd1{0.1};
LowPassFilter lpf_cmd2{0.1};

SerialPIO elrsSerial(TX_INPUT, RX_INPUT);
AlfredoCRSF elrs;

volatile bool motors_ready = false;
volatile float sh_target_current_1 = 0.0;
volatile float sh_target_current_2 = 0.0;
volatile float sh_velocity_1 = 0.0;
volatile float sh_velocity_2 = 0.0;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
void mapChannels();
float accelerationRamp(float actual, float target, float acc, float timestep);

// SETUP CORE 1 MOTORS CONTROL
void setup1()
{
  SPI1.setSCK(SENSOR1_CLK);
  SPI1.setRX(SENSOR1_RX);

  sensor1.init(&SPI1);
  sensor2.init(&SPI1);
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  driver1.voltage_power_supply = BATTERY_VOLTAGE;
  driver2.voltage_power_supply = BATTERY_VOLTAGE;

  driver1.pwm_frequency = 50000;
  driver2.pwm_frequency = 50000;
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
  motor1.voltage_sensor_align = 5;
  motor2.voltage_sensor_align = 5;

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

  motor1.zero_electric_angle = 4.00;        // rad
  motor1.sensor_direction = Direction::CCW; // CW or CCW
  motor2.zero_electric_angle = 2.14;        // rad
  motor2.sensor_direction = Direction::CW;  // CW or CCW

  motor1.initFOC();
  motor2.initFOC();

  motors_ready = true;
}

// LOOP CORE 1 MOTORS CONTROL
void loop1()
{
  motor1.loopFOC();
  motor2.loopFOC();

  motor1.move(sh_target_current_1);
  motor2.move(sh_target_current_2);

  sh_velocity_1 = motor1.shaft_velocity;
  sh_velocity_2 = motor2.shaft_velocity;
}

// SETUP CORE 0 ROBOT CONTROL
void setup()
{
  Serial.begin(115200);

  pidStab.setOutputLimits(-0.5, 0.5);
  pidVel.setOutputLimits(-0.3, 0.3);
  pidYaw.setOutputLimits(-0.1, 0.1);

  pidVel.setDerivativeFilter(0.5);

  led.begin();
  led.setBrightness(100);
  led.setPixelColor(0, led.Color(0, 0, 150));
  led.show();

  elrsSerial.begin(CRSF_BAUDRATE, SERIAL_8N1);
  if (!elrsSerial)
    while (1)
      Serial.println("Invalid crsfSerial configuration");
  elrs.begin(elrsSerial);

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
unsigned long lastTimeRadio = 0;
unsigned long lastTimeSensor = 0;
sensors_event_t absolute, gyro;
float speedTarget = 0;
float speedRamped = 0;
float yawTarget = 0;
float pitchAnchor = 0;
float pitchSpeed = 0;

// LOOP CORE 0 ROBOT CONTROL
void loop()
{

  if (millis() > lastTimeLoop + TIMESTEP)
  {
    // time update for cycle
    lastTimeLoop = millis();
    // update the radio commands
    if (lastTimeLoop > lastTimeRadio + TIMESTEPRADIO)
    {
      elrs.update();
      mapChannels();
      lastTimeRadio = millis();
    }

    // update the sensor status
    bno.getEvent(&absolute, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
    float pitchSensorNew = (float)(((absolute.orientation.y) - PITCH_OFFSET) * DEG_TO_RAD);
    float pitchSpeedSensorNew = (float)gyro.gyro.y;

    if (abs(pitchSensorNew - pitchAnchor) > 0.0001f)
    {
      pitchAnchor = pitchSensorNew;
      pitchSpeed = pitchSpeedSensorNew;
      lastTimeSensor = millis();
    }

    unsigned dtSensor = (millis() - lastTimeSensor) / 1000.0f;
    float pitch = pitchAnchor + pitchSpeed * dtSensor;

    float calcCurrent = 0;
    float calcCurrentDiff = 0;
    float pitchTarget = 0;

    if (abs(pitch) < ANGLEMAX)
    {
      speedRamped = accelerationRamp(speedRamped, speedTarget, ACC, TIMESTEP / 1000.0f);
      // limit max wheel speed
      float speedRotation = abs(yawTarget) * WHEELBASE / 2.0f;
      float maxLinearSpeed = SPEEDLIMIT - speedRotation;
      speedRamped = constrain(speedRamped, -maxLinearSpeed, maxLinearSpeed);

      // first PID
      float linearSpeed = lpf_speed((motor1.shaft_velocity + motor2.shaft_velocity) / 2) * WHEEL_RADIUS;
      pitchTarget = -pidVel.run(linearSpeed, speedRamped);
      // pitchTarget = 0;

      // second PID
      calcCurrent = pidStab.run(pitch, pitchTarget, 0.0, pitchSpeed);
      calcCurrentDiff = lpf_yaw(pidYaw.run(((motor1.shaft_velocity - motor2.shaft_velocity) * WHEEL_RADIUS) / WHEELBASE, yawTarget));
    }
    else
    {
      // reset PIDs and motor speed if not up
      sh_target_current_1 = 0;
      sh_target_current_2 = 0;
      led.setPixelColor(0, led.Color(150, 150, 0));
      led.show();
      delay(2000);
      pidStab.reset();
      pidVel.reset();
      pidYaw.reset();
      speedRamped = 0;
    }

    // update variable for core 1
    sh_target_current_1 = calcCurrent + calcCurrentDiff;
    sh_target_current_2 = calcCurrent - calcCurrentDiff;

    if (abs(pitch) < 0.2)
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
    speedTarget = lpf_cmd1(mapFloat(elrs.getChannel(CH_TH), 1000, 2000, MAX_SPEED, -MAX_SPEED));
    yawTarget = lpf_cmd2(mapFloat(elrs.getChannel(CH_DIR), 1000, 2000, MAX_ROTATION_SPEED, -MAX_ROTATION_SPEED));
    if (abs(speedTarget) < MAX_SPEED * 0.05)
      speedTarget = 0;
    if (abs(yawTarget) < MAX_ROTATION_SPEED * 0.05)
      yawTarget = 0;
  }
  else
  {
    speedTarget = 0;
    speedRamped = 0;
    yawTarget = 0;
  }
}

float accelerationRamp(float actual, float target, float acc, float timestep)
{
  float output;
  float step = acc * timestep;
  float diff = target - actual;

  if (abs(diff) <= step)
    return target;
  else if (diff > 0)
    return actual + step;
  else
    return actual - step;
}