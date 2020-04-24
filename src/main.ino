#include "pid.hpp"
#define IBETWEEN(x, a, b) ((a <= x) && (x <= b))
#define EBETWEEN(x, a, b) ((a < x) && (x < b))

#define TAU (2 * PI)
#define DEG_PER_RAD (180.0 / PI)
#define RAD_PER_DEG (PI / 180.0)

#define PEND_A 2
#define PEND_B 3
#define PEND_CPR 600
#define PEND_COUNTS_PER_RADIAN (PEND_CPR / TAU)
#define PEND_RADIANS_PER_COUNT (TAU / PEND_CPR)

#define CART_KP 0
#define CART_KI 0
#define CART_KD 0
#define CART_MIN_OUT -1
#define CART_MAX_OUT 1
#define CART_PID_ACTIVE_RANGE (10 * RAD_PER_DEG)

volatile int pend_count = 0;
volatile float pend_angle = 0;
volatile float pend_angle_last = 0;
volatile float pend_velocity = 0;
volatile float pend_last_timestamp = 0;
volatile byte pend_int_flag = 0;

float main_timestamp_last = 0;

PID cart_pid = PID(CART_KP, CART_KI, CART_KD, CART_MIN_OUT, CART_MAX_OUT);

void setup()
{
  pinMode(PEND_A, INPUT_PULLUP);
  pinMode(PEND_B, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println(pend_count);

  attachInterrupt(0, pendInterrupt, RISING);

  cart_pid.setSetpoint(PI);
}

void loop()
{
  float timestamp = millis() * 1000.0;
  float dt = timestamp - main_timestamp_last;

  if (pend_int_flag == 1)
  {
    pend_int_flag = 0;
  }

  if (IBETWEEN(pend_angle, PI - CART_PID_ACTIVE_RANGE, PI + CART_PID_ACTIVE_RANGE))
  {
    Serial.println("Running PID");
    float output = cart_pid.calculate(pend_angle, dt);
  }

  Serial.print(pend_angle * DEG_PER_RAD);
  Serial.print('\t');
  Serial.println(pend_velocity * DEG_PER_RAD);

  main_timestamp_last = timestamp;
}

void pendInterrupt()
{
  pend_int_flag = 1;
  if (digitalRead(PEND_A) && !digitalRead(PEND_B))
  {
    pend_count++;
  }
  if (digitalRead(PEND_A) && digitalRead(PEND_B))
  {
    pend_count--;
  }
  float timestamp = millis() / 1000.0;
  float dt = timestamp - pend_last_timestamp;
  pend_angle = pend_count * PEND_RADIANS_PER_COUNT;
  pend_velocity = (pend_angle - pend_angle_last) / dt;
  pend_last_timestamp = timestamp;
  pend_angle_last = pend_angle;
}