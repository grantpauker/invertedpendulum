#define PI 3.141592653589793238
#define TAU (2 * PI)
#define DEG_PER_RAD (180 / PI)
#define RAD_PER_DEG (PI / 180)

#define PEND_A 2
#define PEND_B 3
#define PEND_CPR 600
#define PEND_COUNTS_PER_RADIAN (PEND_CPR / TAU)
#define PEND_RADIANS_PER_COUNT (TAU / PEND_CPR)

volatile int pend_count = 0;
volatile float pend_angle = 0;
volatile float pend_angle_last = 0;
volatile float pend_velocity = 0;
volatile float pend_last_timestamp = 0;
volatile byte pend_int_flag = 0;

void setup()
{
  pinMode(PEND_A, INPUT_PULLUP);
  pinMode(PEND_B, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println(pend_count);

  attachInterrupt(0, pendInterrupt, RISING);
}

void loop()
{
  if (pend_int_flag == 1)
  {
    pend_int_flag = 0;
  }
  Serial.print(pend_angle * DEG_PER_RAD);
  Serial.print('\t');
  Serial.println(pend_velocity * DEG_PER_RAD);
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