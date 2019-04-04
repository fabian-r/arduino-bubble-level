#include <Wire.h>

/**
 * Controlling software of an electronic bubble level, designed for mounting on a hat :)
 * It uses 9 LEDs in an L-shape to display the 2-dimensional position of the bubble based on values read from a MPU-6050 accelerometer via I2C.
 * The code for interacting with the accelerometer is heavily inspired by the examples at https://playground.arduino.cc/Main/MPU-6050/#short
 */

/**
 * All LEDs in ascending order. LEDs are expected to be active on a LOW signal.
 */
#define LED0 A0
#define LED1 A1
#define LED2 A2
#define LED3 4
#define LED4 A3
#define LED5 5
#define LED6 6
#define LED7 7
#define LED8 8

/**
 * The total number of LEDs, this should be odd.
 */
#define NUM_LEDS 9

/**
 * LEDs are considered in an L shape with a side length of NUM_LEDS_PER_DIM. This should be ceil(NUM_LEDS/2)
 */
#define NUM_LEDS_PER_DIM 5

/**
 * Position of the "middle" LED per dimension. This should be floor(NUM_LEDS_PER_DIM/2)
 */
#define DIM_MIDDLE 2

/**
 * An array containing all LEDs for dynamic indexing.
 */
const uint8_t all_leds[NUM_LEDS] = {
  LED0, LED1, LED2, LED3, LED4, LED5, LED6, LED7, LED8
};

/**
 * This determines the poll rate to the acceleration sensor.
 */
#define DELAY_TIME 70

/**
 * This determines the speed of the running light on bootup.
 */
#define BOOT_DELAY 100

/**
 * Offsets added to the sensor readings to make up for decentered placement.
 */
#define XOFF 0
#define YOFF 0

/**
 * Making this value larger makes the system less sensitive for sensor readings.
 * Appropriate values depend on the value of IMPREC.
 */
#define CUTOFF 50

/**
 * Increasing this value reduces the sensitivity to measurement noise.
 */
#define IMPREC 50

/**
 * Dimension arrays for write_led_vals().
 * Set entry to a value >= 1 to signal that the led should glow.
 * Notice that the first LED of the y dimension is the same as the last one of the x dimension.
 *
 * Better not have too many set to 1 at the same time to not blast the maximal total current rating of the Arduino.
 */
uint8_t x_led_vals[5] = { 0,0,0,0,0 };
uint8_t y_led_vals[5] = { 0,0,0,0,0 };

/**
 * I2C address of the MPU-6050 (magic number)
 */
const int MPU_addr=0x68;

/**
 * Variables where the sensor readings for the accelerometer are placed by fetch_acc()
 */
int16_t AcX, AcY;

/**
 * Arcane initialization code for the accelerometer.
 */
void setup_acc(void) {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}

/**
 * Read the accelerometer data from the sensor.
 */
void fetch_acc(void) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,4,true);  // request a total of 4 registers
  AcX = Wire.read() << 8;
  AcX = AcX | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8;
  AcY = AcY | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  // more could be read here.
}

/**
 * Set all LED pins as outputs and turn all LEDs off.
 */
void setup_leds(void) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    pinMode(all_leds[i], OUTPUT);
  }
  for (int i = 0; i < NUM_LEDS; ++i) {
    digitalWrite(all_leds[i], HIGH);
  }
}

/**
 * Write the values from the dimension arrays to the LEDs.
 * Better not have too many set to 1 at the same time to not blast the maximal total current rating of the Arduino.
 */
void write_led_vals(void) {
  for (int i = 0; i < NUM_LEDS_PER_DIM - 1; ++i) {
      digitalWrite(all_leds[i], (x_led_vals[i] > 0) ? LOW : HIGH);
  }
  digitalWrite(all_leds[NUM_LEDS_PER_DIM - 1], (x_led_vals[NUM_LEDS_PER_DIM - 1] + y_led_vals[0] > 0) ? LOW : HIGH);
  for (int i = NUM_LEDS_PER_DIM; i < NUM_LEDS; ++i) {
      digitalWrite(all_leds[i], (y_led_vals[i - (NUM_LEDS_PER_DIM - 1)] > 0) ? LOW : HIGH);
  }
}

/**
 * Clear all values in the dimension arrays. Does not write to the LEDs themselves.
 */
void clear_led_vals(void) {
  for (int i = 0; i < NUM_LEDS_PER_DIM; ++i) {
      x_led_vals[i] = 0;
      y_led_vals[i] = 0;
  }
}

/**
 * Clears the LED vals and makes sure that the LED at position idx is glowing.
 */
void turn_on_single_led(int idx) {
    clear_led_vals();
    write_led_vals();
    digitalWrite(all_leds[idx], LOW);
}

/**
 * Have a light run forwards and backwards over the LEDs for fun and functionality testing.
 */
void run_leds(void) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    turn_on_single_led(i);
    delay(BOOT_DELAY);
  }
  for (int i = 1; i < NUM_LEDS; ++i) {
    turn_on_single_led((NUM_LEDS - 1) - i);
    delay(BOOT_DELAY);
  }
  clear_led_vals();
  write_led_vals();
}

/**
 * Compute which LED should be enabled for the sensor reading val, adding the given offset.
 */
int16_t compute_bucket(int16_t val, int16_t offset) {
  val = val + offset;
  int16_t res = ((val/IMPREC) * NUM_LEDS_PER_DIM) / CUTOFF;
  if (res > DIM_MIDDLE) {
    res = DIM_MIDDLE;
  }
  if (res < -DIM_MIDDLE) {
    res = -DIM_MIDDLE;
  }
  return (-res) + DIM_MIDDLE;
}

void setup(void) {
  setup_leds();
  run_leds();
  setup_acc();

}

/**
 * Current locations of the bubble markers.
 */
int16_t current_x = DIM_MIDDLE;
int16_t current_y = DIM_MIDDLE;

#define SIGNUM(V) ((V) == 0 ? 0 : ((V) < 0 ? -1 : 1))

void loop(void) {
  fetch_acc();
  int16_t target_x = compute_bucket(AcX, XOFF);
  int16_t target_y = compute_bucket(AcY, YOFF);
//  Serial.print("AcX = "); Serial.print(effective_acx);
//  Serial.print(" | AcY = "); Serial.println(effective_acy);

  // avoid rapidly jumping LEDs to make transitions look smoother
  current_x += SIGNUM(target_x - current_x);
  current_y += SIGNUM(target_y - current_y);

  clear_led_vals();
  x_led_vals[current_x] = 1;
  y_led_vals[current_y] = 1;
  write_led_vals();
  delay(DELAY_TIME);
}
