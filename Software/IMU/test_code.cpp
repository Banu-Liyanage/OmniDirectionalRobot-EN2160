#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

#define N 5 //setup the moving average filter 
float ax_buffer[N], ay_buffer[N], az_buffer[N]; //buffers to store the values
int index = 0;

float movingAvg(float* buffer) {
  float sum = 0;
  for (int i = 0; i < N; i++) sum += buffer[i];
  return sum / N;
}

float vx = 0, vy = 0;  
unsigned long lastTime = 0;

// Drift correction parameters
const float ACCEL_THRESHOLD = 0.05;  // Threshold for "stationary" detection (m/s²)
const float VELOCITY_DECAY = 0.95;   // Velocity decay factor when stationary
const float ZERO_VELOCITY_THRESHOLD = 0.02; // Velocity threshold for zero correction

// Bias correction variables
float ax_bias = 0, ay_bias = 0;
bool bias_calibrated = false;
int calibration_samples = 0;
const int CALIBRATION_COUNT = 50;

void setup(void)
{ 
  lastTime = millis(); 
  Serial.begin(115200);
  Serial.println("X\tY\tZ");
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); 
  Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
  
  // Calibrate bias
  Serial.println("Calibrating bias... keep sensor still for 5 seconds");
  calibrateBias();
  Serial.println("Bias calibration complete!");
}

void calibrateBias() {
  float ax_sum = 0, ay_sum = 0;
  
  for (int i = 0; i < CALIBRATION_COUNT; i++) {
    sensors_event_t lin;
    bno.getEvent(&lin, Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    ax_sum += lin.acceleration.x;
    ay_sum += lin.acceleration.y;
    
    delay(100);
  }
  
  ax_bias = ax_sum / CALIBRATION_COUNT;
  ay_bias = ay_sum / CALIBRATION_COUNT;
  bias_calibrated = true;
  
  Serial.print("Bias - X: "); Serial.print(ax_bias);
  Serial.print(" Y: "); Serial.println(ay_bias);
}

void loop(void)
{
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);

  sensors_event_t lin; //create an event for linear acc named lin 
  bno.getEvent(&lin, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Apply bias correction
  float ax_raw = lin.acceleration.x - ax_bias;
  float ay_raw = lin.acceleration.y - ay_bias;
  float az_raw = lin.acceleration.z;

  // Update buffers
  ax_buffer[index] = ax_raw;
  ay_buffer[index] = ay_raw;
  az_buffer[index] = az_raw;
  index = (index + 1) % N;

  // Moving average filter 
  float ax_avg = movingAvg(ax_buffer);
  float ay_avg = movingAvg(ay_buffer);
  float az_avg = movingAvg(az_buffer);
  
  // Apply deadband filter
  if (abs(ax_avg) < ACCEL_THRESHOLD) ax_avg = 0;
  if (abs(ay_avg) < ACCEL_THRESHOLD) ay_avg = 0;
  
  // Print filtered values
  Serial.print(ax_avg); Serial.print('\t');
  Serial.print(ay_avg); Serial.print('\t');
  Serial.println(az_avg); Serial.print('\t');

  delay(25);      // 50 samples s‑1 (adjust as you like)

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // convert ms to seconds
  lastTime = currentTime;

  // Integrate acceleration to get velocity
  vx += ax_avg * dt;
  vy += ay_avg * dt;

  // Check if sensor appears to be stationary
  float total_accel = sqrt(ax_avg*ax_avg + ay_avg*ay_avg);
  
  if (total_accel < ACCEL_THRESHOLD) {
    // Apply velocity decay when stationary
    vx *= VELOCITY_DECAY;
    vy *= VELOCITY_DECAY;
    
    // Zero out very small velocities
    if (abs(vx) < ZERO_VELOCITY_THRESHOLD) vx = 0;
    if (abs(vy) < ZERO_VELOCITY_THRESHOLD) vy = 0;
  }

  // Print velocity
  Serial.print("Vel - X: "); Serial.print(vx); 
  Serial.print(" Y: "); Serial.println(vy);
}
void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}
