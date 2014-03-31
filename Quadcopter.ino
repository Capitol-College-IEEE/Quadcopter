#include <Servo.h>
#include <XBee.h>
#include <Wire.h>
#include <MMA8452Q.h>
#include <ITG3200.h>

/******************************************************************************
/* Data & Variables
/*/

// cosine and sine of the angular offset of the sensors (~25 deg)
#define COS 0.906
#define SIN 0.423

#define BAUD 9600










/******************************************************************************
/* Setup & Loop
/*/

void setup() {
  initComms();
  while (!Serial);
  
  Serial.println("Initializing");
  
  Wire.begin();
  
  initProps(9, 10, 11, 6); // don't change
  initStructs();
  
  updateTarget();
  updateSignals();
  updateMotors();
  
  delay(1000);
  
  initGyro();
  initAccel();
  
  Serial.println("Initialization Complete");
  Serial.println("-----------------------");
}

void loop() {
  checkComms();
  updateSensors();
  updateTarget();
  updateSignals();
  updateMotors();
  delay(250);
}










/******************************************************************************
/* Propellers
/*/

struct {
  Servo north, west, south, east;
} props;

void initProps(int a, int b, int c, int d) {
  props.north.attach(a);
  props.east.attach(b);
  props.south.attach(c);
  props.west.attach(d);
}

void updateProps() {
  props.north.write(signals.north);
  props.east.write(signals.east);
  props.south.write(signals.south);
  props.west.write(signals.west);
}










/******************************************************************************
/* Gyroscope
/*/

ITG3200 gyro = ITG3200();

struct {
  float x, y, z;
} _gyro_data[3] = {0,0,0,0,0,0,0,0,0};

uint8_t _gyro_index = 0;

#define gyro_next() (_gyro_index++)
#define gyro_data(offset) (_gyro_data + (_gyro_index + offset + 2) % 3)

void initGyro() {
  Serial.print("Initializing gyro... ");
  gyro.init(ITG3200_ADDR_AD0_HIGH);
  Serial.print("Calibrating gyro... ");
  gyro.zeroCalibrate(500, 2);
  Serial.println("done");
  
}

void updateGyro() {
  float x, y, z;
  
  if (gyro.isRawDataReady()) {
    gyro.readGyro(&x, &y, &z);
    
    gyro_next();
    gyro_data(0)->x = COS*x + SIN*y;
    gyro_data(0)->y = COS*y - SIN*x;
    gyro_data(0)->z = z;
  }
}










/******************************************************************************
/* Accelerometer
/*/

MMA8452Q accel;

float accel_scale;

struct {
  float x, y, z;
} _accel_data[3] = {0,0,0,0,0,0,0,0,0};

uint8_t _accel_index = 0;

#define accel_next() (_accel_index++)
#define accel_data(offset) (_accel_data + (_accel_index + offset + 2) % 3)

void initAccel() {
  Serial.print("Initializing accel... ");
  accel.begin();
  
  Serial.print("Calibrating accel... ");
  int vals[3] = {0,0,0};
  accel_scale = 0;
  for (int i = 0; i < 10; i++) {
    accel.axes(vals);
    accel_scale += vals[2];
    delay(2);
  }
  accel_scale /= 10;
  
  Serial.println("done");
}

void updateAccel() {
  float x, y;
  int vals[3];
  accel.axes(vals, 10);
  x = ((float) vals[0]) / accel_scale;
  y = ((float) vals[1]) / accel_scale;
  
  accel_next();
  accel_data(0)->x = COS*x + SIN*y;
  accel_data(0)->y = COS*y - SIN*x;
  accel_data(0)->z = ((float) vals[2]) / accel_scale;
}










/******************************************************************************
/* Commumications & Control
/*/

XBee xbee = XBee();
Rx16IoSampleResponse io16 = Rx16IoSampleResponse();
Rx64IoSampleResponse io64 = Rx64IoSampleResponse();

void initComms() {
  Serial.begin(BAUD);
  Serial1.begin(BAUD);
  
  xbee.setSerial(Serial1);
  
  controls.l.x =
  controls.l.y =
  controls.r.x =
  controls.r.y = 0;
}

void updateComms() {
  XBeeResponse response;
  
  xbee.readPacket();
  response = xbee.getResponse();
  
  if (!response.isAvailable())
    return;
  
  if (response.getApiId() == RX_16_IO_RESPONSE) {
    response.getRx16IoSampleResponse(io16);
    updateControls(io16.getAnalog(0,0),
                    io16.getAnalog(1,0),
                    io16.getAnalog(2,0),
                    io16.getAnalog(3,0));
  } else if (response.getApiId() == RX_64_IO_RESPONSE) {
    response.getRx64IoSampleResponse(io64);
    updateControls(io64.getAnalog(0,0),
                    io64.getAnalog(1,0),
                    io64.getAnalog(2,0),
                    io64.getAnalog(3,0));
  } else {
    Serial.print("API Packet: ");
    Serial.println(response.getApiId(), HEX);
  }
}










/******************************************************************************
/* Control
/*/

struct {
  struct {
    int x, y;
  } l, r;
} controls;

struct {
  int rise, rotation;
  struct {
    int northsouth, eastwest;
  } tilt;
} target;

struct {
  int north, east, south, west;
} signals;

void updateControls(int y1, int x1, int y2, int x2) {
  controls.l.x = 512 - x2;
  controls.l.y = 512 - y2;
  controls.r.x = x1 - 512;
  controls.r.y = y1 - 512;
  
  if (-50 < controls.l.x && controls.l.x < 50)
    controls.l.x = 0;
  
  if (-50 < controls.l.y && controls.l.y < 50)
    controls.l.y = 0;
  
  if (-50 < controls.r.x && controls.r.x < 50)
    controls.r.x = 0;
  
  if (-50 < controls.r.y && controls.r.y < 50)
    controls.r.y = 0;
}

void updateTarget() {
  target.rise = controls.l.y/10 + 10;
  target.rotation = controls.l.x / 20;
  target.tilt.northsouth = controls.r.y / 20;
  target.tilt.eastwest = controls.r.x / 20;
}

void updateSignals() {
  signals.north = target.rise + target.rotation + target.tilt.northsouth;
  signals.east  = target.rise - target.rotation + target.tilt.eastwest;
  signals.south = target.rise + target.rotation - target.tilt.northsouth;
  signals.west  = target.rise - target.rotation - target.tilt.eastwest;
}










/******************************************************************************
/* Miscellaneous
/*/

void printSensors() {
  Serial.print("Gyro x:");
  Serial.print(sensors.gyro.x);
  Serial.print("  y:");
  Serial.print(sensors.gyro.y);
  Serial.print("  z:");
  Serial.println(sensors.gyro.z);
  
  Serial.print("Accel x: ");
  Serial.print(sensors.accel.x);
  Serial.print(" y: ");
  Serial.print(sensors.accel.y);
  Serial.print(" z: ");
  Serial.println(sensors.accel.z);
}

void printSignals() {
  Serial.print("(N,E,S,W): (");
  Serial.print(signals.north);
  Serial.print(',');
  Serial.print(signals.east);
  Serial.print(',');
  Serial.print(signals.south);
  Serial.print(',');
  Serial.print(signals.west);
  Serial.println(')');
}
