#include <Servo.h>
#include <ITG3200.h>
#include <XBee.h>
#include <nbwire.h>

/******************************************************************************
/* Data & Variables
/*/

// cosine and sine of the angular offset of the sensors (~25 deg)
#define COS 0.906
#define SIN 0.423

#define BAUD 9600

#define ACCEL_SAMPLES 10
#define ACCEL_DELAY 2

#define GYRO_SAMPLES 500
#define GRYO_DELAY 2

typedef void (*i2c_callback)(int);










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
  scheduleAccel();
  
  Serial.println("Initialization Complete");
  Serial.println("-----------------------");
}

void loop() {
  checkComms();
  updateGyro();
  updateTarget();
  updateSignals();
  updateMotors();
  scheduleAccel();
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
  gyro.zeroCalibrate(GYRO_SAMPLES, GYRO_DELAY);
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

#define MMA8452Q_ADDRESS 0x1D

enum MMA8452Q_REGISTERS {
  STATUS       = 0x00,

  OUT_X_MSB    = 0x01,
  OUT_X_LSB    = 0x02,

  OUT_Y_MSB    = 0x03,
  OUT_Y_LSB    = 0x04,

  OUT_Z_MSB    = 0x05,
  OUT_Z_LSB    = 0x06,

  SYSMOD       = 0x0B,
  INT_SOURCE   = 0x0C,
  WHO_AM_I     = 0x0D,
  XYZ_DATA_CFG = 0x0E,

  PL_STATUS    = 0x10,
  PL_CFG       = 0x11,
  PL_COUNT     = 0x12,

  CTRL_REG1    = 0x2A,
  CTRL_REG2    = 0x2B,
  CTRL_REG3    = 0x2C,
  CTRL_REG4    = 0x2D,
  CTRL_REG5    = 0x2E,

  OFF_X        = 0x2F,
  OFF_Y        = 0x30,
  OFF_Z        = 0x31
};

enum MMA8452Q_STATUS {
  ZYX_OW = 0b10000000,
  Z_OW   = 0b01000000,
  Y_OW   = 0b00100000,
  X_OW   = 0b00010000,
  ZYX_DR = 0b00001000,
  Z_DR   = 0b00000100,
  Y_DR   = 0b00000010,
  X_DR   = 0b00000001
};

enum MMA8452Q_CTRL_REG1 {
  ACTIVE = 0,
  F_READ = 1,
  LNOISE = 2,
  DR0    = 3,
  DR1    = 4,
  DR2    = 5
};

enum MMA8452Q_CTRL_REG2 {
  MODS0  = 0,
  MODS1  = 1,
  SLPE   = 2,
  SMODS0 = 3,
  SMODS1 = 4,
  RST    = 6,
  ST     = 7
};

enum MMA8452Q_CTRL_REG3 {
  PP_OD       = 0,
  IPOL        = 1,
  WAKE_FF_MT  = 3,
  WAKE_PULSE  = 4,
  WAKE_LNDPRT = 5,
  WAKE_TRANS  = 6
};

enum MMA8452Q_CTRL_REG4 {
  INT_EN_DRDY   = 0,
  INT_EN_FF_MT  = 2,
  INT_EN_PULSE  = 3,
  INT_EN_LNDPRT = 4,
  INT_EN_TRANS  = 5,
  INT_EN_ASLP   = 7
};

enum MMA8452Q_CTRL_REG5 {
  INT_CFG_DRDY   = 0,
  INT_CFG_FF_MT  = 2,
  INT_CFG_PULSE  = 3,
  INT_CFG_LNDPRT = 4,
  INT_CFG_TRANS  = 5,
  INT_CFG_ASLP   = 7
};

enum MMA8452Q_PL_STATUS {
  BAFRO = 0,
  LAPO0 = 1,
  LAPO1 = 2,
  LO    = 6,
  NEWLP = 7
};

enum MMA8452Q_PL_CFG {
  PL_EN  = 6,
  DBCNTM = 7
};

float accel_scale;
int accel_has_low_byte;

struct {
  float x, y, z;
} _accel_data[3] = {0,0,0,0,0,0,0,0,0};

uint8_t _accel_index = 0;

#define accel_next() (_accel_index++)
#define accel_data(offset) (_accel_data + (_accel_index + offset + 2) % 3)
  
void initAccel() {
  Serial.print("Initializing accel... ");
  
  if (registerRead(MMA8452Q_ADDRESS, WHO_AM_I) != 0x2A)
    return -1;
  
  uint8_t reg = registerRead(MMA8452Q_ADDRESS, CTRL_REG1);
  bitWrite(reg, ACTIVE, true);
  registerWrite(MMA8452Q_ADDRESS, CTRL_REG1, reg);
  
  accel_has_low_byte = !bitRead(reg, F_READ);
  
  Serial.print("Calibrating accel... ");
  
  accel_scale = 0;
  
  uint8_t data* = new uint8_t[accel_has_low_byte ? 6 : 3];
  for (int i = 0; i < ACCEL_SAMPLES; i++) {
    registersRead(MMA8452Q_ADDRESS, OUT_Z_MSB, data, accel_has_low_byte ? 2 : 1);
    
    accel_scale += data[0] << 8
    if (accel_has_low_byte)
      accel_scale += data[1];
    
    delay(ACCEL_DELAY);
  }
  accel_scale /= ACCEL_SAMPLES;
  
  Serial.println("done");

  return 0;
}

void scheduleAccel() {
  registersReadNonblocking(MMA8452Q_ADDRESS, OUT_X_MSB, accel_has_low_byte ? 6 : 3);
}

void updateAccel(int readResult) {
  float x, y;
  uint8_t *data = new uint8_t[readResult];
  
  for (uint8_t i = 0; i < readResult; i++)
    data[i] = Wire.read();
  registersReadNonblocking_unlock();
  
  if (accel_has_low_byte && readResult < 6 || !accel_has_low_byte && readResult < 3) {
    delete data;
    return;
  }
  
  uint16_t vals[3];
  for (uint8_t i = 0; i < 3; i++) {
    vals[i] = data; data++;
    
    if (accel_has_low_byte) {
      vals[i] += data; data++;
    }
  }
  
  delete data;
  
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

struct {
  uint8_t lock;
  uint8_t device;
  size_t count;
  i2c_callback callback;
} i2c_nonblocking_data = {0, 0, 0, 0};

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

static inline uint8_t registerRead(uint8_t device, uint8_t addr) {
  Wire.beginTransmission(device);
  Wire.write(addr);
  Wire.endTransmission(false);

  Wire.requestFrom(device, 1);

  while (!Wire.available());
  return Wire.read();
}

static inline void registersRead(uint8_t device, uint8_t addr, uint8_t data[], size_t count) {
  Wire.beginTransmission(device);
  Wire.write(addr);
  Wire.endTransmission(false);

  Wire.requestFrom(device, count);

  while (Wire.available() < count);

  for (size_t i = 0; i < count; i++)
    data[i] = Wire.read();
}

static inline void registerWrite(uint8_t device, uint8_t addr, uint8_t value) {
  Wire.beginTransmission(device);
  Wire.write(addr);
  Wire.write(value);
  Wire.endTransmission();
}

static inline void registersWrite(uint8_t device, uint8_t addr, uint8_t data[], size_t count) {
  Wire.beginTransmission(device);
  Wire.write(addr);

  for (int i = 0; i < count; i++)
    Wire.write(data[i]);

  Wire.endTransmission();
}

static uint8_t registersReadNonblocking(uint8_t device, uint8_t addr, size_t count, i2c_callback callback) {
  if (i2c_nonblocking_data.lock)
    return 0;
  
  i2c_nonblocking_data.lock = 1;
  i2c_nonblocking_data.device = device;
  i2c_nonblocking_data.count = count;
  i2c_nonblocking_data.callback = callback;
  
  Wire.beginTransmission(device);
  Wire.write(addr);
  Wire.nbendTransmission(registersReadNonblocking_request);
  
  return 1;
}

static uint8_t registersReadNonblocking_request(int writeResult) {
  Wire.nbrequestFrom(i2c_nonblocking_data.device, i2c_nonblocking_data.count, i2c_nonblocking_data.callback);
}

static void registersReadNonblocking_unlock() {
  i2c_nonblocking_data.lock = 0;
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
