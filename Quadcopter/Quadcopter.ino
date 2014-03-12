#include <Servo.h>
#include <XBee.h>

XBee xbee = XBee();

Rx16IoSampleResponse io16 = Rx16IoSampleResponse();
Rx64IoSampleResponse io64 = Rx64IoSampleResponse();

Servo north, west, south, east;
int v = 10;

void setup() {
  north.attach(6);
  west.attach(9);
  south.attach(10);
  east.attach(11);
  
  writeMotors(v);
  
  Serial.begin(9600);
  Serial1.begin(9600);
  
  xbee.setSerial(Serial1);
  
  delay(1000);
}

void loop() {
  XBeeResponse response;
  
  xbee.readPacket();
  
  response = xbee.getResponse();
  if (response.isAvailable()) {
    if (response.getApiId() == RX_16_IO_RESPONSE) {
      response.getRx16IoSampleResponse(io16);
      joystickControl(io16.getAnalog(0,0),
                      io16.getAnalog(1,0),
                      1024 - io16.getAnalog(2,0),
                      1024 - io16.getAnalog(3,0));
    } else if (response.getApiId() == RX_64_IO_RESPONSE) {
      response.getRx64IoSampleResponse(io64);
      joystickControl(io64.getAnalog(0,0),
                      io64.getAnalog(1,0),
                      1024 - io64.getAnalog(2,0),
                      1024 - io64.getAnalog(3,0));
    } else {
      Serial.print("API Packet: ");
      Serial.println(response.getApiId(), HEX);
    }
  }
}

void joystickControl(int y1, int x1, int y2, int x2) {
  int base = y1/10 - 20;
  writeMotors(base);
}

void writeMotors(int base) {
  writeMotors(base, 0, 0, 0);
}

void writeMotors(int base, int northsouth, int eastwest, int clockwise) {
  north.write(base + northsouth + clockwise);
   west.write(base + eastwest   - clockwise);
  south.write(base - northsouth + clockwise);
   east.write(base - eastwest   - clockwise);
}
