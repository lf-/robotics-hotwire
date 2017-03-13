#include <SoftwareSerial.h>

const int MD49_RX = 8;
const int MD49_TX = 9;
const int SABER_TX = 10;
const int SABER_UNUSED = 11;
const int CMD_MD49 = 0x01;
const int CMD_SABER = 0x02;

SoftwareSerial md49(MD49_RX, MD49_TX);
SoftwareSerial sabertooth(SABER_UNUSED, SABER_TX);

int mode = 0;


void setup() {
  Serial.begin(9600);
  md49.begin(9600);
  sabertooth.begin(9600);
  
  // REQUISITE REQUISITE REQUISITE
  md49.listen();
  // REQUISITE REQUISITE REQUISITE
  
  pinMode(LED_BUILTIN, OUTPUT);
  while (!Serial);
}

void loop() {
  if (md49.available() > 0) {
    digitalWrite(LED_BUILTIN, 1);
    Serial.write(md49.read());
  }

  if (Serial.available() > 0) {
    if (mode == 0) {
      mode = Serial.read();
    }
    if (mode == CMD_MD49) {
      byte received = Serial.read();
      md49.write(received);
      mode = 0;
    } else if (mode == CMD_SABER) {
      byte received = Serial.read();
      sabertooth.write(received);
      mode = 0;
    }
  }
}
