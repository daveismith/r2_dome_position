#include <SPI.h>

#define CHIP_SELECT 10

SPISettings rotaryEncoderSettings(1000000, MSBFIRST, SPI_MODE1); 

void setup() {
  // put your setup code here, to run once:
  pinMode(CHIP_SELECT, OUTPUT);

  // Start SPI
  SPI.begin();

  Serial.begin(115200);
}

uint8_t l, h;
uint16_t val;

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(CHIP_SELECT, LOW);

  SPI.beginTransaction(rotaryEncoderSettings);
  h = SPI.transfer(0);
  l = SPI.transfer(0);

  digitalWrite(CHIP_SELECT, HIGH);
  SPI.endTransaction();

  val = (h << 4) | (l >> 4);

  Serial.print(h);
  Serial.print(",");
  Serial.print(l);
  Serial.print(",");
  Serial.print(val);
  Serial.print(",");
  Serial.print((l & 0x08) >> 3);
  Serial.print(",");
  Serial.print((l & 0x04) >> 2);
  Serial.print(",");
  Serial.print((l & 0x02) >> 1);  
  Serial.print(",");
  Serial.print(l & 0x01);
  Serial.println();
  delay(100);
}
