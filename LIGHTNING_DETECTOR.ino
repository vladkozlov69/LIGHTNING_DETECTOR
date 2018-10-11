/*

Copyright (c) 2017, Embedded Adventures
All rights reserved.

Contact us at source [at] embeddedadventures.com
www.embeddedadventures.com

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

- Neither the name of Embedded Adventures nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.

*/

// AS3935 MOD-1016 Lightning Sensor Arduino test sketch
// Written originally by Embedded Adventures

#include <Wire.h>
#include <SPI.h>
#include "AS3935_mod1016.h"
#include <U8g2lib.h>

#define IRQ_PIN 2
#define CS_PIN 10

volatile bool detected = false;

U8G2_PCD8544_84X48_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 8, /* dc=*/ 7, /* reset=*/ 9);     // Nokia 5110 Display

int noiseCount = 0;
int disturberCount = 0;
int lightningsCount = 0;
int distance = 0;


void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  Serial.println("Welcome to the MOD-1016 (AS3935) Lightning Sensor test sketch!");
  Serial.println("Embedded Adventures (www.embeddedadventures.com)\n");

  u8g2.begin();
  //I2C
  Wire.begin();
  mod1016.init(IRQ_PIN);
  //SPI
  //SPI.begin();
  //mod1016.init(IRQ_PIN, CS_PIN);

  //Tune Caps, Set AFE, Set Noise Floor
  autoTuneCaps(IRQ_PIN);

  //mod1016.setTuneCaps(7);
  //mod1016.setOutdoors();
  mod1016.setIndoors();
  mod1016.setNoiseFloor(5);


  Serial.println("TUNE\tIN/OUT\tNOISEFLOOR");
  Serial.print(mod1016.getTuneCaps(), HEX);
  Serial.print("\t");
  Serial.print(mod1016.getAFE(), BIN);
  Serial.print("\t");
  Serial.println(mod1016.getNoiseFloor(), HEX);
  Serial.print("\n");

  pinMode(IRQ_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), alert, RISING);
  mod1016.getIRQ();
  Serial.println("after interrupt");
}

char buf[30];

void loop()
{
  if (detected)
  {
    translateIRQ(mod1016.getIRQ());
    detected = false;
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_profont12_tf);

  sprintf(buf, "Noise: %d", noiseCount);
  u8g2.drawStr(0, 10,buf);

  sprintf(buf, "Dists: %d", disturberCount);
  u8g2.drawStr(0, 20,buf);

  sprintf(buf, "Lightnings: %d", lightningsCount);
  u8g2.drawStr(0, 30,buf);

  if (lightningsCount > 0)
  {
      sprintf(buf, "Distance: %d", distance);
      u8g2.drawStr(0, 40,buf);
  }

  u8g2.sendBuffer();
}

void alert()
{
	detected = true;
}

void translateIRQ(uns8 irq) {
  switch(irq) {
      case 1:
        Serial.println("NOISE DETECTED");
        noiseCount++;
        break;
      case 4:
        Serial.println("DISTURBER DETECTED");
        disturberCount++;
        break;
      case 8:
        Serial.println("LIGHTNING DETECTED");
        lightningsCount++;
        printDistance();
        break;
    }
}

void printDistance() {
  distance = mod1016.calculateDistance();
  if (distance == -1)
    Serial.println("Lightning out of range");
  else if (distance == 1)
    Serial.println("Distance not in table");
  else if (distance == 0)
    Serial.println("Lightning overhead");
  else {
    Serial.print("Lightning ~");
    Serial.print(distance);
    Serial.println("km away\n");
  }
}
