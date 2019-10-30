#define INT_BUFFER_SIZE 100
const byte interruptPin = 23;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint16_t intIndex = 0;
volatile uint16_t intBuffer[INT_BUFFER_SIZE];

void IRAM_ATTR handleInterrupt() {
  static uint64_t lastTime = 0;
  uint16_t newTime = esp_timer_get_time();
  
  portENTER_CRITICAL_ISR(&mux);
  intBuffer[intIndex] = newTime-lastTime;
  intIndex = (intIndex + 1) % INT_BUFFER_SIZE;
  lastTime = newTime;
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Monitoring interrupts: ");
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, CHANGE);
}


uint16_t lastIndex = 0;
int smpteBit() {
  static bool bitLocked = false;
  static bool bitStage = false;
  
  while (intIndex != lastIndex) {
    portENTER_CRITICAL(&mux);
    int intStep = intBuffer[lastIndex];
    lastIndex = (lastIndex + 1) % INT_BUFFER_SIZE;
    portEXIT_CRITICAL(&mux);

    if(intStep < 104 || intStep > 520) {
      bitLocked = false;
      return -2; // error
    }

    if(bitLocked) {
      if(bitStage) {
        if(intStep > 312) {
          bitLocked = false;
          return -2; // error
        }
        bitStage = false;
      }
      else {
        if(intStep > 312) {
          return 0;
        }
        else {
          bitStage = true;
          return 1;
        }
      }
    }
    else {
      if(intStep > 312) {
        bitLocked = true;
        bitStage = false;
      }
    }

  }
  return -1; // no bit
}


uint8_t frameBuf[10];

int smpteFrame() {
  static bool frameSync = false;
  static int frameBit;
  static int oneCount = 0;

  while(true) {
    int b = smpteBit();

    if(b == -1) {
      return 0;   // no output
    }
    if(b == -2) {
      frameSync = false;
      return -1;  // error
    }

    if(frameSync) {
      int index = frameBit / 8;
      int bitNum = frameBit % 8;
      if(b) {
        frameBuf[index] |= 1 << bitNum;
      }
      else {
        frameBuf[index] &= ~(1 << bitNum);
      }
      frameBit++;

      if(frameBit == 80) {
        frameBit = 0;

        if(frameBuf[8] == 0xFC && frameBuf[9] == 0xBF) {
          return 1;
        }
        else {
          frameSync = false;
          return -1;  // error
        }
      }
    }
    else {
      if(b) {
        oneCount++;
      }
      else {
        if(oneCount == 12) {
          // valid sync
          oneCount++;
        }
        else {
          // invalid sync
          oneCount = 0;
        }
      }
      if(oneCount == 14) {
        frameSync = true;
        frameBit = 0;
        oneCount = 0;
      }
    }

  }
}

void loop() {
  int frameStat = smpteFrame();
  if(frameStat == 1) {
    int hour = (frameBuf[7] & 0x3) * 10 + (frameBuf[6] & 0xF);
    int minute = (frameBuf[5] & 0x7) * 10 + (frameBuf[4] & 0xF);
    int second = (frameBuf[3] & 0x7) * 10 + (frameBuf[2] & 0xF);
    int frameNum = (frameBuf[1] & 0x3) * 10 + (frameBuf[0] & 0xF);
    Serial.printf("%02i:%02i:%02i:%02i", hour, minute, second, frameNum);

//    for(int i=0; i<80; i++) {
//      Serial.print((frameBuf[i/8] >> (i%8)) & 1);
//    }
    Serial.print("\n");
  }
  if(frameStat == -1) {
    Serial.println("Error");
  }
}
