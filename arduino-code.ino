/*
  Not the prettiest code, but it works!

  Details:
  - This code was made for a TTGO LCD 240*135 ESP32 board.
  - You can type hex into the serial terminal and it will send the commands to the lens (useful for debugging)
  - The lens is connected to the board using a SPI interface

  Huge thanks to http://jp79dsfr.free.fr/ for their incredible reverse engineering of the Canon lens protocol.
  Check out the document they made here: http://jp79dsfr.free.fr/_Docs%20et%20infos/Photo%20Tech%20_%20Canon%20EOS-EF%20Protocol.pdf

  Read my blog post for more details on the project: https://joshuabird.com/blog/post/3d-printed-film-video-camera

  Joshua Bird
*/

#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>


/* 
  PIN DEFINITIONS 
*/
#define STEP_PIN    27
#define EN_PIN      33

#define HSPI_MISO   13
#define HSPI_MOSI   25
#define HSPI_SCLK   15
#define HSPI_CS     39

#define TRIGGER_PIN 17
#define TOP_BUT_PIN 0
#define BOT_BUT_PIN 35

#define SENSOR_PIN  26

#define BAT_PIN     2

#define TFT_WIDTH   135
#define TFT_HEIGHT  240


/*
  GLOBAL VARIABLES
*/
int fps = 18;
const float shutterAngle = 150.0;
const int microsteps = 8;

int lowerBound = 500;
int upperBound = 2000;

int selected = 0;
bool selectedChanged = false;
bool changeSelected = false;

uint8_t minAv, maxAv;
uint8_t av;

uint16_t focalLength;

bool isLensInit = false;

int handleTriggerInterrupt = 0;

enum RecordingState {
  shouldInitRecording,
  speedingUp,
  recording,
  slowingDown,
  shouldDeinitRecording,
  waiting,
};
RecordingState recordingState = waiting;

float curRps = 0;
bool sensorIsLow = true;
int sprocketCount = 0;
int prevSprocketCount = 0;
int sprocketSkips = 0;

bool error = false;
bool errorOverride = false;

int timer1 = 0;
int timer2 = 0;

const int accel = 35;

int absolutePosition = 0;

SPIClass * lens_spi = NULL;
static const int spiClk = 500000;
SPISettings spiSettings = SPISettings(spiClk, MSBFIRST, SPI_MODE3);

TFT_eSPI tft = TFT_eSPI();


/*
  INTERRUPT HANDLING
*/
void IRAM_ATTR toggleSelectedInterrupt() {
  if (!debounce()) return;
  selected++;
  selected %= 7;
  selectedChanged = true;
}

void IRAM_ATTR changeSelectedInterrupt() {
  if (!debounce()) return;
  changeSelected = true;
}

void IRAM_ATTR triggerInterrupt() {
  if (!debounce()) return;
  handleTriggerInterrupt = millis();
}


void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, INPUT_PULLDOWN);
  pinMode(TOP_BUT_PIN, INPUT_PULLUP);
  pinMode(BOT_BUT_PIN, INPUT_PULLUP);
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(TOP_BUT_PIN, toggleSelectedInterrupt, FALLING);
  attachInterrupt(BOT_BUT_PIN, changeSelectedInterrupt, RISING);
  attachInterrupt(TRIGGER_PIN, triggerInterrupt, CHANGE);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(EN_PIN, HIGH);
  
  Serial.begin(2000000);
  
  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  lens_spi = new SPIClass(HSPI);
  lens_spi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS);

  tft.fillScreen(TFT_BLACK);
  
  displayParams();

  delay(500);
}

int displayCount = 0;

void loop() {
  // Handle interrupts
  if (handleTriggerInterrupt != 0 && millis() - handleTriggerInterrupt >= 20) {
    if (digitalRead(TRIGGER_PIN) == HIGH && recordingState == waiting) {
      recordingState = shouldInitRecording;
    }
    else if (digitalRead(TRIGGER_PIN) == LOW && recordingState != waiting) {
      recordingState = slowingDown;
    }
    handleTriggerInterrupt = 0;
  }
  
  // Do stuff depending on recording state
  switch(recordingState) {
    case shouldInitRecording: {
      digitalWrite(EN_PIN, LOW);
    
      curRps = 0.1;
      
      stopDown(av);
      
      tft.fillScreen(TFT_MAGENTA);
      tft.setTextColor(TFT_WHITE, TFT_MAGENTA);
      tft.drawString("RECORD", 0, 0, 4);
      
      recordingState = speedingUp;

      return;
    }
    case speedingUp: {
      float maxRps = fps/5.0;
      recordingState = recording;
      changeMotorSpeed(0, maxRps, accel, true);

      return;
    }
    case recording: {
      runMotor(fps/5.0, 200*microsteps/5);
      checkFilmJam();
      return;
    }
    case slowingDown: {
      changeMotorSpeed(curRps, 0, accel, false);
      recordingState = shouldDeinitRecording;
      returnHome();

      return;
    }
    case shouldDeinitRecording: {
      digitalWrite(EN_PIN, HIGH);
      
      openUp();

      if (error) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString("ERROR:", 0, 0, 4);
        tft.drawString("film jam", 0, 26, 4);
        
        char buf[50];
        dtostrf(sprocketSkips, 4, 0, buf);
        strcat(buf," skips");
        tft.drawString(buf, 0, 78, 4);
      }
      else {
        tft.fillScreen(TFT_BLACK);
        displayParams();
      }

      delay(100);

      recordingState = waiting;

      break;
    }
    case waiting: {
      break;
    }
  }
  
  // Update screen stuff
  if (selectedChanged) {
    displayParams();
    selectedChanged = false;
  }
  if (changeSelected) {
    changeSelectedHandler();
    displayParams();
    changeSelected = false;
  }

  // Try to connect to lens
  if (!isLensInit) {
    isLensInit = tryInitLens();
    
    if (isLensInit) displayParams();
  }

  // Check if lens status has changed every once in a while
  if (millis() - timer1 > 100) {
    timer1 = millis();

    bool newIsLensInit = checkIsLensInit();
    if (newIsLensInit != isLensInit) {
      isLensInit = newIsLensInit;
      displayParams();
    }

    updateFocalLength();
  }

  // Update battery reading once in a while?
  if (millis() - timer2 > 10000) {
    timer2 = millis();

    // Update battery
    //displayParams(); 
  }
  
  // Read commands from serial
  while (Serial.available() > 0) {
    uint8_t incomingByte1 = Serial.read();

    if (incomingByte1 == 10) break;
    
    uint16_t firstByte = charToHex(incomingByte1) << 4;
    uint8_t secondByte = charToHex(Serial.read());
    
    uint16_t dataToSend = firstByte | secondByte;

    sendData(dataToSend);
  }
}

/**
 * @param accel rps/s
 */
void changeMotorSpeed(float startRps, float endRps, float accel, bool shouldCheckFilmJam) {
  if (startRps == endRps) return;

  int stepCount = 0;

  bool speedingUp = startRps < endRps;
  if (!speedingUp) accel *= -1;

  curRps = startRps+0.1;

  int prevFrameTime = 0;
  while ((speedingUp && curRps < endRps) || (!speedingUp && curRps > endRps)) {
    int delayTime = 1000000/(200*microsteps*curRps*2); // In microseconds

    while (micros() - prevFrameTime < delayTime*2) delayMicroseconds(1);
    prevFrameTime = micros();

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(STEP_PIN, LOW);
    if (shouldCheckFilmJam) readSensor();

    stepCount++;

    absolutePosition += 1;
    absolutePosition %= 200*microsteps/5;

    if (stepCount % 200 == 0 && shouldCheckFilmJam) {
      checkFilmJam();
      if (recordingState == slowingDown) return;
    }

    curRps += accel * delayTime*1e-6*2;
  }

  curRps = endRps;
}

void runMotor(float rps, int steps) {
  int delayTime = 1000000/(200*microsteps*rps*2);
  int prevFrameTime = 0;
  for (int i=0; i < steps; i++) {
    while (micros() - prevFrameTime < delayTime*2) delayMicroseconds(1);
    prevFrameTime = micros();

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(STEP_PIN, LOW);
    
    readSensor();

    absolutePosition += 1;
    absolutePosition %= 200*microsteps/5;
  }
}

void returnHome() {
  runMotor(0.5, 200*microsteps/5 - absolutePosition);
}

void checkFilmJam() {
  sprocketSkips += 2-min(sprocketCount-prevSprocketCount, 2);
  if (sprocketCount == prevSprocketCount && !errorOverride) {
    error = true;
    recordingState = slowingDown;
  }
  prevSprocketCount = sprocketCount;
}

bool checkIsLensInit() {
  sendData(0x00);
  sendData(0x0a);
  return sendData(0x00) == 0xaa;
}

void updateFocalLength() {
  if (!isLensInit) return;

  sendData(0xa0);
  uint16_t result1 = sendData(0x00) << 8;
  uint8_t result2 = sendData(0x00);

  uint16_t result = result1 | result2;

  // If focal len has changed
  if (result != focalLength) {
    focalLength = result;

    setMinMaxAv();

    av = max(av, minAv);
    av = min(av, maxAv);

    displayParams();
  }
}

float avToFStop(int av) {
  return sqrt(pow(2,(av/8.0 - 1)));
}

void changeSelectedHandler() {
  switch(selected) {
    case 0: // Nothing selected
      break;
    case 1: // FPS
      fps %= 48;
      fps += 2;
      break;
    case 3: // F-Stop
      av += 4;
      if (av > maxAv) av = minAv;
      break;
    case 4: // Error override
      errorOverride = !errorOverride;
      break;
  }
}

void stopDown(int av) {
  int deltaAv = av - minAv;

  sendData(0x12);
  sendData(0xff+deltaAv); // Overflows if av is pos, subtracts from 0xFF if av is neg
  sendData(0x00);
}

void setMinMaxAv() {
  sendData(0xb0);
  sendData(0x00);
  minAv = sendData(0x00);
  maxAv = sendData(0x00);
}

bool tryInitLens() {
  if (!checkIsLensInit()) return false;
  
  setMinMaxAv();
 
  openUp();
  av = minAv;

  // enable IS
  sendData(0x00);
  sendData(0x0a);
  sendData(0x00);
  sendData(0x91);
  sendData(0xb9);
  sendData(0x00);
  sendData(0x00);
  sendData(0x00);

  updateFocalLength();

  return true;
}

void openUp() {
  sendData(0x12);
  sendData(0x80);
  sendData(0x00);
}

float batVoltage() {
  float value = analogRead(BAT_PIN);

  return value * 0.0071746 / 6;
}

void displayParams() {
  displayParam(0, 0, "FPS", fps, 2, 0, selected==1); 
  displayParam(1, 0, "SEC LEFT", (144.0-sprocketCount/2)/fps, 2, 1, selected==2);
  displayParam(0, 1, "F-STOP", avToFStop(av), 3, 1, selected==3);
  displayParam(1, 1, "ERR OVRD", errorOverride ? "ON" : "OFF", selected==4);
  isLensInit 
    ? displayParam(0, 2, "FOCL LEN", focalLength*2.5, 2, 0, selected==5)
    : displayParam(0, 2, "FOCL LEN", "ERR", selected==5);
  displayParam(1, 2, "BAT VOLT", batVoltage(), 3, 1, selected==6);
}

void displayParam(int x, int y, char* paramName, float param, int paramWidth, int paramPrec, boolean selected) {
  int margin = 2;
  int screenX = x*(TFT_WIDTH/2);
  int screenY = y*(TFT_HEIGHT/3);
    
  tft.fillRect(screenX, screenY, (TFT_WIDTH/2), (TFT_HEIGHT/3), TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  if (selected) { 
    tft.fillRect(screenX, screenY, (TFT_WIDTH/2), (TFT_HEIGHT/3), TFT_MAGENTA);
    tft.setTextColor(TFT_WHITE, TFT_MAGENTA);
  }
  
  tft.drawString(paramName, screenX+margin, screenY, 2);
  char buf[10];
  dtostrf(param, paramWidth, paramPrec, buf);
  tft.drawString(buf, screenX+margin, screenY+18, 6);
}

void displayParam(int x, int y, char* paramName, const char* param, boolean selected) {
  int margin = 6;
  int screenX = x*(TFT_WIDTH/2);
  int screenY = y*(TFT_HEIGHT/3);
    
  tft.fillRect(screenX, screenY, (TFT_WIDTH/2), (TFT_HEIGHT/3), TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  if (selected) { 
    tft.fillRect(screenX, screenY, (TFT_WIDTH/2), (TFT_HEIGHT/3), TFT_MAGENTA);
    tft.setTextColor(TFT_WHITE, TFT_MAGENTA);
  }
  
  tft.drawString(paramName, screenX+margin, screenY, 2);
  tft.drawString(param, screenX+margin, screenY+18+10, 4);
}

void readSensor() {
  int value = digitalRead(SENSOR_PIN);
  //Serial.println(value);

  if (sensorIsLow && value == HIGH) {
    sensorIsLow = false;
  }
  else if (!sensorIsLow && value == LOW) {
    sensorIsLow = true;
    sprocketCount++;
  } 
}

uint8_t charToHex(uint8_t val) {
  if (val >= 48 && val <= 57) {
    return val - 48;
  }
  else {
    return val - 87;
  }
}

uint8_t sendData(uint16_t dataToSend) {
    uint8_t result;
    lens_spi->beginTransaction(spiSettings);
    result = lens_spi->transfer(dataToSend);
    lens_spi->endTransaction();
    delay(1);

    Serial.print("sent: ");
    Serial.print(dataToSend, HEX);
    Serial.print("  received: ");
    Serial.println(result, HEX);

    return result;
}

int lastPushTime = 0;
boolean debounce() {
 int now = millis();
 int timeGap = now - lastPushTime;

 lastPushTime = now;
 return timeGap > 5;
}
