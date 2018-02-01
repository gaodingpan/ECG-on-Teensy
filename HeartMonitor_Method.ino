/* ADCpdbDMA
PDB triggers the ADC which requests the DMA to move the data to a buffer
*/
#include <SD.h>
#include <SPI.h>
#include <Filters.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include "ILI9341_t3.h"
// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10
#define SD_CS 15
#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01
 
uint16_t buf[7500];
const int pausePin = 2;
const int ReadPin = 3;
const int PupPin = 4;
const int PdownPin = 5;
int bufIndex;
int k;
int pauseState;
int pauseTemp;
bool pause;
int ReadState;
int ReadTemp;
bool Read;
int PupState;
int PupTemp;
bool Pup;
int PdownState;
int PdownTemp;
bool Pdown;
int currentFile;
long int PrevTime;
bool refresh;
long Btemp;
long Ttemp;
long Qtemp;
long rise;
long int QRS;
long HRindex;
long HR[5]{0,0,0,0,0};
uint16_t x;
uint16_t xPrev;
bool stable;
bool stabilizing = false;
bool RiseDetect;
bool drawFinish;
long Arrhythmia[5]; 

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);
FilterOnePole LowPass(LOWPASS,7.0);
FilterOnePole HighPass(HIGHPASS, 0.001);
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;
 
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
 
void setup() {
  // Setting up default value for global variables
  pause = false;
  refresh = false;
  pinMode(pausePin, INPUT_PULLUP);
  pinMode(ReadPin, INPUT_PULLUP);
  pinMode(PupPin, INPUT_PULLUP);
  pinMode(PdownPin, INPUT_PULLUP);
  bufIndex = 0;
  k = 0;
  x = 0;
  xPrev = 0;
  Btemp = 0;
  Ttemp = 0;
  rise = 0;
  stable = true;
  HRindex = 0;
  currentFile = 1;
  RiseDetect = false;
  drawFinish = true;
 
  // Initializing serial monitor and LCD screen
  Serial.begin(11520);
  tft.begin();
  tft.fillScreen(ILI9341_GREEN);
  while (!Serial); // wait until the serial debug window is opened
  Serial.print("Initializing SD card..."); // Initializing SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("failed!");
  } else {
    Serial.println("OK!");
  }
  bluetoothInit();
  screenInit();
  adcInit();
  pdbInit();
  PrevTime = millis();
}
 
void loop() {
  pauseState = digitalRead(pausePin);
  if (!pauseState && pauseTemp){
    pause = !pause;
  }
  ReadState = digitalRead(ReadPin);
  if (!ReadState && ReadTemp){
    Read = !Read;
    tft.fillScreen(ILI9341_YELLOW);
    tft.setCursor(20, 150);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK);
    tft.print("Mode Changed");
    delay(800);
    screenInit();
    drawFinish = true;
  }
  PupState = digitalRead(PupPin);
  PdownState = digitalRead(PdownPin);
  if(Read) {
    if(drawFinish) {
      readFromFile();
      drawFinish = false;
    }
    x = buf[bufIndex];
    if(k < 320) {
      k++;
    } else {
      k = 0;
    }
    ECG();
    if(bufIndex == 7500) {
      drawFinish = true;
      bufIndex = 0;
    }
    PrevTime = millis();
    pauseTemp = pauseState;
    ReadTemp = ReadState;
  }else{
    ifStable();
    ECG();
    pauseTemp = pauseState;
    ReadTemp = ReadState;
    bluetoothSend(HR[4]);
    writeSD();
    buf[bufIndex] = x;
  }
  long int delayT = millis();
  while(millis() - delayT < 4);
}

void buttonStateChange(int state, int temp, int pin, bool control){
  state = digitalRead(pin);
    if (state && temp){
    control = !control;
  }
}

void screenInit(){
  // Drawing Grid for display
  tft.fillScreen(ILI9341_WHITE);
  tft.setRotation(3);
  cleanScreen();
  tft.fillRect(0, 120, 320, 120, ILI9341_BLACK);
  printHR(0);
  printQRS(0);
}
 
void cleanScreen() {
  int i, j;
  tft.fillRect(0, 0, 320, 120, ILI9341_WHITE);
  for (i=0; i<340; i+=10) {
    tft.drawLine(i, 0, i, 120, ILI9341_RED);
  }
  for (j=0; j<120; j+=10) {
    tft.drawLine(0, j, 340, j, ILI9341_RED);
  }
}

void ECG() {
  if (!pause && stable) {
    tft.setRotation(3);
    bufIndex = bufIndex % 7500;
  
    if (k == 320) {
      refresh = true;
      cleanScreen();
      refresh = false;
      k = 0;
    }
    if (x/20+10 > 120) {
      x = 2200;
    }
   
    tft.drawLine(k, xPrev/20+10, k+1, x/20+10, ILI9341_BLACK);
    tft.drawLine(k, xPrev/20+9, k+1, x/20+9, ILI9341_BLACK);
 
       
    if (buf[bufIndex] - x > 50 && abs(Qtemp - k) > 40) {
      tft.drawLine(k, 0, k, 120, ILI9341_BLACK);
      Qtemp = k;
      RiseDetect = true;
      rise = millis();
    }
 
    if (x - buf[bufIndex] > 60 && RiseDetect && abs(Btemp - k) > 40) {
      long int beat = millis() - Ttemp;
      if (rise != 0) {
       long TQRS = (millis() - rise) * 3;
        if (TQRS < 200) {
          QRS = TQRS;
        }
      }
      Btemp = k;
      if (beat > 600) {
        movingAvg(60000/beat);
      }
      printQRS(QRS);
      printHR(HR[4]);
      detectStatus();
      Ttemp = millis();
      tft.drawLine(k, 0, k, 120, ILI9341_GREEN);
      RiseDetect = false;
    }else if ( millis() - Ttemp > 2000) {
      movingAvg(HR[4] / 6);
      QRS = 0;
      printQRS(QRS);
      printHR(HR[4]);
      detectStatus();
    }
  }
  xPrev = x;
  bufIndex += 1;
  
}

void detectStatus() {
  tft.setCursor(120, 200);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("ECG Status: ");
  tft.fillRect(180,200,140,20,ILI9341_BLACK);
  tft.setCursor(190, 200);
  tft.setTextSize(1);
  if ( HR[4] == 0) {
    tft.print("Asystole detected");
    Arrhythmia[0]++;
  }else if (HR[4] < 60) {     
     tft.print("Bradycardia detected");
     Arrhythmia[1]++;
  }else if (HR[4] > 100) {
     tft.print("Tachycardia detected");
     Arrhythmia[2]++;
  }else if (QRS > 120){
     tft.print("PVC detected");
     Arrhythmia[3]++;
  }else{
     tft.print("Normal");
     Arrhythmia[4]++;
  }
}

void ifStable() {
  if (abs(x-xPrev) > 150) {
    stable = false;
    stabilizing = true;
  }else if(stabilizing && abs(x-xPrev) < 150) {
    stable = true;
    stabilizing = false;
    //refresh = true;
    cleanScreen();
    k = 0;
    //refresh = false;
  }
  if (!stable && stabilizing) {
    //refresh = true;
    tft.fillRect(0, 0, 320, 120, ILI9341_BLUE);
    tft.setCursor(0, 0);
    tft.setTextSize(4);
    tft.setTextColor(ILI9341_RED);
    tft.print("Stablizing");
    //refresh = false;
  }
}

void movingAvg(long newData) {
   HR[HRindex] = newData;
   HR[4] = (HR[0] + HR[1] + HR[2] + HR[3])/4;
   HRindex++;
   HRindex = HRindex % 4;
   Ttemp = millis();    
}
 
void printHR(long HR) {
  tft.setCursor(20, 120);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("HR (BPM)");
  tft.fillRect(20, 140, 80, 80, ILI9341_WHITE);
  tft.setCursor(40, 160);
  tft.setTextSize(5);
  tft.setTextColor(ILI9341_GREEN);
  tft.print(HR); 
}
 
void printQRS(long QRS) {
  tft.setCursor(120, 120);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("QRS(ms): ");
  tft.fillRect(220,120,40,20,ILI9341_BLACK);
  tft.setCursor(220, 120);
  tft.setTextSize(2);
  tft.print(QRS);
}
 
void bluetoothSend(long data) {
  refresh = true;
  /* Command is sent when \n (\r) or println is called */
  /* AT+GATTCHAR=CharacteristicID,value */
  ble.print( F("AT+GATTCHAR=") );
  ble.print( hrmMeasureCharId );
  ble.print( F(",00-") );
  ble.println(data, HEX);
  refresh = false;
}
 
void writeSD() {
  // Storing 30s Data into SD card
  long int Now = millis();
  int i = 0;
  int maxIndex = 0;
  long maxCount = 0;
  if (Now - PrevTime >= 33000) {
    // Creating file name
    String fileNum = String(currentFile);
    String fileName = String("Data" + fileNum + ".txt");
    char charFileName[fileName.length()];
    fileName.toCharArray(charFileName, fileName.length() + 1);
    if (SD.exists(charFileName)) { // Delete the pre-existing file
      SD.remove(charFileName);     // with the desired file Name
    }
    File myFile = SD.open(charFileName, FILE_WRITE);
    String ECGstatus = "";  
    for (i = 0; i < 5; i++) {
        if (Arrhythmia[i] > maxCount) {
            maxIndex = i;
            maxCount = Arrhythmia[i];
            Arrhythmia[i] = 0;
        }
    }
    if (maxIndex == 0) {
      ECGstatus = "Asystole detected";
    }else if (maxIndex == 1) {
      ECGstatus = "Bradycardia detected";
    }else if (maxIndex == 2) {
      ECGstatus = "Tachycardia detected";
    }else if (maxIndex == 3) {
      ECGstatus = "PVC detected";
    }else{
      ECGstatus = "normal";
    }
    
    if(myFile) {
      Serial.print("SD Card Printing...");
      int i;
      // Printing file header with the Initials of the author and
      // File Number and the sampling frequency
      myFile.println(String("LYW,DGP-Record" + fileNum + "-250"));
      myFile.println("ECG status: " + ECGstatus);
      for (i = 1; i <= 7500; i++) {
        myFile.print(buf[i - 1]);
        myFile.print(", ");
        if ((i%8) == 0) {
          myFile.println();
        }
      }
      myFile.println("EOF");      
    }
    myFile.close();
    Serial.println("Done");
    currentFile++;
    PrevTime = Now;
  } 
}

void readFromFile() {
  SD.begin(SD_CS);
  File root = SD.open("/");
  printDirectory(root, 0);
  tft.fillScreen(ILI9341_WHITE);
  tft.setCursor(20, 150);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_BLACK);
  tft.print("Waiting for monitor input");
  SD.begin(SD_CS);
  Serial.print("Please enter file number: DATA");
  String fileNum = "-1";
  while (fileNum.toInt() <= 0 && Read){
    fileNum = Serial.readString();
    if (fileNum == "stop") {
      Read = !Read;
      screenInit();
      break;
    }
  }
  Serial.println(fileNum);
  String fileName = String("DATA" + fileNum + ".txt");
  char charFileName[fileName.length()];
  fileName.toCharArray(charFileName, fileName.length() + 1);
  File myFile = SD.open(charFileName);
  
  if (myFile) {
    int skipHeader = 0;
    int index = 0;
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      if (skipHeader < 3) {
        skipHeader++;
        myFile.parseInt();
      } else {
        String dataRead = myFile.parseInt();
        uint16_t dataSave = dataRead.toInt();
        buf[index] = dataSave;
      }
      index++;
    }
    // close the file:
    myFile.close();
    screenInit();
    k = 0;
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }
}

void printDirectory(File root, int numTabs) {
  while (true) {
    File entry =  root.openNextFile();
    if (! entry) {
      // no more files
      break;
    }

    if (!entry.isDirectory()) {
      Serial.println(entry.name());
    } 
    entry.close();
  }
}

void bluetoothInit() {
  bool success;
  Serial.println(F("Adafruit Bluefruit Heart Rate Monitor (HRM) Example"));
  Serial.println(F("---------------------------------------------------"));
 
  randomSeed(micros());
 
  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));
 
  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );
 
  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }
 
  /* Disable command echo from Bluefruit */
  ble.echo(false);
 
  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
 
  // this line is particularly required for Flora, but is a good idea
  // anyways for the super long lines ahead!
  // ble.setInterCharWriteDelay(5); // 5 ms
 
  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Bluefruit HRM': "));
 
  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Bluefruit HRM")) ) {
    error(F("Could not set device name?"));
  }
 
  /* Add the Heart Rate Service definition */
  /* Service ID should be 1 */
  Serial.println(F("Adding the Heart Rate Service definition (UUID = 0x180D): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x180D"), &hrmServiceId);
  if (! success) {
    error(F("Could not add HRM service"));
  }
 
  /* Add the Heart Rate Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
  Serial.println(F("Adding the Heart Rate Measurement characteristic (UUID = 0x2A37): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &hrmMeasureCharId);
    if (! success) {
    error(F("Could not add HRM characteristic"));
  }
 
  /* Add the Body Sensor Location characteristic */
  /* Chars ID for Body should be 2 */
  Serial.println(F("Adding the Body Sensor Location characteristic (UUID = 0x2A38): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A38, PROPERTIES=0x02, MIN_LEN=1, VALUE=3"), &hrmLocationCharId);
    if (! success) {
    error(F("Could not add BSL characteristic"));
  }
 
  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
  Serial.print(F("Adding Heart Rate Service UUID to the advertising payload: "));
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );
 
  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();
  Serial.println();
}
 
static const uint8_t channel2sc1a[] = {
  5, 14, 8, 9, 13, 12, 6, 7, 15, 4,
  0, 19, 3, 21, 26, 22
};
/*
  ADC_CFG1_ADIV(2)         Divide ratio = 4 (F_BUS = 48 MHz => ADCK = 12 MHz)
  ADC_CFG1_MODE(1)         Single ended 12 bit mode
  ADC_CFG1_ADLSMP          Long sample time
*/
#define ADC_CONFIG1 (ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(1) | ADC_CFG1_ADLSMP)
/*
  ADC_CFG2_MUXSEL          Select channels ADxxb
  ADC_CFG2_ADLSTS(3)       Shortest long sample time
*/
#define ADC_CONFIG2 (ADC_CFG2_MUXSEL | ADC_CFG2_ADLSTS(3))
void adcInit() {
  ADC0_CFG1 = ADC_CONFIG1;
  ADC0_CFG2 = ADC_CONFIG2;
  // Voltage ref vcc, hardware trigger, DMA
  ADC0_SC2 = ADC_SC2_REFSEL(0) | ADC_SC2_ADTRG | ADC_SC2_DMAEN;
  // Enable averaging, 4 samples
  ADC0_SC3 = ADC_SC3_AVGE | ADC_SC3_AVGS(0);
  adcCalibrate();
  Serial.println("calibrated");
  // Enable ADC interrupt, configure pin
  ADC0_SC1A = ADC_SC1_AIEN | channel2sc1a[6];
  NVIC_ENABLE_IRQ(IRQ_ADC0);
}
void adcCalibrate() {
  uint16_t sum;
 
  // Begin calibration
  ADC0_SC3 = ADC_SC3_CAL;
  // Wait for calibration
  while (ADC0_SC3 & ADC_SC3_CAL);
  // Plus side gain
  sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
  sum = (sum / 2) | 0x8000;
  ADC0_PG = sum;
 // Minus side gain (not used in single-ended mode)
  sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
  sum = (sum / 2) | 0x8000;
  ADC0_MG = sum;
}
/*
  PDB_SC_TRGSEL(15)        Select software trigger
  PDB_SC_PDBEN             PDB enable
  PDB_SC_PDBIE             Interrupt enable
  PDB_SC_CONT              Continuous mode
  PDB_SC_PRESCALER(7)      Prescaler = 128
  PDB_SC_MULT(1)           Prescaler multiplication factor = 10
*/
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE \
  | PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1))
// 48 MHz / 128 / 10 / 1 Hz = 37500
#define PDB_PERIOD (F_BUS / 128 / 10 / 1) / 250
void pdbInit() {
  // Enable PDB clock
  SIM_SCGC6 |= SIM_SCGC6_PDB;
  // Timer period
  PDB0_MOD = PDB_PERIOD;
  // Interrupt delay
  PDB0_IDLY = 0;
  // Enable pre-trigger
  PDB0_CH0C1 = PDB_CH0C1_TOS | PDB_CH0C1_EN;
  // PDB0_CH0DLY0 = 0;
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
  // Software trigger (reset and restart counter)
  PDB0_SC |= PDB_SC_SWTRIG;
  // Enable interrupt request
  //NVIC_ENABLE_IRQ(IRQ_PDB);
}

void adc0_isr() {
  //HighPass.input(ADC0_RA);
  LowPass.input(ADC0_RA);
  //HighPass.input(LowPass.output());
  //LowPass.input(HighPass.output());
  x = LowPass.output();
  x = x % 3500;
  if (!pause && !refresh && !Read){
      if ( k != 320) {
          k +=1;
      }
  }
}

