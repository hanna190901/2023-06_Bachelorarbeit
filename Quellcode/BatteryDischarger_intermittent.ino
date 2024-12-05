// Batteriekapazität messen mit Arduino
// HK 2023-04-17 11.34 
// changelog at end
#define VERSIONMAJOR 5 // Haupt-Versionsnummer; wird u.a. für Dateinamen verwendet, einstellig
#define VERSIONMINOR 8 // Unter-Versionsnummer, einstellig

#define LCDELEGOO 1

// hardware
const int BATTS = 4;                  // Anzahl Batterien
const long R1 = 10000;                // Oberer Widerstand des Spannungsteilers in Ohm
const long R2 = 5600;                 // Unterer Widerstand des Spannungsteilers in Ohm
const long R3 = 220;                  // Entlade-Widerstand in Ohm
const long MAX_U_BAT = 9000;          // 100% Spannung (mV), maximal geladene Batterie
const long ADC_REF_VOLT = 3300;       // Betriebsspannung (mV) des Arduinos 5000 or 3300
const long AD_RESOLUTION = 1024;      // Auflösung des AD-Wandlers in Stufen
const int ADC_PINS[] = {A15, A14, A13, A12, A11, A10, A9, A8}; // Pins der AD-Wandler Batterien

// BATTERY measurement
const long U_BAT_START_DISCHARGE = 7600;  // -> Entladen wird gestartet
const long U_BAT_STOP_DISCHARGE = 5600;   // 0% Spannung (mV), entleerte Batterie -> Entladen wird beendet
//const long MAX_U_INPUT = MAX_U_BAT * R2 / (R1 + R2);  // Spannung (mV) am Arduino nach Spannungsteiler
long AD[BATTS] = {0};                     // measurement of voltage (ADC units)
long UBat[BATTS] = {0};                   // voltage (V)
long I[BATTS] = {0};                      // current (mA)
long Q_total[BATTS] = {0};                // aktuelle Batteriekapazität in mAs, also milliAmpereSekunden; /3600 ergibt mAh
long Time_Bat[BATTS] = {0};               // Laufzeit in s der einzelnen Batterien

// intervals
const long PERIOD_MEASURE = 1;            // 1s Takt der Display- und seriellen Ausgaben in s
const long PERIOD_LOGFILE = 5;            // 5s Takt für Logfile-Ausgaben in s
const long PERIOD_LOG_END = 3 * 60 * 60;  // continue logging 5*60min after last battery is discharged

// intervals intermittent discharge
const long PERIOD_RELAX_BEGIN [BATTS] = {5,50,600,5}; // cycle with continuous discharge, after that, relax
const long PERIOD_RELAX_END [BATTS] = {0, 0, 0, 0}; 
//const long PERIOD_RELAX_END [BATTS] = {PERIOD_RELAX_BEGIN, PERIOD_RELAX_BEGIN, PERIOD_RELAX_BEGIN, PERIOD_RELAX_BEGIN}; // kontinuierliche Messung
//const long PERIOD_RELAX_END [BATTS] = {4*PERIOD_RELAX_BEGIN[0], 4*PERIOD_RELAX_BEGIN[1], 4*PERIOD_RELAX_BEGIN[2], PERIOD_RELAX_BEGIN[3]}; // PERIOD_RELAX_BEGIN[j] + Relaxdauer

// output parameter
long LinenumberS = 1;                   // Zeilennummer serielle Ausgabe
long LinenumberF = 0;                   // Zeilennummer Datei
boolean Logging[BATTS] = {false};       // diese Batterien werden geloggt
int Logging_count = 0;                  // Anzahl Batterien, die gerade geloggt werden
boolean Discharging[BATTS] = {false};   // diese Batterien werden gerade entladen
boolean FileOK = false;                 // Schreiben auf SD-Karte möglich ja/nein
unsigned long now_ds;                   // runtime in 1/10s
unsigned long MeasureTimer_ds;          // Counter for display and serial output (deciseconds)
unsigned long LogTimer_ds;              // Counter for logfile output (ds)
unsigned long Log_end_timer_ds;         // timer for logging after last battery is discharged

// output selection
const int OUTPUT_ADC_ONOFF = 1;    // Ausgabe von ADC-Werten ja=1 nein=0; betrifft serielle Ausgabe und Logfile, nicht Display
const int OUTPUT_U_ONOFF = 1;      // Ausgabe von Spannungs-Werten ja=1 nein=0
const int OUTPUT_I_ONOFF = 1;      // Ausgabe von Strom-Werten ja=1 nein=0
const int OUTPUT_Q_ONOFF = 1;      // Ausgabe von Entladungs-Werten ja=1 nein=0

// TEMPERATURE measurement
#include <OneWire.h>
#include <DallasTemperature.h>
#define TEMP_PIN 44                 // Data wire is plugged into port 44 on the Arduino
OneWire tempWire(TEMP_PIN);         // Setup a oneWire instance to communicate with any OneWire devices 
DallasTemperature temp(&tempWire);  // Pass our oneWire reference to Dallas Temperature. 
float tempC = 0;

// Relais
const int RELAIS_PINS[] = {32, 33, 34, 35, 36, 37, 38, 39}; // note: relais are active LOW

// SD & File
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
const int SDchipSelect = 53;        // SD-Kartenleser: 9=DataLoggerShield; 10=TFTshield_Uno; 48=TFTshield_Mega; 53=SD_Adapter
File LogfileHandle;
String LogfileName = "...";
#define VERSION4DIGITS (VERSIONMAJOR*1000+VERSIONMINOR*100)
const String VERSIONSTRING = (String(VERSIONMAJOR) + "." + String(VERSIONMINOR));

// LCD
#if LCDELEGOO
// LCD TFT size is 240x320
// Using Elegoo 2.8" TFT Breakout Board Pinout
#include <Elegoo_TFTLCD.h>  // Hardware-specific library for 320x240 display
#include <Elegoo_GFX.h>     // Core graphics library
#define LCD_CS A3           // Chip Select goes to Analog 3
#define LCD_CD A2           // Command/Data goes to Analog 2
#define LCD_WR A1           // LCD Write goes to Analog 1
#define LCD_RD A0           // LCD Read goes to Analog 0
#define LCD_RESET A4        // Can alternately just connect to Arduino's reset pin 
Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
#define LCD_TYPE 2

#else
#include <Adafruit_GFX.h>     // Core graphics library Original Adafruit
#include <Adafruit_ST7735.h>  // Hardware-specific library for ST7735 small 160x128 display
#define TFT_CS        10
#define TFT_RST        8      // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         9
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
#define LCD_TYPE 1
#endif

// Colors: https://learn.adafruit.com/adafruit-gfx-graphics-library/coordinate-system-and-units
// sixteen-bit-color= (RED << 11) + (GREEN << 6) + BLUE // RGB 0..31 each
#define DARKGRAY 0x4208   //  (8<<11)+(8<<6)+8
#define GRAY 0x8410       // (16<<11)+(16<<6)+16
#define LIGHTGRAY 0xC618  // (24<<11)+(24<<6)+24
#define ORANGE 0xFE00     // (31<<11)+(24<<6)+0, slightly less yellow than 0xFD00
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF


// Funktionen
void battery_log(int j, boolean onoff);
void serial_header();
void serial_line();
void file_create();
void file_header();
void file_line();
void LCD_SplashScreen();
void LCD_background();
void LCD_filename();
void LCD_fileline_nr();
void LCD_line();
void LCD_led(int redled, int yellowled, int greenled, int waitms);
String getHHMMSS (long time);
void temp_measurement();



void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);     // define GPIOs
  digitalWrite(LED_BUILTIN, HIGH);
  analogReference(EXTERNAL);        // external reference https://www.arduino.cc/reference/en/language/functions/analog-io/analogreference/
  delay(500);

#if LCDELEGOO
  tft.reset();                      // init LCD hardware
  tft.begin(0x9341);
#else
  tft.initR(INITR_BLACKTAB);        // Init ST7735S chip, black tab
#endif

  tft.setRotation(1);
  LCD_SplashScreen();               // start screen

  for (int j = 0; j < BATTS; j++) {     
    pinMode(ADC_PINS[j], INPUT);        // define ADC inputs
    pinMode(RELAIS_PINS[j], OUTPUT);    // define relais pins
    digitalWrite(RELAIS_PINS[j], LOW);  // turn relais on briefly during startup (active low)
    delay(100);
    digitalWrite(RELAIS_PINS[j], HIGH); // turn off again (active low)
    Logging[j] = false;                 // no discharge yet
  }

  temp.begin();                         // begin temperature measurement

  Serial.begin(9600);                   // init serial output, set baud-Rate
  delay(1000);
  if (!SD.begin(SDchipSelect))          // init SD card reader
  {
    Serial.println("SD-Karte nicht ansprechbar, kein Logfile.");
    LCD_led(1, 0, 0, 3000);             // rot an 3s
    LogfileName = "SD-Fehler";
  }

  serial_header();                      // serielle Ausgabe auf Konsole
  LCD_background();
}

/*
  Vollautomatik:
  a) gemessen wird im regelmäßigen häufigen Zyklus
  b) wenn Batterie z.B. mind. 7,2 V hat, wenn nötig Logfile starten und Batterie entladen
  c) wenn Batterie z.B. auf 5,5 V entladen, Entladung beenden
  d) wenn keine Batterie mehr entladen wird, ist der Entladevorgang beendet, Logfile wird geschlossen

  Loop runs once a second.
  when PeriodMeasureCnt >= PERIOD_MEASURE display and serial output, check discharge status
  when PeriodLogCnt >= PERIOD_LOGFILE logfile output
  when PERIOD_RELAX_BEGIN relax until == PERIOD_RELAX_END
*/


void loop() {
  now_ds = millis() / 100;
  if (now_ds >= MeasureTimer_ds) {                // *** Measure
    MeasureTimer_ds += PERIOD_MEASURE * 10;       // set next timer
    for (int j = 0; j < BATTS; j++) {
      delay(1);                                   // wait a millisecond between measurements to reduce interference
      AD[j] = analogRead(ADC_PINS[j]);
      // UBat[j] = AD[j] * ADC_REF_VOLT * (R1 + R2) / (AD_RESOLUTION * R2);       // Spannung (V) vor Spannungsteiler berechnen ÜBERLAUF
      UBat[j] = (((AD[j]  * (R1 + R2)) / AD_RESOLUTION) * ADC_REF_VOLT) / R2;     // Überlauf vermieden durch definierte Berechnungsreihenfolge 
      // e.g. (0..1023) *15600 /1024 *3300 /5600
      if (Logging[j]) Time_Bat[j] += PERIOD_MEASURE;                              // add up logging time (s)

      if (Discharging[j]) I[j] = ((UBat[j] * (R1 + R2 + R3)) / (R3 * (R1 + R2))); // Strom (mA) berechnen mit Entladewiderstand R3
      else I[j] = UBat[j] / (R1 + R2);            // Strom (mA) berechnen ohne Entladewiderstand R3, kann unter 1mA liegen
      
      Q_total[j] +=  I[j] * PERIOD_MEASURE;       // I= U/R; Batteriekapazität aufsummieren (mA * Period); Einheit mAs, wenn PERIOD_MEASURE==1

      if (UBat[j] >= U_BAT_START_DISCHARGE) battery_log(j, true);   // start logging when full
      if (UBat[j] <= U_BAT_STOP_DISCHARGE) battery_log(j, false);   // end logging when empty

      if (Logging[j]) {                           // if relax status changed during logging, turn relais on/off
        boolean relaxing = ((Time_Bat[j] % PERIOD_RELAX_END[j]) >= PERIOD_RELAX_BEGIN[j]); // relaxcycle = (Time_Bat[j] % PERIOD_RELAX_END) pause/continue discharge
        if (relaxing && Discharging[j]) {         // during relaxing, discharge should be turned off; goto sleep
          Discharging[j] = false;
          digitalWrite(RELAIS_PINS[j], HIGH);     // turn discharge off (active low)
          Serial.println("Pause beginnen bei: " + String(j));
          delay(5); // wait a few ms to reduce interference
        } else if (!relaxing && !Discharging[j]) {  // during non-relaxing, discharge should be turned on; wake up
          Discharging[j] = true;
          digitalWrite(RELAIS_PINS[j], LOW);      // turn discharge on (active low)
          Serial.println("Pause beenden bei: " + String(j));
          delay(5); // wait a few ms to reduce interference
        }
      }
    } // end for

    temp_measurement();

    serial_line();           // *** console output
    LCD_line();              // *** display output

    if (FileOK) // LED signal 
      LCD_led(0, 1, 1, 50);             // gelb+grün an 0,05s
    else
      LCD_led(1, 0, 1, 50);             // rot+grün an 0,05s
  } // end measure

  now_ds = millis() / 100;
  if (now_ds >= LogTimer_ds) {          // file_line
    LogTimer_ds += PERIOD_LOGFILE * 10; // set next timer
    if (FileOK) {
      LCD_led(0, 1, 0, 50);             // gelb an 0,1s, signalisiere Schreiben
      file_line();
    }
    else
      LCD_led(0, 0, 1, 50);             // rot an 0,5s, signalisiere SD-Fehler
    Serial.println();
  }

  if (Log_end_timer_ds > 0) {           // if continue logging after discharge is active
    LCD_led(1, 0, 1, 100);              // rot grün an 0,1s
    now_ds = millis() / 100;
    if (now_ds > Log_end_timer_ds) {
      Serial.println("Logfile wird fertiggestellt.");
      if (FileOK) {
        LogfileHandle.println("end of file");
        LogfileHandle.close();
        LogfileName = "(fertig)    ";
        LCD_filename();
        FileOK = false;
        Log_end_timer_ds = 0;           // disable timer
      }
    }
  }
  LCD_led(0, 0, 1, 50);                 // grün an 0,5s, 1 x pro Messung
}


void battery_log(int j, boolean onoff) {  // turn logging on (true) or off (false)
  if ( Logging[j] == onoff) return;       // already in correct state
  Logging[j] = onoff;
  Discharging[j] = onoff;

  if (onoff)  {                           // turn logging and discharge ON (active low)
    digitalWrite(RELAIS_PINS[j], LOW);
    Logging_count++;
    Serial.println("Entladung beginnen bei: " + String(j));
    Q_total[j] = 0;
    Time_Bat[j] = 0;
    if (!FileOK) file_create();           // start logging
  }
  else {                                  // turn logging and discharge off (active low)
    digitalWrite(RELAIS_PINS[j], HIGH);
    Logging_count--;
    Serial.println("Entladung beenden bei: " + String(j));
    if (Logging_count <= 0)  {            // alle Batterien wurden gerade entladen, Logfile wird ggf. geschlossen
      Serial.println("Batterien fertig entladen.");
      Log_end_timer_ds = millis() / 100 + 10 * PERIOD_LOG_END; // prepare to stop logging
    }
  } // end else
}


// Ausgabe seriell

void serial_header() {
  Serial.print("\n\n*** Batteriekapazität messen mit Arduino " + VERSIONSTRING + " - ");
  Serial.print(F(__DATE__));
  Serial.print(" ");
  Serial.println(F(__TIME__));
  Serial.print("R1= " + String(R1) + ", R2= " + String(R2) + ", R3= " + String(R3));
  Serial.print(", t= " + String(PERIOD_MEASURE) + "s, tLog= " + String( PERIOD_LOGFILE) + "s, ");
  //Serial.print("Zyklus= " + String(PERIOD_RELAX_BEGIN[j]) + "+" + String((PERIOD_RELAX_END - PERIOD_RELAX_BEGIN)) + "s, ");
  Serial.println(String(MAX_U_BAT) + ".." + String(U_BAT_STOP_DISCHARGE) + "mV\n");
  // Zeile AD1 AD2 AD3 AD4 U1 U2 U3 U4 Q1 Q2 Q3 Q4
  String os = "Zeile\tt[h/100]\tTemp[C]";
  for (int j = 1; j <= BATTS * OUTPUT_ADC_ONOFF; j++) os += "\tADC" + String(j);
  for (int j = 1; j <= BATTS * OUTPUT_U_ONOFF; j++) os += "\tU" + String(j);
  for (int j = 1; j <= BATTS * OUTPUT_I_ONOFF; j++) os += "\tI" + String(j);
  for (int j = 1; j <= BATTS * OUTPUT_Q_ONOFF; j++) os += "\tQ" + String(j);
  Serial.println(os);
}

void serial_line() {
  String os = String(LinenumberS);                          // Beginn jeder Zeile mit neuer Zeilennummer
  os += "\t" + String((LinenumberS * PERIOD_MEASURE) / 36); // Zeit in h/100 // NEU
  LinenumberS++;
  //os += "\t" + String((LinenumberF * PERIOD_MEASURE * PERIOD_LOGFILE) / 36);    // Zeit in h/100 // NEU
  os += "\t\t" + String(tempC) + "\t";                      // Temperatur in °C
  for (int j = 1; j <= BATTS * OUTPUT_ADC_ONOFF; j++) os += "\t" + String(AD[j - 1]); // ADC aktuell
  for (int j = 1; j <= BATTS * OUTPUT_U_ONOFF; j++) os += "\t" + String(UBat[j - 1]); // Spannung aktuell
  for (int j = 1; j <= BATTS * OUTPUT_I_ONOFF; j++) os += "\t" + String(I[j - 1]); // Strom (mA) aktuell
  for (int j = 1; j <= BATTS * OUTPUT_Q_ONOFF; j++) os += "\t" + String(Q_total[j - 1] / 36); // Ladungs-Messwert mAh/100
  Serial.println(os);
}


// Ausgabe Logfile

void file_create() {
  FileOK = false; // file system is not working yet
  LinenumberF = 1;
  for (int i = VERSION4DIGITS; (i < 9999) && SD.exists(LogfileName = "LOG" + String(i) + ".CSV"); i++); // probiere aufsteigende Dateinamen ab 1.txt
  LogfileHandle = SD.open(LogfileName, FILE_WRITE);   // Datei erstellen
  if (LogfileHandle) {                                // file was created successfully, so output header to file
    FileOK = true;
    file_header();
    LCD_led(0, 1, 1, 200);                            //  gelb grün an 0,2s
  }
  else {
    Serial.print("Datei konnte nicht geöffnet werden");
    LogfileName = "Dateifehler ";
    LCD_led(1, 0, 0, 1000);                           // rot an 1s
  }
  LCD_filename();
  Serial.print(" - neues Logfile: ");
  Serial.println(LogfileName);
}

void file_header() {
  // R1 R2 AD1 AD2 AD3 AD4 U1 U2 U3 U4 Q1 Q2 Q3 Q4
  String os = "t[h] R1=" + String(R1) + " R2=" + String(R2) + " R3=" + String(R3);
  os += " tLog=" + String( PERIOD_LOGFILE) + "s ";
  //os += "Zyklus=" + String(PERIOD_RELAX_BEGIN ) + "+" + String((PERIOD_RELAX_END - PERIOD_RELAX_BEGIN)) + "s ";
  os += String(MAX_U_BAT) + ".." + String(U_BAT_STOP_DISCHARGE) + "mV,t[s],Line,Temp[C]";

  for (int j = 1; j <= BATTS * OUTPUT_ADC_ONOFF; j++) os += ",ADC" + String(j);
  for (int j = 1; j <= BATTS * OUTPUT_U_ONOFF; j++) os += ",U" + String(j);
  for (int j = 1; j <= BATTS * OUTPUT_I_ONOFF; j++) os += ",I" + String(j);
  for (int j = 1; j <= BATTS * OUTPUT_Q_ONOFF; j++) os += ",Q" + String(j);
  os += ",";
  os += (F(__DATE__));        // write compile date into logfile header
  os += " ";
  os += (F(__TIME__));        // write compile time into logfile header
  LogfileHandle.println(os);
  LogfileHandle.flush();      // sicherstellen, dass tatsächlich geschrieben wird; Puffer wird entleert
}

void file_line() {
  LCD_fileline_nr();          // show file log activity on screen
  String os = String((LinenumberF * PERIOD_LOGFILE) / 3600);  // Zeit in h
  long sec = millis() / 1000;
  os += "," + String(sec);    // Laufzeit in sec
  os += "," + String(LinenumberF);  // neue Zeilennummer
  os += "," + String(tempC);  // Temperatur in °C
  LinenumberF++;
  for (int j = 1; j <= BATTS * OUTPUT_ADC_ONOFF; j++) os += "," + String(AD[j - 1]); // ADC
  for (int j = 1; j <= BATTS * OUTPUT_U_ONOFF; j++) os += "," + String(UBat[j - 1]); // Spannung
  for (int j = 1; j <= BATTS * OUTPUT_I_ONOFF; j++) os += "," + String(I[j - 1]); // Strom (mA)
  for (int j = 1; j <= BATTS * OUTPUT_Q_ONOFF; j++) os += "," + String(Q_total[j - 1] / 3600); // Ladungs-Messwert mAh
  LogfileHandle.println(os);
  LogfileHandle.flush();      // sicherstellen, dass tatsächlich geschrieben wird; Puffer wird entleert
  if (!SD.exists(LogfileName)) {
    LCD_led(1, 1, 1, 2000);           // rot+gelb+grün an 2s
    FileOK = false;
    Serial.println("Logfile konnte nicht fortgesetzt werden");
  }
}


/* LCD 320x240 or 160x128
  Font size 1: 6x8; size 2: 12x16 -> Grid 26,66 columns, 10 lines
  0123456789+123456789+123456
  BeST  filename.txt 999999 o
  .#.mV...mA..mAh..h:m:s   o
  12.9999.999.9999.12:34:56
*/

const int LCDtabs[] = {0, 3, 8, 12, 17};
#define LCD_CHARWIDTH 6*LCD_TYPE
#define LCD_HMAX 160*LCD_TYPE
#define LCD_LINEHEIGHT 12*LCD_TYPE
#define LCD_LEDRADIUS 2*LCD_TYPE
#define LCD_LEDHPOS (LCD_HMAX-LCD_LEDRADIUS-1)
#define LCD_TITLEVPOS LCD_LINEHEIGHT
#define LCD_FILENAMEHPOS 6
#define LCD_FILELINESHPOS 19


void LCD_SplashScreen() {
  tft.fillScreen(BLACK);

  tft.setTextColor(ORANGE);
  tft.setTextSize(4);
  tft.setCursor(0, 0);
  tft.print("BeST");            // BeST

  tft.setTextColor(WHITE);
  tft.setTextSize(LCD_TYPE);
  tft.setCursor(0, 5 * LCD_LINEHEIGHT);
  tft.print("Batterieanalyse " + VERSIONSTRING);  // Batterieanalyse
  tft.setCursor(0, 6 * LCD_LINEHEIGHT);
  tft.print(F(__DATE__));       // Datum
  tft.setCursor(LCD_HMAX / 2, 6 * LCD_LINEHEIGHT);
  tft.print(F(__TIME__));       // Uhrzeit
  tft.setCursor(0, 7 * LCD_LINEHEIGHT);
  tft.print("R1=" + String(R1) + " R2=" + String(R2) + " R3=" + String(R3));  // Widerstände R1, R2, R3
  tft.setCursor(0, 8 * LCD_LINEHEIGHT);
  tft.print("t= " + String(PERIOD_MEASURE) + "s tLog= " + String(PERIOD_LOGFILE) + "s");
  delay(1600);
}

void LCD_background() {
  tft.fillScreen(BLACK);
  tft.drawFastHLine(0, LCD_LINEHEIGHT * 2 - 4, LCD_HMAX, ORANGE); // Tabellenlinien
  for (int k = 1; k <= 4; k++)
    tft.drawFastVLine((LCDtabs[k] - 1) * LCD_CHARWIDTH + 2, LCD_TITLEVPOS, (BATTS + 2) * LCD_LINEHEIGHT - LCD_TITLEVPOS, ORANGE);

  tft.setTextColor(ORANGE);
  tft.setCursor(0, 0);
  tft.print("BeST");            // BeST

  tft.setTextColor(WHITE);
  tft.setCursor(LCDtabs[4] * LCD_CHARWIDTH, 9 * LCD_LINEHEIGHT);
  tft.print("T/C:");

  tft.setTextColor(LIGHTGRAY);
  int column = 1;
  tft.setCursor(LCDtabs[column++] * LCD_CHARWIDTH, LCD_TITLEVPOS);
  tft.print("mV");
  tft.setCursor(LCDtabs[column++] * LCD_CHARWIDTH, LCD_TITLEVPOS);
  tft.print("mA");
  tft.setCursor(LCDtabs[column++] * LCD_CHARWIDTH, LCD_TITLEVPOS);
  tft.print("mAh");
  tft.setCursor(LCDtabs[column++] * LCD_CHARWIDTH, LCD_TITLEVPOS);
  tft.print("h:m:s");

  const String Fill2 = "  ";        // two filler spaces for three-digit values
  String s; // number converted to string that will be attached to filler chars to achieve fixed length
  for (int j = 0; j < BATTS; j++) { // Battery numbers
    tft.setCursor(0, (j + 2) * LCD_LINEHEIGHT);
    s = (String(j + 1));
    tft.print(Fill2.substring(s.length()) + s);  // Batterienr. zweistellig rechtsbündig
  }
}

void LCD_filename() {
  tft.setTextColor(LIGHTGRAY, BLACK);
  tft.setCursor(LCD_FILENAMEHPOS * LCD_CHARWIDTH, 0);
  tft.print(LogfileName);       // Dateiname
}

void LCD_fileline_nr() {
  tft.setTextColor(LIGHTGRAY, BLACK);
  tft.setCursor(LCD_FILELINESHPOS * LCD_CHARWIDTH, 0);
  const String Fill6 = "      ";  // filler spaces for digit values
  String s = String(LinenumberF);
  tft.print(Fill6.substring(s.length()) + s); // Strom (mA) aktuell; max. sechsstellig rechtsbündig
}

void LCD_line() {
  const String Fill4 = "    ";  // four filler spaces for four-digit values
  const String Fill3 = "   ";   // three filler spaces for three-digit values
  String s;                     // number converted to string that will be attached to filler chars to achieve fixed length

  LCD_fileline_nr();

  int column;
  for (int j = 0; j < BATTS; j++)  {
    tft.setTextColor(GRAY, BLACK);                // black background erases previous display
    //if (Discharging[j]) 
      tft.setTextColor(WHITE, BLACK);
    //else if (Logging[j]) tft.setTextColor(CYAN, BLACK); // logging but not discharging? must be relaxing
    column = 1;
    tft.setCursor(LCDtabs[column++] * LCD_CHARWIDTH, (j + 2)*LCD_LINEHEIGHT); // go to next screen tab position
    s = String(UBat[j]);
    tft.print(Fill4.substring(s.length()) + s);   // Spannung aktuell; vierstellig rechtsbündig

    tft.setCursor(LCDtabs[column++] * LCD_CHARWIDTH, (j + 2)*LCD_LINEHEIGHT);
    s = String(I[j]);
    tft.print(Fill3.substring(s.length()) + s);   // Strom (mA) aktuell; dreistellig rechtsbündig

    tft.setCursor(LCDtabs[column++] * LCD_CHARWIDTH, (j + 2)*LCD_LINEHEIGHT);
    s = String(Q_total[j] / 3600);
    tft.print(Fill4.substring(s.length()) + s);   // Ladungs-Messwert mAh; vierstellig rechtsbündig

    tft.setCursor(LCDtabs[column++] * LCD_CHARWIDTH, (j + 2)*LCD_LINEHEIGHT);
    tft.print(getHHMMSS(Time_Bat[j]));            // Laufzeit (HH:MM:SS)
  }

  tft.setTextColor(WHITE, BLACK);
  tft.setCursor(LCDtabs[4] * LCD_CHARWIDTH + 4 * LCD_CHARWIDTH, 9 * LCD_LINEHEIGHT);
  s = String(tempC);
  tft.print(Fill4.substring(s.length()) + s);
}

void LCD_led(int redled, int yellowled, int greenled, int waitms) {
  if (redled != 0) tft.fillCircle(LCD_LEDHPOS, LCD_LEDRADIUS, LCD_LEDRADIUS, RED); else tft.fillCircle(LCD_LEDHPOS, LCD_LEDRADIUS, LCD_LEDRADIUS, BLACK);     // rote "LED"
  if (yellowled != 0) tft.fillCircle(LCD_LEDHPOS, LCD_LEDRADIUS * 4, LCD_LEDRADIUS, YELLOW); else tft.fillCircle(LCD_LEDHPOS, LCD_LEDRADIUS * 4, LCD_LEDRADIUS, BLACK); // gelbe "LED"
  if (greenled != 0) tft.fillCircle(LCD_LEDHPOS, LCD_LEDRADIUS * 7, LCD_LEDRADIUS, GREEN); else tft.fillCircle(LCD_LEDHPOS, LCD_LEDRADIUS * 7, LCD_LEDRADIUS, BLACK); // grüne "LED"
  delay (waitms);
}

String getHHMMSS (long time) {      // Ausgabeformat für Laufzeit
  String output = "";
  int Sec = time % 60;              // Zeit (s) aufteilen in Stunden, Minuten, Sekunden
  time /= 60;
  int Min = time % 60;
  time /= 60;
  int Hour = time;

  if (Hour < 10) output = " ";      // Ausgabe vorbereiten
  output += Hour;
  output += ":";
  if (Min < 10) output += "0";
  output += Min;
  output += ":";
  if (Sec < 10) output += "0";
  output += Sec;

  return output;
}

void temp_measurement() {
  temp.requestTemperatures();
  tempC = temp.getTempCByIndex(0);
  if(tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature data");
    LCD_led(1, 0, 0, 10);
  }
}


// end.


/*
  Sept. 9:
  new functions led_state, led_toggle
  new functions output serial/file header/line
  changed MinUBat from 6800 to 6000
  changed RTC library from DS1307RTC.h (Paul Stoffregen) to RTClib (Limor Fried, Adafruit)
  RTC date is set to compile date/time automatically if clock needs setting

  Sept. 10:
  remove RTC
  generate filenames with incrementing numbers
  wait extra seconds at start for stable serial connection
  LED output collides with 13 CLK for microSD; removed
  ADC_PINS[] independent from number of connected batteries
  simplify loop
  new arrays AD_single U_single I[]
  added CR to last line of logfile
  checked Q mAh mAmin measurements
  output of U, I, Q instead of ADC, U, Q
  LogfileHandle.flush() ensures lines are written to card
  test for stand-alone operation
  built-in LED on after use, failed
  test with Arduino Mega
  output with Strings
  choose output of AD, Voltage, Discharge to serial, logfile
  test TFT module SD card readers
  file line numbers corrected
  filename LOG1nn.TXT
  output csv format https://en.wikipedia.org/wiki/Comma-separated_values
  write version into file name

  Sept. 11:
  LED signal red/yellow/green
  output battery charge in units of mAh/100
  output logging interval in logfile
  note: output of Q only if U>MinUBat
  output of Q as mAh/100
  output MAX_U_BAT MIN_U_BAT in logfile
  signal no SD card with LED
  fixed output of compile date in header line of logfile

  Sept. 14:
  connect to Mega 2560

  Sept. 16:
  functions for LCD output

  Sep. 19:
  modify hardware
  test Relais module

  Sep. 20:
  running time per battery, format HH:MM:SS
  troubleshooting LCD touchscreen & Relais

  new init sequence, LCD after serial and SD
  touchscreen removed
  new flag "try_new" for testing all batteries

  Sep. 21:
  LCD output of Q (mAh) instead of Q (mAh/100)
  column headings added LCD output
  discharging resistor R3 added
  U_BAT_STOP_DISCHARGE / U_BAT_START_DISCHARGE (6 V / 6,8 V) for different purposes
  new function: new_file
  New R1=10000, R2=5600, R3 220R

  depleted[] changed to not discharging[]
  new_file renamed to file_create
  loop_counter renamed to Loop_file_counter
  MinUBat1 MinUBat2 renamed to U_BAT_STOP_DISCHARGE U_BAT_START_DISCHARGE
  references to hardware LED removed
  extra comma in CSV file header output removed
  main loop rearranged
  bug: overflow in voltage calculation removed
  fixed handling of all batteries depleted
  removed variable try_new
  new variable "discharge_active"
  version number as numeric constant
  don't create file at startup if there are no batteries to discharge

  Sept. 23:
  minor edits
  LCD_SplashScreen() shows resistor values; improved
  filename output to serial improved
  LCD_background() when a new file is created, to display file name

  Sept. 24:
  chipSelect renamed to SDchipSelect
  filename renamed to LogfileName
  myFile renamed to LogfileHandle
  output_LCD_header renamed to LCD_background
  SplashScreen reformatted
  text background erase with setTextColor(foreground, background); https://learn.adafruit.com/adafruit-gfx-graphics-library/graphics-primitives
  LCD_COLUMNWIDTH defined as 60; LCD_LINEHEIGHT as 23; LCD_TITLE_YPOS as 42
  global variables now begin with capital letters
  variable discharge_active removed
  LogFilename initialised
  wait at end of loop, not at start
  output.. prefix removed from function names

  Sept. 25:
  output timing in SplashScreen
  mV mA mAH output on display right-aligned with filling spaces
  changelog moved from begin to end
  Q is correctly displayed even after switching off discharge
  main loop simplified
  set U_BAT_START_DISCHARGE to 7200 to avoid discharge/recover loop
  test for failing SD card

  Sept. 26:
  external AD reference 3300 mV
  VCC renamed to ADC_REF_VOLT
  main loop: wait a millisecond between measurements to reduce interference
  display hours as " 0:" instead of "00:"
  set U_BAT_START_DISCHARGE from 7200 to 7000 mV

  Sept. 27:
  setRotation(3) changed to setRotation(1) to improve contrast
  file_create() resets LinenumberF to 1
  new function LCD_fileline_nr(); linecount is permanently displayed
  LCD_led moved 10 pixels to the right
  leftmost two columns moved one char to the right
  now also compatible with 160x128 display

  Sept. 28:
  additional color definitions

  Sept. 29:
  logfile output with mAh, not mAh/100
  longer wait period at begin for serial

  Sept. 30:
  serial and logfile output of time as t[h/100]
  ADC_, V_, I_sumup removed, no more average, as it did not improve smoothness of output
  ADC_single I_single U_single renamed to without "single"
  U_BAT_STOP_DISCHARGE = 5600
  PERIOD_LOGFILE = 36x5s = 180s (3 min) output period

  Oct. 2:
  Logfile Period 120s
  PeriodLoop renamed to PERIOD_MEASURE, PeriodSD renamed to PERIOD_LOGFILE
  new const PERIOD_RELAX_BEGIN, PERIOD_RELAX_END

  Oct. 5:
  Constants written in FULL CAPS
  timers in main loop newly arranged

  Oct. 6, 7:
  discharge with relax periods; main loop rewritten
  Output relax begin & end into serial and logfile, corrected
  Filename & title display light gray instead of gray

  Oct. 8:
  bugs fixed: PERIOD_.. as long const, int is not sufficient; also LineNumberS and F as long
  Cycle 8 min ON, 12 min OFF

  Oct. 18, 19:
  Cycle 4 min ON, 16 min OFF
  enter relax phases only with battery below U_BAT_START_RELAX
  first column of logfile is now t[h], after that line number

  Oct. 26:
  raise U_BAT_START_DISCHARGE from 7200 to 7500 to avoid cycling
  output runtime in minutes with logfile lines

  Nov. 2:
  main loop exactly one second cycle
  output timing in seconds for every line into logfile
  "end-of-file" line is output when file is properly completed and closed
  bugfix timing, loop 5s instead of 6s: if (++PeriodMeasureCnt >= PERIOD_MEASURE) ..
  continue logging for 3h after discharge completed
  start relax under 6500mV, cycle 4+20 min

  Nov. 3:
  inactive display lines changed from dark grey to medium grey

  Nov. 10:
  timer for measurement/display and logfile redesigned with deci seconds
  timer redesigned for log after discharge
  long changed to unsigned long https://www.arduino.cc/reference/en/language/variables/data-types/unsignedlong/
  timing and voltage thresholds changed slightly

  Dec. 09:
  Added to GitHub HK

  Mar. 30:
  Added different intervals for intermittent discharge per battery (PERIOD_RELAX_END) HK
  new order of parameter initialisierung HK

  Apr. 03:
  removed bool RelaxAllowed, not necessary for bachelor thesis HK

  Apr. 12:
  Changed PERIOD_RELAX_END for first measurement HK

  Apr. 13:
  Added temperature measurement with TMP36GZ 
  Added output of temperature value to SerialOutput & SD

  Apr. 17:
  Changed temperature measurement to digital sensor: DS18B20
  Added output of temperature value to LCD display

  TODO:
  check for connected/disconnected batteries
  redesign timers for discharging and relax phases
  turn ADC raw value output off

  */
