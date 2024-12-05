// Batteriekapazität messen mit Arduino
// HK 2023-05-19 11.34 
// changelog at end
#define VERSIONMAJOR 5 // Haupt-Versionsnummer; wird u.a. für Dateinamen verwendet, einstellig
#define VERSIONMINOR 9 // Unter-Versionsnummer, einstellig

#define LCDELEGOO 1

// hardware
const int BATTS = 2;                  // Anzahl Batterien
const long R1 = 10000;                // Oberer Widerstand des Spannungsteilers in Ohm
const long R2 = 5600;                 // Unterer Widerstand des Spannungsteilers in Ohm
const long R3 = 220;                  // Entlade-Widerstand in Ohm
const long MAX_U_BAT = 9000;          // 100% Spannung (mV), maximal geladene Batterie
const long ADC_REF_VOLT = 3300;       // Betriebsspannung (mV) des Arduinos 5000 or 3300
const long AD_RESOLUTION = 1024;      // Auflösung des AD-Wandlers in Stufen
const int ADC_PINS[] = {A15, A14, A13, A12, A11, A10, A9, A8}; // Pins der AD-Wandler Batterien
const int PWM_pin = 45;

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

// output parameter
long LinenumberS = 1;                   // Zeilennummer serielle Ausgabe
long LinenumberF = 0;                   // Zeilennummer Datei
boolean Logging[BATTS] = {true};       // diese Batterien werden geloggt
int Logging_count = 1;                  // Anzahl Batterien, die gerade geloggt werden
boolean Discharging[BATTS] = {true};   // diese Batterien werden gerade entladen
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
//void battery_log(int j, boolean onoff);
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
    //Logging[j] = false;                 // no discharge yet
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

  if (!FileOK) file_create();           // start logging

  pinMode(PWM_pin, OUTPUT);
  analogWrite(PWM_pin, 127);
}

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
      //else I[j] = UBat[j] / (R1 + R2);            // Strom (mA) berechnen ohne Entladewiderstand R3, kann unter 1mA liegen
      
      Q_total[j] +=  I[j] * PERIOD_MEASURE;       // I= U/R; Batteriekapazität aufsummieren (mA * Period); Einheit mAs, wenn PERIOD_MEASURE==1

      //if (UBat[j] >= U_BAT_START_DISCHARGE) battery_log(j, true);   // start logging when full
      //if (UBat[j] <= U_BAT_STOP_DISCHARGE) battery_log(j, false);   // end logging when empty
    } // end for

    //temp_measurement();

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

  /*for(int i=0; i<400; i++){
    digitalWrite(PWM_pin, HIGH);
    delayMicroseconds(1250);
    digitalWrite(PWM_pin, LOW);
    delayMicroseconds(1250);
  }*/
  
  LCD_led(0, 0, 1, 50);                 // grün an 0,5s, 1 x pro Messung
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
    if (Discharging[j]) tft.setTextColor(WHITE, BLACK);
    else if (Logging[j]) tft.setTextColor(CYAN, BLACK); // logging but not discharging? must be relaxing
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
  CHANGE LOG
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

  May 11:
  Changed for continous discharge

  May 19: 
  Added pulse width modulation (PWM) for intermittent discharge with frequency = 400Hz

  TODO:
  check for connected/disconnected batteries
  redesign timers for discharging and relax phases
  turn ADC raw value output off

  */
