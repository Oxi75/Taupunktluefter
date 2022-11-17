//////////////////////////////////////////////////////////////////////////////
// Das Taupunkt-Lüftungssystem
// mit Datenlogging
//
// veröffentlicht in der MAKE 1/2022 und 2/2022
//
// Ulrich Schmerold
// 3/2022
//////////////////////////////////////////////////////////////////////////////
// Extensions by Andreas Schmack
// 09/2022 - V3.0 TÜR-Sensor and Power saving Extensions by Andreas Schmack
// 11/2022 - V3.1 Sensor-Restart on Startup 
//////////////////////////////////////////////////////////////////////////////
#define Software_version "Version: 3.1"

// Dieser Code benötig zwingend die folgenden Libraries:
#include "DHT.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>
#include <DS1307RTC.h>
#include <SD.h>
#include <SPI.h>

tmElements_t tm;

#define RELAIPIN 6     // Anschluss des LüfterRelais
#define DHTPIN_I 4     // Datenleitung für den DHT Sensor I (innen)
#define DHTPIN_A 5     // Datenleitung für den DHT Sensor A (außen)
#define SENSPWR  8     // Spannungsversorgung der Sensoren via Ditital-Out

#define TUER1PIN A3    // Tür1-Kontakt 
#define TUER2PIN A1    // Tür1-Kontakt 
#define TUER3PIN A2    // Tür1-Kontakt 
#define TUER4PIN A0    // Tür1-Kontakt 
#define OFFEN HIGH     // Tür ist offen
#define ZU    LOW      // Tür ist zu
#define TUERLEDPIN 7   // Zeigt an, dass mindestens eine Tür offen ist

#define TFTANPIN 2     // TFT Backlight on
#define TFTEINZEIT 20000 // TFT bleibt nach Aktivierung ms an

#define RELAIS_EIN HIGH
#define RELAIS_AUS LOW
#define LED_EIN LOW
#define LED_AUS HIGH
#define SENS_AUS LOW
#define SENS_EIN HIGH
#define TFT_EIN LOW
#define TFT_AUS HIGH

bool rel;
bool fehler = true;

#define DHTTYPE_I DHT22   // DHT 22 
#define DHTTYPE_A DHT22   // DHT 22  

// ***************************   Korrekturwerte der einzelnen Sensorwerten.  ***************
#define Korrektur_t_1  0    // Korrekturwert Innensensor Temperatur
#define Korrektur_t_2  0    // Korrekturwert Außensensor Temperatur
#define Korrektur_h_1  0    // Korrekturwert Innensensor Luftfeuchtigkeit
#define Korrektur_h_2  0    // Korrekturwert Außensensor Luftfeuchtigkeit
//******************************************************************************************

#define SCHALTmin   5.0   // minimaler Taupuntunterschied, bei dem das Relais schaltet
#define HYSTERESE   1.0   // Abstand von Ein- und Ausschaltpunkt
#define TEMP1_min  10.0   // Minimale Innentemperatur, bei der die Lüftung aktiviert wird
#define TEMP2_min -10.0   // Minimale Außentemperatur, bei der die Lüftung aktiviert wird

DHT dht1(DHTPIN_I, DHTTYPE_I);  //Der Innensensor wird ab jetzt mit dht1 angesprochen
DHT dht2(DHTPIN_A, DHTTYPE_A);  //Der Außensensor wird ab jetzt mit dht2 angesprochen

LiquidCrystal_I2C lcd(0x27,20,4); // LCD Display I2C Addresse und Displaygröße setzen

uint32_t TFT_AN_ZEIT;   //Zeitstempel für TFT-Backlight-Abschaltung

//++++++++++++++++++++++++++++++++ Variablen für das Datenlogging ***************************************
#define Headerzeile F("Datum|Zeit;Temperatur_S1;Feuchte_H1;Taupunkt_1;Temperatur_S2;Feuchte_H2;Taupunkt_2;Luefter_Ein/Aus;Laufzeit_Luefter;")

#define logFileName F("Luefter1.csv")  // Name der Datei zum Abspeichern der Daten (Dateinamen-Format: 8.3)!!!!
#define LogInterval 10                   // Wie oft werden die Messwerte aufgezeichnet ( 5 = alle 5 Minuten)

bool logging = true;                    // Sollen die Daten überhaupt protokolliert werden?
String LogData = "" ;                   // Variable zum Zusammensetzen des Logging-Strings.
char stamp[17];                         // Variable für den Zeitstempel.
unsigned int LuefterStart = 0;          // Wann wurde der Lüfter eingeschaltet?
unsigned int LuefterLaufzeit = 0;       // Wie lange lief der Lüfter?
char StrLuefterzeit[6];                 // Lüfterlaufzeit als String zur weiteren Verwendung.
uint8_t Today = 0;                      // Das heutige Datum (nur Tag), zur Speicherung bei Tageswechsel.
bool Tageswechsel=false;
//********************************************************************************************************

void setup()
{
    wdt_enable(WDTO_8S); // Watchdog timer auf 8 Sekunden stellen

    pinMode(RELAIPIN, OUTPUT);          // Relaispin als Output definieren
    digitalWrite(RELAIPIN, RELAIS_EIN); // Relais während dem Start zur Kontrolle einschalten

    pinMode(SENSPWR, OUTPUT);           // Sensor-Power als Output definieren
    digitalWrite(SENSPWR, SENS_AUS);    // Sensoren einschalten
    

    pinMode(TUERLEDPIN, OUTPUT);        // LED-Pin als Output definieren
    digitalWrite(TUERLEDPIN, LED_EIN);  // LED während dem Starten einschalten

    pinMode(TUER1PIN, INPUT);           // Türkontakt1-Pin als Input definieren
    pinMode(TUER2PIN, INPUT);           // Türkontakt2-Pin als Input definieren
    pinMode(TUER3PIN, INPUT);           // Türkontakt3-Pin als Input definieren
    pinMode(TUER4PIN, INPUT);           // Türkontakt4-Pin als Input definieren
    
    pinMode(TFTANPIN, INPUT_PULLUP);    // Anmerkung: Backlight soll per default aus sein

    lcd.init();         //LCD initialisieren
    lcd.clear();        //LCD löschen
    lcd.backlight();    //LCD Backlight während der Startphase einschalten

    wdt_reset();        // Watchdog zurücksetzen

    //--------------------- Logging ------------------------------------------------------------------------------------
    if (logging == true)
    {
        lcd.setCursor(0,1);            // Curser auf Zeile 1, 2. Zeichen
        lcd.print(Software_version);   // Welche Softwareversion läuft gerade
        RTC_start();                   // RTC-Modul testen. Wenn Fehler, dann kein Logging
        delay (4000);                  // Zeit um das Display zu lesen
        lcd.clear();                   // LCD wieder löschen
        wdt_reset();                   // Watchdog zurücksetzen
        test_SD();                     // SD-Karte suchen. Wenn nicht gefunden, dann kein Logging ausführen
        Today = tm.Day ;
    }
    
    //------------------------------------------------ Neustart aufzeichnen -------------------------------------
    if (logging == true) // kann sich ja geändert haben wenn Fehler bei RTC oder SD
    {
        make_time_stamp();
        File logFile = SD.open(logFileName, FILE_WRITE);
        logFile.print(stamp);
        logFile.println(F(": Neustart"));                    // Damit festgehalten wird, wie oft die Steuerung neu gestartet ist
        logFile.close();
     } //---------------------------------------------------------------------------------------------------------------------

    //---------------------- diverse Sonderzeichen definieren --------------------------------------------------
    //Anm.: Nicht ganz klar, warum kein lcd.write(223); verwendet wird um z.B. das Grad-Zeichen auszugeben
    byte Grad[8] = {B00111,B00101,B00111,B0000,B00000,B00000,B00000,B00000};      // Sonderzeichen ° definieren
    lcd.createChar(0, Grad);
    byte Strich[8] = {B00100,B00100,B00100,B00100,B00100,B00100,B00100,B00100};   // Sonderzeichen senkrechter Strich definieren    
    lcd.createChar(1, Strich);
    byte KleinUE[8] = {B10010, B00000, B10010, B10010, B10010, B10010, B01101, B00000};   // Sonderzeichen ü
    lcd.createChar(2, KleinUE);
    byte GrossUE[8] = {B10001, B00000, B10001, B10001, B10001, B10001, B01110, B00000};   // Sonderzeichen Ü
    lcd.createChar(3, GrossUE);

    //--------------------------- Sensoren starten ------------------------------------------------------
    digitalWrite(SENSPWR, SENS_EIN);    // Sensoren einschalten
    delay (500);                        // Zeit um Sensoren zu versorgen
    dht1.begin();                       // Sensoren starten
    dht2.begin();

    TFT_AN_ZEIT = millis();
    digitalWrite(RELAIPIN, RELAIS_AUS); // Relais ausschalten
}

// ----------------- Beginn Main-Loop --------------------------
void loop()
{
    LCD_EIN_AUS();  // Prüfen ob TFT Backlight aktiviert bzw. deaktiviert werden muss

    // Sensoren auslesen
    RTC.read(tm);
    float h1 = dht1.readHumidity()+Korrektur_h_1;       // Innenluftfeuchtigkeit auslesen und unter „h1“ speichern
    float t1 = dht1.readTemperature()+ Korrektur_t_1;   // Innentemperatur auslesen und unter „t1“ speichern
    float h2 = dht2.readHumidity()+Korrektur_h_2;       // Außenluftfeuchtigkeit auslesen und unter „h2“ speichern
    float t2 = dht2.readTemperature()+ Korrektur_t_2;   // Außentemperatur auslesen und unter „t2“ speichern

    if (fehler == true)  // Prüfen, ob gültige Werte von den Sensoren kommen
    {
        lcd.clear();
        lcd.setCursor(2,0);
        lcd.print(F("Teste Sensoren.."));
        fehler = false;
        if (isnan(h1) || isnan(t1) || h1 > 100 || h1 < 1 || t1 < -40 || t1 > 80 )  {
            // Serial.println(F("Fehler beim Auslesen vom 1. Sensor!"));
            lcd.setCursor(0,1);
            lcd.print(F("Fehler Sensor Innen"));
            fehler = true;
        } else {
            lcd.setCursor(0,1);
            lcd.print(F("Sensor I in Ordnung"));
        }

        LCD_EIN_AUS();  // Prüfen ob TFT Backlight aktiviert bzw. deaktiviert werden muss
        delay(2000);  // Zeit um das Display zu lesen
        LCD_EIN_AUS();  // Prüfen ob TFT Backlight aktiviert bzw. deaktiviert werden muss

        if (isnan(h2) || isnan(t2) || h2 > 100 || h2 < 1 || t2 < -40 || t2  > 80) 
        {
            // Serial.println(F("Fehler beim Auslesen vom 2. Sensor!"));
            lcd.setCursor(0,2);
            lcd.print(F("Fehler Sensor Aussen"));
            fehler = true;
        } else {
            lcd.setCursor(0,2);
            lcd.print(F("Sensor A in Ordnung"));
        }

        delay(2000);  // Zeit um das Display zu lesen
    }
    if (isnan(h1) || isnan(t1) || isnan(h2) || isnan(t2)) fehler = true;

    LCD_EIN_AUS();  // Prüfen ob TFT Backlight aktiviert bzw. deaktiviert werden muss

    if (fehler == true) {
        digitalWrite(RELAIPIN, RELAIS_AUS); // Relais ausschalten
        lcd.setCursor(0,3);
        lcd.print(F("CPU Neustart....."));
        while (1);  // Endlosschleife um das Display zu lesen und die CPU durch den Watchdog neu zu starten
    }
    wdt_reset();  // Watchdog zurücksetzen


    /**************************************************************************/
    /* Türkontakte prüfen. Wenn eine Tür geöffnet, System nicht aktivieren!
    /**************************************************************************/
    int TUER1 = digitalRead(TUER1PIN);                 //Status Tür 1
    int TUER2 = digitalRead(TUER2PIN);                 //Status Tür 2
    int TUER3 = digitalRead(TUER3PIN);                 //Status Tür 3
    int TUER4 = digitalRead(TUER4PIN);                 //Status Tür 4
    int TUERSTATUS;

    lcd.clear();
    LCD_EIN_AUS();  // Prüfen ob TFT Backlight aktiviert bzw. deaktiviert werden muss
    lcd.setCursor(0,0);
    lcd.print("T");         //Türkontakte:
    lcd.write((uint8_t)3);  //
    lcd.print("rkontakte:");

    if ((TUER1 == OFFEN) || (TUER2 == OFFEN) || (TUER3 == OFFEN) || (TUER4 == OFFEN))
    {
      TUERSTATUS = OFFEN;
      digitalWrite(TUERLEDPIN, LED_EIN);  // Signal-LED einschalten

      lcd.setCursor(0,1);
      lcd.print(" offen: ");
      if (TUER1) lcd.print("1  ");
      if (TUER2) lcd.print("2  ");
      if (TUER3) lcd.print("3  ");
      if (TUER4) lcd.print("4");

      lcd.setCursor(0,2);
      lcd.print(" zu:    ");
      if (!TUER1) lcd.print("1  ");
      if (!TUER2) lcd.print("2  ");
      if (!TUER3) lcd.print("3  ");
      if (!TUER4) lcd.print("4");

      delay(2000);  // Zeit um das Display zu lesen
    }
    else
    {
       TUERSTATUS = ZU;
       digitalWrite(TUERLEDPIN, LED_AUS);  // Signal-LED einschalten
       //lcd.setCursor(0,2);
       //lcd.print("  Alle geschlossen");    //Alle geschlossen
       //delay(2000);  // Zeit um das Display zu lesen
    }

    wdt_reset();  // Watchdog zurücksetzen


//**** Taupunkte errechnen********
    float Taupunkt_1 = taupunkt(t1, h1);   //Taupunkt innen
    float Taupunkt_2 = taupunkt(t2, h2);   //Taupunkt außen

    // Werteausgabe auf dem I2C-Display
    lcd.clear();
    LCD_EIN_AUS();  // Prüfen ob TFT Backlight aktiviert bzw. deaktiviert werden muss
    lcd.setCursor(0,0);
    lcd.print(F("SI: "));
    lcd.print(t1);
    lcd.write((uint8_t)0); // Sonderzeichen °C
    lcd.write(('C'));
    lcd.write((uint8_t)1); // Sonderzeichen |
    lcd.print(h1);
    lcd.print(F(" %"));

    lcd.setCursor(0,1);
    lcd.print(F("SA: "));
    lcd.print(t2);
    lcd.write((uint8_t)0); // Sonderzeichen °C
    lcd.write(('C'));
    lcd.write((uint8_t)1); // Sonderzeichen |
    lcd.print(h2);
    lcd.print(F(" %"));

    lcd.setCursor(0,2);
    lcd.print(F("Taupunkt I: "));
    lcd.print(Taupunkt_1);
    lcd.write((uint8_t)0); // Sonderzeichen °C
    lcd.write(('C'));

    lcd.setCursor(0,3);
    lcd.print(F("Taupunkt A: "));
    lcd.print(Taupunkt_2);
    lcd.write((uint8_t)0); // Sonderzeichen °C
    lcd.write(('C'));

    delay(5000); // Zeit um das Display zu lesen
    wdt_reset(); // Watchdog zurücksetzen

    lcd.clear();
    lcd.setCursor(0,0);

    float DeltaTP = Taupunkt_1 - Taupunkt_2;

    if (DeltaTP > (SCHALTmin + HYSTERESE)) rel = true;    //Taupunkt-Unterschied überhalb Schaltschwelle  => Einschalten
    if (DeltaTP < (SCHALTmin)) rel = false;               //Taupunkt-Unterschied unterhalb Schaltschwelle => Ausschalten
    if (t1 < TEMP1_min) rel = false;                      //Temperatur kleiner Minimal-Temperatur => Ausschalten
    if (t2 < TEMP2_min) rel = false;                      //Temperatur kleiner Minimal-Temperatur => Ausschalten
    if (TUERSTATUS == OFFEN) rel = false;                 //Tür(en) nicht geschlossen => Ausschalten

    if (rel == true)
    {
        digitalWrite(RELAIPIN, RELAIS_EIN); // Relais einschalten
        lcd.print("L");         //Lüftung: AN
        lcd.write((uint8_t)3);  // ü
        lcd.print("ftung: AN");
        if (LuefterStart <=0 && logging == true) {
            LuefterStart = tm.Hour*60 + tm.Minute;
        }
    } else {
        digitalWrite(RELAIPIN, RELAIS_AUS); // Relais ausschalten
        lcd.print("L");         //Lüftung: AUS
        lcd.write((uint8_t)3);  // ü
        lcd.print("ftung : AUS");
        if( LuefterStart > 0 && logging == true) {
            LuefterLaufzeit += (tm.Hour*60 + tm.Minute - LuefterStart);
            LuefterStart = 0;
        }
    }

    LCD_EIN_AUS();  // Prüfen ob TFT Backlight aktiviert bzw. deaktiviert werden muss
    lcd.setCursor(0,1);
    lcd.print(F("Delta TP: "));
    lcd.print(DeltaTP);
    lcd.write((uint8_t)0); // Sonderzeichen °C
    lcd.write('C');

    make_time_stamp();
    lcd.setCursor(0,3);
    lcd.print(stamp);
    lcd.setCursor(0,2);
    if (logging == true)lcd.print(F("Logging: AN"));
    else lcd.print(F("Logging: AUS"));

    delay(4000);   // Wartezeit zwischen zwei Messungen
    wdt_reset();   // Watchdog zurücksetzen

//--------------------------------------------logging-----------------------------------------------------
    if (logging == true)
    {
        if  ( Today  != tm.Day)                                                     // Tageswechsel ==> Lüfterzeit abspeichern
        {
            Tageswechsel = true;                                                    // ==> Sofort speichern (siehe SD.ino) ==> Nicht erst wenn LogIntervall abgelaufen ist
            if (LuefterStart > 0 ) LuefterLaufzeit += (1440 - LuefterStart);       // ==>Lüfter läuft gerade
            snprintf(StrLuefterzeit,sizeof(StrLuefterzeit),"%d;",LuefterLaufzeit);
            Today = tm.Day;
            LuefterLaufzeit = 0;
        } else {
            strcpy( StrLuefterzeit, "0;");     // Kein Tageswechsel, nur Platzhalter abspeichern
        }

        char buff[4];
        LogData="";
        dtostrf(t1, 2, 1, buff);
        LogData += buff ;
        LogData += ';';
        dtostrf(h1, 2, 1, buff);
        LogData += buff ;
        LogData += ';';
        dtostrf(Taupunkt_1, 2, 1, buff);
        LogData += buff ;
        LogData += ';';
        dtostrf(t2, 2, 1, buff);
        LogData += buff ;
        LogData += ';';
        dtostrf(h2, 2, 1, buff);
        LogData += buff;
        LogData += ';';
        dtostrf(Taupunkt_2, 2, 1, buff);
        LogData += buff;
        LogData += ';';
        if (rel == true) LogData +="1;";
        else LogData += "0;";
        LogData += StrLuefterzeit;

        save_to_SD(); // Daten auf die SD Karte speichern
    }
}
//---------------------------Ende Main-Loop ----------------------------------------------------------------




float taupunkt(float t, float r) {

    float a, b;

    if (t >= 0) {
        a = 7.5;
        b = 237.3;
    } else if (t < 0) {
        a = 7.6;
        b = 240.7;
    }

    // Sättigungsdampfdruck in hPa
    float sdd = 6.1078 * pow(10, (a*t)/(b+t));

    // Dampfdruck in hPa
    float dd = sdd * (r/100);

    // v-Parameter
    float v = log10(dd/6.1078);

    // Taupunkttemperatur (°C)
    float tt = (b*v) / (a-v);
    return { tt };
}


void software_Reset() // Startet das Programm neu, nicht aber die Sensoren oder das LCD
{
    asm volatile ("  jmp 0");
}


void LCD_EIN_AUS()
{
  static bool TFT_STATUS = TFT_AUS;
  uint32_t JETZT;
  uint32_t DIFF;
  
  JETZT = millis();                                           //aktuelle Systemzeit  
  if (digitalRead(TFTANPIN) == TFT_EIN) TFT_AN_ZEIT = JETZT;  //Wenn Display-Pin noch gedrückt ist, Zeitstempel zurück setzen  
  DIFF = JETZT - TFT_AN_ZEIT;                                 //Zeitdifferenz berechnen

  if ((DIFF < TFTEINZEIT) && (TFT_STATUS == TFT_AUS))         //Noch keine 10 Sek vergangen und TFT ausgeschaltet?
  {
    lcd.backlight();                                          //TFT Backlight einschalten
    TFT_STATUS = TFT_EIN;                                     //TFT Status merken
    TFT_AN_ZEIT = JETZT;                                      //die 10 Sekunden starten ab jetzt
  }

  if ((DIFF >= TFTEINZEIT) && (TFT_STATUS == TFT_EIN))        //Die Zeit ist abgelaufen, Display wieder ausschalten
  {
    lcd.noBacklight();                                        //TFT Backlight ausschalten
    TFT_STATUS = TFT_AUS;                                     //TFT Status merken
    attachInterrupt(digitalPinToInterrupt(TFTANPIN), INT_PINisr, LOW);  //Interrupt Wiedereinschalten des LCD-Backlight 
  }
}



// ********************************************************************
// ****  INT_PINisr()
// ****  is called on interrupt from ligth sensor
// ********************************************************************
void INT_PINisr(void)
/* ISR fuer Pin 2 */
{
  /* detach Interrupt, damit er nur einmal auftritt */
  detachInterrupt(digitalPinToInterrupt(TFTANPIN));
  //detachInterrupt(0);

  TFT_AN_ZEIT = millis();
}

