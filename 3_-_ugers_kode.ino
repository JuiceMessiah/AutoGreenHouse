#include "DHT.h" //Forskellige libraries, denne her er DHT.h
#include <SPI.h> //For at kunne kommunikere med SPI komponenter/devices, SPI: Serial Peripheral Interface
#include <SD.h> //Library til SD-kort
#include <Wire.h> //For at kunne kommunikere med I^2C, komponenter/devices 
#include <DS3231.h> //Library til sensor
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

#define DHTPINOUT 47     // what pin we're connected to

#define DHTPININ 49  // what pin we're connected to

#define DHTTYPE DHT11   // DHT 11, den lille blå humidity og temperatur sens<or
// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dhtin(DHTPININ, DHTTYPE);
DHT dhtout(DHTPINOUT, DHTTYPE);
DS3231 clock; //Klokken
RTCDateTime dt;

//Datalogger
const int chipSelect = 53;
const int interruptPin = 2;
volatile int yorn = 0; //initiliasere en volatile int, som kan afbryde og ikke kan røres
volatile int i = 0;

//CO2 sensor
int smokeA0 = A5;
int sensorThreshold = 125;
int blaeser = 4;

//Moisture sensor
int thresholdValue = 800;

//Waterpump
int ACWATERPUMP = 13; //You can remove this line, it has no use in the program.
int val; //This variable stores the value received from Soil moisture sensor.
int pump = 5;
int moist = A0;

void document() { //Dette er interrupt-funktionen, hvor yorn er sat til 1
  yorn = 1;
  digitalWrite(13, HIGH);

}

void datalog() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dhtin.readHumidity(); //Til første Humidity måling
  float t = dhtin.readTemperature(); //Til første temperatur måling
  float h2 = dhtout.readHumidity(); //Til anden Humidity måling
  float t2 = dhtout.readTemperature(); //Til anden temperatur

  // make a string for assembling the data to log:
  String dataString = "";
  dataString += String(dt.year); //Specificere dataString, til at inkludere alle følgende: dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second
  dataString += F("-");
  dataString += String(dt.month);
  dataString += F("-");
  dataString += String(dt.day);
  dataString += F(" ");
  dataString += String(dt.hour);
  dataString += F(":");
  dataString += String(dt.minute);
  dataString += F(":");
  dataString += String(dt.second);

  dataString += F("; "); //For at få print til at se pænere ud, derfor ": ", for at give mellemrum
  dataString += String(h, 0);
  dataString += F("; ");
  dataString += String(t, 0);


  dataString += F("; "); //For at få print til at se pænere ud, derfor ": ", for at give mellemrum
  dataString += String(h2, 0);
  dataString += F("; ");
  dataString += String(t2, 0);


  /*dt = clock.getDateTime();
    dtostrf(t * 10, 3, 0, TextT1);
    dtostrf(h * 10, 3, 0, TextH1);
    dtostrf(t2 * 10, 3, 0, TextT2);
    dtostrf(h2 * 10, 3, 0, TextH2);
    // make a string for assembling the data to log:
    char dataString[40] = "";
    sprintf(dataString, "%s-%s;%s;%s;%s;%s", clock.dateFormat("Y-m-d,H:i:s", dt), "23", TextT1, TextH1, TextT2, TextH2);
  */

  // read three sensors and append to the string:
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString); //Print dataString
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt"); //Hvis der kommer en error print dette
  }

  // For leading zero look to DS3231_dateformat example

  Serial.print("Raw data: ");
  Serial.print(dt.year);   Serial.print("-"); //For at printe år
  Serial.print(dt.month);  Serial.print("-"); //For at printe måned
  Serial.print(dt.day);    Serial.print(" "); //For at printe dag
  Serial.print(dt.hour);   Serial.print(":"); //For at printe time
  Serial.print(dt.minute); Serial.print(":"); //For at printe minut
  Serial.print(dt.second); Serial.println(""); //For at printe i sekunder

  delay(1000);

  clock.forceConversion();

  // print date
  Serial.print("Temperature: ");
  Serial.println(clock.readTemperature());
  delay(2000);

  Serial.println("ALARM 2 TRIGGERED!"); //Den er så triggered her
  if (i < 59) { //Hvis i er mindre end <indsæt tal>
    i += 1; //Ligger 10 til i hver gang, givet at den skal lave et interrupt hvert 10'ende minut
  }
  else {
    i = 0; //Sætter i til at være 0, da 60 ikke findes, og man tæller fra 00 igen efter 50 er nået.
  }
  clock.setAlarm2(0, 0, i,     DS3231_MATCH_M); //Sætter isAlarm2 til at være det nye i
  dt = clock.getDateTime();
  Serial.println("It works");
  Serial.print("Raw data: ");
  Serial.print(dt.year);   Serial.print("-");
  Serial.print(dt.month);  Serial.print("-");
  Serial.print(dt.day);    Serial.print(" ");
  Serial.print(dt.hour);   Serial.print(":");
  Serial.print(dt.minute); Serial.print(":");
  Serial.print(dt.second); Serial.println("");
  yorn = 0;
}
void setup() {

  //Datalogger setup//

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Initialize DS3231");;
  clock.begin();
  // Set sketch compiling time
  clock.setDateTime(__DATE__, __TIME__);
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  String header = "Date; 1st: Humidity (%); 1st: Temperatur °C; 2nd: Humidity (%); 2nd: Temperatur °C"; //For at se de 2 forskellige Humidity samt temperature, dette er bare til udskrift på serialmonitor
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(header); //Print header, som er specifiseret længere oppe ved "String header"
    dataFile.close(); //Luk filen
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt"); //Hvis filen ikke kan åbnes
  }
  Serial.println("DHTxx test!");

  dhtin.begin();
  dhtout.begin();

  clock.enableOutput(false); //Bare en clock-cycle som kører, den kører på interrupt-pinen

  //attachInterrupt(digitalPinToInterrupt(interruptPin), document, FALLING);
  digitalWrite(interruptPin, HIGH);
  // Disarm alarms and clear alarms for this example, because alarms is battery backed.
  // Under normal conditions, the settings should be reset after power and restart microcontroller.
  clock.armAlarm1(false);
  clock.armAlarm2(false);
  clock.clearAlarm1();
  clock.clearAlarm2();

  // Check alarm settings
  Serial.println(clock.isAlarm2()); //Printer isAlarm2, isAlarm2 den var på RTC'en, denne er indbygget

  dt = clock.getDateTime();
  i = dt.minute + 1;
  clock.setAlarm2(0, 0, i,     DS3231_MATCH_M);

  //Co2 setup//
  pinMode(smokeA0, INPUT);
  Serial.begin(9600);
  pinMode(blaeser, OUTPUT);

  //Moisture sensor setup//
  Serial.begin(9600);

  //Waterpump setup//
  pinMode(pump, OUTPUT); //Set pin 13 as OUTPUT pin
  pinMode(moist, INPUT); //Set pin 8 as input pin, to receive data from Soil moisture sensor.

}

void loop() {
  if (yorn) { //Hvis yorn påbegynd nedenstående
    datalog();
  }

  int analogSensor = analogRead(smokeA0);  //Read data from soil moisture sensor
  if (analogSensor > sensorThreshold) //Hvis værdien på co2 sensor er større end det threshold værdien så kør nedenstående
  {
    digitalWrite(blaeser, HIGH); //Start blæser, og kør blæseren indtil threshold værdien er nået
    delay(10000);
  }
  else
  {
    digitalWrite(blaeser, LOW); //Sluk blæser hvis co2 ikke er større end Threshold værdien
  }
  delay(400); //Wait for few second and then continue the loop.
  Serial.print("Co2 value: "); //For at printe det på serial monitor
  Serial.println(analogSensor); //Print værdien af co2 sensoren


  delay(1000);
  Serial.println(digitalRead(blaeser));
  int val = analogRead(moist);  //Read data from soil moisture sensor
  if (val > thresholdValue) //Fuldstændig samme princip med co2 sensoren, men hvor der bare aktiveres og slukkes for vandpumpen i stedet for blæseren
  {
    digitalWrite(pump, HIGH); 
  }
  else
  {
    digitalWrite(pump, LOW); 
  }
  delay(400); //Wait for few second and then continue the loop.

  Serial.print("Moisture value: ");
  Serial.println(val);


  delay(1000);
  Serial.println(digitalRead(pump));

  Serial.println(i); //For at printe i, som er alarm-tælleren, i holder styr på næste interrupt
  delay(3000);
  sleepNow();                                      // Call the sleep routine: sleepNow()
  After_Wakeup_Now();                            // do something after wakeup

}

void sleepNow ()
{
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  noInterrupts ();                                                      // make sure we don't get interrupted before we    sleep
  power_all_disable();                                                  // disables all modules
  MCUCR = 0x84;                                                      // BODS | BODSE
  MCUCR = 0x80;                                                      // Disable BOD
  ADCSRA = 0;                                                        // disable ADC
  sleep_enable ();                                                      // enables the sleep bit in the mcucr register
  attachInterrupt (digitalPinToInterrupt (2), document, LOW);  // wake up on RISING level on D2
  interrupts ();                                                        // interrupts allowed now, next instruction WILL be executed
  // Clear the ledPin to indicate going to sleep
  sleep_cpu ();                                                         // here the device is put to sleep
}  // end of sleepNow

void After_Wakeup_Now() {
  sleep_disable ();
  power_all_enable();                                                     // first thing after waking from sleep:
  detachInterrupt (digitalPinToInterrupt (2));
  delay(100);// stop RISING interrupt on D2
  Serial.println ("Back from Sleep");
  delay(100);
  // Indicate the processor is Awake again
} // end of wakeup_Now
