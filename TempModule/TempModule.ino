/*
 * BE482 Sensor Logger to integrate w/ all known sensors
 * Design of a Non-Destructive Temperature Sensing Module for Food Preservation
 * (should) work with 
 *  -2 IR sensor readings, (I2C)
 *  -1 thermocouple readings, (SPI)
 *  -1 humidity reading (I2C)
 *  Untested due to COVID-19 outbreak
 */

#include <Adafruit_MAX31856.h> //thermocouple
#include <Adafruit_MLX90614.h> //infrared
#include <Adafruit_Sensor.h>  //humidity dependency
#include <DHT.h> //humidity sensor
#include <DHT_U.h>  //humidity dependency
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#define LOG_INTERVAL 10000 //log per second
#define SYNC_INTERVAL 1000
#define TCAADDR 0x70 //define multiplexer address

uint32_t syncTime = 0;

#define ECHO_TO_SERIAL    1 //turn on echoing
#define SERIAL_TO_VIEW    1 //turn on comm to ABE_VIEW
#define WAIT_TO_START     0 //Wait for serial input in setup()
#define ANALOG_TO_SSR     0 //input 1 to turn PID on

#define redLEDpin 2
#define greenLEDpin 3

#define MAX1CS 53      //was 53
#define MAXSDI 11      //was 51
#define MAXSDO 12      //was 50
#define MAXSCK 13      //was 52
#define DHTPIN 49      //Digital pin connected to DHT sensor
#define SSRPIN 0       //Analog pin connected to SSR for PID control
#define aref_voltage 3.3
#define DHTTYPE DHT22

//define RTC
RTC_PCF8523 RTC;

const int chipSelect = 10;
char command;

File logfile; //define logging file

//define thermo sensors
//Use software SPI: CS, DI, DO, CLK
//digital pins
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(53, 11, 12, 13); //Box Underside

//IR thermosensor
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614(1);
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614(2);
Adafruit_MLX90614 mlx3 = Adafruit_MLX90614(3);

//Humidity sensor
DHT_Unified dht(DHTPIN, DHTTYPE);

byte ATuneModeRemember=2;
double input=25, output=50, setpoint=5;
double kp=2, ki=.5, kd=2;
double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;
boolean tuning = false;
unsigned long modelTime, serialTime;
int n;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
PID_ATune aTune(&input, &output);
boolean useSimulation = false;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while(1);
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  Serial.begin(115200);
  //Serial.println("Test");
#if SERIAL_TO_VIEW
  Serial3.begin(115200);
#endif //SERIAL_TO_VIEW

  //use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);

  //
  //initialize MAX31856 + set thermocouple type
  maxthermo.begin();
  maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);

  //print thermocouple type to Serial (not logged)
  Serial.print("Thermocouple type: ");
  switch (maxthermo.getThermocoupleType() ) {
    case MAX31856_TCTYPE_B: Serial.println("B Type"); break;
    case MAX31856_TCTYPE_E: Serial.println("E Type"); break;
    case MAX31856_TCTYPE_J: Serial.println("J Type"); break;
    case MAX31856_TCTYPE_K: Serial.println("K Type"); break;
    case MAX31856_TCTYPE_N: Serial.println("N Type"); break;
    case MAX31856_TCTYPE_R: Serial.println("R Type"); break;
    case MAX31856_TCTYPE_S: Serial.println("S Type"); break;
    case MAX31856_TCTYPE_T: Serial.println("T Type"); break;
    case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
    case MAX31856_VMODE_G32: Serial.println("Voltage x32 Gain mode"); break;
    default: Serial.println("Unknown"); break;
  }
//

  tcaselect(1);
  if(!mlx1.begin()){
    error("MLX at S1 not found. Please check connections.");
  }

  tcaselect(2);
  if(!mlx2.begin()){
    error("MLX at S2 not found. Please check connections.");
  }

  tcaselect(3);
  if(!mlx3.begin()){
    error("MLX at S3 not found. Please check connections.");
  }
  
  dht.begin();
  
#if WAIT_TO_START
  //possible plan: ask for thermocouple type to start
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

  // initialize the SD card
  Serial.print("Initializing SD card...");
  //make sure that the default chip select pin is set to output, even if you don't use it
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  //see if the card is present and can be initialized
  //int s = SD.begin(chipSelect); // bad function
 /* if(SD.begin(chipSelect)){
    Serial.println("true");}*/
  if(!SD.begin(chipSelect)){
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");

  //create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  if (! logfile) {
    error("couldnt create file");
  }

  //Confirm logging to serial
  Serial.print("Logging to: ");
  Serial.println(filename);

  // connect RTC to I2C (Wire)
  Wire.begin();  
  if (!RTC.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }

  //Initialize column headers in csv
  //vacate for 2 IR, 4 TC, color data (TSC), and humidity
  logfile.println("datetime,IR1,IR2,Box Underside,Freezer Ambient,Box Ambient,Above Meat,colorTemp,lux,r,g,b,Humidity");
  //logfile.println("datetime,TCJ,TTC");
#if ECHO_TO_SERIAL
  Serial.println("datetime,IR1,IR2,Box Underside,Freezer Ambient,Box Ambient,Above Meat,colorTemp,lux,r,g,b,Humidity");
  //Serial.println("datetime,TCJ,TTC");
#endif //ECHO_TO_SERIAL

  //Calibrate time (must readjust "17(s)" in final code)
  RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  DateTime t = DateTime(RTC.now().unixtime()+17);
  RTC.adjust(t);

  myPID.SetMode(AUTOMATIC);
  if(tuning){
    tuning = false;
    changeAutoTune();
    tuning=true;
  }

  serialTime=0;
}

void loop() {
  //fetch time
  DateTime now = RTC.now();

  //# mlx sensors
  n=0;

#if SERIAL_TO_VIEW
  //Read input from the android app.
  while (Serial3.available()) {
    command = Serial3.read();
    switch (command) {
      case 's':// code from Android that it has just connected- requests information about how to populate the Android Interface
        Populate_Android_Interface();
        break;
      case 'b':// code from Android that one of the buttons on the Android interface was pushed
        Parse_Button();
        break;
      case 'c':// code from Android that one of the control input values has been updated
        Parse_Control_Input();
        break;
      case 'r':// code from Android that one of the Radio buttons was selected
        Parse_Radio_Buttons();
        break;
      default: break;
    }
  }
#endif //SERIAL_TO_VIEW
  
  //fetch humidity data
  sensors_event_t event;
  dht.humidity().getEvent(&event);

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));

  //written LOW at end to blink
  digitalWrite(greenLEDpin, HIGH);

  //print timestamp
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  if(now.month()<10){
    logfile.print("0");
  }
  logfile.print(now.month(), DEC);
  logfile.print("/");
  if(now.day()<10){
    logfile.print("0");
  }
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  if(now.hour()<10){
    logfile.print("0");
  }
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  if(now.minute()<10){
    logfile.print("0");
  }
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  if(now.second()<10){
    logfile.print("0");
  }
  logfile.print(now.second(), DEC);
  logfile.print('"');
  logfile.print(", ");
  
#if ECHO_TO_SERIAL
  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  if(now.month()<10){
    Serial.print("0");
  }
  Serial.print(now.month(), DEC);
  Serial.print("/");
  if(now.day()<10){
    Serial.print("0");
  }
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  if(now.hour()<10){
    Serial.print("0");
  }
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  if(now.minute()<10){
    Serial.print("0");
  }
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  if(now.second()<10){
    Serial.print("0");
  }
  Serial.print(now.second(), DEC);
  Serial.print('"');
  Serial.print(", ");
#endif //ECHO_TO_SERIAL

#if SERIAL_TO_VIEW
  Serial3.print("d0\t");
  //Serial3.print('"');
  Serial3.print(now.year(), DEC);
  Serial3.print("/");
  if(now.month()<10){
    Serial3.print("0");
  }
  Serial3.print(now.month(), DEC);
  Serial3.print("/");
  if(now.day()<10){
    Serial3.print("0");
  }
  Serial3.print(now.day(), DEC);
  Serial3.print(" ");
  if(now.hour()<10){
    Serial3.print("0");
  }
  Serial3.print(now.hour(), DEC);
  Serial3.print(":");
  if(now.minute()<10){
    Serial3.print("0");
  }
  Serial3.print(now.minute(), DEC);
  Serial3.print(":");
  if(now.second()<10){
    Serial3.print("0");
  }
  Serial3.print(now.second(), DEC);
  //Serial3.print('"');
  //Serial3.print(", ");
  Serial3.print('\t');
#endif //SERIAL_TO_VIEW

  //IR1
  tcaselect(1);
  logfile.print(mlx1.readObjectTempC());
  logfile.print(", ");
  float IR1 = mlx1.readObjectTempC();
  n++;
#if ECHO_TO_SERIAL
  Serial.print(mlx1.readObjectTempC());
  Serial.print(", ");
#endif //ECHO_TO_SERIAL
#if SERIAL_TO_VIEW
  Serial3.print("d1\t");
  Serial3.print(mlx1.readObjectTempC());
  Serial3.print("\t");
#endif //SERIAL_TO_VIEW


  //IR2
  tcaselect(2);
  logfile.print(mlx2.readObjectTempC());
  logfile.print(", ");
  float IR2 = mlx2.readObjectTempC();
  n++;
#if ECHO_TO_SERIAL
  Serial.print(mlx2.readObjectTempC());
  Serial.print(", ");
#endif //ECHO_TO_SERIAL
#if SERIAL_TO_VIEW
  Serial3.print("d2\t");
  Serial3.print(mlx2.readObjectTempC());
  Serial3.print("\t");
#endif //SERIAL_TO_VIEW

  //IR3
  tcaselect(3);
  logfile.print(mlx3.readObjectTempC());
  logfile.print(", ");
  float IR3 = mlx3.readObjectTempC();
  n++;
#if ECHO_TO_SERIAL
  Serial.print(mlx3.readObjectTempC());
  Serial.print(", ");
#endif //ECHO_TO_SERIAL
#if SERIAL_TO_VIEW
  Serial3.print("d3\t");
  Serial3.print(mlx3.readObjectTempC());
  Serial3.print("\t");
#endif //SERIAL_TO_VIEW

  //thermocouple 1 - Box Ambient
  maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);
  logfile.print(maxthermo.readThermocoupleTemperature());
  logfile.print(", ");
#if ECHO_TO_SERIAL
  Serial.print(maxthermo.readThermocoupleTemperature());
  Serial.print(", ");
#endif //ECHO_TO_SERIAL
#if SERIAL_TO_VIEW
  Serial3.print("d4\t");
  Serial3.print(maxthermo.readThermocoupleTemperature());
  Serial3.print("\t");
#endif //SERIAL_TO_VIEW

  //Humidity Sensor HDT
  logfile.println(event.relative_humidity);
#if ECHO_TO_SERIAL
  Serial.println(event.relative_humidity);
#endif //ECHO_TO_SERIAL
#if SERIAL_TO_VIEW
  Serial3.print("d5\t");
  Serial3.print(event.relative_humidity);
  Serial3.print("\t");
#endif //SERIAL_TO_VIEW

  input = (IR1 + IR2 + IR3)/n;
  if(tuning){
    byte val = (aTune.Runtime());
    if(val!=0){
      tuning = false;
    }
    if(!tuning){
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
    else myPID.Compute();
  }

#if ANALOG_TO_SSR
  analogWrite(SSRPIN, output);
#endif //ANALOG_TO_SSR

  digitalWrite(greenLEDpin, LOW);

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  
  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);

#if SERIAL_TO_VIEW
  Serial3.print('u'); //refresh VIEW
#endif //SERIAL_TO_VIEW
}

void Populate_Android_Interface()
{
  //First label and populate available buttons (8 available)
  //Serial.print("b0\tActivate\t");  // activates button "0" on Android interface, with a label "Activate"
  //Serial.print("b1\tButton 1\t");
  //Serial.print("b2\tButton 2\t");
  //Serial.print("b3\tButton 3\t");
  //Serial.print("b4\tButton 4\t");
  //Serial.print("b5\tButton 5\t");
  //Serial.print("b6\tButton 6\t");
  //Serial.print("b7\tButton 7\t");
  
  //Populate & configure available numerical controls on interface (total of 8 are available)
  //Parameters separated by tabs are 1)control number, 2)decimal places, 3)minimum value, 4)default value, 5)maximum value, 6)name.
  //Serial.print("c0\t1\t1\t5\t10\tRevolutions\t");  // activate the numeric control "Revolutions" input on the Android interface
  //Serial.print("c1\t0\t1\t1000\t1000\tControl 1)\t");
  //Serial.print("c2\t0\t1\t1000\t1000\tControl 2)\t");
  //Serial.print("c3\t0\t1\t1000\t1000\tControl 3)\t");
  //Serial.print("c4\t0\t1\t1000\t1000\tControl 4)\t");
  //Serial.print("c5\t0\t1\t1000\t1000\tControl 5)\t");
  //Serial.print("c6\t0\t1\t1000\t1000\tControl 6)\t");
  //Serial.print("c7\t0\t1\t1000\t1000\tControl 7)\t");

  //Populate available data headings (total of 16 available). These correspond to the data values exported in loop().
  //Parameter separated by tabs are 1) check box to plot data '1' or none '0' and 2) heading name.
  Serial3.print("h0\t0\tTime\t");  // activate headings for data exported in loop() with "d#" commands.
  Serial3.print("h1\t1\tMLX1 (C)\t");
  Serial3.print("h2\t1\tMLX2 (C)\t");
  Serial3.print("h3\t1\tMLX3 (C)\t");
  Serial3.print("h4\t1\tThermocouple (C)\t");
  Serial3.print("h5\t0\tHumidity\t");
  //Serial.print("h7\t1\tHeading 1\t");

  //Populate a graph on the Android interface.
  Serial3.print("g1\t");

  //Populat radio buttons and options.
  //Parameters separated by tabs are 1) button name, 2) selction name 1, 3) selction name 2, 4) selction name 3, 5) selction name 4.
  //If using less than 4 selections for a radio button, keep all tabs in command but include no characters between the later tabs.
  //Serial.print("r0\tMode\tOpt1\tOpt2\tOpt3\tOpt4\t");  // At total of 4 selections for each radio button
  //Serial.print("r1\tSpecification\tRevolutions\tVolume\t\t\t");
}

void Parse_Button()
{
  int button_id = Serial3.parseInt();
  switch (button_id) {
   // case 0:
    //  ACTIVATE = HIGH;  // toggle the state of pipette activation button
   //   break;
    default:
      break;
  }
}

void Parse_Control_Input()
{
  float value;
  int control_id = Serial3.parseInt();
  switch (control_id) {
    case 0:
      
      break;
    //case 1:
     // volume = Serial.parseFloat();
      //break;
     // case 2:
     // vol_m = Serial.parseFloat();
     // break;
     // case 3:
     // vol_o = Serial.parseFloat();
     // break;
    default:
      value = Serial3.parseFloat();
      break;
  }
}

void Parse_Radio_Buttons() {
  int rb_id = Serial3.parseInt();
  switch (rb_id) {
    //case 11:  // this is the first radio button in our first radio group
    //  user_input = 1; // clockwise motion
    //  break;
    //case 12: // this is the second radio button in our first radio group
    //  user_input = 2; // couter clockwise motion
    //  break;
   // case 13: // this is the third radio button in our first radio group
   //   user_input = 3; // pipette mode
  //    break;
   // case 14: // this is the third radio button in our first radio group
   //   user_input = 0; // revolutions mode
   //   break;
    default:
      break;
  }
}

void changeAutoTune(){
  if(!tuning){ //set output to desired starting frequency
    output = aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else{ //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start){
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}
