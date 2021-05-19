// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Copyright 2015, 2016 Ideetron B.V.
//
// Modified by Brent Rubell for Adafruit Industries, 2018
//
// Modified by Ondřej Knebl for LoRaWAN Meteostation, 2021
/*************************************************************************/

#include <TinyLoRa.h>
#include <SPI.h>
TinyLoRa lora = TinyLoRa(7, 8, 4);  // Pinout for Adafruit Feather 32u4 LoRa

// Cayenne Low Power Payload (LPP)
#include <CayenneLPP.h>
CayenneLPP lpp(51);

// Thermometer and humidity meter DHT22
#include <TroykaDHT.h>
DHT dht(12, DHT22);             // pin and DTH type

// Thermometer DS18B20
#include <OneWire.h> 
#include <DS18B20.h>
#define ONE_WIRE_BUS 23         // Data wire is plugged into pin 11 on the Arduino  
OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices
DS18B20 sensors(&oneWire);      // Pass our oneWire reference to Dallas Temperature. 

// Light intensity BH1750
#include <BH1750.h>             // OneTime-BH1750
BH1750 luxSenzor = BH1750();    // initialization of the BH1750 sensor from the library

// Battery
#define batteryPin A1


//--------------------------------------------------------------------------------


// Time
unsigned long previousMillis = 0;   // previous time
const long interval = 1000;         // interval 1000 ms
unsigned int countSeconds = 1;      // counting seconds


// Thermometer and humidity meter DHT22
float humi = 0.0;                   // variable for humidity
float temp = 0.0;                   // variable for adding temperatures
unsigned int numberOfSamplesDHT = 0;// variable for number of measured samples by DHT


// Thermometer DS18B20
float groundTemp = 0.0;             // variable for ground temperature


// Light intensity BH1750
unsigned long lightIntensity = 0;    // variable for light intensity


// Amount of precipitation
const int rainPin = 10;                 // sensor connected to digital pin 10
unsigned int switchState = 0;           // switch state inside sensor     
unsigned int switchPreviousState = 1;   // previous switch state inside sensor
float rain = 0.0;                       // variable for amount of precipitation


// Wind direction
const int analogPin = 0;                    // sensor connected to analog pin 0
unsigned int raw = 0;                       // resolution of the analog to digital converter 0-1023
unsigned int windDirection = 0;             // variable for wind direction
int directionArray[8] = {0,0,0,0,0,0,0,0};  // wind direction array
int directionDegreesArray[8] = {0,45,90,135,180,225,270,315};  // wind direction array
unsigned int maxValueDirection = 0;         // maximum value in wind direction array


// Wind speed
const int windSpeedPin = 9;             // sensor connected to digital pin 9
unsigned int switchState2 = 0;          // switch state inside sensor
unsigned int switchPreviousState2 = 1;  // previous switch state inside sensor
unsigned int impulses = 0;              // number of impulses
float addSpeedOfWind = 0.0;             // variable for adding wind speed 
float speedOfWind = 0.0;                // variable for wind speed 
float maxSpeedOfWind = 0.0;             // variable for maximum wind speed


// Battery
float batteryVoltage = 0.0;

//----------------------------------------------------------------------------------------


// Network Session Key (MSB)
uint8_t NwkSkey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Application Session Key (MSB)
uint8_t AppSkey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Device Address (MSB)
uint8_t DevAddr[4] = { 0x00, 0x00, 0x00, 0x00 };



void do_send() {
        lpp.reset();
        lpp.addTemperature(1, temp / numberOfSamplesDHT);       // add the average temperature into channel 1
        lpp.addTemperature(2, groundTemp / 60);                 // add the average ground temperature into channel 2
        lpp.addRelativeHumidity(3, humi / numberOfSamplesDHT);  // add the average humidity into channel 3
        lpp.addLuminosity(4, uint16_t(lightIntensity / 10));    // add the average light intensity into channel 4
        lpp.addAnalogInput(5, rain);                            // add the amount of precipitation into channel 5
        lpp.addAnalogInput(6, float(windDirection));            // add the wind direction into channel 6
        lpp.addAnalogInput(7, addSpeedOfWind / 600);            // add the average speed of wind into channel 7
        lpp.addAnalogInput(8, maxSpeedOfWind);                  // add the maximum speed of wind into channel 8
        lpp.addAnalogInput(9, batteryVoltage);                  // add the battery voltage into channel 9


        Serial.println("Sending LoRa Data...");
        lora.sendData(lpp.getBuffer(), lpp.getSize(), lora.frameCounter);
        Serial.print("Frame Counter: ");
        Serial.println(lora.frameCounter);
        lora.frameCounter++;

        resetValues();                                          // reset values
}



void resetValues() {                    // reset values
    temp = 0.0;
    humi = 0.0;  
    numberOfSamplesDHT = 0;
    groundTemp = 0.0;
    lightIntensity = 0.0;
    rain = 0.0;
    windDirection = 0;
    maxValueDirection = 0;
    resetDirectionArray();
    addSpeedOfWind = 0.0;
    maxSpeedOfWind = 0.0;
    batteryVoltage = 0.0;
}

void resetDirectionArray() {            // reset values
    for (byte i = 0; i < 8; i++) {
        directionArray[i] = 0;
    }
}




void measureTempsAndHumi(){
    dht.read();
    if(dht.getState() == DHT_OK){               // if there is no error get temperature and humidity from DHT22
        temp = temp + dht.getTemperatureC();    // adding current temperature to all measured temperatures
        humi = humi + dht.getHumidity();        // adding current humidity to all measured humidities
        numberOfSamplesDHT++;                   // add measured sample
    }

    sensors.requestTemperatures();                  // Send the command to get temperature readings from DS18B20
    groundTemp = groundTemp + sensors.getTempC();   // adding current ground temperature to all measured ground temperatures
}



void measureLuminosity(){
    lightIntensity = lightIntensity + luxSenzor.getLightIntensity();      // adding current light intensity to all measured light intensities
}




void measureWindDirection(){                        
    raw = analogRead(analogPin);                          // analog reading on the pin

    if(809 <= raw && raw <= 818){                         // if raw is in this range
        directionArray[0] = directionArray[0] + 1;        // add one value to value with index 0 in wind direction array - north
    }else if(958 <= raw && raw <= 966){                   
        directionArray[1] = directionArray[1] + 1;        // northeast
    }else if(1012 <= raw && raw <= 1019){                 
        directionArray[2] = directionArray[2] + 1;        // east
    }else if(1003 <= raw && raw <= 1010){
        directionArray[3] = directionArray[3] + 1;        // southeast
    }else if(990 <= raw && raw <= 997){                       
        directionArray[4] = directionArray[4] + 1;        // south
    }else if(905 <= raw && raw <= 914){                   
        directionArray[5] = directionArray[5] + 1;        // southwest
    }else if(523 <= raw && raw <= 533){               
        directionArray[6] = directionArray[6] + 1;        // west
    }else if(675 <= raw && raw <= 684){                      
        directionArray[7] = directionArray[7] + 1;        // northwest
  }
}

void findWindDirection() {                                  // find the highest value in direction array
    for (byte i = 0; i < 8; i++) {
        if (directionArray[i] > maxValueDirection) {        // if value with index i in direction array is higher than highest finded value  
            maxValueDirection = directionArray[i];          // set highest finded value to current value
            windDirection = directionDegreesArray[i];;      // set wind direction to degrees with index of highest finded value in direction array
        }
    }
}



void measureWindSpeed(){
    switchState2 = digitalRead(windSpeedPin);                       // read value from digital pin

    if(switchState2 == LOW){                                        // switch closed 
        switchPreviousState2 = 0;                                   // set that switch was closed
    }
    if(switchState2 == HIGH & switchPreviousState2 == LOW ){        // switch open
        impulses++;                                                 // add impuls
        switchPreviousState2 = 1;                                   // set that switch was opened
    }
}


void countWindSpeed(){
    speedOfWind = impulses * 0.33;                           // 1 impuls = 0.33 m/s

    if(speedOfWind > maxSpeedOfWind){                       // if current speed of wind is maximum speed of wind
        maxSpeedOfWind = speedOfWind;
    }

    addSpeedOfWind = addSpeedOfWind + speedOfWind;          // adding current speed of wind to all measured speeds of wind

    impulses = 0;                                           // set impulses to zero
    speedOfWind = 0;                                        // set speed of wind to zero
}



void measureRain(){
    switchState = digitalRead(rainPin);                             // read value from digital pin

    if(switchState == LOW){                                         // switch closed  
        switchPreviousState = 0;                                    // set that switch was closed
    }
    if(switchState == HIGH & switchPreviousState == LOW ){          // switch open  
        rain = rain + 0.30;                                         // adding 0,3 mm to total amount of rain
        switchPreviousState = 1;                                    // set that switch was opened
    }  
}


void measureBattery(){
    batteryVoltage = analogRead(batteryPin);
    batteryVoltage = batteryVoltage * 3.1;
    batteryVoltage = batteryVoltage * 3.3;    // reference voltage 3.3 V
    batteryVoltage = batteryVoltage / 1024;   // convert to voltage
}



void measure(){

    measureRain();
    measureWindSpeed();
 
    unsigned long currentMillis = millis();             // current millis

    if(currentMillis - previousMillis >= interval) {    // timer set to 1 second
        previousMillis = currentMillis;

        countWindSpeed();
      
        if(countSeconds % 10 == 0){                     // every 10 seconds
            measureWindDirection();
            measureTempsAndHumi();
        }

        if(countSeconds % 60 == 0){                     // every 1 minute
            measureLuminosity();
        }
      
        if(countSeconds % 600 == 0){                    // every 10 minutes
            findWindDirection();
            measureBattery();
            do_send();                                  // send measured data
            countSeconds = 0;                           // zero seconds counter
        }
        countSeconds++;                                 // + 1 second
    }
}




void setup()
{
  // Initialize LoRa
  Serial.print("Starting LoRa...");
  lora.setChannel(MULTI);                   // define multi-channel sending
  lora.setDatarate(SF7BW125);               // set datarate
  if(!lora.begin())
  {
    Serial.println("Failed");
    Serial.println("Check your radio");
    while(true);
  }
  Serial.println("OK");

  dht.begin();
  sensors.begin(); 
  luxSenzor.begin();

  pinMode(rainPin, INPUT_PULLUP);
  pinMode(windSpeedPin, INPUT_PULLUP);
}


void loop()
{
    measure();
}
