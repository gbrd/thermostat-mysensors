#include <MySensor.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "HeatMode.h"

#define ONE_WIRE_BUS 4 // Pin where dallas sensor is connected 
#define PIECE "salon"
#define NODE_ID 2

#define TEMP_SENSOR_ID 0
#define HT_SENSOR_ID 1
#define LT_SENSOR_ID 2
#define MODE_SENSOR_ID 3
#define STATUS_SENSOR_ID 4
#define LS_SENSOR_ID 63 //load shedding


const int RELAY_PIN =  3; 
const float TEMP_MIN = 0.0;
const float TEMP_MAX = 30.0;


const int ledPin =  13; 
int loadSheddingLevel = 0;
const int myLoadSheddingLevel = 1;


unsigned long currentMillis = millis();
int ledState = LOW;
long previousMillisA = 0; 
long intervalA = 500;
long previousMillisB = 0; 
long intervalB = 10000;
long previousMillisC = 0; 
long intervalC = 600000;


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
MySensor gw;
float lastTemperature;
HeatMode mode;

MyMessage tempMsg(TEMP_SENSOR_ID,V_TEMP);
MyMessage htMsg(HT_SENSOR_ID,V_TEMP);
MyMessage ltMsg(LT_SENSOR_ID,V_TEMP);
MyMessage modeMsg(MODE_SENSOR_ID,V_VAR1);
MyMessage statusMsg(STATUS_SENSOR_ID,V_STATUS);
MyMessage lsStatusMsg(LS_SENSOR_ID,V_VAR1);


boolean heatingStateOn = false;

float offTemp = -273.1;
float ecoDeltaTemp = -3.2;
float hgTemp = 7.0;
float instructionTemp = 19.0;
float tolerance = 0.5;
float highThreshhold = instructionTemp+tolerance;
float lowThreshhold = instructionTemp-tolerance;


const char* zOffString = "Off";
const char* zConfString = "Conf";
const char* zEcoString = "Eco";
const char* zHGString = "HG";

HeatMode string2HeatMode(const char *input){
  if (strcmp (input,zOffString) == 0) return zOff;
  //if (strcmp (input,zConfString) == 0) return zConf;
  if (strcmp (input,zEcoString) == 0) return zEco;
  if (strcmp (input,zHGString) == 0) return zHG;
  return zConf;
}

const char *heatMode2String(HeatMode mode){
  switch (mode) {
    case zOff:  return zOffString;
    case zConf: return zConfString;
    case zEco: return zEcoString;
    case zHG: return zHGString;
    default: return zConfString;
  }
}

void recalcThresholds(){  
  float riTemp;
  switch(mode){
    case zOff: riTemp=offTemp;break;
    case zConf: riTemp=instructionTemp;break;
    case zEco: riTemp=instructionTemp+ecoDeltaTemp;break;
    case zHG: riTemp=hgTemp;break;
    default:riTemp=instructionTemp;
  }
  if(loadSheddingLevel >= myLoadSheddingLevel){
    riTemp=offTemp;
  }
  
  lowThreshhold = riTemp-tolerance;
  highThreshhold = riTemp+tolerance;

  sendHighThreshholdTemp();
  sendLowThreshholdTemp();
  sendMode();  
}
 
// setup()
void setup() {
  Serial.begin(115200); 
  Serial.println("start"); 
  
  pinMode(RELAY_PIN, OUTPUT);
  sensors.begin();   
  gw.begin(incomingMessage, NODE_ID, true);
  gw.sendSketchInfo("Thermostat", "1.0");
  
  int numSensors = sensors.getDeviceCount();
  if(numSensors != 1){
    Serial.println(String("WARN numSensors is not 1: ") + numSensors);  
  } 

  const char *tempDesc = (String("Temperature ") + String(PIECE)).c_str();
  gw.present(TEMP_SENSOR_ID, S_TEMP,tempDesc);
  const char *ltDesc = (String("Seuil haut ") + String(PIECE)).c_str();
  gw.present(LT_SENSOR_ID, S_TEMP,ltDesc);;
  const char *htDesc = (String("Seuil bas ") + String(PIECE)).c_str();
  gw.present(HT_SENSOR_ID, S_TEMP,htDesc);
  const char *modeDesc =  (String("Mode ") + String(PIECE)).c_str();
  gw.present(MODE_SENSOR_ID, S_CUSTOM,modeDesc);
  const char *statusDesc = (String("Status ") + String(PIECE)).c_str();
  gw.present(STATUS_SENSOR_ID, S_CUSTOM,statusDesc);
  
  
  turnOffHeating();    
  
  Serial.println("start2");
}

// loop()
void loop() {
  gw.process(); 
  sensors.requestTemperatures(); 
  currentMillis = millis();

  checkPeriodicA();
  checkPeriodicB();
  checkPeriodicC();

}


//TODO check old state to avoid to much switches ?
void turnOffHeating(){
  digitalWrite(RELAY_PIN, LOW);
  sendHeatingOff();
  heatingStateOn = false;
}

void turnOnHeating(){
  digitalWrite(RELAY_PIN, HIGH);    
  sendHeatingOn();
  heatingStateOn = true;  
}

void sendCurrentHeatingStatus(){
  if(heatingStateOn){
    sendHeatingOn();
  }else{
    sendHeatingOff();
  }
}

void sendHeatingOff(){
  gw.send(statusMsg.set(false));  
}
void sendHeatingOn(){
  gw.send(statusMsg.set(true));  
}

void sendTemp(){
  // Send in the new temperature
  gw.send(tempMsg.set(lastTemperature,1));  
}

void sendHighThreshholdTemp(){
  gw.send(htMsg.set(highThreshhold,1));
}
void sendLowThreshholdTemp(){
  gw.send(ltMsg.set(lowThreshhold,1));
}
void sendMode(){
  gw.send(modeMsg.set(heatMode2String(mode)));
}



void printTemp(float temp){
    // Affiche la température
    Serial.print("Temperature : ");
    Serial.print(lastTemperature);
    Serial.write(176); // caractère °
    Serial.write('C');
    Serial.println();
}



boolean getTemperature(){    
  // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric?sensors.getTempCByIndex(0):sensors.getTempFByIndex(0)) * 10.)) / 10.;
    // Only send data if temperature has changed and no error
    boolean changed = lastTemperature != temperature && temperature != -127.00;
    if (changed) {
      lastTemperature=temperature;
    }
    return changed;
}



void incomingMessage(const MyMessage &message) {
  if (message.sensor == MODE_SENSOR_ID && message.type==V_VAR1){
    mode = string2HeatMode(message.getString());
    recalcThresholds();
  }else if (message.sensor == STATUS_SENSOR_ID && message.type==V_STATUS){
    bool bmode = message.getBool();
    if(bmode){
      mode = zConf;
    }else{
      mode = zOff;
    }       
    recalcThresholds();
  }else if(message.sensor == TEMP_SENSOR_ID && message.type == V_TEMP){
    float it = message.getFloat();
    if(it > TEMP_MIN && it < TEMP_MAX){
      instructionTemp = it;
      recalcThresholds();
    }
  }else if((message.sensor == LT_SENSOR_ID || message.sensor == HT_SENSOR_ID) && message.type == V_TEMP){
    tolerance = message.getFloat();
    recalcThresholds();       
  }else if(message.sensor == LS_SENSOR_ID && message.type == V_VAR1){
    loadSheddingLevel = message.getInt();
    recalcThresholds();
    sendLoadSheddingStatus();
  }
}

void sendLoadSheddingStatus(){
  //send status of loadshedding 
  gw.send(lsStatusMsg.set(loadSheddingLevel)); 
}





//500 ms
void checkPeriodicA(){
  if(currentMillis - previousMillisA > intervalA) {
    previousMillisA = currentMillis;
    changeLedState();
  }
}

//10 s 
void checkPeriodicB(){
  if(currentMillis - previousMillisB > intervalB) {
    previousMillisB = currentMillis;

    // Lit la température ambiante 
    if(getTemperature()) {
      
      sendTemp();
      sendHighThreshholdTemp();
      sendLowThreshholdTemp();
      
      if (lastTemperature < lowThreshhold) {
        turnOnHeating();
      } else if (lastTemperature > highThreshhold){
        turnOffHeating();      
      } else {
        sendCurrentHeatingStatus(); //TODO send current temp instruction too ??
      }
    }
  }
}

//10 min
void checkPeriodicC(){
  if(currentMillis - previousMillisC > intervalC) {
    previousMillisC = currentMillis;   

    sendTemp();
    sendHighThreshholdTemp();
    sendLowThreshholdTemp();
    sendCurrentHeatingStatus();
    sendMode();
    sendLoadSheddingStatus();

  }
}

void changeLedState(){
    if (ledState == LOW){
      ledState = HIGH;
    }else{
      ledState = LOW;
    }
    digitalWrite(ledPin, ledState);
}
