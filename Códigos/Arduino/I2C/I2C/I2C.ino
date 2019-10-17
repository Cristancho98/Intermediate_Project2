#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire ourWire(13);
DallasTemperature sensors(&ourWire);

#define MASTER_ADDRESS 0x04
#define SLAVE_ADDRESS 0X03
int x=0;

void setup(){
    pinMode(8, OUTPUT);
    Serial.begin(9600);
    Wire.begin(MASTER_ADDRESS);
    Wire.onReceive(receiveData);
    Wire.onRequest(rpi);
    sensors.begin();
}
void loop(void){
  sensors.requestTemperatures();
  x=sensors.getTempCByIndex(0);
  Serial.println(x);
  delay(1000);
  Wire.onRequest(rpi);
}
void receiveData(int byteCount){
    while(Wire.available()) {
         byteCount = Wire.read();
         Serial.println(byteCount);
         if(byteCount==1){
         digitalWrite(8,HIGH);
         }
         if(byteCount==0){
           digitalWrite(8,LOW);
         }        
     }
}
void rpi(){
  Wire.write(x);
}
