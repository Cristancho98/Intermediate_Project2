# Intermediate Project #2 : Monitoring Iot based system for indoor applications using a controller of two levels

* Realizado por: Daniel Cristancho
* Universida Sergio Arboleda, Bogotá-Colombia
* Ingeniería electrónica

## INTRODUCCIÓN



## MATERIALES //LISTO
* 1 PC
* Chrome/Mozilla/Internet Explorer
* Acceso a servicio de Internet
* 1 actuador LED
* 1 Raspberry Pi
* 1 sensor digital DS18B20 (https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf)
* 1 protoboard y cables
* Elementos eléctricos para el circuito de acondicionamiento del sensor DS18B20 y actuador LED

## OBJETIVOS //LISTO
* Afianzar los conceptos básicos asociados a Internet de las Cosas (IoT).
* Familiarizarse con el manaje de gestores de bases de datos y servidores Web en sistemas embebidos.
* Familiarizarse con una arquitectura de dos niveles compuesta por un controlador de bajo nivel y un
sistema embebido para la monitorización de temperatura.
* Desarrollar un controlador de dos niveles para la monitorización de temperatura utilizando la
plataforma Raspberry Pi, un microcontrolador (Arduino), el sensor digital de temperatura DS18B20, una
base de datos e interface Web local y remota.
* Implementar un servidor central en la plataforma Raspberry Pi compuesto al menos por un servidor web
y una base de datos (gestionada en SQLite).
* Implementar un controlador de bajo para la lectura del sensor DS18B20 y transferencia de las
mediciones al servidor central.
* Proponer y codificar un programa para la adquisición, almacenamiento, procesamiento y visualización
de los valores arrojados por el controlador de bajo utilizando scripts.
* Presentar y sustentar la solución alcanzada.
* Elaborar un informe del proyecto empleando el formato IEEE.

# DESARROLLO Y PROCEDIMIENTO


### CRITERIOS DE DISEÑO //LISTO

1. La solución debe usar el sensor DS18B20 para medir la temperatura
2. Los datos recibidos por el sensor deben almacenarse en en una base de datos local en la rapsberry
3. Generar la conexión maestro-esclavo utilizando el protocolo de comunicación I2C.
4. Generar una alerta visual cuando el dato fue entregado exitosamente 
5. Realizar el Script en raspbian por comunicación SSH 

### DIAGRAMA DE BLOQUES DE LA SOLUCIÓN

%% <a href="I2C"><img src="../master/Diagramas/Diagrama_Bloques.PNG"  width="50%" align="justify"></a>

## COMUNICACIÓN I2C //LISTO
1. Iniciar la raspberry y acceder a la consola de comando
2. Configurar la raspberry
  * sudo raspi-config
  * selesccionar interfacing options
  * seleccionar i2c 
  * seleccionar si
  * seleccionar aceptar
  * seleccionar finish
  
3. Verificar la comunicación I2C usando sudo i2cdetect -y 1
4. Instalar python sudo apt-get install python-smbus
5. Instalar librerias usando sudo apt-get install rpi.gpio 
6. Crear el script de Python usando sudo nano NOMBREDELSCRIPT.py (Tener presente en que directorio se guardo)
7. Compilar el script usando python NOMBREDELSCRIPT.py

**DIAGRAMA DE CONEXIÓN I2C** //LISTO

<a href="I2C"><img src="../master/Esquemáticos/I2C_esquematico.PNG"  width="70%" align="center"></a>

## PROCEDIMIENTO DE LA SOLUCIÓN

%%%%%%%%%%%%%%%

**DIAGRAMA DE CONEXIÓN** 

%% <a href="I2C"><img src="../master/Diagramas/Conexión.PNG"  width="70%" align="center"></a>

**SCRIPT DE PYTHON** 
     
     import time
     import paho.mqtt.client as mqtt
     import paho.mqtt.client as paho  
     import sqlite3
     import datetime
     import smbus
     from time import sleep
     channel = 1
     address = 0x04#arduino uno address
     address2 = 0x03
     bus = smbus.SMBus(channel) #se define el bus
     def InsertData(temp):#funcion para la insercion de datos en la base de datos local
         ts = time.time()
         st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S') 
         conn=sqlite3.connect('/home/pi/iot4.db') #ubicacion de la base de datos
         curs=conn.cursor()
         curs.execute("INSERT INTO temperatura (temperatura,fecha) VALUES ((?),(?))",(temp,st)) 
         conn.commit()    
     broker="iot.eclipse.org" #broker
      #define callback
     def on_message(client, userdata, message):
         time.sleep(1)    
     while(True):
             temp=bus.read_byte(address) #temperatura que le llega del arduino
             print(temp)
             InsertData(temp) 
             time.sleep(1)
             client= paho.Client("client-001") 
             client.on_message=on_message
             client.connect(broker)     #connect
             client.loop_start()      #start loop to process received messages
             print("publishing ")
             client.publish("house/bulb1",temp)    #publish
             time.sleep(4)
             client.disconnect()    #disconnect
             client.loop_stop()     #stop loop
             led=bus.write_byte(address,0)
             if(temp&#62;=25):
                 print('Alerta de temperatura alta')
                 temp=bus.write_byte(address,1)
                 InsertData(temp)        
                 time.sleep(2)
                 client= paho.Client("client-001") 
                 client.on_message=on_message
                 client.connect(broker)     #connect
                 client.loop_start()      #start loop to process received messages
                 print("publishing ")
                 client.publish("house/bulb1","Alerta de temperatura alta")     #publish
                 time.sleep(4)
                 client.disconnect()     #disconnect
                 client.loop_stop()       #stop loop   

**SCRIPT DE ARDUINO** 

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
    

**PRUEBAS DE FUNCIONAMIENTO** 
