# Intermediate Project #2 : Monitoring Iot based system for indoor applications using a controller of two levels

* Realizado por: Daniel Cristancho
* Universida Sergio Arboleda, Bogotá-Colombia
* Ingeniería electrónica

## INTRODUCCIÓN
Este pryecto nace como compendio de todo lo aprendido hasta la fecha, se plantea desarrollar un sistema de control de temperatura de doble nivel que permia visualizar los datos históricos captados por el sensor DS18B20 en una página web Hotspot.

## MATERIALES 
* 1 PC
* Chrome/Mozilla/Internet Explorer
* Acceso a servicio de Internet
* 1 actuador LED
* 1 Raspberry Pi
* 1 sensor digital DS18B20 (https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf)
* 1 protoboard y cables
* Elementos eléctricos para el circuito de acondicionamiento del sensor DS18B20 y actuador LED

## OBJETIVOS 
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
### DIAGRAMA DE CONEXIÓN DEL SENSOR
%% <a href="I2C"><img src="../master/Imagenes/Esquemático.PNG"  width="50%" align="justify"></a>

### CRITERIOS DE DISEÑO 

1. La solución debe usar el sensor DS18B20 para medir la temperatura
2. Los datos recibidos por el sensor deben almacenarse en en una base de datos local en la rapsberry
3. Generar la conexión maestro-esclavo utilizando el protocolo de comunicación I2C.
4. Generar una alerta visual cuando el dato fue entregado exitosamente 
5. Realizar el Script en raspbian por comunicación SSH 

### DIAGRAMA DE BLOQUES DE LA SOLUCIÓN

%% <a href="I2C"><img src="../master/Diagramas/BLOQUES.PNG"  width="50%" align="justify"></a>

## COMUNICACIÓN I2C
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
**DIAGRAMA DE CONEXIÓN** 

%% <a href="I2C"><img src="../master/Diagramas/Conexión.PNG"  width="70%" align="center"></a>

**CONFIGURACIÓN DE RASPBERRY COMO ACCESS POINT**
* sudo apt install dnsmasq hostapd
* sudo systemctl stop dnsmasq
* sudo systemctl stop hostapd
* sudo nano /etc/dhcpcd.conf

copiar en el final del archivo 

    interface wlan0
           static ip_address=192.168.4.1/24
           nohook wpa_supplicant
           
* sudo service dhcpcd restart
* sudo service dhcpcd restart
* sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig
* sudo nano /etc/dnsmasq.conf

COPIAR EN EL ARCHIVO

    interface=wlan0      # Use the require wireless interface - 
    usually wlan0
    dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h

**Configuración del software host del punto de acceso (hostapd)**

* sudo nano /etc/hostapd/hostapd.conf

AGREGAR ESTO

    interface=wlan0
    driver=nl80211
    ssid= NOMBREDELARED
    hw_mode=g
    channel=7
    wmm_enabled=0
    macaddr_acl=0
    auth_algs=1
    ignore_broadcast_ssid=0
    wpa=2
    wpa_passphrase=AardvarkBadgerHedgehog
    wpa_key_mgmt= CONTRASEÑADELARED
    wpa_pairwise=TKIP
    rsn_pairwise=CCMP

* sudo nano /etc/default/hostapd

reemplazar la linea #DAEMON_CONF 

    DAEMON_CONF="/etc/hostapd/hostapd.conf"

**PONERLO EN MARCHA**
* sudo systemctl unmask hostapd
* sudo systemctl enable hostapd
* sudo systemctl start hostapd
* sudo nano /etc/sysctl.conf

DESCOMENTE ESTA lÍNEA

    net.ipv4.ip_forward=1

* sudo iptables -t nat -A  POSTROUTING -o eth0 -j MASQUERADE
* sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"

EDITAR /etc/rc.local y agregar justo antes de exit 0

    iptables-restore < /etc/iptables.ipv4.nat

* Reiniciar raspberry

**SCRIPT DE PYTHON** 
     
    import time
    import sqlite3
    import datetime
    import smbus
    from time import sleep
    import time
    import RPi.GPIO as GPIO

    bus = smbus.SMBus(1)
    address = 0x04
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def InsertData(temp):
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        conn=sqlite3.connect('/home/pi/iot4.db')
        curs=conn.cursor()
        ##curs.execute("INSERT INTO temperatura (temperatura,fecha) VALUES ((?),(?))",(temp,st))
        conn.commit()
    broker="iot.eclipse.org"

    def on_message(client, userdata, message):
        time.sleep(1)

    while(True):

            temp=bus.read_byte(address) #Recibo la temperatura desde el arduino


            if(temp>25):
                print('Alerta temperatura alta : ',temp)
                temp = bus.write_byte(address, 0)
                InsertData(temp)
                time.sleep(2)

            else:
                print(temp)
                time.sleep(1)
                led=bus.write_byte(address,1)
  

**SCRIPT DE ARDUINO** 

    #include <Wire.h>
    #include <OneWire.h>
    #include <DallasTemperature.h>

    OneWire ourWire(2);
    DallasTemperature sensors(&ourWire);

    #define SLAVE_ADDRESS 0X04
    int x=0;

    void setup(){
        pinMode(13, OUTPUT);
        Serial.begin(9600);
        Wire.begin(SLAVE_ADDRESS);
        Wire.onReceive(receiveData);
        Wire.onRequest(rpi);
        sensors.begin();
    }
    void loop(void){
      sensors.requestTemperatures();
      x=sensors.getTempCByIndex(0);
      Serial.print("envio : ");
      Serial.println(x);
      delay(1000);
      Wire.onRequest(rpi);
    }
    void receiveData(int byteCount){

        while(Wire.available()) {
             byteCount = Wire.read();
             Serial.println(byteCount);
             if( byteCount==0 ){
             digitalWrite(13,HIGH);
             }
             if( byteCount==1 ){
               digitalWrite(13,LOW);
             }        
         }
    }
    void rpi(){
      Wire.write(x); 
    }
    

**PRUEBAS DE FUNCIONAMIENTO** 
