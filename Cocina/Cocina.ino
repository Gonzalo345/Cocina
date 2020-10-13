#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHTesp.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Cocina.h"


DHTesp dht;
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

unsigned long previousMillis = 0;

char PLACA[50];


String strext = "";
char TEMPERATURA[50];
char CALDERA[50];
char TEMPDESEADA[50];
bool HabilitarCaldera = 0;
float tempDeseada = 15.0;

//----------------------------------------------------------

int estadoPulsador = 0;             // Variable que almacena el estado actual del pulsador
int estadoAnteriorPulsador = 0;     // Variable que almacena el estado anterior del pulsador

//----------------------------------------------------------
WiFiClient espClient;
PubSubClient client(espClient);

//------------------------SETUP-----------------------------
void setup() {

    pinMode(releCaldera,	   OUTPUT);
    pinMode(luzVerde,   	   OUTPUT);
    pinMode(luzAzul,    	   OUTPUT);
    pinMode(luzRoja,    	   OUTPUT);
    pinMode(movimiento, 	   INPUT);

    digitalWrite(luzVerde,	    HIGH);
    digitalWrite(luzAzul,       HIGH);
    digitalWrite(luzRoja,       HIGH);
    digitalWrite(releCaldera,   LOW);

    delay(10);
    dht.setup(dht_dpin, DHTesp::DHT11); // Connect DHT sensor to GPIO 17

    // Inicia Serial
    Serial.begin(115200);
    Serial.println("");

    // Conexión WIFI
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED and contconexion < 50) { //Cuenta hasta 50 si no se puede conectar lo cancela
        ++contconexion;
        delay(500);
        Serial.print(".");
    }
    if (contconexion < 50) {
        Serial.println("WiFi conectado");
        Serial.println(WiFi.localIP());
    }
    else {
        Serial.println("");
        Serial.println("Error de conexion");
    }

    client.setServer(SERVER, SERVERPORT);
    client.setCallback(callback);

    String temperatura = "/" + USERNAME + "/" + "temperatura";
    temperatura.toCharArray(TEMPERATURA, 50);

    String caldera = "/" + USERNAME + "/" + "caldera";
    caldera.toCharArray(CALDERA, 50);

    String tempDeseada = "/" + USERNAME + "/" + "";
    tempDeseada.toCharArray(TEMPDESEADA, 50);

}

//--------------------------LOOP--------------------------------
void loop() {

    char valueStr[15];
    String strtemp;
    bool mov;


    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    unsigned long currentMillis = millis();

    //-----------------------------------------
    estadoPulsador = digitalRead(movimiento);      // Se lee y almacena el estado del pulsador

    if (estadoPulsador != estadoAnteriorPulsador) { // Se compara la variable estadoPulsador 
                                                      //// con su valor anterior
        if (estadoPulsador == HIGH) {                 // Si el estado cambió, se incrementa el contador
            Serial.println("Encedido");
            ledOn();
        } 
        else {                                        // Si el estado actual es LOW entonces el pulsador
            Serial.println("Apagado");                  //// paso de un estado ON a OFF
            ledOff();
        }
    }
                                                      // Se guarda el estado actual como ultimo estado
    estadoAnteriorPulsador = estadoPulsador; 
    //-----------------------------------------

    if (currentMillis - previousMillis >= 10000) { //envia la temperatura cada 10 segundos
        previousMillis = currentMillis;

        delay(dht.getMinimumSamplingPeriod());

        sensors.requestTemperatures();
        float tempPieza = sensors.getTempCByIndex(0);
        float tempExterior = sensors.getTempCByIndex(1);
        Serial.println("tempPieza    [" +  String(tempPieza) + " ºC] ");
        Serial.println("tempExterior [" +  String(tempExterior) + " ºC] ");
        float tempCocina = dht.getTemperature();
        float humidity = dht.getHumidity();
        tempCocina -= 1.0;
        strtemp =  "T:" + String(tempCocina, 1) + " H:" + String(humidity, 0);
        strtemp.toCharArray(valueStr, 15);


        Serial.println("Mensaje enviando: [" +  String(TEMPERATURA) + "] " + strtemp);
        client.publish(TEMPERATURA, valueStr);
        digitalWrite(releCaldera, LOW);
        if (HabilitarCaldera){
            if (tempDeseada > tempPieza){
                digitalWrite(releCaldera,   HIGH);
            }
            else
            digitalWrite(releCaldera,   LOW);
        }
    }

}


//------------------------CALLBACK-----------------------------
void callback(char* topic, byte* payload, unsigned int length) {

    char PAYLOAD[5] = "    ";

    Serial.print("Mensaje recibido: [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
        PAYLOAD[i] = (char)payload[i];
    }
    Serial.println(PAYLOAD);

    if (String(topic) ==  String(CALDERA)) {
        if (payload[0] == '1') {
            HabilitarCaldera = 1;
            Serial.println("Caldera habilitada");
        }
        if (payload[0] == '0') {
            HabilitarCaldera = 0;
            Serial.println("Caldera deshabilitada");
        }
    }

    if (String(topic) ==  String(TEMPDESEADA)) {
        tempDeseada = atof(PAYLOAD);
    }
}

//------------------------RECONNECT-----------------------------
void reconnect() {
    uint8_t retries = 3;
    // Loop hasta que estamos conectados
    while (!client.connected()) {
        Serial.print("Intentando conexion MQTT...");
        // Crea un ID de cliente al azar
        String clientId = "ESP8266Client-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        USERNAME.toCharArray(PLACA, 50);
        if (client.connect("", PLACA, PASSWORD)){
            Serial.println("conectado");
            client.subscribe(CALDERA);
            client.subscribe(TEMPDESEADA);
        }
        else {
            Serial.print("fallo, rc=");
            Serial.print(client.state());
            Serial.println(" intenta nuevamente en 5 segundos");
            // espera 5 segundos antes de reintentar
            delay(5000);
        }
        retries--;
        if (retries == 0) {
            // esperar a que el WDT lo reinicie
            while (1);
        }
    }
}

void ledOn(){

    int sensorValue;
    float voltage;
    Serial.print("Mov = Si ");
    sensorValue = analogRead(A0); //Lectura del ADC
    voltage = sensorValue * (3.3 / 1023.0); //escalamos a voltaje
    Serial.print("    ADC = ");
    Serial.print(sensorValue);
    if ( sensorValue < 5 )   { sensorValue = 1000;}
    else{						           //  64
        if ((sensorValue > 5) && (sensorValue < 10  ))   { sensorValue = 800 ;}	 //  128
        else{
            if ((sensorValue > 10) && (sensorValue < 30  ))   { sensorValue = 700;}	 //  256
            else{
                if ((sensorValue > 30) && (sensorValue < 40  ))   { sensorValue = 600;}	 //  512
                else{
                    if ((sensorValue > 40) && (sensorValue < 50  ))   { sensorValue = 500;}	 //  768
                    else{
                        if ((sensorValue > 50) && (sensorValue < 1024))   { sensorValue = 0  ;}	 // 1024}
                    }
                }
            }
        }
    }
    Serial.print("    ADC2= ");
    Serial.print(sensorValue);
    analogWrite(luzAzul, sensorValue );
    analogWrite(luzVerde, sensorValue);
    analogWrite(luzRoja, sensorValue );

}
void ledOff(){  
    analogWrite(luzAzul, 1024 );
    analogWrite(luzVerde,1024 );
    analogWrite(luzRoja, 1024 );
}
