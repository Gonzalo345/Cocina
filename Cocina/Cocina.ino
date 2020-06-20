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


//-------------------------------------------------------------------------
WiFiClient espClient;
PubSubClient client(espClient);

//------------------------SETUP-----------------------------
void setup() {

  pinMode(caldera,    OUTPUT);
  pinMode(luzVerde,   OUTPUT);
  pinMode(luzAzul,    OUTPUT);
  pinMode(luzRoja,    OUTPUT);
  pinMode(movimiento, INPUT);

  digitalWrite(luzVerde,  HIGH);
  digitalWrite(luzAzul,   HIGH);
  digitalWrite(luzRoja,   HIGH);
  digitalWrite(caldera,   LOW);

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
  mov = digitalRead(movimiento);

  if (currentMillis - previousMillis >= 10000) { //envia la temperatura cada 10 segundos
    tomaLuminosidad(mov);
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


    previousMillis = currentMillis;

    Serial.println("Mensaje enviando: [" +  String(TEMPERATURA) + "] " + strtemp);
    client.publish(TEMPERATURA, valueStr);
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
      digitalWrite(caldera, HIGH);
    }
    if (payload[0] == '0') {
      digitalWrite(caldera, LOW);
    }
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
    if (client.connect("", PLACA, PASSWORD))
    {
      Serial.println("conectado");
      client.subscribe(CALDERA);
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

void tomaLuminosidad(bool mov){

  int sensorValue;
  float voltage;

  if ( mov == 1 ){
    Serial.print("Mov = Si ");
    sensorValue = analogRead(A0); //Lectura del ADC
    voltage = sensorValue * (3.3 / 1023.0); //escalamos a voltaje
    Serial.print("    ADC= ");
    Serial.print(sensorValue);
    if (sensorValue < 256){ sensorValue = 256;}
    if (sensorValue > 256 && sensorValue < 512) { sensorValue = 512;}
    if (sensorValue > 512 && sensorValue < 768) { sensorValue = 768;}
    if (sensorValue > 768 && sensorValue < 1024){ sensorValue = 1024;}

    analogWrite(luzAzul, 1023 - sensorValue);
    analogWrite(luzVerde,1023 - sensorValue);
    analogWrite(luzRoja, 1023 - sensorValue);

    Serial.print("  LED= ");
    Serial.print(1023 - sensorValue);
    Serial.print("  Voltaje= ");
    Serial.println(voltage);
  }
  else {
    Serial.println("Mov = No ");
    analogWrite(luzAzul, 1023 );
    analogWrite(luzVerde,1023 );
    analogWrite(luzRoja, 1023 );
  }

}
