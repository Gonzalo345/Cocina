/*
   Instancia MONITOREO cloudmqtt
*/
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include "DHTesp.h"     // DHT11 temperature and humidity sensor Predefined library
#include <NTPClient.h>
#include <WiFiUdp.h>

#define DHTTYPE DHT11   // DHT 11
#define dht_dpin 0      //GPIO-0 D3 pin of nodemcu

#define led_1 D0
#define led_2 D1        //D4 led de la placa
#define pinRele D8
#define pulsador D2


//-------------------VARIABLES GLOBALES--------------------------
int contconexion = 0;

const char *ssid = "Gonzalo5toA";
const char *password = "0141271819";


const long utcOffsetInSeconds = -10800;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

char   SERVER[50]   = "tailor.cloudmqtt.com";
int    SERVERPORT   = 12034;
String USERNAME = "Gonzalo";
char   PASSWORD[50] = "12345";

unsigned long previousMillis = 0;

char charPulsador [15];
String strPulsador;
String strPulsadorUltimo;

char PLACA[50];

char valueStr[15];
String strtemp = "";
String strext = "";
char TEMPERATURA[50];
char HUMEDAD[50];
char PULSADOR[50];
char LED_1[50];
char LED_2[50];
//char LED_3[50];
char RELE[50];
char EXTRACTOR[50];

int led2state;
float temp;

//----------------------WEBSERVER------------------------------------------
ESP8266WebServer server(80);
float Temperature;
float Humidity;
float Hora;
float Minutos;
//-------------------------------------------------------------------------
WiFiClient espClient;
PubSubClient client(espClient);


// Temperature sensonr

DHTesp dht;
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

  if (String(topic) ==  String(LED_1)) {
    if (payload[0] == '1') {
      digitalWrite(led_1, HIGH);
    }
    if (payload[0] == '0') {
      digitalWrite(led_1, LOW);
    }
  }
  
   if (String(topic) ==  String(LED_2)) {
    if (payload[0] == '1') {
      digitalWrite(led_2, HIGH);
    }
    if (payload[0] == '0') {
      digitalWrite(led_2, LOW);
    }
  }

  if (String(topic) ==  String(RELE)) {
    if (payload[0] == '1') {
      digitalWrite(pinRele, HIGH);
    }
    if (payload[0] == '0') {
      digitalWrite(pinRele, LOW);
    }
  }


}
void handle_OnConnect() {


  Temperature = dht.getTemperature();   // Gets the values of the temperature
  Humidity = dht.getHumidity();         // Gets the values of the humidity
  Hora = timeClient.getHours();
  Minutos = timeClient.getMinutes();
  server.send(200, "text/html", SendHTML(Temperature, Hora, Minutos, Humidity));

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
      client.subscribe(LED_1);
      client.subscribe(LED_2);
      client.subscribe(RELE);
      client.subscribe(EXTRACTOR);

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

//------------------------SETUP-----------------------------
void setup() {

  timeClient.update();
  delay(10);
  dht.setup(0, DHTesp::DHT11); // Connect DHT sensor to GPIO 17

  pinMode(pulsador, INPUT);
  pinMode(led_1, OUTPUT);
  pinMode(led_2, OUTPUT);
  pinMode(pinRele, OUTPUT);

  digitalWrite(led_1, LOW);
  digitalWrite(led_2, LOW);
  digitalWrite(pinRele, LOW);


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
    server.on("/", handle_OnConnect);
    server.onNotFound(handle_NotFound);

    server.begin();
    Serial.println("HTTP server started");
  }
  else {
    Serial.println("");
    Serial.println("Error de conexion");
  }

  client.setServer(SERVER, SERVERPORT);
  client.setCallback(callback);

  String temperatura = "/" + USERNAME + "/" + "temperatura";
  temperatura.toCharArray(TEMPERATURA, 50);

  String humedad = "/" + USERNAME + "/" + "humedad";
  humedad.toCharArray(HUMEDAD, 50);

  String pulsador = "/" + USERNAME + "/" + "pulsador";
  pulsador.toCharArray(PULSADOR, 50);

  String led_1 = "/" + USERNAME + "/" + "led_1";
  led_1.toCharArray(LED_1, 50);
  
  String led_2 = "/" + USERNAME + "/" + "led_2";
  led_2.toCharArray(LED_2, 50);

  String rele = "/" + USERNAME + "/" + "rele";
  rele.toCharArray(RELE, 50);

  String extractor = "/" + USERNAME + "/" + "extractor";
  extractor.toCharArray(EXTRACTOR, 50);


}
void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

//--------------------------LOOP--------------------------------
void loop() {

  server.handleClient();
  Serial.println(timeClient.getFormattedTime());

  unsigned long currentMillis = millis();
  

  delay(1000); //delay 1 Minuto

  if (currentMillis - previousMillis >= 20000) { //envia la temperatura cada 1 minuto
      
    timeClient.update();
    Serial.print(daysOfTheWeek[timeClient.getDay()]);
    Serial.print(", ");
    Serial.print(timeClient.getHours());
    Serial.print(":");
    Serial.println(timeClient.getMinutes());

   
    delay(dht.getMinimumSamplingPeriod());

    float humidity = dht.getHumidity();
    float temperature = dht.getTemperature();

    previousMillis = currentMillis;
    strtemp = String(temperature, 1); //1 decimal
    strtemp.toCharArray(valueStr, 15);
    Serial.println("Mensaje enviando: [" +  String(TEMPERATURA) + "] " + strtemp);
    client.publish(TEMPERATURA, valueStr);
    strtemp = String(humidity, 1); //1 decimal
    strtemp.toCharArray(valueStr, 15);
    Serial.println("Mensaje enviando: [" +  String(HUMEDAD) + "    ] " + strtemp);
    client.publish(HUMEDAD, valueStr);

    if (temp >= 30)
    {
      digitalWrite(led_2, HIGH);
      led2state = digitalRead(led_2);
      strext = String(led2state, 1); //1 decimal
      strext.toCharArray(valueStr, 15);
      client.publish(EXTRACTOR, valueStr);
    }
    else
    {
      digitalWrite(led_2, LOW);
      led2state = digitalRead(led_2);
      strext = String(led2state, 1); //1 decimal
      strext.toCharArray(valueStr, 15);
      client.publish(EXTRACTOR, valueStr);
    }
    
  }



  if (digitalRead(pulsador) == 0) {
    strPulsador = "SI";
  } else {
    strPulsador = "NO";
  }

  if (strPulsador != strPulsadorUltimo) { //envia el estado del pulsador solamente cuando cambia.
    strPulsadorUltimo = strPulsador;
    strPulsador.toCharArray(valueStr, 15);
    Serial.println("Mensaje enviando: [" +  String(PULSADOR) + "] " + strPulsador);
    client.publish(PULSADOR, valueStr);
  }

}
String SendHTML(float TempCstat, float Hora, float Minutos, float Humiditystat) {
  
  String ptr = "<!DOCTYPE html> <html>\n";

  ptr += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr += "<meta http-equiv=\"refresh\" content=\"30\">\n";
  ptr += "<link href=\"https://fonts.googleapis.com/css?family=Open+Sans:300,400,600\" rel=\"stylesheet\">\n";
  ptr += "<title>ESP8266 Informe del clima</title>\n";
  ptr += "<style>html { font-family: 'Open Sans', sans-serif; display: block; margin: 0px auto; text-align: center;color: #333333;}\n";
  ptr += "body{margin-top: 50px;}\n";
  ptr += "h1 {margin: 50px auto 30px;}\n";
  ptr += ".side-by-side{display: inline-block;vertical-align: middle;position: relative;}\n";
  ptr += ".humidity-icon{background-color: #3498db;width: 30px;height: 30px;border-radius: 50%;line-height: 36px;}\n";
  ptr += ".humidity-text{font-weight: 600;padding-left: 15px;font-size: 19px;width: 160px;text-align: left;}\n";
  ptr += ".humidity{font-weight: 300;font-size: 60px;color: #3498db;}\n";
  
  ptr += ".hora-icon{background-color: #26B99A;width: 30px;height: 30px;border-radius: 50%;line-height: 36px;}\n";
  
  ptr += ".hora-text{font-weight: 600;padding-left: 15px;font-size: 19px;width: 110px;text-align: left;}\n";
  ptr += ".hora{font-weight: 300;font-size: 60px;color: #3498db;}\n";
  
  ptr += ".temperature-icon{background-color: #f39c12;width: 30px;height: 30px;border-radius: 50%;line-height: 40px;}\n";
  ptr += ".temperature-text{font-weight: 600;padding-left: 15px;font-size: 19px;width: 160px;text-align: left;}\n";
  ptr += ".temperature{font-weight: 300;font-size: 60px;color: #f39c12;}\n";
  ptr += ".superscript{font-size: 17px;font-weight: 600;position: absolute;right: -20px;top: 15px;}\n";
  ptr += ".data{padding: 10px;}\n";
  ptr += "</style>\n";
  ptr += "</head>\n";
  ptr += "<body>\n";

  ptr += "<div id=\"webpage\">\n";

  ptr += "<h1>ESP8266 Informe del clima</h1>\n";
  ptr += "<div class=\"data\">\n";
  ptr += "<div class=\"side-by-side temperature-icon\">\n";
  ptr += "<svg version=\"1.1\" id=\"Layer_1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" x=\"0px\" y=\"0px\"\n";
  ptr += "width=\"9.915px\" height=\"22px\" viewBox=\"0 0 9.915 22\" enable-background=\"new 0 0 9.915 22\" xml:space=\"preserve\">\n";
  ptr += "<path fill=\"#FFFFFF\" d=\"M3.498,0.53c0.377-0.331,0.877-0.501,1.374-0.527C5.697-0.04,6.522,0.421,6.924,1.142\n";
  ptr += "c0.237,0.399,0.315,0.871,0.311,1.33C7.229,5.856,7.245,9.24,7.227,12.625c1.019,0.539,1.855,1.424,2.301,2.491\n";
  ptr += "c0.491,1.163,0.518,2.514,0.062,3.693c-0.414,1.102-1.24,2.038-2.276,2.594c-1.056,0.583-2.331,0.743-3.501,0.463\n";
  ptr += "c-1.417-0.323-2.659-1.314-3.3-2.617C0.014,18.26-0.115,17.104,0.1,16.022c0.296-1.443,1.274-2.717,2.58-3.394\n";
  ptr += "c0.013-3.44,0-6.881,0.007-10.322C2.674,1.634,2.974,0.955,3.498,0.53z\"/>\n";
  ptr += "</svg>\n";
  ptr += "</div>\n";
  
  ptr += "<div class=\"side-by-side temperature-text\">Temperatura</div>\n";
  ptr += "<div class=\"side-by-side temperature\">";
  ptr += (int)TempCstat;
  ptr += "<span class=\"superscript\">C</span></div>\n";
  ptr += "</div>\n";
  ptr += "<div class=\"data\">\n";
  ptr += "<div class=\"side-by-side humidity-icon\">\n";
  ptr += "<svg version=\"1.1\" id=\"Layer_2\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" x=\"0px\" y=\"0px\"\n\"; width=\"12px\" height=\"17.955px\" viewBox=\"0 0 13 17.955\" enable-background=\"new 0 0 13 17.955\" xml:space=\"preserve\">\n";
  ptr += "<path fill=\"#FFFFFF\" d=\"M1.819,6.217C3.139,4.064,6.5,0,6.5,0s3.363,4.064,4.681,6.217c1.793,2.926,2.133,5.05,1.571,7.057\n";
  ptr += "c-0.438,1.574-2.264,4.681-6.252,4.681c-3.988,0-5.813-3.107-6.252-4.681C-0.313,11.267,0.026,9.143,1.819,6.217\"></path>\n";
  ptr += "</svg>\n";
  ptr += "</div>\n";
  
  ptr += "<div class=\"side-by-side humidity-text\">Humedad</div>\n";
  ptr += "<div class=\"side-by-side humidity\">";
  ptr += (int)Humiditystat;
  ptr += "<span class=\"superscript\">%</span></div>\n";
  ptr += "</div>\n";

  ptr += "<div class=\"data\">\n";
  ptr += "<div class=\"side-by-side hora-icon\">\n";
  ptr += "<svg version=\"1.1\" id=\"Layer_2\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" x=\"0px\" y=\"0px\"\n\"; width=\"23px\" height=\"18px\" viewBox=\"0 0 24 24\">\n";
  ptr += "<path fill=\"#FFFFFF\" d=\"M12 2c5.514 0 10 4.486 10 10s-4.486 10-10 10-10-4.486-10-10 4.486-10 10-10zm0-2c-6.627 0-12 5.373-12 12s5.373 12 12 12 12-5.373 12-12-5.373-12-12-12zm5.848 12.459c.202.038.202.333.001.372-1.907.361-6.045 1.111-6.547 1.111-.719 0-1.301-.582-1.301-1.301 0-.512.77-5.447 1.125-7.445.034-.192.312-.181.343.014l.985 6.238 5.394 1.011z\"></path>\n";
  ptr += "</svg>\n";
  ptr += "</div>\n";
  
  ptr += "<div class=\"side-by-side hora-text\">Hora</div>\n";
  ptr += "<div class=\"side-by-side hora\">";
  ptr += (int)Hora;
  ptr += ":";
  ptr += (int)Minutos;
  ptr += "<span class=\"superscript\"> Hs</span></div>\n";
  ptr += "</div>\n";

  ptr += "</div>\n";
  ptr += "</body>\n";
  ptr += "</html>\n";
  return ptr;
}
