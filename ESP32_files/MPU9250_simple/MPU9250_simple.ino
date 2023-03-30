#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
////////////////////////////////
/////VARIABLES
////////////////////////////////


int contconexion=0;
const byte led_conn=27;

// MPU 9250
#define MPU9250_ADDRESS 0x68
#define TCA9548A_ADDRESS 0x70

long accelX, accelY, accelZ;
long gyroX, gyroY, gyroZ;
int16_t ax, ay, az, Tmp, gx, gy, gz;
const float AccelScaleFactor = 16384.0;
const float GyroScaleFactor =131;


//const char* ssid = "Ventana 2";
//const char* password = "2858351Qv"; 

const char* ssid = "Agrosavia2.4G";
const char* password = "Agrosavia"; 

int Id_client= 1; //Identificador del cliente
int p = 0; // Identificador del paquete enviado 
int cont_conexion=0; //Contador intentos conexion

WiFiUDP Udp;

void setup() {
////////////////////////SET UP ACELEROMETRO////////////////////////////////////////
  // put your setup code here, to run once:
   Serial.begin(115200);
   Wire.begin();
   Serial.println("setting up");
   //setupMPU();
 //Seting pin mode 
pinMode(led_conn, OUTPUT);
// pinMode(LED1, OUTPUT);



////////////////////////SET UP WIFI////////////////////////////////////////

 WiFi.mode(WIFI_STA);
 WiFi.begin(ssid,password);
 

  while (WiFi.status() != WL_CONNECTED and contconexion < 50) //Cuenta hasta 50 si no se puede conectar lo cancela
  {
    ++contconexion;
    delay(250);
    Serial.print(".");
  }
  if (contconexion < 50)
  {
    //Se está usando el código con IP dinámica por problemas con la ip fija. (funciona igualmente bien con IP dinámica)
    Serial.println("");
    Serial.println("WiFi conectado");
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.gatewayIP());
    digitalWrite(led_conn, HIGH);  //El led se queda predido si se conecta a la red
  }
  else
  {
    Serial.println("");
    Serial.println("Error de conexion");
    digitalWrite(led_conn, LOW); 
  }

}

String msg = "";
void loop() {



   // ---  Lectura acelerometro y giroscopio --- 
     for (int channel = 0; channel < 3; channel++) {
    
    Wire.beginTransmission(TCA9548A_ADDRESS);
    Wire.write(1 << channel);
    Wire.endTransmission();
    delay(1);

    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDRESS, 14, true);

    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    int16_t temp = Wire.read() << 8 | Wire.read();
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();


}



//  printData();

  Udp.beginPacket("192.168.0.108", 9001); 
  //Udp.beginPacket("127.0.0.1", 9001);  //// Esta ip es la ip del computador servidor y el puerto debe coincidir
  digitalWrite(led_conn, HIGH);
  Serial.println("Start envio paquete");
  //msg = String(ax) + "#" + String(ay) + "#" + String(az) + "#" + String(gx) + "#" + String(gy) + "#" + String(gz) +  "#" + String(ax) +  "#" + String(ay) +  "#" + String(az) + "#" + String(Id_client) +  "#" + String(p); //El mensaje completo contiene el id del cliente y el numero de paquete enviad
  msg = String(ax) + "#" + String(ay) + "#" + String(az) + "#" + String(gx) + "#" + String(gy) + "#" + String(gz) +  "#" + String(Id_client) +  "#" + String(p); //El mensaje completo contiene el id del cliente y el numero de paquete enviad
  Serial.print(msg);
  for (int i = 0; i < msg.length(); i++)
  {
    int old = micros();
    Udp.write(msg[i]);
    while (micros() - old < 87);
  }

  Udp.endPacket();
  Serial.println("End envio paquete");
  digitalWrite(led_conn, LOW);

  p = p + 1;
  delay(10);
}


void printData() {
   Serial.print(ax); Serial.print(" , ");
   Serial.print(ay); Serial.print(" , ");
   Serial.print(az); Serial.print(" , ");
   Serial.print(gx); Serial.print(" , ");
   Serial.print(gy); Serial.print(" , ");
   Serial.print(gz); Serial.print("\n");
   
   // Fin medicion
   Serial.println("");
}
