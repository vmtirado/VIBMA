#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
////////////////////////////////
/////VARIABLES
////////////////////////////////


int contconexion=0;
const byte led_conn=27;

// MPU 9250
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

float Ax, Ay, Az;
long gyroX, gyroY, gyroZ;
int16_t ax, ay, az, Tmp, gx, gy, gz;
const float AccelScaleFactor = 8192.0;
const float GyroScaleFactor =65.5;

const float accelerationThreshold = 2.5 ;
const int numSamples = 62;
int samplesRead = numSamples;


//const char* ssid = "QV_2G";
//const char* password = "2858351qv"; 

//const char* ssid = "Agrosavia2.4G";
//const char* password = "Agrosavia"; 

const char* ssid = "FLIA-TIRADO-GOMEZ";
const char* password = "14080515"; 

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


 // Configurar acelerometro
 I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);
 // Configurar giroscopio
 I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_500_DPS);
 // Configurar magnetometro
 I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
 I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);


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

  //Once the samples needed are accquired stop the take
  while (samplesRead == numSamples) {
    Serial.println("Estoy en hold");
    
   uint8_t buff[14];
   I2Cread(MPU9250_ADDRESS, 0x3B, 14, buff);
    ax = (buff[0] << 8 | buff[1]);
    ay = (buff[2] << 8 | buff[3]);
    az = (buff[4] << 8 | buff[5]);
   processAccelData();
   
   //In order to start taking taking data again motion has to be detected. 
   float aSum = fabs(Ax) + fabs(Ay) + fabs(Az);
   Serial.println(aSum);

  // check if it's above the threshold if it is the device is moving samples are reset data is taken once again
  if (aSum >= accelerationThreshold) {
    // reset the sample read count
    samplesRead = 0;
    break;
  }

    }

while (samplesRead < numSamples) {
  Serial.println("Empiezo a tomar datos");

     // ---  Lectura acelerometro y giroscopio --- 
   uint8_t buff[14];
   I2Cread(MPU9250_ADDRESS, 0x3B, 14, buff);

   // Convertir registros acelerometro
   //Serial.println("Acelerometro");
   int16_t ax = (buff[0] << 8 | buff[1]);
   int16_t ay = (buff[2] << 8 | buff[3]);
   int16_t az = (buff[4] << 8 | buff[5]);

   // Convertir registros giroscopio
   //Serial.println("Giroscopio");
   int16_t gx = (buff[8] << 8 | buff[9]);
   int16_t gy = (buff[10] << 8 | buff[11]);
   int16_t gz = (buff[12] << 8 | buff[13]);



//   // ---  Lectura del magnetometro --- 
//   uint8_t ST1;
//   do
//   {
//      I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
//   } while (!(ST1 & 0x01));
//
//   uint8_t Mag[7];
//   I2Cread(MAG_ADDRESS, 0x03, 7, Mag);
//
//
//   // Convertir registros magnetometro
//   //Serial.println("Magnometro");
//   int16_t mx = -(Mag[3] << 8 | Mag[2]);
//   int16_t my = -(Mag[1] << 8 | Mag[0]);
//   int16_t mz = -(Mag[5] << 8 | Mag[4]);

//  printData();

//Envio de datos 
  Udp.beginPacket("192.168.10.10", 9001); 
  //Udp.beginPacket("127.0.0.1", 9001);  //// Esta ip es la ip del computador servidor y el puerto debe coincidir
  digitalWrite(led_conn, HIGH);
  Serial.println("Start envio paquete");
  //Se mandan las variables ax,ay,az,gx,gy,gz sin procesar ya que asi es mas facil normalizarlas para la red neuronal. Temperatura y angulo se envian al final  
  msg = String(ax) + "#" + String(ay) + "#" + String(az) + "#" + String(gx) + "#" + String(gy) + "#" + String(gz) +  "#" + String(Id_client) +  "#" + String(p); //El mensaje completo contiene el id del cliente y el numero de paquete enviad
    //msg = String(ax) + "#" + String(ay) + "#" + String(az) + "#" + String(gx) + "#" + String(gy) + "#" + String(gz) +  "#" + String(Id_client) +  "#" + String(p) + "#" + String(ang_x)+ "#"  +String(ang_y) ; //El mensaje completo contiene el id del cliente y el numero de paquete enviado
  //Serial.print(msg);
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
  samplesRead++;
  delay(10);
  
  
  }


}



//Funcion auxiliar lectura
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
   //Serial.println("Reading I2C");
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.endTransmission();

   Wire.requestFrom(Address, Nbytes);
   uint8_t index = 0;
   while (Wire.available())
      Data[index++] = Wire.read();
}


// Funcion auxiliar de escritura
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.write(Data);
   Wire.endTransmission();
}



void processAccelData(){
 Ax = ax / AccelScaleFactor;
 Ay = ay / AccelScaleFactor; 
Az = az / AccelScaleFactor;

Serial.print(Ax);
Serial.print(Ay);
Serial.print(Az);
}


//void processGyroData() {
// Gx = gyroX / GyroScaleFactor;
// Gy = gyroY / GyroScaleFactor; 
// Gz = gyroZ / GyroScaleFactor;
//}

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
