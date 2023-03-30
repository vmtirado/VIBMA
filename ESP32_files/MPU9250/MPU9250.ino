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


   // Configurar acelerometro
   I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);
   // Configurar giroscopio
   I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
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



   // ---  Lectura acelerometro y giroscopio --- 
   uint8_t Buf[14];
   I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

   // Convertir registros acelerometro
   //Serial.println("Acelerometro");
   int16_t ax = -(Buf[0] << 8 | Buf[1]);
   int16_t ay = -(Buf[2] << 8 | Buf[3]);
   int16_t az = Buf[4] << 8 | Buf[5];

   // Convertir registros giroscopio
   //Serial.println("Giroscopio");
   int16_t gx = -(Buf[8] << 8 | Buf[9]);
   int16_t gy = -(Buf[10] << 8 | Buf[11]);
   int16_t gz = Buf[12] << 8 | Buf[13];



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

void setupMPU(){
  Serial.println("Setting up");
   // Configurar acelerometro
   I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);
   // Configurar giroscopio
   I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
   // Configurar magnetometro
   I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
   I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);
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

void recordRegisters() {
  Serial.println("Record registers :D");
   // ---  Lectura acelerometro y giroscopio --- 
   uint8_t Buf[14];
   I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

   // Convertir registros acelerometro
   //Serial.println("Acelerometro");
   int16_t ax = -(Buf[0] << 8 | Buf[1]);
   int16_t ay = -(Buf[2] << 8 | Buf[3]);
   int16_t az = Buf[4] << 8 | Buf[5];

   // Convertir registros giroscopio
   //Serial.println("Giroscopio");
   int16_t gx = -(Buf[8] << 8 | Buf[9]);
   int16_t gy = -(Buf[10] << 8 | Buf[11]);
   int16_t gz = Buf[12] << 8 | Buf[13];



   // ---  Lectura del magnetometro --- 
   uint8_t ST1;
   do
   {
      I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
   } while (!(ST1 & 0x01));

   uint8_t Mag[7];
   I2Cread(MAG_ADDRESS, 0x03, 7, Mag);


   // Convertir registros magnetometro
   //Serial.println("Magnometro");
   int16_t mx = -(Mag[3] << 8 | Mag[2]);
   int16_t my = -(Mag[1] << 8 | Mag[0]);
   int16_t mz = -(Mag[5] << 8 | Mag[4]);


   //Serial.println("End record registers :D");

}

//void processAccelData(){
//  Ax = accelX / AccelScaleFactor;
//  Ay = accelY / AccelScaleFactor; 
//  Az = accelZ / AccelScaleFactor;
//}
//
//void recordGyroRegisters() {
//  Wire.beginTransmission(0b1101000); //I2C address of the MPU
//  Wire.write(0x43); //Starting register for Gyro Readings
//  Wire.endTransmission();
//  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
//  while(Wire.available() < 6);
//  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
//  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
//  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
//  processGyroData();
//  delay(1000);
//}
//
//void processGyroData() {
//  Gx = gyroX / GyroScaleFactor;
//  Gy = gyroY / GyroScaleFactor; 
//  Gz = gyroZ / GyroScaleFactor;
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
