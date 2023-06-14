#include <Wire.h>
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

long Ax, Ay, Az;
long Gx,Gy,Gz;
int16_t ax, ay, az, Tmp, gx, gy, gz;
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;
const float AccelScaleFactor = 8192.0;
const float GyroScaleFactor =65.5;

const char* ssid = "QV_2G";
const char* password = "2858351qv";

//const char* ssid = "Ventana 2";
//const char* password = "2858351Qv"; 

//const char* ssid = "Agrosavia2.4G";
//const char* password = "Agrosavia"; 

int Id_client= 1; //Identificador del cliente
int p = 0; // Identificador del paquete enviado 
int cont_conexion=0; //Contador intentos conexion

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
}


//
void processGyroData() {
  Gx = gx / GyroScaleFactor;
  Gy = gy / GyroScaleFactor; 
  Gz = gz / GyroScaleFactor;
}

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

}


String msg = "";
void loop() {



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




//Calculo del filtro complementario 
  //Se obtienen convierten los valores para calcular el filtro complementario 
  processAccelData();
  processGyroData();
  //Se calcula el filtro complementario 
  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan(ay/sqrt(pow(Ax,2) + pow(Az,2)))*(180.0/3.14);
  float accel_ang_y=atan(-ax/sqrt(pow(Ay,2) + pow(Az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y filtro complemento  

  delay(100);
//Envio de datos 
  msg = String(ax) + "#" + String(ay) + "#" + String(az) + "#" + String(gx) + "#" + String(gy) + "#" + String(gz); //El mensaje completo contiene el id del cliente y el numero de paquete enviado
  Serial.println(msg);
  delay(10);
}
