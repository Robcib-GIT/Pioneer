#include <Arduino.h>
#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
#include <DualVNH5019MotorShieldMod3.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include <nav_msgs/Odometry.h> //not use this (verify)
#include <tf/transform_broadcaster.h>
#include <ros/time.h>


#define EMERGENCY_PORT  26


DualVNH5019MotorShieldMod3 md;

// ************************* Encoder izquierdo*************************
const byte pin_encoder_L_1 = 18;   // 21 Entrada 1 encoder
const byte pin_encoder_L_2 = 19;   // 20 Entrada 2 encoder
volatile long contador_L = 0;   // En esta variable se guardan los pulsos del encoder y que se los interpretara como el angulo
volatile long contador_encoder_L = 0;
//volatile long contador_anterior_L = 0; //unnecessary variable ********** NATHY

// ************************* Encoder derecho  *************************
const byte pin_encoder_R_1 = 21;   // 18 Entrada 1 encoder
const byte pin_encoder_R_2 = 20;   // 19 Entrada 2 encoder
volatile long contador_R = 0;   // En esta variable se guardan los pulsos del encoder y que se los interpretara como el angulo
volatile long contador_encoder_R = 0;
//volatile long contador_anterior_R = 0; //unnecessary variable ********** NATHY

// ************************* variables para tiempo / calculo de velocidad ***
unsigned long ta1;//, ta2, dif_tiempo; //unnecessary variable ********** NATHY
long int cont1; //,cont2, dif_cont_L=0; //unnecessary variable ********** NATHY
long l_lastT = 0;

// ************************* variables de velocidad y robot *****************
float f_distancePerTick = 0.000027974;
float f_half_wheel_diff = 0.2;
//float f_wheel_diff = 0.41;  //unnecessary variable ********** NATHY
float R = 0.111; // Radio ruedas
float L = 0.41;  // Longitud entre ruedas

//float msg_angular_z = 0.0; //unnecessary variable ********** NATHY
//float msg_linear_x = 0.0; //unnecessary variable ********** NATHY

double d_targetSpeedL = 0;
double d_targetSpeedR = 0;

double d_currentSpeedL = 0;
double d_currentSpeedR = 0;
double d_currentSpeedL_ant = 0;
double d_currentSpeedR_ant = 0;
double d_currentSpeedL_media = 0;
double d_currentSpeedR_media = 0;


int i_m1Speed = 0;
int i_m2Speed = 0;

// ****************************** Variables para cálculo de la odometría
double x = 0.0;
double y = 0.0;
double th = 0.0;
//float deltat=0.025;//cada vez que actualizo la velocidad hago una actualizacion de la posicion
float deltat=0.1;//cada vez que actualizo la velocidad hago una actualizacion de la posicion
double vx;
double vy;
double vth;

char base_link[] = "/base_link"; //FOR TRANSFORM TF ODOMETRY
char odom[] = "/odom"; //FOR TRANSFORM TF ODOMETRY

// ******************************************** variables del controlador
double Kp=5;
double Kd=0.5;

// ******************************************** calculo de 
float read_vel_x = 0;
float read_vel_z = 0;
//float vel_leida_x_ant = 0; //unnecessary variable ********** NATHY
//float vel_leida_y_ant = 0; //unnecessary variable ********** NATHY



// *********************************** funcion para suscribirse al topico 
// *********************************** de velocidad que se envia desde C++
ros::NodeHandle nh;

void velocidad( const geometry_msgs::Twist& velocidad) //cambiar el nombre a la función
{
  read_vel_x = velocidad.linear.x;
  read_vel_z = velocidad.angular.z;
  float angular = f_half_wheel_diff * read_vel_z * 6;//* f_half_wheel_diff
  d_targetSpeedR = read_vel_x - angular;
  d_targetSpeedL = read_vel_x + angular;
}

// ******************************************* creacion de nodos para ROS *****

//Crear el nodo que va a publicarle la odometría al ROS

std_msgs::Float32 vel_lin_act;
ros::Publisher pub_lin("vel_lin_act", &vel_lin_act);

std_msgs::Float32 vel_ang_act;
ros::Publisher pub_ang("vel_ang_act", &vel_ang_act)  ;

//Crear el nodo que va a publicarle la transformada al ROS
geometry_msgs::TransformStamped trans_odom;
tf::TransformBroadcaster broadcaster;

ros::Subscriber<geometry_msgs::Twist> cmdvelSubscriber("/cmd_vel", &velocidad);

std_msgs::Bool emergencyMessage;


// ************************************* funcion de lectura de velocidad en ROS



void stopIfFault()
{
  if (md.getM1Fault())
   {
      Serial.println("M1 fault");
      while(1);
   }
   if (md.getM2Fault())
    {
      Serial.println("M2 fault");
      while(1);
    }
}

void setup()
{
  // ***************************** inicializacion pin de seguridad
  pinMode(EMERGENCY_PORT, INPUT);
  digitalWrite(EMERGENCY_PORT, HIGH);
  
  // ***************************** configuracion encoder izquierda
  pinMode(pin_encoder_L_1, INPUT);      // sets pin A as input
  digitalWrite(pin_encoder_L_1, LOW);  // turn on pullup resistors
  pinMode(pin_encoder_L_2, INPUT);      // sets pin B as input
  digitalWrite(pin_encoder_L_2, LOW);  // turn on pullup resistors

  // ***************************** configuracion encoder derecha
  pinMode(pin_encoder_R_1, INPUT);      // sets pin A as input
  digitalWrite(pin_encoder_R_1, LOW);  // turn on pullup resistors
  pinMode(pin_encoder_R_2, INPUT);      // sets pin B as input
  digitalWrite(pin_encoder_R_2, LOW);  // turn on pullup resistors
  
  // ****************************** inicializacion de motores
  md.init();

  // ****************************** funciones para las interrupciones
  attachInterrupt(digitalPinToInterrupt(pin_encoder_L_1), intEncoder_L, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin_encoder_R_1), intEncoder_R, FALLING);

  // ******************************* inicializacion del tiempo en microsegundos
  ta1=cont1=micros(); //revisar si funciona en teensy
  //Serial.begin(230400);
  Serial.begin(500000);

  // *********************************************** inicializacion de ROS
  nh.initNode();
  // advertiser es para inicializar un publicador

  // ****************************** Inicializo la suscripción a ROS
  nh.initNode();
  nh.subscribe(cmdvelSubscriber);
  // ****************************** Inicializo la publicación a ROS
  nh.initNode();
  nh.advertise(pub_lin);

  nh.initNode();
  nh.advertise(pub_ang);
  // ****************************** Inicializo broadcasting a ROS
  nh.initNode();
  broadcaster.init(nh);

  // *********************************************** inicialización del PID
   
}

  // *************************************************************** LOOP
  // *************************************************************** LOOP
  
void loop()
{

  for(int i = 0; i <9; i++)
  {
   
      nh.spinOnce();  // 1
      updateEncoder(); //get current speed and actualize previous speed

    if (!readEmergency())
      { 
        adjustSpeed(); //control speed
        
        // ************************************** Filtro de media
        d_currentSpeedL_media=(d_currentSpeedL + d_currentSpeedL_ant)/2 ;
        d_currentSpeedR_media=(d_currentSpeedR + d_currentSpeedR_ant)/2 ;

        // ************************************** Calculo de la odometria
      vx= R/2*(d_currentSpeedL_media+d_currentSpeedR_media)*cos(th);
      vy= R/2*(d_currentSpeedL_media+d_currentSpeedR_media)*sin(th);
      vth= R/L*(d_currentSpeedL_media-d_currentSpeedR_media);
    
       // *************************************** Calculo posicion
      x=x+vx*deltat;
      y=y+vy*deltat;
      th=th+vth*deltat;
        
        // **************************************** publicación de los nodos

        // ****************** Armo el mensaje de la transformada para enviarlo
        trans_odom.header.frame_id = odom;
        trans_odom.child_frame_id = base_link;
        trans_odom.transform.translation.x = x;
        trans_odom.transform.translation.y = y;
        trans_odom.transform.translation.z = 0.0;
        trans_odom.transform.rotation.x = contador_encoder_R;
        trans_odom.transform.rotation.y = contador_encoder_L;
        trans_odom.transform.rotation.z = th;
        trans_odom.transform.rotation.w = 1.0; 
        trans_odom.header.stamp = nh.now();

    
  
      }   
    // ********************* publicacion de topicos
  

    delay(10);   
  }
  vel_lin_act.data = d_currentSpeedL_media;
  pub_lin.publish(&vel_lin_act);

  vel_ang_act.data = d_currentSpeedL;
  pub_ang.publish(&vel_ang_act);
  broadcaster.sendTransform(trans_odom);
}


    // ***************************************** PARO DE EMERGENCIA
boolean readEmergency() //check ***********NATHY
//read emergency botton 
{
  if (!digitalReadFast(EMERGENCY_PORT) == LOW)
  {
    emergencyMessage.data = true;
    i_m1Speed = 0;
    i_m2Speed = 0;
    md.setM1Speed(i_m2Speed); 
    md.setM2Speed(i_m2Speed); 
    return true;
  }
  else
  {
    emergencyMessage.data = false;
    return false;
  }
}


    // ****************************************** encoder izquierda
void intEncoder_L() //check ***********NATHY
//Function count the pulses of left enconder
{
  
  if (digitalReadFast(pin_encoder_L_1) == LOW) {
    if (digitalReadFast(pin_encoder_L_2) == HIGH) {
      contador_L --;
      contador_encoder_L --;
    } else {
      contador_L ++;
      contador_encoder_L ++;
    }
  } else {
    if (digitalReadFast(pin_encoder_L_2) == LOW) {  
      contador_L --;
      contador_encoder_L --;
    } else {
      contador_L ++;
      contador_encoder_L ++;
    }
  }

  
}

    // ****************************************** encoder derecha
void intEncoder_R() //check ***********NATHY
//Function count the pulses of right enconder
{
  
  if (digitalReadFast(pin_encoder_R_1) == LOW) {
    if (digitalReadFast(pin_encoder_R_2) == HIGH) {
      contador_R --;
      contador_encoder_R --;
    } else {
      contador_R ++;
      contador_encoder_R ++;
    }
  } else {
    if (digitalReadFast(pin_encoder_R_2) == LOW) {  
      contador_R --;
      contador_encoder_R --;
    } else {
      contador_R ++;
      contador_encoder_R ++;
    }
  }
}

    // ************************************** actualizacion del encoder
void updateEncoder() //check ***********NATHY
/* FUnction converts pulses of encoder to velocity
*/
{
  int left = contador_L;
  int right = contador_R;
  
 // encoderMessage.left += left;
 // encoderMessage.right += right;

  contador_L = 0;
  contador_R = 0;

  long currentTime = millis();
  double deltaTime = currentTime - l_lastT;
  l_lastT = currentTime;

  d_currentSpeedL_ant=d_currentSpeedL;
  d_currentSpeedR_ant=d_currentSpeedR;
  
  d_currentSpeedL = 1000.0 * (f_distancePerTick * left) / deltaTime;
  d_currentSpeedR = 1000.0 * (f_distancePerTick * right) / deltaTime;
  
}

  // ****************************************  Controladora de velocidad

  void adjustSpeed() //check ***********NATHY
  //Function to control speed 
{

  if ((fabs(d_targetSpeedL) < 0.001) && (fabs(d_targetSpeedR) < 0.001))
  {
    i_m1Speed = 0;
    i_m2Speed = 0;
  }
  else
  {
    float additionR = fabs(Kp * fabs(d_targetSpeedR-d_currentSpeedR));
   // Serial.println(additionR);
    float additionL = fabs(Kp * fabs(d_targetSpeedL-d_currentSpeedL));

  // ************************ LIMITADOR
    if(additionR > 0.001 && additionR < 1.0)
    {
      additionR = 1; 
    }
    if(additionL > 0.001 && additionL < 1.0)
    {
      additionL = 1; 
    }

  // ************************ CONTROLADORA DERECHA
    if (d_targetSpeedR > d_currentSpeedR)
    {
      i_m1Speed += additionR;
    }
    else if (d_targetSpeedR < d_currentSpeedR)
    {
      i_m1Speed -= additionR;
    }

  // ************************ CONTROLADORA IZQUIERDA
    if (d_targetSpeedL > d_currentSpeedL)
    {
      i_m2Speed += additionL;
    }
    else if (d_targetSpeedL < d_currentSpeedL)
    {
      i_m2Speed -= additionL;
    }
  }

  // ****************************************** limitación de la tarjeta
    if(i_m1Speed > 200)
      {
        i_m1Speed = 200;
      }
    else if (i_m1Speed < -200)
      {
        i_m1Speed = -200;
      }
    if(i_m2Speed > 200)
      {
        i_m2Speed = 200;
      }
    else if (i_m2Speed < -200)
      {
         i_m2Speed = -200;
      }

  if (!md.getM1Fault() && !md.getM2Fault())
  {
    md.setM1Speed(i_m1Speed); 
    md.setM2Speed(i_m2Speed); 
    stopIfFault();
    
  }
}
