#include <Arduino.h>
#include <Ticker.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <L3G.h>




const char* ssid = "Infinix NOTE 8";
const char* password = "12345678";
const char* mqtt_server = "192.168.43.52
";

/*
const char* ssid = "AeRobotiX INSAT";
const char* password = "Aerobotix2024";
const char* mqtt_server = "10.13.0.59
";
*/
float spacing_encoder = 133.0;  //132
#define T 10
#define rampmax 550.0
#define spacing_wheel 133
#define MaxPami
TimeOut 100
float kp1r = 5.0;
float kd1r = 0.0005;
float kp1m = 3.0;
float kd1m = 0.0005;
float ki1 = 0.0135;
float kp2 = 1.3;
float right_radius = 29.75;
float left_radius = 30.0;
float kbm = 10;
float kbr = 5;
#define MSG_BUFFER_SIZE (30)

L3G gyro;
bool GyroTnek = false;
bool enablegyro = false;
bool UltraActive = true;

unsigned long timer = 0;
float timeStep = 0.01;
float mean, variance;

int prevcountR = 0;
int prevcountL = 0;
int dalpha_counter = 0;
int d_right;
int d_left;
float totalR;
float totalL;
float dL, dC, dR;
float X = 0, Y = 0, alpha = 0, alpha_deg = 0;
float speedL, speedR;
double ZoneX = 0, ZoneY = 0;
int tickmmR, tickmmL;
int prevCountL, prevCountR;
float right_resolution = 700;
float right_precision = 2;

float left_resolution = 700;
float left_precision = 2;

volatile float d_right_counter;
volatile float d_left_counter;
volatile float total_centre;
volatile float right_encoder_speed;
volatile float left_encoder_speed;
volatile float alpha_speed;
volatile float d_alpha_counter;
volatile float right_speed;
volatile float left_speed;
volatile float GyroSpeed = 0;
volatile float GyroAngle = 0;


float rawalpha;
float prev_pos_error;
float prev_vel_error;
float integ_error;
float d_error;
float t;
float first;
float now;
float lastTime;
unsigned long Gtime;
double ramp;
float pos_error;
float consigne_vel;
float vel_error;
float consigne_right;
float consigne_left;
float vitesse_right;
float left_distance;
float bal_error;
int Timeout=50;
int i = 0;
//ESP32

const int encoderLPin1 = 35;
const int encoderLPin2 = 34;
const int encoderRPin1 = 18;
const int encoderRPin2 = 23;


const int motorleft1 = 32;
const int motorleft2 = 33;
const int motorright1 = 26;
const int motorright2 = 25;
//ESP86
/*

const int motorleft1 = 32;
const int motorleft2 = 33;
const int motorright1 = 25;
const int motorright2 = 26;


//*/
unsigned long gtime;
volatile int counterL = 0;
volatile int counterR = 0;
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
char msg[MSG_BUFFER_SIZE];
char SendMsg[25];
char SendPath[100];
int value = 0;
volatile bool IsCalibrating = false;
double yaw = 0;
double speed = 0;


const int trigPin = 12;
const int echoPin = 13;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;




void setup_wifi() {
  delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  randomSeed(micros());
}



void callback(char* topic, byte* payload, unsigned int length) {
  if (!strcmp(topic, "systemCommand")) {
    double distance = 0;
    double rotation = 0;
    strlcpy(SendMsg, (char*)payload, length + 1);
    sscanf(SendMsg, "%lf,%lf", &distance, &rotation);
    move_distance(distance);
    if (rotation != 0) {
      //calibrate(20);
      rotate(rotation);
      //orientate(rotation);
    }
    //Robot_Locate(distance, rotation);
  } 
  else if (!strcmp(topic, "setCoeffs")) {
    double newKpp=0,newKdp=0,newKpv=0,newKiv=0,newKb=0;
    strlcpy(SendMsg, (char*)payload, length + 1);
    sscanf(SendMsg, "%lf,%lf,%lf,%lf,%lf", &newKpp,&newKdp,&newKpv,&newKiv,&newKb); 
    if(newKpp!=-1)
      kp1m=newKpp;
    if(newKdp!=-1)
      kd1m=newKdp;
    if(newKpv!=-1)
      kp2=newKpv;
    if(newKiv!=-1)
      ki1=newKiv;
    if(newKb!=-1)
      kbm=newKb;
    delay(100);
    sprintf(SendMsg, "%lf,%lf,%lf,%lf,%lf", kp1m,kd1m,kp2,ki1,kbm);
    client.publish("tuneVerif",SendMsg);
  }
  else if (!strcmp(topic, "robotLocate")) {
    double x = 0;
    double y = 0;
    strlcpy(SendMsg, (char*)payload, length + 1);
    sscanf(SendMsg, "%lf,%lf", &x, &y);
    Robot_Locate(x, y);
  } 

}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      client.subscribe("Pami
Orders2");
      client.subscribe("TogglePosition2");
      client.subscribe("Pami
Position2");
      client.subscribe("Pami
Timeout2");
      
    } else {
      // Wait 5 seconds before retrying
      delay(1000);
    }
  }
  if (GyroTnek) {
    client.publish("Pami
Feedback", "Gyro tnek");
  }
  client.publish("Pami
Feedback", "Hani Connectit");
}

void IRAM_ATTR handleEncoderA() {
  int a = digitalRead(encoderLPin1);
  int b = digitalRead(encoderLPin2);

  if (a == b) {
    counterL++;
  } else {
    counterL--;
  }
}
void IRAM_ATTR handleEncoderB() {
  int a = digitalRead(encoderRPin1);
  int b = digitalRead(encoderRPin2);

  if (a == b) {
    counterR++;
  } else {
    counterR--;
  }
}

Ticker Timer1;
Ticker Timer2;
void setup() {
  Serial.begin(9600);
  pinMode(encoderLPin1, INPUT_PULLUP);
  pinMode(encoderRPin1, INPUT_PULLUP);
  pinMode(encoderLPin2, INPUT_PULLUP);
  pinMode(encoderRPin2, INPUT_PULLUP);

  pinMode(motorleft1, OUTPUT);
  pinMode(motorleft2, OUTPUT);
  pinMode(motorright1, OUTPUT);
  pinMode(motorright2, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderLPin1), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRPin1), handleEncoderB, CHANGE);
  Wire.begin();

  if (!gyro.init()) {
    Serial.println("Failed to autodetect gyro type!");
    GyroTnek = true;
  }
  if (!GyroTnek) {
    delay(1000);
    gyro.enableDefault();
    calibrate(200);
  }
  Timer1.attach(0.005, odometry);
  gtime = millis();
  Timer2.attach(0.01, UpdateAngle);
  sei();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  enablegyro = true;




}
unsigned long wala = 0;
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
 /* Serial.println(GyroAngle);
  delay(25);*/


}

void UpdateAngle() {
  if (!GyroTnek) {
    if (!IsCalibrating && enablegyro) {
      GyroSpeed = getspeed();
      GyroAngle += GyroSpeed * float(millis() - gtime) / 1000;
      gtime = millis();
      //GyroAngle += GyroSpeed * 0.015;
    }
  } else {
    //GyroAngle = alpha;
  }
}

void odometry() {
  update_position();
  if (i == 2) {
    speed_calcul();
    i = 0;
  }
  i++;
}
double timee1;
double timee2;
int counter_ult;
float target_angle, goal_distance;
int sens;
void orientate(float orientation) {
  target_angle = orientation - alpha_deg;
  if (target_angle > 180) target_angle -= 360;
  if (target_angle < -180) target_angle += 360;
  rotate(target_angle);
}
void Robot_Locate(float goal_x, float goal_y) {
  sens = (asinf((goal_y - Y) / sqrtf((X - goal_x) * (X - goal_x) + (Y - goal_y) * (Y - goal_y))) > 0) ? 1 : -1;
  target_angle = sens * rad_to_deg(acosf((goal_x - X) / sqrtf((X - goal_x) * (X - goal_x) + (Y - goal_y) * (Y - goal_y))));
  orientate(target_angle);
  goal_distance = sqrtf((X - goal_x) * (X - goal_x) + (Y - goal_y) * (Y - goal_y));
  move_distance(goal_distance);
}

void move_distance(double target) {
  double AccumilatedTarget = 0;
  bool edged = false;
  while (true) {
    initi();
    edged = false;
    target -= AccumilatedTarget;
    int signe = (int)(target / fabs(target));
    float now = millis();
    bool breaked = false;
    double dist;
    first = now;
    while ((fabs(target - totalR) > 2) || (fabs(target - totalL) > 2)) {
      if (counter_ult == 5) {
        counter_ult = 0;
        dist = Ultrasonic_dist();
        if (dist < 10 && dist != 0) {
          breaked = true;
          break;
        }
      }
      counter_ult++;
      lastTime = millis();
      ramp = signe * calculateRamp(now - first,0.75);
      pos_error = target - (totalR + totalL) / 2;
      d_error = (pos_error - prev_pos_error);
      prev_pos_error = pos_error;
      consigne_vel = kp1m * pos_error + kd1m * d_error;
      consigne_vel = constraint(consigne_vel, -rampmax, rampmax);
      if (signe == 1)
        consigne_vel = fmin(ramp, consigne_vel);
      else
        consigne_vel = fmax(ramp, consigne_vel);
      Report();
      vel_error = consigne_vel - right_speed;
      integ_error += vel_error * T;
      consigne_right = kp2 * vel_error + ki1 * integ_error;
      vel_error = consigne_vel - left_speed;
      integ_error += vel_error * T;
      consigne_left = kp2 * vel_error + ki1 * integ_error;
      bal_error = totalR - totalL;
      set_motors((int)constrain(consigne_right - kbm * bal_error, -255, 255), (int)constrain(consigne_left + kbm * bal_error, -255, 255));
      now = millis();
      delay(T - now + lastTime);
    }
    stop_motors();
    if (breaked) {
      for(int i = 0; i < Timeout;i++){
        delay(10);
        dist = Ultrasonic_dist();
        if (!(dist < 10 && dist != 0)) {
          edged = true;
          break;
        }
      }
      dist = Ultrasonic_dist();
      if (dist < 10 && dist != 0) {
        UltraActive = false || edged;
      }
      AccumilatedTarget = (totalR + totalL) / 2;
    }else{
      return;
    }
  }
}
float sangle;
int timecounter = 0;
void rotate(double goal) {
  if (goal == 0) { return; }
  initi();
  timecounter = 0;
  enablegyro = true;
  int signe = (int)(goal / fabs(goal));
  float now = millis();
  //sangle = GyroAngle;
  double inter;
  double target;
  double salpha = GyroAngle;
  double odemetry_start_angle = alpha_deg;
  target = goal * PI * spacing_encoder / 360;
  first = now;
  Gtime = millis();
  while (((fabs(target - totalR) > 2) || (fabs(target + totalL) > 2)) && (fabs((GyroAngle - salpha) - goal) > 1)) {  //(fabs(target-totalR)>2) || (fabs(target+totalL)>2)
                                                                                                                     // while (fabs((GyroAngle-sangle)-goal)>2){
    lastTime = millis();                                                                                             /*
        GyroSpeed = getspeed();
        GyroAngle += GyroSpeed*float(millis()-Gtime)/1000;
		    Gtime=millis();*/
    ramp = signe * calculateRamp(now - first,0.4);
   // pos_error = target - (GyroAngle - salpha) / 180 * PI * spacing_encoder / 2;  //(alpha-salpha)*spacing_encoder/2;
     pos_error=target -(totalR-totalL)/2;
    d_error = (pos_error - prev_pos_error);
    prev_pos_error = pos_error;
    consigne_vel = kp1r * pos_error + kd1r * d_error;
    consigne_vel = constraint(consigne_vel, -rampmax, rampmax);
    //Serial.println(consigne_vel);
    if (signe == 1)
      consigne_vel = fmin(ramp, consigne_vel);
    else
      consigne_vel = fmax(ramp, consigne_vel);
    vel_error = consigne_vel - right_speed;
    integ_error += vel_error * T;
    consigne_right = kp2 * vel_error + ki1 * integ_error;
    //Report();
    vel_error = (-consigne_vel) - left_speed;
    integ_error += vel_error * T;
    consigne_left = kp2 * vel_error + ki1 * integ_error;

    bal_error = totalR + totalL;
    set_motors((int)constraint_rot(consigne_right - kbr * bal_error, -255, 255), (int)constraint_rot(consigne_left - kbr * bal_error, -255, 255));
    timecounter++;
    if(timecounter>MaxPami
TimeOut){
      break;
    }
    now = millis();
    delay(T - now + lastTime);
  }
  stop_motors();
  alpha_deg = odemetry_start_angle + GyroAngle - salpha;
  alpha = alpha_deg * PI / 180;
  enablegyro = false;
}

void stop_motors() {
  analogWrite(motorleft1, 0);
  analogWrite(motorleft2, 0);
  analogWrite(motorright1, 0);
  analogWrite(motorright2, 0);
}
void set_motors(int cmdR, int cmdL) {
  if (cmdR > 0) {
    analogWrite(motorright1, cmdR);
    analogWrite(motorright2, 0);
  } else {
    analogWrite(motorright2, abs(cmdR));
    analogWrite(motorright1, 0);
  }
  if (cmdL > 0) {
    analogWrite(motorleft1, cmdL);
    analogWrite(motorleft2, 0);
  } else {
    analogWrite(motorleft2, abs(cmdL));
    analogWrite(motorleft1, 0);
  }
}
void update_position(void) {
  d_right = counterR - prevcountR;
  prevcountR = counterR;
  dR = ticks_to_distance(d_right, right_radius, right_resolution, right_precision);  // d_right count - prevvount en 10 ms
  totalR += dR;
  d_right_counter += dR;
  d_left = counterL - prevcountL;
  prevcountL = counterL;
  dL = ticks_to_distance(d_left, left_radius, left_resolution, left_precision);
  totalL += dL;
  d_left_counter += dL;
  dC = (dR + dL) / 2;
  total_centre += dC;

  X += dC * cos(alpha);
  Y += dC * sin(alpha);
  alpha += ((dR - dL) / spacing_encoder);
  dalpha_counter += ((dR - dL) / spacing_encoder);
  while (alpha > PI) {
    alpha -= 2 * PI;
  }
  while (alpha < -PI) {
    alpha += 2 * PI;
  }
  alpha_deg = rad_to_deg(alpha);
}
void speed_calcul(void) {
  right_speed = d_right_counter / T * 1000;
  d_right_counter = 0;
  left_speed = d_left_counter / T * 1000;
  d_left_counter = 0;
  alpha_speed = d_alpha_counter / T * 1000;
  d_alpha_counter = 0;
  /* right_speed = (right_encoder_speed + left_encoder_speed) / 2 + alpha_speed * spacing_wheel / 2;
  left_speed = (right_encoder_speed + left_encoder_speed) / 2 - alpha_speed * spacing_wheel / 2;*/
}
float ticks_to_distance(int x, float r, int resolution, int precision) {
  return (x * 2 * PI * r / (resolution * precision));
}
float rad_to_deg(double x) {
  return (x * 360 / (2 * PI));
}
void initi(void) {
  totalR = 0;
  totalL = 0;
  prev_pos_error = 0;
  prev_vel_error = 0;
  integ_error = 0;
  d_error = 0;
  t = 0;
  vitesse_right = 0;
  left_distance = 0;
  d_right_counter = 0;
  d_left_counter = 0;
  d_alpha_counter = 0;
}

double constraint(double val, double min, double max) {
  if (val > max) {
    return max;
  } else if (val<40.0 & val> 0) {
    return 40.0;
  } else if (val > -40.0 & val < 0) {
    return -40.0;
  } else if (val < min) {
    return min;
  } else {
    return val;
  }
}
double constraint_rot(double val, double min, double max) {
  if (val > max) {
    return max;
  } else if (val<60.0 & val> 0) {
    return 60.0;
  } else if (val > -60.0 & val < 0) {
    return -60.0;
  } else if (val < min) {
    return min;
  } else {
    return val;
  }
}
double calculateRamp(double t,float rampe) {
  double ramp = rampe * t;
  ramp = constraint(ramp, -rampmax, rampmax);
  return ramp;
}

void calibrate(int attemps) {
  IsCalibrating = true;
  delay(10);
  for (int i = 0; i < attemps; i++) {
    gyro.read();
    mean += gyro.g.z;
    variance += gyro.g.z * gyro.g.z;
    Serial.println(gyro.g.z);
    delay(15);
  }
  mean /= attemps;
  variance = sqrt((variance / attemps) - (mean * mean)) * 2;
  IsCalibrating = false;
}
double getspeed() {
  gyro.read();
  if (abs(gyro.g.z - mean) < variance) {
    return 0;
  }
  return double(gyro.g.z - mean) * 0.066;  //0.066 pami
 1
}

double Ultrasonic_dist() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH, 3500);

  // Calculate the distance
  distanceCm = duration * SOUND_SPEED / 2;
  return distanceCm * Resitrictzone(distanceCm) * UltraActive;
}


bool Resitrictzone(float dist) {
  double EnemyX = X + (65 + dist * 10) * cos(alpha);
  double EnemyY = Y + (65 + dist * 10) * sin(alpha);
  return sqrt(pow(EnemyX - ZoneX, 2) + pow(EnemyY - ZoneY, 2)) > 200;
}
