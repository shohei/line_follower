int left_sensor = A4;
int right_sensor = A3;
int thresh = 600;

#define MAX_DIFF 830 //adhoc val from measurement
#define MID_POWER 120
#define MIN_POWER 20
#define MAX_POWER 220

#define DELAY_MS 50
#define DELAY_MS2 200
#define ML_Ctrl  2   //define the direction control pin of A motor
#define ML_PWM 9   //define the PWM control pin of A motor
#define MR_Ctrl  4   //define the direction control pin of B motor
#define MR_PWM 5   //define the PWM control pin of B motor
#define INIT 9999

//#define DEBUG

float kp = 1.0;
float kd = 0.1;

int left_pwm;
int right_pwm;
int left_pwm_delta;
int right_pwm_delta;
int left_pwm_delta_last=INIT;
int right_pwm_delta_last=INIT;
float left_pwm_delta_dt;
float right_pwm_delta_dt;

float dt = DELAY_MS/1000.0;


void setup() {
Serial.begin(9600);
  pinMode(ML_Ctrl, OUTPUT);//set the direction control pin of A motor to OUTPUT
  pinMode(ML_PWM, OUTPUT);//set the PWM control pin of A motor to OUTPUT
  pinMode(MR_Ctrl, OUTPUT);//set the direction control pin of B motor to OUTPUT
  pinMode(MR_PWM, OUTPUT);//set the PWM control pin of B motor to OUTPUT  
}

void loop() {
  int left_val = analogRead(left_sensor);
  int right_val = analogRead(right_sensor);
//  Serial.print("left:");
//  Serial.print(left_val);
//  Serial.print(",right:");
//  Serial.println(right_val);

  int diff = left_val-right_val;
  turn(diff);  
}
void back(){
  digitalWrite(ML_Ctrl,LOW);//set the direction control pin of A motor to low level
  analogWrite(ML_PWM,100);//set the PWM control speed of A motor to 100
  digitalWrite(MR_Ctrl,LOW);//set the direction control pin of B motor to low level
  analogWrite(MR_PWM,100);//set the PWM control speed of B motor to 100  
  delay(DELAY_MS);//delay in 2000ms
  stop_motor();
}

void front(){
  digitalWrite(ML_Ctrl,HIGH);//set the direction control pin of A motor to high level
  analogWrite(ML_PWM,100);//set the PWM control speed of A motor to 100
  digitalWrite(MR_Ctrl,HIGH);//set the direction control pin of B motor to high level
  analogWrite(MR_PWM,100);//set the PWM control speed of B motor to 100
  delay(DELAY_MS);//delay in 2000ms
  stop_motor();
}

void turn(int diff){  
  if(diff>0){
     //Diverted to left
     //CW rotation
     left_pwm_delta = float(MAX_POWER-MID_POWER) * diff*1.0/MAX_DIFF;
     if (left_pwm_delta_last==INIT) {
       left_pwm_delta_dt = 0;            
     } else {
       left_pwm_delta_dt = (left_pwm_delta-left_pwm_delta_last)/dt; 
     }
     left_pwm = MID_POWER + kp*left_pwm_delta + kd*left_pwm_delta_dt;
     left_pwm = min(MAX_POWER,left_pwm); //clipping

     right_pwm_delta = float(MID_POWER-MIN_POWER) * diff*1.0/MAX_DIFF;
     if (right_pwm_delta_last==INIT) {
       right_pwm_delta_dt = 0;            
     } else {
       right_pwm_delta_dt = (right_pwm_delta-right_pwm_delta_last)/dt; 
     }
     right_pwm = MID_POWER - kp*right_pwm_delta - kd*right_pwm_delta_dt;   
     if(right_pwm < 0){
        Serial.println("Fast CW");
        right_pwm = max(-MAX_POWER, right_pwm);//clipping
     }
  } else {
     //Diverted to right
     //CCW rotation
     left_pwm_delta = float(MID_POWER-MIN_POWER) * abs(diff)*1.0/MAX_DIFF;
     if (left_pwm_delta_last==INIT) {
      left_pwm_delta_dt = 0;
     } else {
       left_pwm_delta_dt = (left_pwm_delta-left_pwm_delta_last)/dt;    
     }
     left_pwm = MID_POWER - kd*left_pwm_delta - kp*left_pwm_delta_dt;
     if(left_pwm < 0){
        Serial.println("Fast CCW");
        left_pwm = max(-MAX_POWER, left_pwm);//clipping
     }
     
     right_pwm_delta =  float(MAX_POWER-MID_POWER) * abs(diff)*1.0/MAX_DIFF;
     if (right_pwm_delta_last==INIT) {
       right_pwm_delta_dt = 0;            
     } else {
        right_pwm_delta_dt = (right_pwm_delta-right_pwm_delta_last)/dt;   
     }
     right_pwm = MID_POWER + kd*right_pwm_delta + kp*right_pwm_delta_dt;
     right_pwm = min(MAX_POWER,right_pwm); //clipping
  }
  Serial.print("diff:");
  Serial.print(diff);  
  Serial.print(",left_pwm:");
  Serial.print(left_pwm);
  Serial.print(",right_pwm:");
  Serial.println(right_pwm);

  if (left_pwm > 0){
     digitalWrite(ML_Ctrl,HIGH);    
  } else {
     digitalWrite(ML_Ctrl,LOW);        
  }
  if (right_pwm > 0){
    digitalWrite(MR_Ctrl,HIGH);  
  } else {
    digitalWrite(MR_Ctrl,LOW);      
  }

  #ifndef DEBUG
  analogWrite(ML_PWM,abs(left_pwm));
  analogWrite(MR_PWM,abs(right_pwm));
  #endif 
  left_pwm_delta_last = left_pwm_delta;
  right_pwm_delta_last = right_pwm_delta;
    
  delay(DELAY_MS);
  stop_motor();  
}

void left(int pwm_val){
  digitalWrite(ML_Ctrl,HIGH);//set the direction control pin of A motor to HIGH level
  analogWrite(ML_PWM,pwm_val);// set the PWM control speed of A motor to 100
  digitalWrite(MR_Ctrl,LOW);//set the direction control pin of B motor to LOW level
  analogWrite(MR_PWM,pwm_val);//set the PWM control speed of B motor to 100
  delay(DELAY_MS);//delay in 2000ms
  stop_motor();
}

void right(int pwm_val){
  digitalWrite(ML_Ctrl,LOW);//set the direction control pin of A motor to LOW level
  analogWrite(ML_PWM,pwm_val);//100 set the PWM control speed of A motor to 100
  digitalWrite(MR_Ctrl,HIGH);//set the direction control pin of B motor to HIGH level
  analogWrite(MR_PWM,pwm_val);//set the PWM control speed of B motor to 100
  delay(DELAY_MS);//delay in 2000ms
  stop_motor();
}

void stop_motor(){
  analogWrite(ML_PWM,0);//set the PWM control speed of A motor to 0
  analogWrite(MR_PWM,0);// set the PWM control speed of B motor to 0
}
