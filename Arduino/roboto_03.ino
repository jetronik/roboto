

//#define USE_USBCON
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>



// Initialize PID paramaters

double Setpoint_fl, Input_fl, Output_fl;
double Setpoint_fr, Input_fr, Output_fr;

unsigned long timer =0;

double rway=0;
double lway=0;

int ctL=0;
int ctR=0;
//double Setpoint_bl, Input_bl, Output_bl;
//double Setpoint_br, Input_br, Output_br;


// Initialize quadrature encoder paramaters

int ticks_per_revolution = 108;
double ticks_to_pi=PI/108;


// Initialize pin numbers
int inPinL=26;
int inPinR=27;



const uint8_t intrR1=19; // interrupt 14
//const uint8_t intrR2=15; // interrupt 15
//const uint8_t intrL1=19; // interrupt 19
const uint8_t intrL2=18; // interrupt 18


const uint8_t R_ENABLE = 42; //34
const uint8_t L_ENABLE = 39; //31 +8

const uint8_t R_DIR = 41; //38
const uint8_t L_DIR = 38; //41

const uint8_t R_PWM = 9; //8
const uint8_t L_PWM = 8; //9

const uint8_t BRAKE = 23;
float x=0;
float z=0;

int left_wheel=0;
int right_wheel=0;





// Initialize ROS paramaters

ros::NodeHandle nh;

std_msgs::Int32 lfcount;
std_msgs::Int32 rfcount;






ros::Publisher lfwheel("lfwheel", &lfcount);
ros::Publisher rfwheel("rfwheel", &rfcount);


// Effort Callback
// Sets the setpoints of the pid for each wheel

void onEffort(const std_msgs::Int32MultiArray &msg)
{
  //nh.loginfo("Inside Callback");
  left_wheel = msg.data[0];
  right_wheel = msg.data[1];
 
 
 //call function to introduce PWM signal 0-255
    
  Move_motor((-1)*left_wheel,L_PWM,L_ENABLE,L_DIR);
  //l_pwm.data= int(Output_fl);
  Move_motor(right_wheel,R_PWM,R_ENABLE,R_DIR);

}

ros::Subscriber<std_msgs::Int32MultiArray> sub("/commander", onEffort);

// Move any motor function with speed_pwm value and pin numbers

void Move_motor(int speed_pwm,const uint8_t pwm,const uint8_t enable, const uint8_t dir)
{
  
 
  if(speed_pwm > 0)
  {
     digitalWrite(BRAKE, HIGH);
    digitalWrite(enable, HIGH);
    //if(dir==R_DIR){digitalWrite(dir, HIGH);}
    //else {digitalWrite(dir, LOW);}
    digitalWrite(dir, HIGH);
    analogWrite(pwm, abs(speed_pwm));
    timer=millis();
  }
  else if(speed_pwm < 0)
  {
     digitalWrite(BRAKE, HIGH);
    digitalWrite(enable, HIGH);
    //if(dir==R_DIR){digitalWrite(dir, LOW);}
    //else {digitalWrite(dir, HIGH);}
    digitalWrite(dir, LOW);
    analogWrite(pwm, abs(speed_pwm));
    timer=millis();
  }

  else if(speed_pwm==0){
   stop();

    
    }

  
  
}



// Initialize pins for forward movement

void setpins()
{
  pinMode(L_ENABLE,OUTPUT);
  pinMode(R_ENABLE,OUTPUT);

  pinMode(L_DIR,OUTPUT);
  pinMode(R_DIR,OUTPUT);
  
  pinMode(L_PWM,OUTPUT);
  pinMode(R_PWM,OUTPUT);

  pinMode(BRAKE,OUTPUT);

   pinMode(inPinL, INPUT);
    pinMode(inPinR, INPUT);
  
  
  digitalWrite(BRAKE, LOW);
  
}

// Encoders tend to reverse regarding of the pins??
// This way we move the robot forward a bit on startup
// And if an encoder has negative value we reverse it.




//void reset Integral error when we stop
void reset_pid_Ki()
{
  
  Output_fl=0;
  Output_fr=0;
  
}


//Define interrupts
void rpin1(){
  if (digitalRead(intrR1)==digitalRead(inPinR)){ rway=rway+1;}
  else {rway=rway-1;}
  }
  
void lpin1(){
  if (digitalRead(inPinL)==digitalRead(intrL2)){lway =lway+1;}
  else {lway =lway-1;}
  }


// stop movement

void stop()
{
  
  
  digitalWrite(BRAKE, LOW);
 

  

}

void setup() {

  
  // 115200 baud rate
  
  // 57600 baud rate
  //nh.getHardware()->setBaud(115200);

  // Pid setup
  
  
   
  setpins();


  stop();

  //attach interupts

  attachInterrupt(digitalPinToInterrupt(intrR1), rpin1, CHANGE); // interrupt called when sensors sends digital 2 high (every wheel rotation)
  attachInterrupt(digitalPinToInterrupt(intrL2), lpin1, CHANGE); // interrupt called when sensors sends digital 2 high (every wheel rotation)
  
  
  // ros node setup

  nh.initNode();
  nh.advertise(lfwheel);
  nh.advertise(rfwheel);
 
  nh.subscribe(sub);
  
}

// Initialize starting loop paramaters for calculating velocity and time



void loop() {
  
  // count encoder ticks
  int ct1 = lway;
  int ct2 = rway;
  if (millis()-timer>1000){stop();}

  

  
  
  
    lfcount.data = ct1;
    
  
    rfcount.data = ct2;
    
  // Publish encoder ticks to calculate odom on Jetson Nano side
  
  
  lfwheel.publish(&lfcount);
  rfwheel.publish(&rfcount);
  
  
 // calculate time and current velocity
  
 




  // Move the motors with the output of the pid
  
 
  
  
  nh.spinOnce();
  // take the old encoder ticks and time for calculating velocity
  
  
  delay(25);

}
