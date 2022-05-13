#include "Adafruit_VL53L0X.h"
#include <Servo.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

Servo esc;

#define heightPotPin A0

#define PPotControl A1
#define IPotControl A2
#define DPotControl A3

const int motor_pin = 9;

const int motor_min = 0;
const int motor_max = 180;

int height_goal = 250; // in mm
int tube_height = 500; // in mm 
float last_err = 0;
float cum_err = 0;
float rate_err = 0;

int motor_out = 0;

float PID_constants [3] = {0, 0, 0}; // respectively for p, i, d
int ball_height;
int motor_correction;
unsigned long last_time = 0;
unsigned long curr_time = 0;

int PID_cycle (int distance, float d_time)
{

  float err = height_goal - distance;

  // PID controls //

  // proportional 
  float p_control = PID_constants[0] * err;
  // integral 
  cum_err += (err + last_err) * d_time / 2.0f;
  float i_control = cum_err * PID_constants[1];
  // derrivative 
  rate_err = (err - last_err) / d_time;
  float d_control = rate_err * PID_constants[2];

  int pid_out = int(p_control + 0 + 0);
  last_err = err;
  delay(1);
  return pid_out;
}

void updatePIDControls ()
{
  int p_pot = analogRead(PPotControl);
  float p_value = float(p_pot) / 1023.0f * 0.01f;
  int i_pot = analogRead(IPotControl);
  float i_value = float(i_pot) / 1023.0f * 0.01f;
  int d_pot = analogRead(DPotControl);
  float d_value = float(d_pot) / 1023.0f * 0.01f;

  PID_constants[0] = p_value;
  PID_constants[1] = i_value;
  PID_constants[2] = d_value;
}

void printPIDConstants ()
{
  Serial.print("p: ");
  Serial.print(PID_constants[0]);
  Serial.print(" i: ");
  Serial.print(PID_constants[1]);
  Serial.print(" d: ");
  Serial.println(PID_constants[2]);
}

void printForGraph ()
{
  Serial.print("target:");
  Serial.print(height_goal);
  Serial.print(" height:");
  Serial.print (ball_height);
  Serial.print (" motor_out:");
  Serial.print (motor_out * 2); 
  Serial.print (" PID_out:");
  Serial.print (min(motor_correction, 500));
  Serial.print (" P_param:");
  Serial.println(PID_constants[0] * 10000);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  esc.attach(motor_pin, 1000, 2000);
  delay(1000);
  esc.write(180);
  delay(1000);
  esc.write(0);
  delay(2000);

  while (! Serial)
  {
    delay(1);
  }
  if (!lox.begin())
  {
    Serial.println(F("Failed to boot VL53L0X"));
    // while(1);
  }
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4)
  {
    
    int distance = measure.RangeMilliMeter - 35; // in mm
   
    ball_height = tube_height - distance;
    curr_time = millis();
    motor_correction = PID_cycle (ball_height, (curr_time - last_time) / 1000.0f);
    motor_out += motor_correction;
    motor_out = min(180, motor_out);
    motor_out = max(0, motor_out);
  } else {
//    Serial.println(" out of range "); 
    motor_out = motor_max;
  }
  
  int pot_value = analogRead(heightPotPin);
  // height_goal = int(float(pot_value) * 500 / 1023);
  
  updatePIDControls();
  printForGraph ();
//  printPIDConstants();

//  Serial.print("Pid out: ");
//  Serial.print(motor_out);
//  Serial.print(" height goal: ");
//  Serial.println(height_goal);  

  esc.write(motor_out);
  last_time = curr_time;

  delay (1);
}
