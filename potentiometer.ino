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
double last_err = 0;
double cum_err = 0;
double rate_err = 0;

double motor_out = 0;

double PID_constants [3] = {0, 0, 0}; // respectively for p, i, d
int ball_height;
double motor_correction;
unsigned long last_time = 0;
unsigned long curr_time = 0;

double PID_cycle (int distance, double d_time)
{

  double err = height_goal - distance;

  // PID controls //

  // proportional 
  double p_control = PID_constants[0] * err;
  // integral 
  cum_err += (err + last_err) * d_time / 2.0f;
  double i_control = cum_err * PID_constants[1];
  // derrivative 
  rate_err = (err - last_err) / d_time;
  double d_control = rate_err * PID_constants[2];

  double pid_out = int(p_control + 0 + 0);
  last_err = err;
  delay(1);
  return pid_out;
}

void updatePIDControls ()
{
  int p_pot = analogRead(PPotControl);
  double p_value = double(p_pot) / 1023.0f * 0.001f;
  int i_pot = analogRead(IPotControl);
  double i_value = double(i_pot) / 1023.0f * 0.001f;
  int d_pot = analogRead(DPotControl);
  double d_value = double(d_pot) / 1023.0f * 0.001f;

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
  // height_goal = int(double(pot_value) * 500 / 1023);
  
  updatePIDControls();
  printForGraph ();
//  printPIDConstants();

//  Serial.print("Pid out: ");
//  Serial.print(motor_out);
//  Serial.print(" height goal: ");
//  Serial.println(height_goal);  

  esc.write((int)motor_out);
  last_time = curr_time;

  delay (1);
}
