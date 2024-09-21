#include <Arduino.h>
#include <Encoder.h>
#include <FireTimer.h>
#include <ArduPID.h>

#define PIN_MOTOR_DIR  5
#define PIN_MOTOR_PWM  6
#define PIN_ENCODER_A  2
#define PIN_ENCODER_B  3

// Looping Variables
  // Setpoint
  unsigned long loop_position_last = 0;
  #define POSITION_TIME 5000
  long motor_position = 0;

  // Display
  unsigned long loop_display_last = 0;
  #define DISPLAY_WAIT_TIME 50
  int header_display = 0;
  #define HEADER_COUNT 50

  // Encoder
  Encoder myEnc(PIN_ENCODER_A,PIN_ENCODER_B);

// PID
  double PID_Input = 0;
  double PID_Output = 0;
  double PID_Setpoint = 0;
  double Kp = 5.0; // 1.5
  double Ki = 5.0; // 0.3
  double Kd = 0.0; // 0.1
  //PID myPID(&PID_Input, &PID_Output, &PID_Setpoint, Kp, Ki, Kd, DIRECT);
  ArduPID myController;
  #define SETPOINT_LIMITS 3000
  #define WINDUP_LIMITS  30
  #define OUTPUT_LIMITS 255
  #define OUTPUT_MIN 00


void RunMotor(int speed) {
  //Serial.print("Motor speed "); Serial.println(speed);
  if (speed > OUTPUT_MIN) {
    digitalWrite(PIN_MOTOR_DIR, HIGH); 
    analogWrite(PIN_MOTOR_PWM, 255-speed);
    //analogWrite(PIN_MOTOR_PWM,255-speed);
  } else if (speed < OUTPUT_MIN) {
    digitalWrite(PIN_MOTOR_DIR, LOW); 
    //digitalWrite(PIN_MOTOR_PWM, HIGH);
    analogWrite(PIN_MOTOR_PWM, -speed);
  } else {
    digitalWrite(PIN_MOTOR_DIR, LOW);
    digitalWrite(PIN_MOTOR_PWM, LOW);
  }
  //analogWrite(PIN_MOTOR_PWM, speed); // Go
  
  return;
}

void setup() {
  Serial.begin(9600);
  Serial.println("Starting motor control program");

  // Setup Motor Pins
  pinMode(PIN_MOTOR_DIR, OUTPUT);
  pinMode(PIN_MOTOR_PWM, OUTPUT);
  digitalWrite(PIN_MOTOR_DIR, LOW);
  analogWrite(PIN_MOTOR_PWM, 0);

  // Setup PID
    //myPID.SetMode(AUTOMATIC);
    //myPID.SetOutputLimits(-OUTPUT_LIMITS,OUTPUT_LIMITS);
    //myPID.SetSampleTime(20);
    myController.begin(&PID_Input, &PID_Output, &PID_Setpoint, Kp, Ki, Kd);
    myController.setOutputLimits(-OUTPUT_LIMITS,OUTPUT_LIMITS);
    myController.setWindUpLimits(-WINDUP_LIMITS, WINDUP_LIMITS);
    //myController.setDeadBand(-OUTPUT_MIN,OUTPUT_MIN);
    myController.start();

  motor_position = random(-SETPOINT_LIMITS,SETPOINT_LIMITS);
  PID_Setpoint = (double) motor_position;
}

void loop() {
  PID_Input = (double)myEnc.read();
  //myPID.Compute();
  myController.compute();
  RunMotor((int)PID_Output);

  if (millis() - loop_position_last >= POSITION_TIME) {
    motor_position = random(-SETPOINT_LIMITS,SETPOINT_LIMITS);
    PID_Setpoint = (double) motor_position;
    //Serial.print("New Setpoint: ");
    //  Serial.println(PID_Setpoint);
    loop_position_last = millis();
  }

  if (millis() - loop_display_last >= DISPLAY_WAIT_TIME) {
    if (header_display >= HEADER_COUNT) {
      //Serial.print("Current PID's, Kp ");
      //  Serial.print(myPID.GetKp());
      //  Serial.print(", Ki ");
      //  Serial.print(myPID.GetKi());
      //  Serial.print(", Kd ");
      //  Serial.println(myPID.GetKd());
      Serial.println("Time [ms] | Setpoint | Position | Output");
      header_display = 0;
    } else {
      header_display++;
    }

    Serial.print(millis());
      Serial.print("|");
      Serial.print((int)PID_Setpoint);
      Serial.print("|");
      Serial.print(PID_Input, 0);
      Serial.print("|");
      Serial.println((int)PID_Output);

    
    loop_display_last = millis();
  }

}