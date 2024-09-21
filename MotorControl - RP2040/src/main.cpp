#include <Arduino.h>
#include <pio_encoder.h>
#include <ArduPID.h>
#include <Wire.h>
#include <Adafruit_INA219.h>


// Encoder
  #define ENCODER_PIN_A D2
  #define ENCODER_PIN_B D3
  PioEncoder encoder(D2); // Encoder on Pins 1/2 (must be sequential)

// Motor
  #define MOTOR_PIN_A D9
  #define MOTOR_PIN_B D8
  #define MOTOR_OUTPUT_MIN 10
  #define ENCODER_CPR 48.0
  #define MOTOR_GEARING 9.6

// Current Sensor
  Adafruit_INA219 ina219;
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;


// Loop
  // Setpoint
  unsigned long loop_position_last = 0;
  #define POSITION_TIME 10000
  //long motor_position = 0;
  double output_angle = 0.0;
  int enc_last_pos = 0;
  unsigned long enc_last_time = 0;

  // Display
  unsigned long loop_display_last = 0;
  #define DISPLAY_WAIT_TIME 50
  int header_display = 0;
  #define HEADER_COUNT 200


// PID
  #define SAMPLE_PERIOD 2
  unsigned long PID_sample_last = 0;
  // Position
    double PID_Pos_Input = 0;
    double PID_Pos_Output = 0;
    double PID_Pos_Setpoint = 0;
    double Pos_Kp = 0.5; // 1.5
    double Pos_Ki = 0.0005; // 0.3
    double Pos_Kd = 0.0; // 0.1
    ArduPID PID_Pos;
    #define POS_SETPOINT_LIMITS 180.0
    #define POS_WINDUP_LIMITS  64
    #define POS_OUTPUT_LIMITS 255
  /*// Speed
    double PID_Speed_Input = 0;
    double PID_Speed_Output = 0;
    double PID_Speed_Setpoint = 0;
    double Speed_Kp = 0.0000001;
    double Speed_Ki = 0.0;//0.000075;
    double Speed_Kd = 0.0;
    ArduPID PID_Speed;
    //#define SPEED_SETPOINT_LIMITS 255
    #define SPEED_WINDUP_LIMITS 150
    #define SPEED_OUTPUT_LIMITS 255
    */
    



void RunMotor(int speed) {
  //Serial.print("Motor speed "); Serial.println(speed);
  if (speed > MOTOR_OUTPUT_MIN) {
    digitalWrite(MOTOR_PIN_A, HIGH); 
    analogWrite(MOTOR_PIN_B, 255-speed);
  } else if (speed < MOTOR_OUTPUT_MIN) {
    digitalWrite(MOTOR_PIN_A, LOW); 
    analogWrite(MOTOR_PIN_B, -speed);
  } else {
    digitalWrite(MOTOR_PIN_A, LOW);
    digitalWrite(MOTOR_PIN_B, LOW);
  }
  
  return;
}

void CurrentSense() {
  shuntvoltage = ina219.getShuntVoltage_mV();
  //busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  //power_mW = ina219.getPower_mW();
  //loadvoltage = busvoltage + (shuntvoltage / 1000);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }
  Serial.println("Starting Sketch");
  
  // Encoder 
  Serial.println("Starting Encoder");
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);
  encoder.begin();

  // Motor
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  digitalWrite(MOTOR_PIN_A, LOW);
  digitalWrite(MOTOR_PIN_B, LOW);

  // Current Sensor
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  // Setup PID - Position
    PID_Pos.begin(&PID_Pos_Input, &PID_Pos_Output, &PID_Pos_Setpoint, Pos_Kp, Pos_Ki, Pos_Kd);
    PID_Pos.setOutputLimits(-POS_OUTPUT_LIMITS, POS_OUTPUT_LIMITS);
    PID_Pos.setWindUpLimits(-POS_WINDUP_LIMITS, POS_WINDUP_LIMITS);
    //PID_Pos.setDeadBand(-OUTPUT_MIN,OUTPUT_MIN);
    PID_Pos.start();
    //motor_position = POS_SETPOINT_LIMITS;
    PID_Pos_Setpoint = POS_SETPOINT_LIMITS;

  // Setup PID - Speed
    /*PID_Speed.begin(&PID_Speed_Input, &PID_Speed_Output, &PID_Speed_Setpoint, Speed_Kp, Speed_Ki, Speed_Kd);
    PID_Speed.setOutputLimits(-SPEED_OUTPUT_LIMITS, SPEED_OUTPUT_LIMITS);
    PID_Speed.setWindUpLimits(-SPEED_WINDUP_LIMITS, SPEED_WINDUP_LIMITS);
    //PID_Pos.setDeadBand(-OUTPUT_MIN,OUTPUT_MIN);
    PID_Speed.start();
    PID_Speed_Setpoint = PID_Pos_Output;*/
    //PID_Pos_Setpoint = (double) motor_position;

  enc_last_pos = encoder.getCount();
  enc_last_time = micros();

}

void loop() {
  double enc_move = 0.0;
  double enc_move_denum = 0.0;
  double enc_move_num = 0.0;
  int enc_position = 0;
  if (millis() - PID_sample_last >= SAMPLE_PERIOD) {
    enc_position = encoder.getCount();
    output_angle = (double)enc_position / (ENCODER_CPR * MOTOR_GEARING);
    output_angle = 360.0 * output_angle;
    PID_Pos_Input = output_angle;
    CurrentSense();
    //unsigned long enc_cur_time = micros();
    //enc_move_num = ((double)enc_position - (double)enc_last_pos);
    //enc_move_denum = (((double)enc_cur_time - (double)enc_last_time)/1000000.0);
    //enc_move = enc_move_num / enc_move_denum;
    //double enc_move = (double)(enc_position - enc_last_pos) / ((double)(enc_cur_time - enc_last_time)/1000000.0) ; //
    
    //PID_Speed_Input = enc_move;
    //enc_last_pos = enc_position;
    //enc_last_time = enc_cur_time;
    PID_Pos.compute();
    //PID_Speed_Setpoint = PID_Pos_Output;
    //PID_Speed.compute();
    RunMotor((int)PID_Pos_Output);
    PID_sample_last = millis();

  

    //if ( enc_position != enc_last_pos ) {
    //  Serial.print("Counts / time = speed : ");
    //    Serial.print(enc_move_num,6);
    //    Serial.print(" / ");
    //    Serial.print(enc_move_denum,6);
    //    Serial.print(" = ");
    //    Serial.println(enc_move,6);
    //}
  }
      
  if (millis() - loop_display_last >= DISPLAY_WAIT_TIME) {
    if (header_display >= HEADER_COUNT) {
      //Serial.print("Current PID's, Kp ");
      //  Serial.print(myPID.GetKp());
      //  Serial.print(", Ki ");
      //  Serial.print(myPID.GetKi());
      //  Serial.print(", Kd ");
      //  Serial.println(myPID.GetKd());
      Serial.println("Time [ms] | Pos_Setpoint | Pos_Position | Encoder Pos | Pos_Output | Shunt Voltage | Current [mA]");
      header_display = 0;
    } else {
      header_display++;
    }

    Serial.print(millis());
      Serial.print("|");
      Serial.print(PID_Pos_Setpoint,2);
      Serial.print("|");
      Serial.print(PID_Pos_Input, 2);
      Serial.print("|");
      Serial.print(enc_position);
      Serial.print("|");
      Serial.print((int)PID_Pos_Output);
      Serial.print("|");
      Serial.print(shuntvoltage);
      Serial.print("|");
      Serial.print(current_mA);
      //Serial.print("|");
      //Serial.print((int)PID_Speed_Setpoint);
      //Serial.print("|");
      //Serial.print(enc_move_num);
      //Serial.print("|");
      //Serial.print((int)PID_Speed_Output);
      Serial.println();
      
    
    loop_display_last = millis();
  }
  

  if (millis() - loop_position_last >= POSITION_TIME) {
    //motor_position = random(-SETPOINT_LIMITS,SETPOINT_LIMITS);
    PID_Pos_Setpoint = -PID_Pos_Setpoint;
    Serial.print("New Setpoint: ");
      Serial.println(PID_Pos_Setpoint);
    loop_position_last = millis();
  }
 
}