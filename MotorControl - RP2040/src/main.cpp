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
  #define MOTOR_OUTPUT_MIN 0
  #define ENCODER_CPR 48.0
  #define MOTOR_GEARING 9.6

// Current Sensor
  Adafruit_INA219 ina219;
  //float shuntvoltage = 0;
  //float busvoltage = 0;
  float current_mA = 0;
  //float loadvoltage = 0;
  //float power_mW = 0;


// Loop
  // Setpoint
  unsigned long loop_position_last = 0;
  #define POSITION_TIME 500
  int enc_position = 0;
  double output_angle = 0.0;
  double output_speed = 0;
  double output_angle_last = 0;
  unsigned long enc_last_time = 0;
  #define POS_SETPOINT_CHANGE 30.0
  #define POS_SETPOINT_MULT    6

  // Display
  unsigned long loop_display_last = 0;
  #define DISPLAY_WAIT_TIME 50
  int header_display = 0;
  #define HEADER_COUNT 200


// PID
  bool PID_latch = false;
  #define PID_LATCH_POINT 45.0
  #define SAMPLE_PERIOD 500 //micros
  unsigned long PID_sample_last = 0;
  // Position
    double PID_Pos_Input = 0;
    double PID_Pos_Output = 0;
    double PID_Pos_Setpoint = 0;
    double Pos_Kp = 1.5; // 1.5
    double Pos_Ki = 0.20; // 0.3
    double Pos_Kd = 0.1; // 0.1
    ArduPID PID_Pos;
    #define POS_WINDUP_LIMITS  64
    #define POS_OUTPUT_LIMITS 255
    

void AngleCalcs() {
    // Get encoder position
    enc_position = encoder.getCount();
    unsigned long encoder_time = micros();
    
    // Calculate Angle @ output
    output_angle = (double)enc_position / (ENCODER_CPR * MOTOR_GEARING); // [%rotation]
    output_angle = 360.0 * output_angle; // [deg]

    // Calculate rotation speed @output
    output_speed = (output_angle - output_angle_last) / (double)(encoder_time - enc_last_time); // [deg/us]
    output_speed = output_speed * 166666.6666666667; // Convert to [rpm]

    // Collect previous values
    output_angle_last = output_angle;
    enc_last_time = encoder_time;
}

void RunMotor(int speed) {
  //Serial.print("Motor speed "); Serial.println(speed);
  if (speed > MOTOR_OUTPUT_MIN) {
    digitalWrite(MOTOR_PIN_A, HIGH); 
    analogWrite(MOTOR_PIN_B, 255-speed);
  } else if (speed < MOTOR_OUTPUT_MIN) {
    analogWrite(MOTOR_PIN_A, 255+speed);
    digitalWrite(MOTOR_PIN_B, HIGH); 
  } else {
    digitalWrite(MOTOR_PIN_A, LOW);
    digitalWrite(MOTOR_PIN_B, LOW);
  }
  
  return;
}

void CurrentSense() {
  //shuntvoltage = ina219.getShuntVoltage_mV();
  //busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  //power_mW = ina219.getPower_mW();
  //loadvoltage = busvoltage + (shuntvoltage / 1000);
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      yield();
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
    PID_Pos_Setpoint = POS_SETPOINT_CHANGE;

  // Prepare for loop
    AngleCalcs();
    if (PID_Pos_Setpoint - output_angle <= PID_LATCH_POINT) {
      PID_latch = true;
    }

}

void loop() {

  if (micros() - PID_sample_last >= SAMPLE_PERIOD) {
    AngleCalcs();
    CurrentSense();
    PID_Pos_Input = output_angle;
    if (abs(PID_Pos_Setpoint - output_angle) <= PID_LATCH_POINT) {
      // Run Motor with PID
      if (!PID_latch) {
        PID_latch = true;
        PID_Pos.reset();
      }
      PID_Pos.compute();
      RunMotor((int)PID_Pos_Output);
    } else {
      // Run Motor Bang-Bang
      if (PID_latch) {
        PID_latch = false;
      }
      if (PID_Pos_Setpoint - output_angle > 0) {
        PID_Pos_Output = 255.0;
      } else {
        PID_Pos_Output = -255.0;
      }
    }
    RunMotor((int)PID_Pos_Output);

    // Loop Timing
    PID_sample_last = millis();

  }
      
  if (millis() - loop_display_last >= DISPLAY_WAIT_TIME) {
    if (header_display >= HEADER_COUNT) {
      Serial.println("Time [us]|Pos_Setpoint|Pos_Position|Error|PID_Latch|Pos_Output|P_out|I_out|D_out|Current [mA]|Output Speed [deg/s]");
      header_display = 0;
    } else {
      header_display++;
    }

    Serial.print(micros());
      Serial.print("|");
      Serial.print(PID_Pos_Setpoint,2);
      Serial.print("|");
      Serial.print(PID_Pos_Input, 2);
      Serial.print("|");
      Serial.print((PID_Pos_Setpoint-PID_Pos_Input),4);
      Serial.print("|");
      Serial.print(PID_latch);
      Serial.print("|");
      Serial.print((int)PID_Pos_Output);
      Serial.print("|");
      Serial.print(PID_Pos.Pout(),4);
      Serial.print("|");
      Serial.print(PID_Pos.Iout(),4);
      Serial.print("|");
      Serial.print(PID_Pos.Dout(),4);
      Serial.print("|");
      Serial.print(current_mA);
      Serial.print("|");
      Serial.print(output_speed);
      Serial.println();
      
    
    loop_display_last = micros();
  }
  

  if (millis() - loop_position_last >= POSITION_TIME) {
    PID_Pos_Setpoint = PID_Pos_Setpoint + random(-POS_SETPOINT_MULT,POS_SETPOINT_MULT+1)*POS_SETPOINT_CHANGE;
    //Serial.print("New Setpoint: ");
    //  Serial.println(PID_Pos_Setpoint);
    loop_position_last = millis();
  }
 
}