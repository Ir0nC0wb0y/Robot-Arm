#include <Arduino.h>
#include <pio_encoder.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <PIDLib.hpp>


// Encoder
  #define ENCODER_PIN_A D2
  #define ENCODER_PIN_B D3
  PioEncoder encoder(D2); // Encoder on Pins 1/2 (must be sequential)
  // Encoder Angle
    int enc_position = 0;
    double output_angle = 0.0;
    double output_speed = 0;
    double output_angle_last = 0;
    unsigned long enc_last_time = 0;
    #define POS_SETPOINT_CHANGE 30.0
    #define POS_SETPOINT_MULT    6

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
  #define POSITION_TIME 5000
  double PID_Pos_Setpoint = 0.0;
  #define POSITION_CHANGE 180

  // Display
    unsigned long loop_display_last = 0;
    #define DISPLAY_WAIT_TIME 50
    int header_display = 0;
    #define HEADER_COUNT 200


// PID
  #define LOOP_HIST_CNT 20
  // Inner loop (current)
    PIDLib::MeasurementType meas_current{};
    PIDLib::SetupType       setup_current{};
    PIDLib::PID             PID_current{};
    #define CURRENT_LIMIT_LOWER   -360.0
    #define CURRENT_LIMIT_UPPER    360.0
    double PIDout_current = 0.0;
    #define TIME_CURRENT 833
    unsigned long time_current_last = 0;
    unsigned long time_current_past[LOOP_HIST_CNT] = {};
    int time_current_past_idx = 0;
    
  // Middle loop (speed)
    PIDLib::MeasurementType meas_speed{};
    PIDLib::SetupType       setup_speed{};
    PIDLib::PID             PID_speed{};
    #define SPEED_LIMIT_LOWER   -12.0
    #define SPEED_LIMIT_UPPER    12.0
    double PIDout_speed = 0.0;
    #define TIME_SPEED 2500
    unsigned long time_speed_last = 0;
    unsigned long time_speed_past[LOOP_HIST_CNT] = {};
    int time_speed_past_idx = 0;

  // Outer loop (position)
    PIDLib::MeasurementType meas_position{};
    PIDLib::SetupType       setup_position{};
    PIDLib::PID             PID_position{};
    #define POSITION_LIMIT_LOWER   -1.0
    #define POSITION_LIMIT_UPPER    1.0
    double PIDout_position = 0.0;
    #define TIME_POSITION 5000
    unsigned long time_position_last = 0;
    unsigned long time_position_past[LOOP_HIST_CNT] = {};
    int time_position_past_idx = 0;

int Loop_Freq_calc(unsigned long time_list[LOOP_HIST_CNT]) {
  int freq_calc = 0;
  unsigned long time_sum = 0;

  // loop through values
  int num_valid_values = 0;
  for (int i=0; i < LOOP_HIST_CNT; i++) {
    if (time_list[i] > 0) {
      time_sum += time_list[i];
      num_valid_values++;
    }
  }

  // Divide by # valid values
  freq_calc = time_sum / num_valid_values; // summed us divided by count -> us
  freq_calc = 1000000 / freq_calc; // s/s-> Hz

  return freq_calc;
}

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

    // Set PID measurement values
    meas_speed.time_usec = encoder_time;
    meas_speed.value = output_speed;

    meas_position.time_usec = encoder_time;
    meas_position.value = output_angle;
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

  meas_current.time_usec = micros();
  meas_current.value = current_mA;
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

  // Setup PID
    // PID Setup: current
      setup_current.gainP = 1.0;
      setup_current.gainI = 0.0;
      setup_current.gainD = 0.0;
      setup_current.outputLowerLimit = CURRENT_LIMIT_LOWER;
      setup_current.outputUpperLimit = CURRENT_LIMIT_UPPER;

      PID_current.begin(setup_current);

    // PID Setup: speed
      setup_speed.gainP = 1.0;
      setup_speed.gainI = 0.0;
      setup_speed.gainD = 0.0;
      setup_speed.outputLowerLimit = SPEED_LIMIT_LOWER;
      setup_speed.outputUpperLimit = SPEED_LIMIT_UPPER;

      PID_speed.begin(setup_speed);

    // PID Setup: position
      setup_position.gainP = 1.0;
      setup_position.gainI = 0.0;
      setup_position.gainD = 0.0;
      setup_position.outputLowerLimit = POSITION_LIMIT_LOWER;
      setup_position.outputUpperLimit = POSITION_LIMIT_UPPER;

      PID_position.begin(setup_position);


  // Prepare for loop
    AngleCalcs();

}

void loop() {

  bool run_current  = false;
  bool run_speed    = false;
  bool run_position = false;

  unsigned long time_loop_run = micros();

  if (micros() - time_current_last >= TIME_CURRENT) {
    run_current = true;
    time_current_past[time_current_past_idx] = micros() - time_current_last;
    time_current_past_idx++;
    if (time_current_past_idx >= LOOP_HIST_CNT) {
      time_current_past_idx = 0;
    }
    time_current_last = micros();
  }

  if (micros() - time_speed_last >= TIME_SPEED) {
    run_speed = true;
    time_speed_past[time_speed_past_idx] = micros() - time_speed_last;
    time_speed_past_idx++;
    if (time_speed_past_idx >= LOOP_HIST_CNT) {
      time_speed_past_idx = 0;
    }
    time_speed_last = micros();
  }

  if (micros() - time_position_last >= TIME_POSITION) {
    run_position = true;
    time_position_past[time_position_past_idx] = micros() - time_position_last;
    time_position_past_idx++;
    if (time_position_past_idx >= LOOP_HIST_CNT) {
      time_position_past_idx = 0;
    }
    time_position_last = micros();
  }

  if (run_speed || run_position) {
    AngleCalcs();
    if (run_position) {
      PIDout_position = PID_position.run(meas_position);
      PID_speed.setpoint(PIDout_position);
    }

    if (run_speed) {
      PIDout_speed    = PID_speed.run(meas_speed);
      PID_current.setpoint(PIDout_speed);
    }
  }

  if (run_current) {
    CurrentSense();
    PIDout_current  = PID_current.run(meas_current);
    RunMotor(PIDout_current);
  }
  

      
  if (millis() - loop_display_last >= DISPLAY_WAIT_TIME) {
    if (header_display >= HEADER_COUNT) {
      Serial.println("Time [us]|Pos_Setpoint|Current_Position|Pos_Error|Position_Freq|Speed_Setpoint|Current_Speed|Speed_Error|Speed_Freq|Pos_Setpoint|Current_Current|Current_Error|Current_Freq");
      header_display = 0;
    } else {
      header_display++;
    }

    Serial.print(micros());
      Serial.print("|");
      Serial.print(PID_Pos_Setpoint,2);
      Serial.print("|");
      Serial.print(meas_position.value, 2);
      Serial.print("|");
      Serial.print((PID_Pos_Setpoint-meas_position.value),4);
      Serial.print("|");
      Serial.print(Loop_Freq_calc(time_position_past));

      Serial.print("|");
      Serial.print(PIDout_position,2);
      Serial.print("|");
      Serial.print(meas_speed.value, 2);
      Serial.print("|");
      Serial.print((PIDout_position-meas_speed.value),4);
      Serial.print("|");
      Serial.print(Loop_Freq_calc(time_speed_past));

      Serial.print("|");
      Serial.print(PIDout_speed,2);
      Serial.print("|");
      Serial.print(meas_current.value, 2);
      Serial.print("|");
      Serial.print((PIDout_speed-meas_current.value),4);
      Serial.print("|");
      Serial.print(Loop_Freq_calc(time_current_past));

      Serial.println();
      
    
    loop_display_last = micros();
  }
  
  // Change Position setpoint
  if (millis() - loop_position_last >= POSITION_TIME) {
    if (PID_Pos_Setpoint > 0) {
      PID_Pos_Setpoint = -POSITION_CHANGE;
    } else {
      PID_Pos_Setpoint = POSITION_CHANGE;
    }
    PID_position.setpoint(PID_Pos_Setpoint);
    loop_position_last = millis();
  }
 
}