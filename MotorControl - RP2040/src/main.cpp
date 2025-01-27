#include <Arduino.h>
#include <pio_encoder.h>
#include <Wire.h>
#include <PIDLib.hpp>
#include <AS5600.h>
#include <RP2040_PWM.h>

#include "motor_params.h"

// Encoder
  #define ENCODER_PIN_A D8
  #define ENCODER_PIN_B D9
  PioEncoder encoder(D2); // Encoder on Pins 1/2 (must be sequential)
  // Encoder Angle
    int enc_position = 0;
    double output_angle = 0.0;
    double output_speed = 0;
    double output_angle_last = 0;
    unsigned long enc_last_time = 0;
    #define POS_SETPOINT_CHANGE 30.0
    #define POS_SETPOINT_MULT    6

// AS5600
  class OutputAngle {
    public:
      int raw = 0;
      double angle = 0.0;
      double output_speed = 0.0;
      //double output_angle_last = 0.0;
      unsigned long angle_time = 0;
  };
  #define ENCODER_PIN_DIR D3
  #define ENCODER_OFFSET 0.0
  AS5600L as5600;
  OutputAngle angle_output;
  #define PRINT_ENCODER_INFO false

// Motor
  #define MOTOR_PIN_A D6
  #define MOTOR_PIN_B D7
  #define MOTOR_OUTPUT_MIN 0
  #define ENCODER_CPR 48.0
  #define MOTOR_GEARING 264

// RP2040 PWM
  float frequency = 25000.0; // Frequency, in Hz
  float dutyCycle_A = 0.0;
  float dutyCycle_B = 0.0;
  RP2040_PWM* PWM_Instance[2];

// Current Sensor
  // ACS712
    #define ACS712_PIN A0
    int ACS712_raw = 0;
    float ACS712_calc = 0.0;
    #define ACS712_CONVERSION 0.006663468
    #define ACS712_MIDPOINT 2086
    int ACS712_midpoint = ACS712_MIDPOINT;
    #define ACS712_READINGS 10
    #define ACS712_CALIBRATE_COUNT 10000

  // BUS VOLTAGE
    #define BUS_READINGS 4
    #define PIN_BUS_VOLTAGE_A A1
    #define PIN_BUS_VOLTAGE_B A2
    #define RESISTOR_A_HIGH 68000
    #define RESISTOR_A_LOW  19620
    #define RESISTOR_B_HIGH 67200
    #define RESISTOR_B_LOW  19470
    #define BUS_ADC_CONVERSION 0.00082275
    #define BUS_A_CONVERSION (RESISTOR_A_HIGH+RESISTOR_A_LOW)/RESISTOR_A_LOW
    #define BUS_B_CONVERSION (RESISTOR_B_HIGH+RESISTOR_B_LOW)/RESISTOR_B_LOW
    class MotorVoltage {
      public:
        int Bus_A_raw = 0;
        int Bus_B_raw = 0;
        float Bus_A_conv = 0.0;
        float Bus_B_conv = 0.0;
        float Bus_voltage = 0.0;
        float Bus_offset = 0.0;
    };
    MotorVoltage motorVoltage;


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

double Loop_Freq_calc(unsigned long time_list[LOOP_HIST_CNT]) {
  double freq_calc = 0.0;
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
  freq_calc = (double)time_sum / (double)num_valid_values; // summed us divided by count -> us
  freq_calc = 1000000.0 / freq_calc; // s/s-> Hz

  return freq_calc;
}

OutputAngle AngleOutput(OutputAngle input_angle, bool reset = false) {
  int32_t enc_position = as5600.getCumulativePosition();
  unsigned long angle_time = micros();

  // Calculate Angle @ output
  double calc_output_angle = (double)enc_position / 4096.0; // [%rotation]
  calc_output_angle = 360.0 * calc_output_angle; // [deg]
  //double calc_output_angle = as5600.getCumulativePosition();

  // Calculate Speed from angle
  double angle_diff = 0;
  double time_diff = 0;
  double calc_speed = 0;
  if (!reset) {
    angle_diff = calc_output_angle - input_angle.angle;
    time_diff = ((double)angle_time - (double)input_angle.angle_time)/1000000;
    calc_speed = angle_diff / time_diff;
  }

  // Set output class
  OutputAngle output_angle;
  output_angle.raw = enc_position;
  output_angle.angle = calc_output_angle;
  output_angle.output_speed = calc_speed;
  output_angle.angle_time = angle_time;

  return output_angle;
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

void RunMotor_RP2040(float speed, bool brake=true) {
  if (speed > MOTOR_OUTPUT_MIN) {
    if (brake) {
      PWM_Instance[0]->setPWM(MOTOR_PIN_A, frequency, 100.0);
    } else {
      PWM_Instance[0]->setPWM(MOTOR_PIN_A, frequency, 0.0);
    }
    PWM_Instance[1]->setPWM(MOTOR_PIN_B, frequency, 100.0-speed);
  } else if (speed < -MOTOR_OUTPUT_MIN) {
    if (brake) {
      PWM_Instance[1]->setPWM(MOTOR_PIN_B, frequency, 100.0);
    } else {
      PWM_Instance[1]->setPWM(MOTOR_PIN_B, frequency, 0.0);
    }
    PWM_Instance[0]->setPWM(MOTOR_PIN_A, frequency, 100.0+speed);
  } else {
    if (brake) {
      PWM_Instance[0]->setPWM(MOTOR_PIN_A, frequency, 100.0);
      PWM_Instance[1]->setPWM(MOTOR_PIN_B, frequency, 100.0);
    } else {
      PWM_Instance[0]->setPWM(MOTOR_PIN_A, frequency, 0.0);
      PWM_Instance[1]->setPWM(MOTOR_PIN_B, frequency, 0.0);
    }
  }
}

/*
MotorVoltage Motor_Voltage() {
  MotorVoltage conversion;
  // Collect ADC data
  for (int i = 0; i < BUS_READINGS; i++) {
    conversion.Bus_A_raw += analogRead(PIN_BUS_VOLTAGE_A);
    conversion.Bus_B_raw += analogRead(PIN_BUS_VOLTAGE_B);
  }
  conversion.Bus_A_raw = conversion.Bus_A_raw / BUS_READINGS;
  conversion.Bus_B_raw = conversion.Bus_B_raw / BUS_READINGS;

  // Calculate Input Voltage
  conversion.Bus_A_conv = conversion.Bus_A_raw * BUS_ADC_CONVERSION * BUS_A_CONVERSION;
  conversion.Bus_B_conv = conversion.Bus_B_raw * BUS_ADC_CONVERSION * BUS_B_CONVERSION;

  // Calculate Voltage Difference
  conversion.Bus_voltage = conversion.Bus_A_conv - conversion.Bus_B_conv;
  if (conversion.Bus_A_conv > conversion.Bus_B_conv) {
    conversion.Bus_offset = conversion.Bus_B_conv;
  } else {
    conversion.Bus_offset = conversion.Bus_A_conv;
  }

  return conversion;
}
*/

void Current_ACS712() {
  int read = 0;
  for (int i=0;i<ACS712_READINGS;i++) {
    read += analogRead(ACS712_PIN);
  }
  meas_current.time_usec = micros();
  ACS712_raw = read / ACS712_READINGS;
  int adjusted_raw = ACS712_raw - ACS712_midpoint;
  ACS712_calc = (float)adjusted_raw * ACS712_CONVERSION;

  
  meas_speed.value = ACS712_calc;
}

void ACS712_Calibrate_Midpoint() {
  //unsigned long calibrate_finish = millis() + ACS712_CALIBRATE_TIME;
  unsigned int reading_count = 0;
  unsigned int read = 0;
  while (reading_count <= ACS712_CALIBRATE_COUNT) {
    reading_count++;
    read += analogRead(ACS712_PIN);
  }
  ACS712_midpoint = (int) (read / reading_count);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      yield();
  }
  delay(3000);
  Serial.println("Starting Sketch");

  // Setup RP2040 PWM
    PWM_Instance[0] = new RP2040_PWM(MOTOR_PIN_A, frequency, dutyCycle_A);
    PWM_Instance[1] = new RP2040_PWM(MOTOR_PIN_B, frequency, dutyCycle_B);

  // ACS712 Setup
    analogReadResolution(12);
    ACS712_Calibrate_Midpoint();
    Serial.print("New ACS712 Midpoint: "); Serial.println(ACS712_midpoint);
  
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

  // Setup I2C Wire
    Wire.setSCL(D5);
    Wire.setSDA(D4);
    Wire.setClock(400000);
    Wire.begin();

  /*
  // Output Encoder
    as5600.setAddress(0x36);
    as5600.setDirection(AS5600_CLOCK_WISE);
      Serial.print("Connected to encoder at address "); Serial.println(as5600.getAddress());
      as5600.begin(ENCODER_PIN_DIR);
      if (!as5600.isConnected()) {
        Serial.println("AS5600 is not connected");
        while (1) {delay(10);}
      } else {
        Serial.println("AS5600 Connected!");
        as5600.setOffset(ENCODER_OFFSET);
        
        if (PRINT_ENCODER_INFO) {  
          Serial.print("Status: "); Serial.println(as5600.readStatus());
          Serial.print("AGC   : "); Serial.println(as5600.readAGC());
          Serial.print("Mag   : "); Serial.println(as5600.readMagnitude());
          Serial.println();
          Serial.print("Mag Detect : "); Serial.println(as5600.readAGC());
          Serial.print("Mag 2Strong: "); Serial.println(as5600.magnetTooStrong());
          Serial.print("Mag 2Weak  : "); Serial.println(as5600.magnetTooWeak());
          Serial.println();
          Serial.print("ZPos: "); Serial.println(as5600.getZPosition());
          Serial.print("MPos: "); Serial.println(as5600.getMPosition());

          angle_output = AngleOutput(angle_output, true);
        }
        delay(3000);
      }
  */

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
    Current_ACS712();
    PIDout_current  = PID_current.run(meas_current);
    RunMotor_RP2040(PIDout_current);
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