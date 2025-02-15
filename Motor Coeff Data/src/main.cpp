#include <Arduino.h>
//#include <pio_encoder.h>
#include <PIDLib.hpp>
#include <Wire.h>
#include <INA219.h>
#include <AS5600.h>
#include <RP2040_PWM.h>

#include "motor_params.h"

// Encoder (Input)
  //#define ENCODER_PIN_A D8
  //#define ENCODER_PIN_B D9
  //PioEncoder encoder(D2); // Encoder on Pins 1/2 (must be sequential)
  class OutputAngle {
    public:
      int raw = 0;
      double angle = 0.0;
      double output_speed = 0.0;
      //double output_angle_last = 0.0;
      unsigned long angle_time = 0;
  };

// Encoder (AS5600)
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

// Current Sensor
  ////INA219
  //  #define INA219_ADDRESS 0x45
  //  INA219 INA(INA219_ADDRESS);
  //  float current_mA = 0.0;
  //  //float shunt_mV = 0.0;

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

// RP2040 PWM
  float frequency = 25000.0; // Frequency, in Hz
  float dutyCycle_A = 0.0;
  float dutyCycle_B = 0.0;
  RP2040_PWM* PWM_Instance[2];

// Loop
  // Display
  unsigned long loop_display_last = 0;
  #define DISPLAY_WAIT_TIME  500 //Output angle and current, [us]
  #define HEADER_COUNT 200
  int header_display = HEADER_COUNT;
  bool display_force = false;
  int motor_loop = 0;

  // Run Speed
  float speed_start = 0;
  float speed_end   = 0;
  float speed_cur = 0;
  float speed_calc_slope = 0.0;
  #define SPEED_MAX 100.0
  //int speed_temp = 0;
  //#define SPEED_CHANGE 5
  bool speed_dir = true;
  bool speed_go = false;
  bool speed_reached = false;
  //#define SPEED_STEP           1
  //#define SPEED_RUN_STEP    1000
  #define SPEED_RUN_GO     5000000
  #define SPEED_RUN_STOP   5000000
  #define SPEED_TIME_MIN   2500000
  #define SPEED_TIME_MAX   2500000
  unsigned long loop_speed_next  = 0;
  unsigned long loop_speed_start = 0;
  unsigned long loop_speed_end   = 0;
  //int freq_tries = 0;
  //#define FREQ_TIMES 2

// PID Setup
  #define PID_INTERVAL_TORQUE   500 //us
  #define PID_INTERVAL_SPEED    PID_INTERVAL_TORQUE * 3
  #define PID_INTERVAL_POSITION PID_INTERVAL_SPEED * 2
  // Position
    unsigned long PID_time_pos = 0;
    PIDLib_MeasurementType Meas_position{};
    PIDLib_PIDSetupType PIDSetup_position{};
    PIDLib_PID PID_position{};
    #define GAIN_POS_P 1.0
    #define GAIN_POS_I 0.0
    #define GAIN_POS_D 0.0
    float PID_Out_pos = 0.0;

  // Speed
    unsigned long PID_time_speed = 0;
    PIDLib_MeasurementType Meas_speed{};
    PIDLib_PIDSetupType PIDSetup_speed{};
    PIDLib_PID PID_speed{};
    #define GAIN_SPEED_P 1.0
    #define GAIN_SPEED_I 0.0
    #define GAIN_SPEED_D 0.0
    float PID_Out_speed = 0.0;

  // Torque/Current
    unsigned long PID_time_torque = 0;
    PIDLib_MeasurementType Meas_torque{};
    PIDLib_PIDSetupType PIDSetup_torque{};
    PIDLib_PID PID_torque{};
    #define GAIN_TORQUE_P 1.0
    #define GAIN_TORQUE_I 0.0
    #define GAIN_TORQUE_D 0.0
    float PID_Out_torque = 0.0;
  
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

/*
OutputAngle AngleInput(OutputAngle input_angle) {
  // Get encoder position
  int enc_position = encoder.getCount();
  //unsigned long encoder_time = micros();
  
  // Calculate Angle @ output
  double calc_output_angle = (double)enc_position / (ENCODER_CPR * MOTOR_GEARING); // [%rotation]
  calc_output_angle = 360.0 * calc_output_angle; // [deg]

  // Calculate rotation speed @output
  output_speed = (output_angle - output_angle_last) / (double)(encoder_time - enc_last_time); // [deg/us]
  output_speed = output_speed * 166666.6666666667; // Convert to [rpm]

  // Collect previous values
  output_angle_last = output_angle;
  enc_last_time = encoder_time;

  // Set output class
  OutputAngle output_angle;
  output_angle.raw = enc_position;
  output_angle.angle = calc_output_angle;

  return output_angle;
}
*/

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
void RunMotor(int speed, bool brake=true) {
  //Serial.print("Motor speed "); Serial.println(speed);
  if (speed > MOTOR_OUTPUT_MIN) {
    if (brake) {
      digitalWrite(MOTOR_PIN_A, HIGH); 
    } else {
      digitalWrite(MOTOR_PIN_A, LOW); 
    }
    analogWrite(MOTOR_PIN_B, 255-speed);
  } else if (speed < MOTOR_OUTPUT_MIN) {
    if (brake) {
      digitalWrite(MOTOR_PIN_B, HIGH);
    } else {
      digitalWrite(MOTOR_PIN_B, LOW);
    }
    analogWrite(MOTOR_PIN_A, 255+speed);
  } else {
    if (brake) {
      digitalWrite(MOTOR_PIN_A, HIGH);
      digitalWrite(MOTOR_PIN_B, HIGH);
    } else {
      digitalWrite(MOTOR_PIN_A, LOW);
      digitalWrite(MOTOR_PIN_B, LOW);
    }
  }
  
  return;
}
*/

void DoMotor() {
  // interpolate speed from Start/End
  //Serial.print("Running DoMotor,");
  unsigned long cur_time = micros();
  double speed_double = speed_end;
  if (cur_time > loop_speed_end) {
    speed_cur = speed_end;
  } else {
    speed_double = ((float)(cur_time - loop_speed_start))*speed_calc_slope + (float)speed_start;
    //speed_cur = (cur_time - loop_speed_start)*((speed_end - speed_start)/(loop_speed_end - loop_speed_start)) + speed_start;
    speed_cur = speed_double;
  }
  
  if (speed_cur > SPEED_MAX){
    speed_cur = SPEED_MAX;
    //Serial.println("Speed out of bounds");
  } else if (speed_cur < -SPEED_MAX) {
    speed_cur = -SPEED_MAX;
    //Serial.println("Speed out of bounds");
  }
  RunMotor_RP2040(speed_cur);
  //Serial.print(" complete. New speed (double, float): ");
  //  Serial.print(speed_double);
  //  Serial.print(", ");
  //  Serial.print(speed_cur);
  //  Serial.println();
}

void MotorSetpoint(float speed, int duration) {
  Serial.print("Received new setpoint, speed, duration: ");
    Serial.print(speed);
    Serial.print(", ");
    Serial.println(duration);

  speed_start = speed_cur;
  speed_end = speed;
  loop_speed_start = micros();
  loop_speed_end   = loop_speed_start + duration;
  speed_calc_slope = (speed_end - speed_start) / (float)(loop_speed_end - loop_speed_start);
  Serial.print("Calc'd Final Speed: ");
    Serial.println(((speed_end - speed_start)) * speed_calc_slope + speed_start);
}

/*
void CurrentSense() {
  current_mA = INA.getCurrent_mA();
  //shunt_mV = INA.getShuntVoltage_mV();
}
*/

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

void Current_ACS712() {
  int read = 0;
  for (int i=0;i<ACS712_READINGS;i++) {
    read += analogRead(ACS712_PIN);
  }
  ACS712_raw = read / ACS712_READINGS;
  int adjusted_raw = ACS712_raw - ACS712_midpoint;
  ACS712_calc = (float)adjusted_raw * ACS712_CONVERSION;

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

void Display_Info() {
  if (micros() - loop_display_last >= DISPLAY_WAIT_TIME || display_force) {
    loop_display_last = micros();

    //unsigned long perf_current = micros();
    //CurrentSense();
    //perf_current = micros() - perf_current;

    unsigned long perf_ACS = micros();
    Current_ACS712();
    perf_ACS = micros() - perf_ACS;

    unsigned long perf_Bus = micros();
    MotorVoltage mvoltage;
    mvoltage = Motor_Voltage();
    perf_Bus = micros() - perf_Bus;
    
    //unsigned long perf_output = micros();
    //angle_output = AngleOutput(angle_output);
    //perf_output = micros() - perf_output;

    if (header_display >= HEADER_COUNT) {
      Serial.println("Time [us]|PWM|ACS712 Raw|Current ACS712 [A]|Perf ACS712 [us]|Bus Raw A|Bus Raw B|Bus Voltage|Bus Offset|Perf Bus[us]");
      header_display = 0;
    } else {
      header_display++;
    }

    Serial.print(loop_display_last);
      Serial.print("|");
      Serial.print(speed_cur, 3);
      Serial.print("|");
      Serial.print(ACS712_raw);
      Serial.print("|");
      Serial.print(ACS712_calc, 3);
      Serial.print("|");
      Serial.print(perf_ACS);
      Serial.print("|");
      Serial.print(mvoltage.Bus_A_conv);
      Serial.print("|");
      Serial.print(mvoltage.Bus_B_conv);
      Serial.print("|");
      Serial.print(mvoltage.Bus_voltage);
      Serial.print("|");
      Serial.print(mvoltage.Bus_offset);
      Serial.print("|");
      Serial.print(perf_Bus);
      Serial.println();

    display_force = false;
    motor_loop = 0;
    loop_display_last = loop_display_last - loop_display_last%DISPLAY_WAIT_TIME;
  }
}

void setup() {
  Serial.begin(115200);
  //while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
  //    yield();
  //}
  delay(5000); // Wait 5 seconds to start serial output
  Serial.println("Starting Sketch");

  // Setup RP2040 PWM
    PWM_Instance[0] = new RP2040_PWM(MOTOR_PIN_A, frequency, dutyCycle_A);
    PWM_Instance[1] = new RP2040_PWM(MOTOR_PIN_B, frequency, dutyCycle_B);

  // ACS712 Setup
    analogReadResolution(12);
    ACS712_Calibrate_Midpoint();
    Serial.print("New ACS712 Midpoint: "); Serial.println(ACS712_midpoint);

  // Setup I2C Wire
  Wire.setSCL(D5);
  Wire.setSDA(D4);
  Wire.setClock(400000);
  Wire.begin();
  /*
  // Current Sensor
    if (!INA.begin())  {
      Serial.println("INA219 could not connect. Fix and Reboot");
    } else {
      INA.setMaxCurrentShunt(3,0.1);
      INA.setBusVoltageRange(16);
      INA.setGain(8);
      INA.setBusResolution(12);
      INA.setShuntResolution(12);
      INA.setShuntSamples(1);
      INA.setBusSamples(1);
      INA.setModeShuntContinuous();

      Serial.print("CALI:\t");
      Serial.println(INA.isCalibrated());
      Serial.print("CLSB:\t");
      Serial.println(INA.getCurrentLSB(),5);
      Serial.print("SHUNT:\t");
      Serial.println(INA.getShunt(), 4);
      Serial.print("MAXC:\t");
      Serial.println(INA.getMaxCurrent(), 4);
      Serial.print("Shunt ADC:\t");
      Serial.println(INA.getShuntADC());
      
    }
    */

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

  display_force = true;

  // PID Setup
    // Position
      PIDSetup_position.gainP = GAIN_POS_P;
      PIDSetup_position.gainI = GAIN_POS_I;
      PIDSetup_position.gainD = GAIN_POS_D;
      PIDSetup_position.outputLowerLimit = -MOTOR_PARAM_MAX_SPEED;
      PIDSetup_position.outputUpperLimit =  MOTOR_PARAM_MAX_SPEED;
    
    // Speed
      PIDSetup_speed.gainP = GAIN_SPEED_P;
      PIDSetup_speed.gainI = GAIN_SPEED_I;
      PIDSetup_speed.gainD = GAIN_SPEED_D;
      PIDSetup_speed.outputLowerLimit =  -MOTOR_PARAM_STALL_TORQUE;
      PIDSetup_speed.outputUpperLimit =   MOTOR_PARAM_STALL_TORQUE;

    // Torque
      PIDSetup_torque.gainP = GAIN_TORQUE_P;
      PIDSetup_torque.gainI = GAIN_TORQUE_I;
      PIDSetup_torque.gainD = GAIN_TORQUE_D;
      PIDSetup_torque.outputLowerLimit = -100.0; // Inverse PWM
      PIDSetup_torque.outputUpperLimit =  100.0; // Max PWM
}

void loop() {

  /*
  if (micros() >= loop_speed_next ) {
    //Serial.println("Calculating new Setpoint");
    if (speed_go) {
      // Motor Running, Stop Motor
      unsigned long setpoint_rampup = random(SPEED_TIME_MIN,SPEED_TIME_MAX);
      MotorSetpoint(0, setpoint_rampup);
      loop_speed_next = micros() + setpoint_rampup + SPEED_RUN_STOP;
      speed_go = false;
    } else {
      // Motor Stopped, get going!
      unsigned long setpoint_rampup = random(SPEED_TIME_MIN,SPEED_TIME_MAX);
      if (!speed_dir) {
        MotorSetpoint(SPEED_MAX, setpoint_rampup);
        speed_dir = true;
      } else {
        MotorSetpoint(-SPEED_MAX, setpoint_rampup);
        speed_dir = false;
      }
      loop_speed_next = micros() + setpoint_rampup + SPEED_RUN_GO;
      speed_go = true;
    }
  }
  */

  // Check Position PID Time
  if (micros() - PID_time_pos >= PID_INTERVAL_POSITION) {
    // CHANGE THIS SET POINT ALGORITHM
    float position_setpoint = random(-360, 360);
    PID_position.setpoint(position_setpoint);

    PID_time_pos = micros();
    // Collect measurement
    angle_output = AngleOutput(angle_output);
    Meas_position.time_usec = angle_output.angle_time;
    Meas_position.value = angle_output.angle;

    // compute speed setpoint
    PID_Out_pos = PID_position.run(Meas_position);
    PID_speed.setpoint(PID_Out_pos);

  }

  // Check Speed PID Time
  if (micros() - PID_time_speed >= PID_INTERVAL_SPEED) {
    // check measurement time (s/b w/in the speed interval)
    if (micros() - angle_output.angle_time > PID_INTERVAL_SPEED / 2) {
      angle_output = AngleOutput(angle_output);
    }
    Meas_speed.time_usec = angle_output.angle_time;
    Meas_speed.value = angle_output.output_speed;

    // compute torque setpoint
    PID_Out_speed = PID_position.run(Meas_speed);
    PID_torque.setpoint(PID_Out_speed);
  }
  
  // Check Torque PID Time
  if (micros() - PID_time_torque >= PID_INTERVAL_TORQUE) {
    // collect measurement
    Current_ACS712();
    Meas_torque.time_usec = micros();
    Meas_torque.value = ACS712_calc;

    // compute PWM setpoint
    PID_Out_torque = PID_position.run(Meas_torque);

    // Run Motor
    RunMotor_RP2040(PID_Out_torque);
    
  }

  //DoMotor();
  //motor_loop++;

  //Display_Info(); // Check this does what you are trying to do before re-enabling
}