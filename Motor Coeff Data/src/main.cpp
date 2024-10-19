#include <Arduino.h>
//#include <pio_encoder.h>
//#include <ArduPID.h>
#include <Wire.h>
#include <INA219.h>
#include <AS5600.h>
#include <RP2040_PWM.h>


// Encoder (Input)
  #define ENCODER_PIN_A D2
  #define ENCODER_PIN_B D3
  //PioEncoder encoder(D2); // Encoder on Pins 1/2 (must be sequential)
  class OutputAngle {
    public:
      int raw = 0;
      double angle = 0.0;
      //double output_speed = 0.0;
      //double output_angle_last = 0.0;
      //unsigned long last_time = 0;
  };
  OutputAngle angle_before;
  OutputAngle angle_after;
// Encoder (Output)
  #define ENCODER_PIN_DIR D0
  #define ENCODER_OFFSET 0.0
  AS5600L as5600;
  OutputAngle angle_output;
  #define PRINT_ENCODER_INFO false

// Motor
  #define MOTOR_PIN_A D9
  #define MOTOR_PIN_B D8
  #define MOTOR_OUTPUT_MIN 0
  #define ENCODER_CPR 48.0
  #define MOTOR_GEARING 264

// Current Sensor
  #define INA219_ADDRESS 0x45
  INA219 INA(INA219_ADDRESS);
  float current_mA = 0.0;
  //float shunt_mV = 0.0;

// RP2040 PWM
  float frequency = 25000.0; // Frequency, in Hz
  float dutyCycle_A = 0.0;
  float dutyCycle_B = 0.0;
  RP2040_PWM* PWM_Instance[2];

// Loop
  // Display
  unsigned long loop_display_last = 0;
  #define DISPLAY_WAIT_TIME  5000 //Output angle and current, [us]
  #define HEADER_COUNT 200
  int header_display = HEADER_COUNT;
  bool display_force = false;
  int motor_loop = 0;

  // Speed
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
  #define SPEED_RUN_STOP   1000000
  #define SPEED_TIME_MIN    100000
  #define SPEED_TIME_MAX  10000000
  unsigned long loop_speed_next  = 0;
  unsigned long loop_speed_start = 0;
  unsigned long loop_speed_end   = 0;
  int freq_tries = 0;
  #define FREQ_TIMES 2
  
OutputAngle AngleOutput(OutputAngle input_angle) {
  int32_t enc_position = as5600.getCumulativePosition();

  // Calculate Angle @ output
  double calc_output_angle = (double)enc_position / 4096.0; // [%rotation]
  calc_output_angle = 360.0 * calc_output_angle; // [deg]
  //double calc_output_angle = as5600.getCumulativePosition();

  // Set output class
  OutputAngle output_angle;
  output_angle.raw = enc_position;
  output_angle.angle = calc_output_angle;

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


void CurrentSense() {
  current_mA = INA.getCurrent_mA();
  //shunt_mV = INA.getShuntVoltage_mV();
}


void setup() {
  Serial.begin(115200);
  //while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
  //    yield();
  //}
  delay(10000); // Wait 10 seconds to start serial output
  Serial.println("Starting Sketch");
  
  /*
  // Encoder 
    Serial.println("Starting Encoder");
    pinMode(ENCODER_PIN_A, INPUT);
    pinMode(ENCODER_PIN_B, INPUT);
    encoder.begin();
  */

  // Motor
    //pinMode(MOTOR_PIN_A, OUTPUT);
    //pinMode(MOTOR_PIN_B, OUTPUT);
    //digitalWrite(MOTOR_PIN_A, LOW);
    //digitalWrite(MOTOR_PIN_B, LOW);
  
  // Setup RP2040 PWM
    PWM_Instance[0] = new RP2040_PWM(MOTOR_PIN_A, frequency, dutyCycle_A);
    PWM_Instance[1] = new RP2040_PWM(MOTOR_PIN_B, frequency, dutyCycle_B);

  // Setup I2C Wire
  Wire.setSCL(D5);
  Wire.setSDA(D4);
  Wire.setClock(400000);
  Wire.begin();
  // Current Sensor
    if (!INA.begin())  {
      Serial.println("INA219 could not connect. Fix and Reboot");
    } else {
      INA.setMaxCurrentShunt(3,0.1);
      INA.setBusVoltageRange(16);
      INA.setGain(8);
      INA.setBusResolution(10);
      INA.setShuntResolution(10);
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
      }
      delay(3000);
    }

  display_force = true;

}

void loop() {

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

  DoMotor();
  motor_loop++;

      
  if (micros() - loop_display_last >= DISPLAY_WAIT_TIME || display_force) {
    loop_display_last = micros();
    loop_display_last = loop_display_last - loop_display_last%DISPLAY_WAIT_TIME;
    //angle_before = AngleInput(angle_before);
    unsigned long perf_current = micros();
    CurrentSense();
    perf_current = micros() - perf_current;
    //angle_after = AngleInput(angle_after);
    unsigned long perf_output = micros();
    angle_output = AngleOutput(angle_output);
    perf_output = micros() - perf_output;

    if (header_display >= HEADER_COUNT) {
      Serial.println("Time [us]|PWM|Frequency|Current [mA]|Perf Current [us]|Angle [deg]|Perf Angle [us]");
      header_display = 0;
    } else {
      header_display++;
    }

    Serial.print(micros());
      Serial.print("|");
      Serial.print(speed_cur, 3);
      Serial.print("|");
      Serial.print(frequency,0);
      Serial.print("|");
      Serial.print(current_mA,0);
      Serial.print("|");
      Serial.print(perf_current);
      Serial.print("|");
      Serial.print(angle_output.angle);
      Serial.print("|");
      Serial.print(perf_output);
      Serial.println();

    display_force = false;
    motor_loop = 0;
  }
 
}