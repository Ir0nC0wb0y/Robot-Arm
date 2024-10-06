#include <Arduino.h>
#include <pio_encoder.h>
#include <ArduPID.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <AS5600.h>


// Encoder (Input)
  #define ENCODER_PIN_A D2
  #define ENCODER_PIN_B D3
  PioEncoder encoder(D2); // Encoder on Pins 1/2 (must be sequential)
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

// Motor
  #define MOTOR_PIN_A D9
  #define MOTOR_PIN_B D8
  #define MOTOR_OUTPUT_MIN 0
  #define ENCODER_CPR 48.0
  #define MOTOR_GEARING 264

// Current Sensor
  Adafruit_INA219 ina219(0x45);
  //float shuntvoltage = 0;
  //float busvoltage = 0;
  float current_mA = 0;
  //float loadvoltage = 0;
  //float power_mW = 0;


// Loop
  // Display
  unsigned long loop_display_last = 0;
  #define DISPLAY_WAIT_TIME  2000 //Output angle and current, [us]
  int header_display = 0;
  #define HEADER_COUNT 200
  bool display_force = false;
  int motor_loop = 0;

  // Speed
  int speed_start = 0;
  int speed_end   = 0;
  int speed_cur = 0;
  double speed_calc_slope = 0.0;
  #define SPEED_MAX 255
  //int speed_temp = 0;
  //#define SPEED_CHANGE 5
  bool speed_dir = true;
  bool speed_go = false;
  bool speed_reached = false;
  //#define SPEED_STEP           1
  //#define SPEED_RUN_STEP    1000
  #define SPEED_RUN_GO   5000000
  #define SPEED_RUN_STOP  250000
  #define SPEED_TIME_MIN  100000
  #define SPEED_TIME_MAX 1000000
  unsigned long loop_speed_next  = 0;
  unsigned long loop_speed_start = 0;
  unsigned long loop_speed_end   = 0;
  
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

OutputAngle AngleInput(OutputAngle input_angle) {
  // Get encoder position
  int enc_position = encoder.getCount();
  //unsigned long encoder_time = micros();
  
  // Calculate Angle @ output
  double calc_output_angle = (double)enc_position / (ENCODER_CPR * MOTOR_GEARING); // [%rotation]
  calc_output_angle = 360.0 * calc_output_angle; // [deg]

  /*
  // Calculate rotation speed @output
  output_speed = (output_angle - output_angle_last) / (double)(encoder_time - enc_last_time); // [deg/us]
  output_speed = output_speed * 166666.6666666667; // Convert to [rpm]

  // Collect previous values
  output_angle_last = output_angle;
  enc_last_time = encoder_time;
  */

  // Set output class
  OutputAngle output_angle;
  output_angle.raw = enc_position;
  output_angle.angle = calc_output_angle;

  return output_angle;
}

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

void DoMotor() {
  /*// Change Speed Set Point
  if (micros() >= loop_speed_next && speed_reached) {
    if (speed_go) {
      // Stop
      speed_go = false;
      speed_set = 0;
      //loop_speed_next = micros() + SPEED_RUN_STOP;
      display_force = true;
    } else {
      // Go new speed
      speed_go = true;
      if (speed_dir) {
        speed_temp = speed_temp + SPEED_CHANGE;
        if (speed_temp > 255) {
          speed_temp = -SPEED_CHANGE;
          speed_dir = false;
        }
      } else {
        speed_temp = speed_temp - SPEED_CHANGE;
        if (speed_temp < -255) {
          speed_temp = SPEED_CHANGE;
          speed_dir = true;
        }
      }
      speed_set = speed_temp;
      //loop_speed_next = micros() + SPEED_RUN_GO;
      display_force = true;
    }
    speed_reached = false;
  }*/

  /*// Ramp speed to Set Point
  if (!speed_reached) {
    //Serial.print("1 Speed not reached: "); Serial.println(!speed_reached);
    if (speed_go) {
      //Serial.print("1.1 Speed Forward: "); Serial.println(speed_go);
      if (micros() >= loop_speed_step) {
        //Serial.println("1.1.1 Speed Step Timing");
        if (abs(speed_cur) < abs(speed_set)) {
          //Serial.println("1.1.1.1 Change Speed");
          if (speed_dir) {
            speed_cur = speed_cur + SPEED_STEP;
          } else {
            speed_cur = speed_cur - SPEED_STEP;
          }
          RunMotor(speed_cur);
          loop_speed_step = micros() + SPEED_RUN_STEP;
        } else {
          //Serial.println("1.1.1.2 Speed Reached");
          loop_speed_next = micros() + SPEED_RUN_GO;
          speed_reached = true;
        }
      }
    } else {
      //Serial.print("1.2 Reverse: "); Serial.println(!speed_go);
      if (micros() >= loop_speed_step) {
        //Serial.println("1.2.1 Speed Step");
        if (abs(speed_cur) > 0) {
          //Serial.println("1.2.1.1 Change Speed");
          if (speed_dir) {
            speed_cur = speed_cur - SPEED_STEP;
          } else {
            speed_cur = speed_cur + SPEED_STEP;
          }
          RunMotor(speed_cur);
          loop_speed_step = micros() + SPEED_RUN_STEP;
        } else {
          //Serial.println("1.2.1.2 Speed Reached");
          loop_speed_next = micros() + SPEED_RUN_STOP;
          speed_reached = true;
        }
      }
    }
  }*/

  // interpolate speed from Start/End
  //Serial.print("Running DoMotor,");
  unsigned long cur_time = micros();
  double speed_double = speed_end;
  if (cur_time > loop_speed_end) {
    speed_cur = speed_end;
  } else {
    speed_double = ((float)(cur_time - loop_speed_start))*speed_calc_slope + (float)speed_start;
    //speed_cur = (cur_time - loop_speed_start)*((speed_end - speed_start)/(loop_speed_end - loop_speed_start)) + speed_start;
    speed_cur = (int)speed_double;
  }
  
  if (speed_cur > 255){
    speed_cur = 255;
    //Serial.println("Speed out of bounds");
  } else if (speed_cur < -255) {
    speed_cur = -255;
    //Serial.println("Speed out of bounds");
  }
  RunMotor(speed_cur);
  //Serial.print(" complete. New speed (double, float): ");
  //  Serial.print(speed_double);
  //  Serial.print(", ");
  //  Serial.print(speed_cur);
  //  Serial.println();
}

void MotorSetpoint(int speed, int duration) {
  Serial.print("Received new setpoint, speed, duration: ");
    Serial.print(speed);
    Serial.print(", ");
    Serial.println(duration);

  speed_start = speed_cur;
  speed_end = speed;
  loop_speed_start = micros();
  loop_speed_end   = loop_speed_start + duration;
  speed_calc_slope = (float)(speed_end - speed_start) / (float)(loop_speed_end - loop_speed_start);
  Serial.print("Calc'd Final Speed: ");
    Serial.println(((float)(speed_end - speed_start)) * speed_calc_slope + (float)speed_start);
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
  //while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
  //    yield();
  //}
  delay(10000); // Wait 10 seconds to start serial output
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

  // Setup I2C Wire
  Wire.setSCL(D5);
  Wire.setSDA(D4);
  Wire.begin();
  // Current Sensor
    if (! ina219.begin()) {
      Serial.println("Failed to find INA219 chip");
      while (1) { delay(10); }
    } else {
      Serial.println("INA219 Connected!");
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
      Serial.println("Time [us]|PWM|Current[mA]|Read Angle[count]|getCumPosition[deg]|Perf Output Angle[us]|Perf Current [us]");
      header_display = 0;
    } else {
      header_display++;
    }

    Serial.print(micros());
      Serial.print("|");
      Serial.print(speed_cur);
      //Serial.print("|");
      //Serial.print(angle_before.encoder_position);
      //Serial.print("|");
      //Serial.print(angle_before.output_angle);
      Serial.print("|");
      Serial.print(current_mA);
      //Serial.print("|");
      //Serial.print(angle_after.encoder_position);
      //Serial.print("|");
      //Serial.print(angle_after.output_angle);
      Serial.print("|");
      Serial.print(angle_output.raw);
      Serial.print("|");
      Serial.print(angle_output.angle);
      //Serial.print("|");
      //Serial.print(motor_loop);
      Serial.print("|");
      Serial.print(perf_output);
      Serial.print("|");
      Serial.print(perf_current);
      Serial.println();

    display_force = false;
    motor_loop = 0;
  }
 
}