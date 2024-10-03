#include <Arduino.h>
#include <pio_encoder.h>
#include <ArduPID.h>
#include <Wire.h>
#include <Adafruit_INA219.h>


// Encoder
  #define ENCODER_PIN_A D2
  #define ENCODER_PIN_B D3
  PioEncoder encoder(D2); // Encoder on Pins 1/2 (must be sequential)
  int enc_position = 0;
  double output_angle = 0.0;
  double output_speed = 0;
  double output_angle_last = 0;
  unsigned long enc_last_time = 0;

// Motor
  #define MOTOR_PIN_A D9
  #define MOTOR_PIN_B D8
  #define MOTOR_OUTPUT_MIN 0
  #define ENCODER_CPR 48.0
  #define MOTOR_GEARING 264

// Current Sensor
  Adafruit_INA219 ina219;
  //float shuntvoltage = 0;
  //float busvoltage = 0;
  float current_mA = 0;
  //float loadvoltage = 0;
  //float power_mW = 0;


// Loop
  // Display
  unsigned long loop_display_last = 0;
  #define DISPLAY_WAIT_TIME 100
  int header_display = 0;
  #define HEADER_COUNT 200
  bool display_force = false;

  // Speed
  int speed_start = 0;
  int speed_end   = 0;
  int speed_cur = 0;
  double speed_calc_slope = 0.0;
  //int speed_temp = 0;
  //#define SPEED_CHANGE 5
  bool speed_dir = true;
  bool speed_go = false;
  bool speed_reached = false;
  //#define SPEED_STEP           1
  //#define SPEED_RUN_STEP    1000
  #define SPEED_RUN_GO   5000000
  #define SPEED_RUN_STOP  100000
  #define SPEED_TIME_MIN    5000
  #define SPEED_TIME_MAX  300000
  unsigned long loop_speed_next  = 0;
  unsigned long loop_speed_start = 0;
  unsigned long loop_speed_end   = 0;
  
    

void AngleCalcs() {
    // Get encoder position
    enc_position = encoder.getCount();
    //unsigned long encoder_time = micros();
    
    // Calculate Angle @ output
    output_angle = (double)enc_position / (ENCODER_CPR * MOTOR_GEARING); // [%rotation]
    output_angle = 360.0 * output_angle; // [deg]

    /*
    // Calculate rotation speed @output
    output_speed = (output_angle - output_angle_last) / (double)(encoder_time - enc_last_time); // [deg/us]
    output_speed = output_speed * 166666.6666666667; // Convert to [rpm]

    // Collect previous values
    output_angle_last = output_angle;
    enc_last_time = encoder_time;
    */
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

  //loop_speed_next = micros() + 1000000; // Wait 10 seconds before moving motor
}

void loop() {

  if (micros() >= loop_speed_next ) {
    //Serial.println("Calculating new Setpoint");
    if (speed_go) {
      // Motor Running, Stop Motor
      //Serial.println("Motor Running, Stop Motor");
      unsigned long setpoint_rampup = random(SPEED_TIME_MIN,SPEED_TIME_MAX);
      MotorSetpoint(0, setpoint_rampup);
      loop_speed_next = micros() + setpoint_rampup + SPEED_RUN_STOP;
      speed_go = false;
    } else {
      // Motor Stopped, get going!
      //Serial.println("Motor Stopped, get going!");
      unsigned long setpoint_rampup = random(SPEED_TIME_MIN,SPEED_TIME_MAX);
      MotorSetpoint(255, setpoint_rampup);
      loop_speed_next = micros() + setpoint_rampup + SPEED_RUN_GO;
      speed_go = true;
    }
  }

  DoMotor();

      
  if (micros() - loop_display_last >= DISPLAY_WAIT_TIME || display_force) {
    unsigned long perf_angle = micros();
    AngleCalcs();
    perf_angle = micros() - perf_angle;
    unsigned long perf_current = micros();
    CurrentSense();
    perf_current = micros() - perf_current;

    if (header_display >= HEADER_COUNT) {
      Serial.println("Time [us]|PWM|Encoder Counts|Angle|Current[mA]|Perf Angle [us]|Perf Current[us]");
      header_display = 0;
    } else {
      header_display++;
    }

    Serial.print(micros());
      Serial.print("|");
      Serial.print(speed_cur);
      Serial.print("|");
      Serial.print(enc_position);
      Serial.print("|");
      Serial.print(output_angle);
      Serial.print("|");
      Serial.print(current_mA);
      Serial.print("|");
      Serial.print(perf_angle);
      Serial.print("|");
      Serial.print(perf_current);
      Serial.println();
      
    
    loop_display_last = micros();
    display_force = false;
  }
 
}