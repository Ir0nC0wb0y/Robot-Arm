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
  #define MOTOR_GEARING 9.6

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
  int speed_set = 0;
  int speed_cur = 0;
  int speed_temp = 0;
  #define SPEED_CHANGE 5
  bool speed_dir = true;
  bool speed_go = false;
  #define SPEED_STEP           1
  #define SPEED_RUN_STEP    1000
  #define SPEED_RUN_GO    500000
  #define SPEED_RUN_STOP  500000
  unsigned long loop_speed_next = 0;
  unsigned long loop_speed_step = 0;
  
    

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
  // Ramp speed to Set Point
  if (speed_go) {
    if (micros() >= loop_speed_step) {
      if (abs(speed_cur) < abs(speed_set)) {
        if (speed_dir) {
          speed_cur = speed_cur + SPEED_STEP;
        } else {
          speed_cur = speed_cur - SPEED_STEP;
        }
        RunMotor(speed_cur);
        loop_speed_step = micros() + SPEED_RUN_STEP;
      }
    }
  }

  // Change Speed Set Point
  if (micros() >= loop_speed_next) {
    if (speed_go) {
      // Stop
      speed_go = false;
      speed_set = 0;
      speed_cur = 0;
      //speed_cur = speed_set;
      RunMotor(speed_cur);
      loop_speed_next = micros() + SPEED_RUN_STOP;
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
      loop_speed_next = micros() + SPEED_RUN_GO;
      display_force = true;
    }
  }
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

  loop_speed_next = micros() + 1000000; // Wait 1second before moving motor
}

void loop() {

  DoMotor();

      
  if (micros() - loop_display_last >= DISPLAY_WAIT_TIME || display_force) {
    AngleCalcs();
    CurrentSense();

    if (header_display >= HEADER_COUNT) {
      Serial.println("Time [us]|PWM|Encoder Counts|Angle|Current[mA]");
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
      Serial.println();
      
    
    loop_display_last = micros();
    display_force = false;
  }
 
}