#include <util/atomic.h>

// Pin definitions
#define ENCODER_A 2      // Encoder Channel A (Interrupt)
#define ENCODER_B 3      // Encoder Channel B
#define MOTOR_PWM 5      // ENA pin of L298N
#define MOTOR_IN1 6      // IN1 pin of L298N
#define MOTOR_IN2 7      // IN2 pin of L298N

// Constants
const int PULSES_PER_REVOLUTION = 500;  // Encoder pulses per revolution
const unsigned long DEBOUNCE_MS = 2;    // Debounce time in milliseconds
const unsigned long PRINT_INTERVAL_MS = 100;  // Serial print interval

// Motor direction constants
const int DIRECTION_CW = 1;
const int DIRECTION_CCW = -1;
const int DIRECTION_STOP = 0;

// Global variables
volatile int encoder_position = 0;
volatile unsigned long last_encoder_time = 0;
volatile bool new_encoder_pulse = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Configure pins
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  // Set up interrupt for encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, RISING);
  
  // Start motor: clockwise, 150/255 PWM
  setMotor(DIRECTION_CW, 150);
}

void loop() {
  static unsigned long last_print_time = 0;
  static int last_position = 0;
  
  // Check for new encoder data
  if (new_encoder_pulse) {
    int current_position;
    unsigned long current_time;
    
    // Atomic block to safely read volatile variables
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      current_position = encoder_position;
      current_time = last_encoder_time;
      new_encoder_pulse = false;
    }
    
    // Calculate speed only if enough time has passed
    unsigned long current_millis = millis();
    if (current_millis - last_print_time >= PRINT_INTERVAL_MS) {
      float delta_time_ms = current_time - last_print_time;
      float delta_time_sec = delta_time_ms / 1000.0;
      
      // Calculate RPM (avoid division by zero)
      float speed_rps = delta_time_sec > 0 ? 
                       (float)(current_position - last_position) / PULSES_PER_REVOLUTION / delta_time_sec : 
                       0.0;
      float speed_rpm = speed_rps * 60.0;
      
      // Print status
      Serial.print("Position: ");
      Serial.print(current_position);
      Serial.print(" pulses | Speed: ");
      Serial.print(speed_rpm, 2);
      Serial.println(" RPM");
      
      last_print_time = current_millis;
      last_position = current_position;
    }
  }
}

// Set motor direction and speed
void setMotor(int direction, int pwm_value) {
  // Validate PWM value
  pwm_value = constrain(pwm_value, 0, 255);
  
  // Set PWM
  analogWrite(MOTOR_PWM, pwm_value);
  
  // Set direction
  switch (direction) {
    case DIRECTION_CW:
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
      break;
    case DIRECTION_CCW:
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, HIGH);
      break;
    default:
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, LOW);
      break;
  }
}

// Interrupt handler for encoder
void readEncoder() {
  static unsigned long last_interrupt_time = 0;
  unsigned long current_time = millis();
  
  // Debounce encoder
  if (current_time - last_interrupt_time >= DEBOUNCE_MS) {
    // Read encoder B to determine direction
    if (digitalRead(ENCODER_B) == LOW) {
      encoder_position++;  // Clockwise
    } else {
      encoder_position--;  // Counter-clockwise
    }
    
    last_encoder_time = current_time;
    new_encoder_pulse = true;
    last_interrupt_time = current_time;
  }
}
