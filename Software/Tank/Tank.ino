#include <PS2X_lib.h>
#include <Servo.h>

#define PS2_DAT_PIN       51 // MOSI   
#define PS2_CMD_PIN       50 // MISO
#define PS2_SEL_PIN       53 
#define PS2_CLK_PIN       52
#define PS2_PRESSURE      false
#define PS2_RUMBLE        false

#define FAN_PIN 2
#define ARM_PIN 3

#define LEFT_ROLLER_FIRE_PIN 4
#define LEFT_ROLLER_LOAD_PIN 5
#define RIGHT_ROLLER_FIRE_PIN 6
#define RIGHT_ROLLER_LOAD_PIN 7

#define LOADER_PUSH_PIN 8
#define LOADER_PULL_PIN 9

#define LEFT_WHEEL_FORWARD_PIN 10
#define LEFT_WHEEL_BACKWARD_PIN 11
#define RIGHT_WHEEL_FORWARD_PIN 12
#define RIGHT_WHEEL_BACKWARD_PIN 13

#define BUZZER_PIN 48
#define INDICATOR_LED_PIN 49

#define SPREAD_POSITION 0
#define FOLD_POSITION 105

#define LEFT_ROLLER_FIRE_RATE 254
#define LEFT_ROLLER_LOAD_RATE 100

#define RIGHT_ROLLER_FIRE_RATE 254
#define RIGHT_ROLLER_LOAD_RATE 100

#define FAN_RATE 254
#define LOADER_MOVE_RATE 254

#define STICK_THRESHOLD 50

#define OFF 0

PS2X ps2x;
Servo arm;

int error = 0;
byte type = 0;

int LY = 0;
int RX = 0;

int mappedLY = 0;
int mappedRX = 0;

int leftWheelSpeed = 0;
int rightWheelSpeed = 0;

void configPinout();
void configController();
void buzz(int repeat, int duration);
void move(unsigned int leftWheelPWM, unsigned int rightWheelPWM);
void push();
void pull();
void fire();
void load();
void turnOnFan();
void turnOffFan();
void turnOffLoader();
void turnOffRollers();

void setup() {
  Serial.begin(115200); 
  configPinout();
  configController();
}

void loop() {
  if(error == 1)    // Skip loop if no controller found
    return; 
  
  if(type == 2)   // Also skip if controller is Guitar Hero
    return;

  else {    // DualShock Controller found
    ps2x.read_gamepad(false, false);  // read controller

    if(ps2x.Button(PSB_SELECT)) {
      Serial.println("STAND BY");
      turnOffFan();
      turnOffRollers();
      turnOffLoader();
      arm.write(FOLD_POSITION);
    }

    if(ps2x.Button(PSB_START)) {
      Serial.println("START");
      load();
      turnOnFan();
      arm.write(SPREAD_POSITION);
    }

    LY = ps2x.Analog(PSS_LY);
    RX = ps2x.Analog(PSS_RX);
    
    mappedLY = map(LY, 0, 255, 254, -254);
    mappedRX = map(RX, 0, 255, -254, 254);

    if (abs(mappedLY) < STICK_THRESHOLD)
      mappedLY = 0;
    if (abs(mappedRX) < STICK_THRESHOLD)
      mappedRX = 0;

    leftWheelSpeed = mappedLY + mappedRX;
    rightWheelSpeed = mappedLY - mappedRX;

    leftWheelSpeed = constrain(leftWheelSpeed, -254, 254);
    rightWheelSpeed = constrain(rightWheelSpeed, -254, 254);

    Serial.print(leftWheelSpeed);
    Serial.print(' ');
    Serial.println(rightWheelSpeed);
    
    move(leftWheelSpeed, rightWheelSpeed);

    if(!ps2x.Button(PSB_PAD_UP) && !ps2x.Button(PSB_PAD_DOWN)) {
      turnOffLoader();
    }
    else {
      if (ps2x.Button(PSB_PAD_UP)) {
        Serial.println("PUSH");
        fire();
        push();
      }
      else if (ps2x.Button(PSB_PAD_DOWN)) {
        Serial.println("PULL");
        load();
        pull();
      }
    }

    if(ps2x.Button(PSB_TRIANGLE)) {
      Serial.println("FOLD UP");
      fire();
      arm.write(FOLD_POSITION);
    }

    if(ps2x.Button(PSB_CROSS)) {
      arm.write(SPREAD_POSITION);
      Serial.println("SPREAD OUT");
    }

    if(ps2x.Button(PSB_PAD_RIGHT)) {
      Serial.println("FIRE");
      fire();
    }

    if(ps2x.Button(PSB_PAD_LEFT)) {
      Serial.println("LOAD");
      load();
    }

    if(ps2x.Button(PSB_SQUARE)) {
      Serial.println("FAN ON");
      turnOnFan();
    }
    
    if(ps2x.Button(PSB_CIRCLE)) {
      Serial.println("FAN OFF");
      turnOffFan();
    }
  }
}

void configPinout() {
  pinMode(LEFT_WHEEL_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_WHEEL_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_WHEEL_FORWARD_PIN, OUTPUT);
  pinMode(RIGHT_WHEEL_BACKWARD_PIN, OUTPUT);

  pinMode(FAN_PIN, OUTPUT);

  pinMode(LOADER_PUSH_PIN, OUTPUT);
  pinMode(LOADER_PULL_PIN, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(INDICATOR_LED_PIN, OUTPUT);

  arm.attach(ARM_PIN);
}

void configController() {
  error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_SEL_PIN, PS2_DAT_PIN, PS2_PRESSURE, PS2_RUMBLE);
  
  if(error == 0)
    Serial.print("Controller found, configured successful");
  else if(error == 1)
    Serial.println("No controller found");   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands");
  else if(error == 3)
    Serial.println("Controller found but refusing to enter Pressures mode");
  
  type = ps2x.readType(); 
  switch(type) {
    case 0:
      Serial.print("Unknown Controller");
      break;
    case 1:
      Serial.print("DualShock Controller");
      break;
    case 2:
      Serial.print("GuitarHero Controller");
      break;
    case 3:
      Serial.print("Wireless Sony DualShock Controller");
      break;
  }
}

void buzz(unsigned int repeat, unsigned int duration) {
  if ((repeat == 0) || (duration == 0))
    return;
  for (unsigned int i = 0; i < duration; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(INDICATOR_LED_PIN, HIGH);
    delay(duration);

    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(INDICATOR_LED_PIN, LOW);
    delay(duration);
  }
}

void move(int leftWheelPWM, int rightWheelPWM) {
  if (leftWheelPWM > 0) {
    analogWrite(LEFT_WHEEL_FORWARD_PIN, leftWheelPWM);
    analogWrite(LEFT_WHEEL_BACKWARD_PIN, OFF);
  }
  else {
    analogWrite(LEFT_WHEEL_FORWARD_PIN, OFF);
    analogWrite(LEFT_WHEEL_BACKWARD_PIN, abs(leftWheelPWM));
  }

  if (rightWheelPWM > 0) {
    analogWrite(RIGHT_WHEEL_FORWARD_PIN, rightWheelPWM);
    analogWrite(RIGHT_WHEEL_BACKWARD_PIN, OFF);
  }
  else {
    analogWrite(RIGHT_WHEEL_FORWARD_PIN, OFF);
    analogWrite(RIGHT_WHEEL_BACKWARD_PIN, abs(rightWheelPWM));
  }
}

void push() {
  analogWrite(LOADER_PUSH_PIN, LOADER_MOVE_RATE);
  analogWrite(LOADER_PULL_PIN, OFF);
}

void pull() {
  analogWrite(LOADER_PUSH_PIN, OFF);
  analogWrite(LOADER_PULL_PIN, LOADER_MOVE_RATE);
}

void fire() {
  analogWrite(LEFT_ROLLER_FIRE_PIN, LEFT_ROLLER_FIRE_RATE);
  analogWrite(LEFT_ROLLER_LOAD_PIN, OFF);
  analogWrite(RIGHT_ROLLER_FIRE_PIN, RIGHT_ROLLER_FIRE_RATE);
  analogWrite(RIGHT_ROLLER_LOAD_PIN, OFF);
}

void load() {
  analogWrite(LEFT_ROLLER_FIRE_PIN, OFF);
  analogWrite(LEFT_ROLLER_LOAD_PIN, LEFT_ROLLER_LOAD_RATE);
  analogWrite(RIGHT_ROLLER_FIRE_PIN, OFF);
  analogWrite(RIGHT_ROLLER_LOAD_PIN, LEFT_ROLLER_LOAD_RATE);
}

void turnOnFan() {
  analogWrite(FAN_PIN, FAN_RATE);
}

void turnOffFan() {
  analogWrite(FAN_PIN, OFF);
}

void turnOffLoader() {
  analogWrite(LOADER_PUSH_PIN, OFF);
  analogWrite(LOADER_PULL_PIN, OFF);
}

void turnOffRollers() {
  analogWrite(LOADER_PUSH_PIN, OFF);
  analogWrite(LOADER_PULL_PIN, OFF);
}