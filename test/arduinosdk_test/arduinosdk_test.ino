/* Interface L298N With NodeMCU
 * By TheCircuit
 */

 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver *pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 0;


//motor 1
//int IN1 = 12; 
//int IN2 = 14; 
//uint8_t MOTOR_PWM = 8;

//motor 2
int IN1 = 13; 
int IN2 = 15; 
uint8_t MOTOR_PWM = 7;

void setup() {
  Serial.begin(9600);
  Serial.println("Super Test");

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  // set all the motor control pins to outputs
  //pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

// this function will run the motors in both directions at a fixed speed

void testOne() {
// turn on motor
pwm.setPWM(MOTOR_PWM, 0, 4095);
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);

delay(5000); // now change motor directions

digitalWrite(IN1, LOW);
digitalWrite(IN2, HIGH);

delay(5000); // now turn off motors

digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
}

// this function will run the motors across the range of possible speeds
// note that maximum speed is determined by the motor itself and the operating voltage
// the PWM values sent by analogWrite() are fractions of the maximum speed possible by your hardware

void testTwo() {

// turn on motors
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);

// accelerate from zero to maximum speed
for (int i = 0; i < 4095; i=i+20)
  {
    Serial.printf("i=%d\n",i);
    pwm.setPWM(MOTOR_PWM, 0, i);
    delay(50);
   }

// decelerate from maximum speed to zero
for (int i = 4095; i >= 0; i=i-20)
    {
      Serial.printf("i=%d\n",i);
      pwm.setPWM(MOTOR_PWM, 0, i);
      delay(50);
     }

// now turn off motors
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}


void loop() {
  //testOne();   
  //delay(1000);   
  testTwo();   
  delay(1000);

  // TEST SERVO ON POSITION 0
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(1000);


  
}
