/* * * * * * * * * * * * * * * * * * * * * * *
 * MOTOR CONTROLLER CLASS
 *
 * Code by: Simon B.
 * Email: 	hello@chillibasket.com
 * Version: 1
 * Date: 	20/4/19
 * * * * * * * * * * * * * * * * * * * * * * */

#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include "mgos_arduino_PWMServoDriver.h"

// MOTOR CONTROLLER CLASS
class MotorController {
public:
	// Constructor
	MotorController(uint8_t _in1, uint8_t _in2, uint8_t _pwmPin, Adafruit_PWMServoDriver *_pwm);
	
	// Functions
	void setSpeed(int pwmValue);

	// Default destructor
	~MotorController();

private:
	uint8_t in1, in2, pwmPin;
	bool reverse, brake, brakeEnabled;
	Adafruit_PWMServoDriver *pwm;
};


/*
 * \Func 	MotorController(uint8_t _dirPin, uint8_t _pwmPin, uint8_t _brkPin)
 * \Desc 	Default constructor
 */
MotorController::MotorController(uint8_t _in1, uint8_t _in2, uint8_t _pwmPin, Adafruit_PWMServoDriver *_pwm) {
	in1 = _in1;
	in2 = _in2;
	pwmPin = _pwmPin;
	pwm = _pwm;

	mgos_gpio_set_mode(in1, MGOS_GPIO_MODE_OUTPUT);
	mgos_gpio_set_mode(in2, MGOS_GPIO_MODE_OUTPUT);

	//pinMode(dirPin, OUTPUT);     // Motor Direction
	//pinMode(brkPin, OUTPUT);     // Motor Brake
	//digitalWrite(dirPin, HIGH);

	reverse = false;
	/*
	if (brakeEnabled) {
		digitalWrite(brkPin, HIGH);
		brake = true;
	} else {
		digitalWrite(brkPin, LOW);
		brake = false;
	}
	*/
}


/*
 * \Func 	~MotorController()
 * \Desc 	Default destructor
 */
MotorController::~MotorController() {

}


/*
 * \Func 	void setSpeed(uint8_t pwmValue)
 * \Desc 	Update the speed of the motor
 */
void MotorController::setSpeed(int pwmValue) {

	// Bound the PWM value to +-255
	if (pwmValue > 255) pwmValue = 255;
	else if (pwmValue < -255) pwmValue = -255;
	
	// Forward direction
	if (pwmValue > 0) {
		mgos_gpio_write(in1, LOW);
		mgos_gpio_write(in2, HIGH);

	// Reverse direction
	} 
	
	else {
		mgos_gpio_write(in1, HIGH);
		mgos_gpio_write(in2, LOW);
	}
	
	// Send PWM value
	mgos_PWMServoDriver_setPWM(pwm, pwmPin, 0, abs(pwmValue));
}


#endif // MOTOR_CONTROLLER_HPP