#include <stdio.h>

#include "Adafruit_PWMServoDriver.h"
#include "Queue.hpp"
#include "MotorController.hpp"
#include "DFPlayerMini_Fast.h"

#include "common/mg_str.h"
#include "common/json_utils.h"

#include "mgos_config_util.h"
#include "mgos_hal.h"
#include "mgos_sys_config.h"
#include "mgos_utils.h"
#include "mgos.h"
#include "mgos_blynk.h"
#include "mgos_rpc.h"
#include "mgos_http_server.h"
#include "mgos_gpio.h"


/******************************************************************************************************
 * DEFINES
*******************************************************************************************************/

#define STATUS_LED_GPIO                   2								// D4

#define TOHEX(Y) (Y>='0'&&Y<='9'?Y-'0':Y-'A'+10)

//#define UART_NO 0

// Define the pin-mapping
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define IN1_L 14           // Motor direction pins								D5
#define IN2_L 12           // Motor brake pins									D6
#define PWM_L 8            // Motor PWM pins (on PCA9685)

#define IN1_R 13															//  D7
#define IN2_R 15
#define PWM_R 7																//  D8

#define SR_OE 0 	       // Servo shield output enable pin					D3					





// Define other constants
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define FREQUENCY 10       // Time in milliseconds of how often to update servo and motor positions
#define SERVOS 7           // Number of servo motors
#define THRESHOLD 1        // The minimum error which the dynamics controller tries to achieve
#define MOTOR_OFF 6000 	   // Turn servo motors off after 6 seconds
#define MAX_SERIAL 5       // Maximum number of characters that can be received



/******************************************************************************************************
 * GLOBAL VARS
*******************************************************************************************************/


//TIMERS
mgos_timer_id ptr_timer_led_blinker;
mgos_timer_id ptr_timer_loop;

// Instantiate objects
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
// Servo shield controller class - assumes default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Set up motor controller classes
MotorController motorL(IN1_L, IN2_L, PWM_L, &pwm);
MotorController motorR(IN1_R, IN2_R, PWM_R, &pwm);

// Queue for animations
Queue <int> queue(400);


// Motor Control Variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
int pwmspeed = 4095;
int moveVal = 0;
int turnVal = 0;
int turnOff = 0;


// Runtime Variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
unsigned long lastTime = 0;
unsigned long animeTimer = 0;
unsigned long motorTimer = 0;
unsigned long updateTimer = 0;
bool autoMode = mgos_sys_config_get_walle_auto_mode();


// Serial Parsing
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
char firstChar;
char serialBuffer[MAX_SERIAL];
uint8_t serialLength = 0;

DFPlayerMini_Fast mp3_player;

// ****** SERVO MOTOR CALIBRATION *********************
// Servo Positions:  Low,High
int preset[][2] =  {{410, 125},   // head rotation
                    {205, 538},   // neck top
                    {140, 450},   // neck bottom
                    {485, 230},   // eye right
                    {274, 495},   // eye left
                    {355, 137},   // arm left
                    {188, 420}};  // arm right
// *****************************************************


// Servo Control - Position, Velocity, Acceleration
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
// Servo Pins:	     0,   1,   2,   3,   4,   5,   6,   -,   -
// Joint Name:	  head,necT,necB,eyeR,eyeL,armL,armR,motL,motR
float curpos[] = { 248, 560, 140, 475, 270, 250, 290,  2880,  2880};  // Current position (units)
float setpos[] = { 248, 560, 140, 475, 270, 250, 290,    0,    0};  // Required position (units)
float curvel[] = {   0,   0,   0,   0,   0,   0,   0,    0,    0};  // Current velocity (units/sec)
float maxvel[] = { 500, 750, 255,2400,2400, 500, 500, 4095, 4095};  // Max Servo velocity (units/sec)
float accell[] = { 350, 480, 150,1800,1800, 300, 300,  12800,  12800};  // Servo acceleration (units/sec^2)


// Animation Presets 
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
// (Time is in milliseconds)
// (Servo values are between 0 to 100, use -1 to disable the servo)
#define SOFT_LEN 7
// Starting Sequence:              time,head,necT,necB,eyeR,eyeL,armL,armR
const int softSeq[][SERVOS+1] =  {{ 200,  50,  69,  29,   1,   1,  41,  41},
                                  { 200,  50,  70,  29,   1,   1,  41,  41},
                                  { 200,  50,  70,  30,   1,   1,  41,  41},
                                  { 200,  50,  70,  30,   0,   1,  41,  41},
                                  { 200,  50,  70,  30,   0,   0,  41,  41},
                                  { 200,  50,  70,  30,   0,   0,  40,  41},
                                  { 200,  50,  70,  30,   0,   0,  40,  40}};

#define BOOT_LEN 9
// Bootup Eye Sequence:            time,head,necT,necB,eyeR,eyeL,armL,armR
const int bootSeq[][SERVOS+1] =  {{2000,  50,  68,   0,  40,  40,  40,  40},
                                  { 700,  50,  68,   0,  40,   0,  40,  40},
                                  { 700,  50,  68,   0,   0,   0,  40,  40},
                                  { 700,  50,  68,   0,   0,  40,  40,  40},
                                  { 700,  50,  68,   0,  40,  40,  40,  40},
                                  { 400,  50,  68,   0,   0,   0,  40,  40},
                                  { 400,  50,  68,   0,  40,  40,  40,  40},
                                  {2000,  50,  85,   0,  40,  40,  40,  40},
                                  {1000,  50,  85,   0,   0,   0,  40,  40}};

#define INQU_LEN 9
// Inquisitive Movements:          time,head,necT,necB,eyeR,eyeL,armL,armR
const int inquSeq[][SERVOS+1] =  {{3000,  48,  60,   0,  35,  45,  60,  59},
                                  {1500,  48,  60,   0, 100,   0, 100, 100},
                                  {3000,   0,   0,   0, 100,   0, 100, 100},
                                  {1500,  48,   0,   0,  40,  40, 100, 100},
                                  {1500,  48,  60,   0,  45,  35,   0,   0},
                                  {1500,  34,  44,   0,  14, 100,   0,   0},
                                  {1500,  48,  60,   0,  35,  45,  60,  59},
                                  {3000, 100,  60,   0,  40,  40,  60, 100},
                                  {1500,  48, 100,   0,   0,   0,   0,   0}};





/******************************************************************************************************
 * FUNCTION DECLARATIONS
*******************************************************************************************************/

void queueAnimation(const int seq[][SERVOS+1], int len);
void evaluateCommand(const char command, int number);


/******************************************************************************************************
 * UTILS
*******************************************************************************************************/



int util_str2bin(const char *source_str, char *dest_buffer) {
  const char *line = source_str;
  const char *data = line;
  int offset;
  int read_byte;
  int data_len = 0;

  while (sscanf(data, " %02x%n", &read_byte, &offset) == 1) {
    dest_buffer[data_len++] = read_byte;
    data += offset;
  }
  return data_len;
}



void util_bin2str(char *input, char *output,int len)
{

    int i;

    for (i = 0; i < len; i++)
    {
        output += sprintf (output, "%02X", input[i]);
    }


}


void util_save_cfg() {
  char *err = NULL;
  save_cfg(&mgos_sys_config, &err); /* Writes conf9.json */
  LOG(LL_INFO, ("Saving configuration: %s\n", err ? err : "no error"));
  free(err);
}



/******************************************************************************************************
 * WALLE
*******************************************************************************************************/


// ------------------------------------------------------------------
// 		QUEUE ANIMATIONS
// ------------------------------------------------------------------
void queueAnimation(const int seq[][SERVOS+1], int len) {
	for (int i = 0; i < len; i++) {
		for (int j = 0; j < SERVOS+1; j++) {
			queue.push(seq[i][j]);
		}
	}
}



// -------------------------------------------------------------------
// 		SEQUENCE AND GENERATE ANIMATIONS
// -------------------------------------------------------------------
void manageAnimations() {
	// If we are running an animation
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	if ((queue.size() >= SERVOS+1) && (animeTimer <= millis())) {
		// Set the next waypoint time
		animeTimer = millis() + queue.pop();

		// Set all the joint positions
		for (int i = 0; i < SERVOS; i++) {
			int value = queue.pop();

			// Scale the positions using the servo calibration values
			setpos[i] = int(value * 0.01 * (preset[i][1] - preset[i][0]) + preset[i][0]);
		}

	// If we are in autonomous mode, but there are no movements queued, generate new movements
	} else if (autoMode && (queue.size() < SERVOS+1) && (animeTimer <= millis())) {

		// For each of the servos
		for (int i = 0; i < SERVOS; i++) {

			// Randomly determine whether or not to update the servo
			if (mgos_rand_range(0, 2) == 1) {

				// For most of the servo motors
				if (i == 0 || i == 1 || i == 2 || i == 5 || i == 6) {

					// Randomly determine the new position
					unsigned int min = preset[i][0];
					unsigned int max = preset[i][1];
					if (min > max) {
						min = max;
						max = preset[i][0];
					}
					
					setpos[i] = mgos_rand_range(min, max+1);

				// Since the eyes should work together, only look at one of them
				} else if (i == 3) {

					int midPos1 = int((preset[i][1] - preset[i][0])*0.4 + preset[i][0]);
					int midPos2 = int((preset[i+1][1] - preset[i+1][0])*0.4 + preset[i+1][0]);

					// Determine which type of eye movement to do
					// Both eye move downwards
					if (mgos_rand_range(0, 2) == 1) {
						setpos[i] = mgos_rand_range(midPos1, preset[i][0]);
						float multiplier = (setpos[i] - midPos1) / float(preset[i][0] - midPos1);
						setpos[i+1] = ((1 - multiplier) * (midPos2 - preset[i+1][0])) + preset[i+1][0];

					// Both eyes move in opposite directions
					} else {
						setpos[i] = mgos_rand_range(midPos1, preset[i][0]);
						float multiplier = (setpos[i] - preset[i][1]) / float(preset[i][0] - preset[i][1]);
						setpos[i+1] = (multiplier * (preset[i+1][1] - preset[i+1][0])) + preset[i+1][0];
					}
				}

			}
		}

		// Finally, figure out the amount of time until the next movement should be done
		animeTimer = millis() + mgos_rand_range(500, 3000);

	}
}


// -------------------------------------------------------------------
// 		MANAGE THE MOVEMENT OF THE SERVO MOTORS
// -------------------------------------------------------------------
void manageServos(float dt) {
	// SERVO MOTORS
	// -  -  -  -  -  -  -  -  -  -  -  -  -
	bool moving = false;
	for (int i = 0; i < SERVOS; i++) {

		float posError = setpos[i] - curpos[i];

		// If position error is above the threshold
		if (abs(posError) > THRESHOLD && (setpos[i] != -1)) {

			mgos_gpio_write(SR_OE, LOW);
			moving = true;

			// Determine motion direction
			bool dir = true;
			if (posError < 0) dir = false;

			// Determine whether to accelerate or decelerate
			float acceleration = accell[i];
			if ((0.5 * curvel[i] * curvel[i] / accell[i]) > abs(posError)) acceleration = -accell[i];

			// Update the current velocity
			if (dir) curvel[i] += acceleration * dt / 1000.0;
			else curvel[i] -= acceleration * dt / 1000.0;

			// Limit Velocity
			if (curvel[i] > maxvel[i]) curvel[i] = maxvel[i];
			if (curvel[i] < -maxvel[i]) curvel[i] = -maxvel[i];
			
			float dP = curvel[i] * dt / 1000.0;

			if (abs(dP) < abs(posError)) curpos[i] += dP;
			else curpos[i] = setpos[i];

      		LOG(LL_INFO, ("Mandando comando PWD %d para %d", (int) curpos[i], i));
			pwm.setPWM(i, 0, curpos[i]);

		} else {
			curvel[i] = 0;
		}
	}

	// Disable servos if robot is not moving
	// This prevents the motors from overheating
	if (moving) motorTimer = millis() + MOTOR_OFF;
	else if (millis() > motorTimer) mgos_gpio_write(SR_OE, HIGH);
}


// -------------------------------------------------------------------
// 		MANAGE THE MOVEMENT OF THE MAIN MOTORS
// -------------------------------------------------------------------
void manageMotors(float dt) {
  
	// Update Main Motor Values
	setpos[7] = moveVal - turnVal - turnOff;
	setpos[8] = moveVal + turnVal + turnOff;

	// MAIN DRIVING MOTORS
	// -  -  -  -  -  -  -  -  -  -  -  -  -
	for (int i = SERVOS; i < SERVOS + 2; i++) {

		float velError = setpos[i] - curvel[i];

		// If velocity error is above the threshold
		if (abs(velError) > THRESHOLD && (setpos[i] != -1)) {

			// Determine whether to accelerate or decelerate
			float acceleration = accell[i];
			if (setpos[i] < curvel[i] && curvel[i] >= 0) acceleration = -accell[i];
			else if (setpos[i] < curvel[i] && curvel[i] < 0) acceleration = -accell[i]; 
			else if (setpos[i] > curvel[i] && curvel[i] < 0) acceleration = accell[i];

			// Update the current velocity
			float dV = acceleration * dt / 1000.0;
			if (abs(dV) < abs(velError)) curvel[i] += dV;
			else curvel[i] = setpos[i];
		} else {
			curvel[i] = setpos[i];
		}
    
		
		// Limit Velocity
		if (curvel[i] > maxvel[i]) curvel[i] = maxvel[i];
		if (curvel[i] < -maxvel[i]) curvel[i] = -maxvel[i];
	}

	if (curvel[SERVOS] > 0 || curvel[SERVOS+1] > 0 ) {
		LOG(LL_INFO, ("Atualizando Motores: L:%d, R:%d", (int)curvel[SERVOS], (int)curvel[SERVOS+1]));
	}

	// Update motor speeds
	motorL.setSpeed(curvel[SERVOS]);
	motorR.setSpeed(curvel[SERVOS+1]);
}



// -------------------------------------------------------------------
// 		EVALUATE INPUT FROM SERIAL
// -------------------------------------------------------------------
void evaluateCommand(const char command, int number) {

	LOG(LL_INFO, ("evaluateCommand: cmd:%c, val:%d", command, number));
	
	// Motor Inputs and Offsets
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	if      (command == 'X' && number >= -100 && number <= 100) turnVal = int(number * 40.95); 		// Forward/reverse control
	else if (command == 'Y' && number >= -100 && number <= 100) moveVal = int(number * 40.95); 		// Left/right control
	else if (command == 'S' && number >=  100 && number <= 100) turnOff = number; 					// Steering offset
	else if (command == 'O' && number >=    0 && number <= 250) curpos[7] = curpos[8] = int(number); 	// Motor deadzone offset

	// Animations
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	else if (command == 'A' && number == 0) queueAnimation(softSeq, SOFT_LEN);
	else if (command == 'A' && number == 1) queueAnimation(bootSeq, BOOT_LEN);
	else if (command == 'A' && number == 2) queueAnimation(inquSeq, INQU_LEN);

	// Autonomous servo mode
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	else if (command == 'M' && number == 0) autoMode = false;
	else if (command == 'M' && number == 1) autoMode = true;

	// Manual Movements with WASD
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	else if (command == 'w') {		// Forward movement
		moveVal = pwmspeed;
		turnVal = 0;
		setpos[0] = (preset[0][1] + preset[0][0]) / 2;
	}
	else if (command == 'q') {		// Stop movement
		moveVal = 0;
		turnVal = 0;
		setpos[0] = (preset[0][1] + preset[0][0]) / 2;
	}
	else if (command == 's') {		// Backward movement
		moveVal = -pwmspeed;
		turnVal = 0;
		setpos[0] = (preset[0][1] + preset[0][0]) / 2;
	}
	else if (command == 'a') {		// Drive & look left
		moveVal = 0;
		turnVal = -pwmspeed;
		setpos[0] = preset[0][0];
	}
	else if (command == 'd') {   		// Drive & look right
		moveVal = 0;
		turnVal = pwmspeed;
		setpos[0] = preset[0][1];
	}

	// Manual Eye Movements
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	else if (command == 'j') {		// Left head tilt
		setpos[4] = preset[4][0];
		setpos[3] = preset[3][1];
	}
	else if (command == 'l') {		// Right head tilt
		setpos[4] = preset[4][1];
		setpos[3] = preset[3][0];
	}
	else if (command == 'i') {		// Sad head
		setpos[4] = preset[4][0];
		setpos[3] = preset[3][0];
	}
	else if (command == 'k') {		// Neutral head
		setpos[4] = int(0.4 * (preset[4][1] - preset[4][0]) + preset[4][0]);
		setpos[3] = int(0.4 * (preset[3][1] - preset[3][0]) + preset[3][0]);
	}

	// Head movement
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	else if (command == 'f') {		// Head up
		setpos[1] = preset[1][0];
		setpos[2] = (preset[2][1] + preset[2][0])/2;
	}
	else if (command == 'g') {		// Head forward
		setpos[1] = preset[1][1];
		setpos[2] = preset[2][0];
	}
	else if (command == 'h') {		// Head down
		setpos[1] = preset[1][0];
		setpos[2] = preset[2][0];
	}

	else if (command == 'v') {		//look left
		setpos[0] = preset[0][0];
	}
	else if (command == 'c') {		//look right
		setpos[0] = preset[0][1];
	}
	else if (command == 'x') {		//look forward
		setpos[0] = (preset[0][1] + preset[0][0]) / 2;
	}
	
	// Arm Movements
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	else if (command == 'b') {		// Left arm low, right arm high
		setpos[5] = preset[5][0];
		setpos[6] = preset[6][1];
	}
	else if (command == 'n') {		// Both arms neutral
		setpos[5] = (preset[5][0] + preset[5][1]) / 2;
		setpos[6] = (preset[6][0] + preset[6][1]) / 2;
	}
	else if (command == 'm') {		// Left arm high, right arm low
		setpos[5] = preset[5][1];
		setpos[6] = preset[6][0];
	}
}


// -------------------------------------------------------------------
// 		START PROCESSING COMMAND
// -------------------------------------------------------------------
void processCommand(char inchar) {

	// If the string has ended, evaluate the serial buffer
	if (inchar == '\n' || inchar == '\r') {

		if (serialLength > 0) evaluateCommand(firstChar, atoi(serialBuffer));
		serialBuffer[0] = 0;
		serialLength = 0;

	// Otherwise add to the character to the buffer
	} else {
		if (serialLength == 0) firstChar = inchar;
		else {
			serialBuffer[serialLength-1] = inchar;
			serialBuffer[serialLength] = 0;
		}
		serialLength++;

		// To prevent overflows, evalute the buffer if it is full
		if (serialLength == MAX_SERIAL) {
			evaluateCommand(firstChar, atoi(serialBuffer));
			serialBuffer[0] = 0;
			serialLength = 0;
		}
	}
}





/******************************************************************************************************
 * NETWORK 
*******************************************************************************************************/


static void status_led_blink(void *arg) {
  //mgos_gpio_write(STATUS_LED_GPIO, led_blynker);
  //led_blynker = !led_blynker;
  mgos_gpio_toggle(STATUS_LED_GPIO);
  (void) arg;
}

static void status_led_off() {
  mgos_clear_timer(ptr_timer_led_blinker);
  mgos_gpio_write(STATUS_LED_GPIO, 1);
  
}

static void status_led_on() {
  mgos_clear_timer(ptr_timer_led_blinker);
  mgos_gpio_write(STATUS_LED_GPIO, 0);
  
}


static void net_cb(int ev, void *evd, void *arg) {
  switch (ev) {
    case MGOS_NET_EV_DISCONNECTED:
      LOG(LL_INFO, ("%s", "Net disconnected"));
      //mgos_clear_timer(ptr_timer_led_blinker);
      status_led_off();
      break;
    case MGOS_NET_EV_CONNECTING:
      LOG(LL_INFO, ("%s", "Net connecting..."));
      mgos_clear_timer(ptr_timer_led_blinker);
      ptr_timer_led_blinker = mgos_set_timer(500, MGOS_TIMER_REPEAT, status_led_blink, NULL);
      break;
    case MGOS_NET_EV_CONNECTED:
      LOG(LL_INFO, ("%s", "Net connected"));
      mgos_clear_timer(ptr_timer_led_blinker);
      ptr_timer_led_blinker = mgos_set_timer(100, MGOS_TIMER_REPEAT, status_led_blink, NULL);
      break;
    case MGOS_NET_EV_IP_ACQUIRED:
      LOG(LL_INFO, ("%s", "Net got IP address"));
      break;
  }
 
  (void) evd;
  (void) arg;
}

static void cloud_cb(int ev, void *evd, void *arg) {
  struct mgos_cloud_arg *ca = (struct mgos_cloud_arg *) evd;
  switch (ev) {
    case MGOS_EVENT_CLOUD_CONNECTED: {
      LOG(LL_INFO, ("Cloud connected (%d)", ca->type));
      status_led_on();
      break;
    }
    case MGOS_EVENT_CLOUD_DISCONNECTED: {
      LOG(LL_INFO, ("Cloud disconnected (%d)", ca->type));
      mgos_clear_timer(ptr_timer_led_blinker);
      ptr_timer_led_blinker = mgos_set_timer(100, MGOS_TIMER_REPEAT, status_led_blink, NULL);
      break;
    }
  }

  (void) arg;
}


bool net_api_get_param(const char *topic, int topic_len, const char *name, char *value) { 

  char *buffer = (char *)calloc(topic_len+1, 1);
  memcpy(buffer, topic, topic_len);
  
  char *pos1 = strstr(buffer, name); 

  if (pos1) { 
    pos1 += strlen(name); 

    if (*pos1 == '=') { // Make sure there is an '=' where we expect it 
      pos1++; 

      while (*pos1 && *pos1 != '&') { 
        
        if (*pos1 == '%') { // Convert it to a single ASCII character and store at our Valueination 
          *value++ = (char)TOHEX(pos1[1]) * 16 + TOHEX(pos1[2]); 
          pos1 += 3; 
        } 
        
        else if( *pos1=='+' ) { // If it's a '+', store a space at our Valueination 
          *value++ = ' '; 
          pos1++; 
        } 
        else { 
          *value++ = *pos1++; // Otherwise, just store the character at our Valueination 
        } 
        //LOG(LL_INFO, ("pos1= %d max=%d", (int)pos1, (int)topic+topic_len));
        //LOG(LL_INFO, ("value=%s", value));

      } 

      *value++ = 0;
      free(buffer);
      return true; 
      } 

    } 

  // If param not found, then use default parameter 
  strcpy(value, "undefine");	
  free(buffer);
  return false; 
  } 


static void net_api_handler(struct mg_connection *c, int ev, void *p, void *user_data) {
	
	LOG(LL_INFO, ("http event=%d", ev));
	if (ev != MG_EV_HTTP_REQUEST) {
		return;
	}
	
	struct http_message *hm = (struct http_message *)p;

	LOG(LL_INFO, ("URI=%.*s", hm->uri.len, hm->uri.p));
	LOG(LL_INFO, ("QS=%.*s", hm->query_string.len, hm->query_string.p));
	char temp[10];
	
	
	
	

	if (strncmp("/walle/cmnd/move", hm->uri.p, hm->uri.len) == 0) {

		memset(temp, 0, sizeof(temp));
		net_api_get_param(hm->query_string.p, hm->query_string.len, "x", temp);
		int x = atoi(temp);

		memset(temp, 0, sizeof(temp));
		net_api_get_param(hm->query_string.p, hm->query_string.len, "y", temp);
		int y = atoi(temp);

		evaluateCommand('X', x);
		evaluateCommand('Y', y);

	}

	else if (strncmp("/walle/cmnd/exec", hm->uri.p, hm->uri.len) == 0) {

		
		int val = 0;
		char temp[10];
		memset(temp, 0, sizeof(temp));
		if (net_api_get_param(hm->query_string.p, hm->query_string.len, "val", temp)) {
			val = atoi(temp);
		}

		char cmd[10];
		memset(cmd, 0, sizeof(cmd));
		if (net_api_get_param(hm->query_string.p, hm->query_string.len, "cmd", cmd) ) {
			evaluateCommand(cmd[0], val);
		}

		else {
			LOG(LL_INFO, ("ERROR: cmd not found"));
		}

	
		
		
	}
	
	else {
		LOG(LL_INFO, ("Comando desconhecido"));
	}

	


	mg_printf(c, "HTTP/1.1 200 OK\r\n" 
             "Content-Type: application/json\r\n\r\n" 
             "{ \"result\": %d }\n", 200); 
   	c->flags |= MG_F_SEND_AND_CLOSE; 

	LOG(LL_INFO, ("Saindo"));

	(void) user_data;

}


static void net_rpc_walle_move_handler(struct mg_rpc_request_info *ri, void *cb_arg,
                                    struct mg_rpc_frame_info *fi,
                                    struct mg_str args) {

  int x = 0;
  int y = 0;

  
  LOG(LL_INFO, ("RPC args : %.*s", (int)args.len, args.p)); 

  if (json_scanf(args.p, args.len, ri->args_fmt, &x, &y) == 2) { 

    LOG(LL_INFO, ("OK: x=%d y=%d", x, y)); 
	evaluateCommand('X', x);
	evaluateCommand('Y', y);
  }
  else {
    LOG(LL_INFO, ("ERROR"));
  }

  mg_rpc_send_responsef(ri, NULL);
  ri = NULL;

  (void) cb_arg;
  (void) fi;
}



static void net_rpc_walle_exec_handler(struct mg_rpc_request_info *ri, void *cb_arg,
                                    struct mg_rpc_frame_info *fi,
                                    struct mg_str args) {

  int val = 0;
  char *cmd = 0;

  if (json_scanf(args.p, args.len, ri->args_fmt, &cmd, &val) == 2) { 

    LOG(LL_INFO, ("OK: cmd=%s val=%d", cmd, val)); 
	evaluateCommand(cmd[0], val);
	
  }
  else {
    LOG(LL_INFO, ("ERROR"));
  }

  free(cmd);

  mg_rpc_send_responsef(ri, NULL);
  ri = NULL;

  (void) cb_arg;
  (void) fi;
}

 


/******************************************************************************************************
 * BLYNK
*******************************************************************************************************/

/*
static void blynk_report (char *bytes) {
  
  

  mg_connection *c = mgos_get_blynk_connection();

  if (c) {
    LOG(LL_INFO, ("report_to_blynk: sendind data"));

    blynk_printf(c, BLYNK_HARDWARE, 0, "vw%c%d%c%s\\n", 0, 50, 0, bytes);
    //blynk_virtual_write(c, 50, bytes , 0);

  }
  else {
    LOG(LL_INFO, ("report_to_blynk: blynk not connected"));
  }
  
}
*/


static void blynk_handler(struct mg_connection *c, const char *cmd,
                                  int pin, int val, int id, void *user_data, const uint8_t *raw_data, int raw_data_len) {
  
  LOG(LL_INFO, ("blynk_handler: cmd: %s pin:%d val:%d id:%d raw_data_len:%d",cmd, pin, val, id, raw_data_len));

  if (raw_data_len > 0) {

    char temp[256];
    util_bin2str((char*)raw_data, temp, raw_data_len);
    LOG(LL_INFO, ("blynk_handler: raw_data:%s", temp));


  }

  //CONECTION SUCCESS
  if (strcmp(cmd, "cn") == 0) {
	  status_led_on();
  }
  
  
  //READS
  else if (strcmp(cmd, "vr") == 0) {
    
    /*
    //memoria livre
    if (pin == 1) {
      LOG(LL_INFO, ("blynk_handler: mem"));
      blynk_virtual_write(c, pin, (float) mgos_get_free_heap_size() / 1024 , id);
    }

    //numero atual
    else if (pin == 6 || pin == 8 ) {
      LOG(LL_INFO, ("blynk_handler: display number"));
      blynk_virtual_write(c, pin, mgos_sys_config_get_display_number(), id );
    }

    //brilho atual
    else if (pin == 7 || pin == 9) {
      LOG(LL_INFO, ("blynk_handler: display bright"));
      blynk_virtual_write(c, pin, mgos_sys_config_get_display_bright(), id );
    }
    */

  } 
  
  //WRITES
  else if (strcmp(cmd, "vw") == 0) {
    
    LOG(LL_INFO, ("blynk_handler. pin:%d val:%d", pin, val));

    //joystick X
    if (pin == 1) {
      LOG(LL_INFO, ("blynk_handler: clique joystick X"));
	  evaluateCommand('X', val);
      
    }

	//joystick Y
    else if (pin == 2) {
      LOG(LL_INFO, ("blynk_handler: clique joystick Y"));
	  evaluateCommand('Y', val);
      
    }

	//head leaf/right
    else if (pin == 3) {
      LOG(LL_INFO, ("blynk_handler: head l/r val=%d",val));
	  if (val > 0) {
	  	evaluateCommand('c', 0);
	  }
	  else if (val < 0) {
		evaluateCommand('v', 0);  
	  }
	  else {
		evaluateCommand('x', 0);    
	  }
      
    }

	//head leaf/right
    else if (pin == 4) {
      LOG(LL_INFO, ("blynk_handler: head u/d val=%d",val));
	  if (val > 0) {
	  	evaluateCommand('f', 0);
	  }
	  else if (val < 0) {
		evaluateCommand('g', 0);  
	  }
	  else {
		evaluateCommand('h', 0);    
	  }
      
    }

    //comando do terminal
    else if (pin == 50) {
      LOG(LL_INFO, ("blynk_handler: comando do terminal"));

      char command[256];
      memset(command, 0, sizeof(command));
      memcpy(command, &raw_data[11], raw_data_len-11);

	  evaluateCommand(command[0],atoi(command+1));

    }



  }
  
  (void) user_data;
  (void) c;
}




/******************************************************************************************************
 * USB UART
*******************************************************************************************************/




/*
 * Dispatcher can be invoked with any amount of data (even none at all) and
 * at any time. Here we demonstrate how to process input line by line.
 */
/*
static void uart_dispatcher(int uart_no, void *arg) {

  static struct mbuf lb = {0};
  
  assert(uart_no == UART_NO);
  size_t rx_av = mgos_uart_read_avail(uart_no);
  if (rx_av == 0) return;
  mgos_uart_read_mbuf(uart_no, &lb, rx_av);




  LOG(LL_INFO, ("UART%d> Received %d bytes", uart_no, (int) lb.len));
  LOG(LL_INFO, ("UART%d> '%.*s'", uart_no, (int) lb.len, lb.buf));


  for(int i=0; i<(int)lb.len; i++) {
    processCommand(lb.buf[i]);
    manageAnimations();
  }


  mbuf_remove(&lb, lb.len);

  
  (void) arg;
}
*/


/******************************************************************************************************
 * OTHERS
*******************************************************************************************************/

/*
static void gpio_int_handler_flash(int pin, void *arg) {
  LOG(LL_INFO, ("botao flash pressionado"));

  LOG(LL_INFO, ("Memoria livre: %d", mgos_get_free_heap_size()  ));
  blynk_report((char *)"botao flash pressionado");

  //Serial.println("Setting volume to max");
  mp3_player.volume(30);
  mgos_msleep(20);
  
  //Serial.println("Playing track 1 for 5 sec");
  mp3_player.play(1);
  mgos_msleep(5000);

  //Serial.println("Sleeping for 5 sec");
  mp3_player.sleep();
  mgos_msleep(5000);

  //Serial.println("Waking up");
  mp3_player.wakeUp();

  //Serial.println("Looping track 1");
  mp3_player.loop(1);

  (void) arg;
  (void) pin;
}
*/



static void loop(void *arg) {
  
	// Load or generate new animations
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	manageAnimations();

	// Move Servos and wheels at regular time intervals
	// -- -- -- -- -- -- -- -- -- -- -- -- -- --
	
  unsigned long newTime = micros();
  float dt = (newTime - lastTime) / 1000.0;
  lastTime = newTime;

  manageServos(dt);
  manageMotors(dt);


  (void) arg;
}


/******************************************************************************************************
 * STARTUP
*******************************************************************************************************/


enum mgos_app_init_result mgos_app_init(void) {

	struct mgos_uart_config ucfg;
	mgos_uart_config_set_defaults(mgos_sys_config_get_walle_uart_no(), &ucfg);
	ucfg.baud_rate = mgos_sys_config_get_walle_uart_baudrate();
	if (!mgos_uart_configure(mgos_sys_config_get_walle_uart_no(), &ucfg)) {
		return MGOS_APP_INIT_ERROR;
	}
	//mgos_uart_set_dispatcher(UART_NO, uart_dispatcher, NULL /* arg */);
	mgos_uart_set_rx_enabled(mgos_sys_config_get_walle_uart_no(), true);

	mp3_player.begin(mgos_sys_config_get_walle_uart_no());



	/* Network connectivity events */
	mgos_event_add_group_handler(MGOS_EVENT_GRP_NET, net_cb, NULL);

	//botao flash para reset
	//mgos_gpio_set_button_handler(0, MGOS_GPIO_PULL_UP, MGOS_GPIO_INT_EDGE_POS, 50, gpio_int_handler_flash, NULL);

	//led for wifi state
	mgos_gpio_set_mode(STATUS_LED_GPIO, MGOS_GPIO_MODE_OUTPUT);

	mgos_event_add_handler(MGOS_EVENT_CLOUD_CONNECTED, cloud_cb, NULL);
	mgos_event_add_handler(MGOS_EVENT_CLOUD_DISCONNECTED, cloud_cb, NULL);

	blynk_set_handler(blynk_handler, NULL);

	// Output Enable (EO) pin for the servo motors
	mgos_gpio_set_mode(SR_OE, MGOS_GPIO_MODE_OUTPUT);
	mgos_gpio_write(SR_OE, HIGH);

	// Communicate with servo shield (Analog servos run at ~60Hz)
	pwm.begin();
  	pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

	LOG(LL_INFO, ("Starting Program"));

	// Move servos to known starting positions
	queueAnimation(softSeq, SOFT_LEN);

	ptr_timer_loop = mgos_set_timer(FREQUENCY, MGOS_TIMER_REPEAT, loop, NULL);

	mgos_register_http_endpoint("/walle", net_api_handler, NULL);

	struct mg_rpc *c = mgos_rpc_get_global();
  	mg_rpc_add_handler(c, "Walle.Move", "{x: %d, y: %d}", net_rpc_walle_move_handler, NULL);
	mg_rpc_add_handler(c, "Walle.Exec", "{cmd: %Q, val: %d}", net_rpc_walle_exec_handler, NULL);

	LOG(LL_INFO, ("INIT COM SUCESSO"));
	return MGOS_APP_INIT_SUCCESS;

}

