#include <stdio.h>
#include "mgos.h"
#include "common/json_utils.h"
#include "mgos_blynk.h"
#include "mgos_arduino_PWMServoDriver.h"

#include "Queue.hpp"


/******************************************************************************************************
 * DEFINES
*******************************************************************************************************/

#define STATUS_LED_GPIO                   2

#define TOHEX(Y) (Y>='0'&&Y<='9'?Y-'0':Y-'A'+10)

// Define the pin-mapping
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
#define DIR_L 12           // Motor direction pins
#define DIR_R 13
#define PWM_L  3           // Motor PWM pins
#define PWM_R 11
#define BRK_L  9           // Motor brake pins
#define BRK_R  8
#define SR_OE 10           // Servo shield output enable pin


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


//BUFFER
//uint8_t buffer[32];
//char buffer_out[256];

//TIMERS
mgos_timer_id ptr_timer_led_blinker;

// Instantiate objects
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
// Servo shield controller class - assumes default address 0x40
Adafruit_PWMServoDriver *pwm = mgos_PWMServoDriver_create();

// Set up motor controller classes
//MotorController motorL(DIR_L, PWM_L, BRK_L, false);
//MotorController motorR(DIR_R, PWM_R, BRK_R, false);

// Queue for animations
Queue <int> queue(400);


// Motor Control Variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
int pwmspeed = 255;
int moveVal = 0;
int turnVal = 0;
int turnOff = 0;


// Runtime Variables
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
unsigned long lastTime = 0;
unsigned long animeTimer = 0;
unsigned long motorTimer = 0;
unsigned long updateTimer = 0;
bool autoMode = false;


// Serial Parsing
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
char firstChar;
char serialBuffer[MAX_SERIAL];
uint8_t serialLength = 0;


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
float curpos[] = { 248, 560, 140, 475, 270, 250, 290, 180, 180};  // Current position (units)
float setpos[] = { 248, 560, 140, 475, 270, 250, 290,   0,   0};  // Required position (units)
float curvel[] = {   0,   0,   0,   0,   0,   0,   0,   0,   0};  // Current velocity (units/sec)
float maxvel[] = { 500, 750, 255,2400,2400, 500, 500, 255, 255};  // Max Servo velocity (units/sec)
float accell[] = { 350, 480, 150,1800,1800, 300, 300, 800, 800};  // Servo acceleration (units/sec^2)


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
 * NETWORK STATUS
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
 


/******************************************************************************************************
 * BLYNK
*******************************************************************************************************/


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



static void blynk_handler(struct mg_connection *c, const char *cmd,
                                  int pin, int val, int id, void *user_data, const uint8_t *raw_data, int raw_data_len) {
  
  LOG(LL_INFO, ("blynk_handler: cmd: %s pin:%d val:%d id:%d raw_data_len:%d",cmd, pin, val, id, raw_data_len));

  if (raw_data_len > 0) {

    char temp[256];
    util_bin2str((char*)raw_data, temp, raw_data_len);
    LOG(LL_INFO, ("blynk_handler: raw_data:%s", temp));


  }
  
  
  //READS
  if (strcmp(cmd, "vr") == 0) {
    
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

    //clique do controle
    if (pin == 1) {
      LOG(LL_INFO, ("blynk_handler: clique do controle"));
      
    }

    //comando do terminal
    else if (pin == 50) {
      LOG(LL_INFO, ("blynk_handler: comando do terminal"));

      
      
    }



    //2
    else if (pin == 2) {
      LOG(LL_INFO, ("blynk_handler: get channel"));
      
      
    }

    //3
    else if (pin == 3) {
      
    }

    //GUIA
    else if (pin == 11) {
      if (val == 1) {
        LOG(LL_INFO, ("blynk_handler: display number --"));
        
      }
    }

    //reset
    else if (pin == 100) {
      LOG(LL_INFO, ("blynk_handler: restart"));
      mgos_system_restart();
    }

    else {
      LOG(LL_INFO, ("blynk_handler: comando desconhecido"));
    }



  }
  
  (void) user_data;
  (void) c;
}






/******************************************************************************************************
 * OTHERS
*******************************************************************************************************/


static void gpio_int_handler_flash(int pin, void *arg) {
  LOG(LL_INFO, ("botao flash pressionado"));

  LOG(LL_INFO, ("Memoria livre: %d", mgos_get_free_heap_size()  ));
  blynk_report((char *)"botao flash pressionado");

  (void) arg;
  (void) pin;
}


/******************************************************************************************************
 * STARTUP
*******************************************************************************************************/


enum mgos_app_init_result mgos_app_init(void) {


  /* Network connectivity events */
  mgos_event_add_group_handler(MGOS_EVENT_GRP_NET, net_cb, NULL);

  //botao flash para reset
  mgos_gpio_set_button_handler(0, MGOS_GPIO_PULL_UP, MGOS_GPIO_INT_EDGE_POS, 50, gpio_int_handler_flash, NULL);

  //led for wifi state
  mgos_gpio_set_mode(STATUS_LED_GPIO, MGOS_GPIO_MODE_OUTPUT);

  mgos_event_add_handler(MGOS_EVENT_CLOUD_CONNECTED, cloud_cb, NULL);
  mgos_event_add_handler(MGOS_EVENT_CLOUD_DISCONNECTED, cloud_cb, NULL);

  blynk_set_handler(blynk_handler, NULL);

  // Output Enable (EO) pin for the servo motors
  mgos_gpio_set_mode(SR_OE, MGOS_GPIO_MODE_OUTPUT);
  mgos_gpio_write(SR_OE, HIGH);

	// Communicate with servo shield (Analog servos run at ~60Hz)
	mgos_PWMServoDriver_begin(pwm);
	mgos_PWMServoDriver_setPWMFreq(pwm, 60);

	LOG(LL_INFO, ("Starting Program"));

	// Move servos to known starting positions
	queueAnimation(softSeq, SOFT_LEN);

  LOG(LL_INFO, ("INIT COM SUCESSO"));
  return MGOS_APP_INIT_SUCCESS;
}

