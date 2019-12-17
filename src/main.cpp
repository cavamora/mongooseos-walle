#include <stdio.h>
#include "mgos.h"
#include "common/json_utils.h"
#include "mgos_blynk.h"


/******************************************************************************************************
 * DEFINES
*******************************************************************************************************/

#define STATUS_LED_GPIO                   2

#define SKY_CMD_GET_INFO_DATA_SIZE        38

#define SKY_CMD_GET_INFO_SERIAL_ID_POS    35
#define SKY_SERIAL_ID_SIZE                17

#define SKY_CMD_GET_INFO_SMART_CARD_POS   14
#define SKY_SMART_CARD_SIZE               12

#define SKY_CMD_GET_INFO_RECV_ID_POS      2
#define SKY_RECV_ID_SIZE                  12

#define TOHEX(Y) (Y>='0'&&Y<='9'?Y-'0':Y-'A'+10)


/******************************************************************************************************
 * GLOBAL VARS
*******************************************************************************************************/


//BUFFER
uint8_t buffer[32];
char buffer_out[256];

//TIMERS
mgos_timer_id ptr_timer_led_blinker;



/******************************************************************************************************
 * FUNCTION DECLARATIONS
*******************************************************************************************************/



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

  LOG(LL_INFO, ("INIT COM SUCESSO"));
  return MGOS_APP_INIT_SUCCESS;
}

