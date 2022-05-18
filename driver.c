/*
 * 
 * Description: Pulse Width Modulation on an ESP32 MCU through FreeRTOS tasks using hardware timers to control the time, ledc routines to control the PWM and interrupts to trigger the process.
 * Author: David Kumar 
 * 
 */
 
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

typedef enum {
  LED_OFF_T = HIGH,
  LED_ON_T = LOW
} LED_STATE_T ;

#ifndef LED_BUILTIN
#define LED_BUILTIN 17
#endif
#define LED2        19 
#define LED3        18 // blue 

const byte switch_pinG = 0; //corresponds to GPIO0
int PWM_FREQUENCY = 1000; 
int PWM_CHANNEL = 0; 
int PWM_RESOUTION = 8; 
unsigned long count_s = 0;
hw_timer_t * timer = NULL;

void IRAM_ATTR onTimer(){
 count_s++;
}

extern void FadeIn(void* pvParameters);
extern void FadeOut(void* pvParameters);
extern void Steady(void* pvParameters);

static void IRAM_ATTR button_pressed(void);

// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(switch_pinG), button_pressed, FALLING);  
  pinMode(LED3, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOUTION);
  ledcAttachPin(LED3, PWM_CHANNEL);
  //ledcWrite(PWM_CHANNEL, 255);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);
  timerAlarmEnable(timer);
  
  xTaskCreatePinnedToCore(
    FadeIn
    ,  "FadeIn"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  Serial.println("\nStarting the scheduler");
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
  
}

void loop(){
  
  // Empty. Things are done in Tasks.
}

void FadeOut(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  Serial.println("\nCurrently in the FadeOut task");
  
  count_s = 0;
  while(count_s <= 2000){ // decreasing brightness for 2 seconds 
    ledcWrite(PWM_CHANNEL, map(count_s,2000,0,256,0));
  }
    
  Serial.println("Finished fading out.");
  vTaskDelete(NULL);
}

void FadeIn(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  Serial.println("\nCurrently in the FadeIn task");

  count_s=0;
  
  while(count_s < 4000){ // increasing brightness for 4 seconds
    ledcWrite(PWM_CHANNEL, map(count_s,4000,0,0,256));
  }
  
  xTaskCreatePinnedToCore(
    Steady
    ,  "Steady"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  Serial.println("Finished fading in.");
  vTaskDelete(NULL);
}

void vProcessInterface(void* pvParameter1, uint32_t ulParameter2){

  BaseType_t xInterfaceToService;

  xInterfaceToService = (BaseType_t) ulParameter2;

  Serial.println("\nStarting the FadeIn task now.");

  xTaskCreatePinnedToCore(
  FadeIn
  ,  "FadeIn"   // A name just for humans
  ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
  ,  NULL
  ,  1 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  ,  NULL 
  ,  ARDUINO_RUNNING_CORE);

  
}

static void IRAM_ATTR button_pressed(void){ // deferred interrupt 

  BaseType_t xInterfaceToService, xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;
  xTimerPendFunctionCallFromISR(vProcessInterface, NULL, (uint32_t) xInterfaceToService, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(); // RTOS functions must have ISR suffix 
  
}

void Steady(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  Serial.println("\nCurrently in the Steady task");
  TickType_t xTimeInTicks = pdMS_TO_TICKS(6000); // maintaining maximum brightness for 6 seconds

  // initialize digital LED3 on defined pin as an output.
  pinMode(LED3, OUTPUT);

  for (;;) {
    
    digitalWrite(LED3, LED_ON_T);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay(xTimeInTicks);  // delays for 6 seconds 
    digitalWrite(LED3, LED_OFF_T);    // turn the LED off by making the voltage LOW

    xTaskCreatePinnedToCore(
    FadeOut
    ,  "FadeOut"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
    
  }

  Serial.println("Finished Steady.");
  vTaskDelete(NULL);
}
