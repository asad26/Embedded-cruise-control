#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"
#include "time.h"
#include "sys/alt_timestamp.h"
#include "sys/alt_cache.h"


#define DEBUG 1

#define HW_TIMER_PERIOD 100 /* 100ms */

/* Button Patterns */

#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x02
/* Switch Patterns */

#define TOP_GEAR_FLAG       0x00000002
#define ENGINE_FLAG         0x00000001
#define ENGINE_GEAR_FLAG    0x00000003

/* LED Patterns */

#define LED_RED_0 0x00000001 // Engine
#define LED_RED_1 0x00000002 // Top Gear
#define LED_RED_01 0x00000003 // For both active

#define LED_GREEN_0 0x0001 // Cruise Control activated
#define LED_GREEN_2 0x0002 // Cruise Control Button
#define LED_GREEN_4 0x0010 // Brake Pedal
#define LED_GREEN_6 0x0040 // Gas Pedal

/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK ButtonIO_Stack[TASK_STACKSIZE];
OS_STK SwitchIO_Stack[TASK_STACKSIZE];

OS_STK Watchdog_Stack[TASK_STACKSIZE];
OS_STK Overload_Stack[TASK_STACKSIZE];

OS_STK Extraload_Stack[TASK_STACKSIZE];


// Task Priorities
 
#define STARTTASK_PRIO     5    // Highest priority
#define WATCHDOG_PRIO   6

#define VEHICLETASK_PRIO  7
#define CONTROLTASK_PRIO  8
#define BUTTONIO_PRIO   9
#define SWITCHIO_PRIO   10


#define OVERLOAD_PRIO   12
#define EXTRALOAD_PRIO  11


// Task Periods

#define CONTROL_PERIOD  300
#define VEHICLE_PERIOD  300
//#define BUTTON_PERIOD 100
//#define SWITCH_PERIOD 100
/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;

// Semaphores
OS_EVENT *Semaphore1;
OS_EVENT *Semaphore2;

//OS_EVENT *Semaphore4;

// SW-Timer
OS_TMR *Timer;
//OS_TMR *Timer_Control;
//OS_TMR *Timer_Button;
//OS_TMR *Timer_Switch;
OS_TMR *overload_timer;
BOOLEAN start;

/*
 * Types
 */
enum active {on, off};

enum active gas_pedal = off;
enum active brake_pedal = off;
enum active top_gear = off;
enum active engine = off;
enum active cruise_control = off; 

/*
 * Global variables
 */
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs

char check_signal = 0;
unsigned char switch_value = 0;
unsigned char signal = 0, utilization = 1;
unsigned char vehiclet = 0, controlt = 0, switcht = 0, buttont = 0;

unsigned int loading_effect = 7000;
unsigned int i = 0, counter = 0;
char chk = 1;   // For controlling timer and semaphores

// Global variables for precise timer
alt_u32 ticks;
alt_u32 time_1;
alt_u32 time_2;
alt_u32 timer_overhead;

int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_KEYS4_BASE);    
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);    
}

/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */
  
  return delay;
}


// Functions for precise time management

void start_measurement()
{
      /* Flush caches */
      alt_dcache_flush_all();
      alt_icache_flush_all();   
      /* Measure */
      alt_timestamp_start();
      time_1 = alt_timestamp();
}

void stop_measurement()
{
      time_2 = alt_timestamp();
      ticks = time_2 - time_1;
}


void callbackVehicle (void* ptmr, void* callback_arg)
{
    
    if (chk == 1)
    {
        OSSemPost (Semaphore1);
       // printf("Vehicle\n");
    }
    if (chk == 2)
    {
        OSSemPost (Semaphore2);
        //printf("Control\n");
    }
}

void callbackOverload (void* ptmr, void* callback_arg)
{
       if(check_signal == 1)
       {
            check_signal = 0;
       }
       else
       {
            signal = 2;
       }
}

/*void callbackControl (void* ptmr, void* callback_arg)
{
    OSSemPost (Semaphore2);
}

void callbackButton (void* ptmr, void* callback_arg)
{
    OSSemPost (Semaphore3);
}

void callbackSwitch (void* ptmr, void* callback_arg)
{
    OSSemPost (Semaphore4);
}*/


static int b2sLUT[] = {0x40, //0
                 0x79, //1
                 0x24, //2
                 0x30, //3
                 0x19, //4
                 0x12, //5
                 0x02, //6
                 0x78, //7
                 0x00, //8
                 0x18, //9
                 0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
    return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
void show_velocity_on_sevenseg(INT8S velocity){
  int tmp = velocity;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(velocity < 0){
     out_sign = int2seven(10);
     tmp *= -1;
  }else{
     out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);
  
  out = int2seven(0) << 21 |
            out_sign << 14 |
            out_high << 7  |
            out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,out);
}

/* Show position on seven segment*/

void show_position_on_sevenseg(INT16U position)
{
    unsigned int i, q, r;
    INT16U tmp = position;
    int out;
    int array[4] = {1000, 100, 10, 1};
    for (i=0; i < 4; i++)
    {
        q = tmp / array[i];     // Quotient
        r = tmp % array[i];     // Remeinder
        tmp = r;
        array[i] = q;
        //printf("quotient %d\n", q);
        //printf("reminder %d\n", r);
    }
    
    out = (int2seven(array[0]) << 21) |
          (int2seven(array[1]) << 14) |
          (int2seven(array[2]) << 7)  |
          (int2seven(array[3]));
          
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
     
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT8U target_vel)
{
    int tmp = target_vel;
    int out;
    INT8U out_high = 0;
    INT8U out_low = 0;
    
    out_high = int2seven(tmp / 10);
    out_low = int2seven(tmp - (tmp/10) * 10);
    
    out = int2seven(0) << 21 |
          int2seven(0) << 14 |
            out_high << 7  |
            out_low;
    
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE, out);
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position)
{
    //INT32U value;
    if (position >= 0 && position < 400)   // For LEDR17
        led_red = (led_red & 0x3f000) | 0x20000;
    else if (position >= 400 && position < 800)
        led_red = (led_red & (0x3f000-0x20000)) | 0x10000;     
    else if (position >= 800 && position < 1200)
        led_red = (led_red & (0x3f000-0x30000)) | 0x08000;
    else if (position >= 1200 && position < 1600)
        led_red = (led_red & (0x3f000-0x38000)) | 0x04000;
    else if (position >= 1600 && position < 2000)
        led_red = (led_red & (0x3f000-0x3C000)) | 0x02000;  
    else 
        led_red = (led_red & (0x3f000-0x3E000)) | 0x01000;
        
    IOWR_ALTERA_AVALON_PIO_DATA (DE2_PIO_REDLED18_BASE, led_red);         
}

/*
 * The function 'adjust_position()' adjusts the position depending on the
 * acceleration and velocity.
 */
 INT16U adjust_position(INT16U position, INT16S velocity,
                        INT8S acceleration, INT16U time_interval)
{
  INT16S new_position = position + velocity * time_interval / 1000
    + acceleration / 2  * (time_interval / 1000) * (time_interval / 1000);

  if (new_position > 24000) {
    new_position -= 24000;
  } else if (new_position < 0){
    new_position += 24000;
  }
  
  show_position(new_position / 10);
  return new_position;
}
 
/*
 * The function 'adjust_velocity()' adjusts the velocity depending on the
 * acceleration.
 */
INT16S adjust_velocity(INT16S velocity, INT8S acceleration,  
               enum active brake_pedal, INT16U time_interval)
{
  INT16S new_velocity;
  INT8U brake_retardation = 200;

  if (brake_pedal == off)
    new_velocity = velocity  + (float) (acceleration * time_interval) / 1000.0;
  else 
  {
    if (brake_retardation * time_interval / 1000 > velocity)
      new_velocity = 0;
    else
      new_velocity = velocity - brake_retardation * time_interval / 1000;
  }
  
  return new_velocity;
}

/*
 * The task 'VehicleTask' updates the current velocity of the vehicle
 */
void VehicleTask(void* pdata)
{ 
  INT8U err;  
  void* msg;
  INT8U* throttle; 
  INT8S acceleration;  /* Value between 40 and -20 (4.0 m/s^2 and -2.0 m/s^2) */
  INT8S retardation;   /* Value between 20 and -10 (2.0 m/s^2 and -1.0 m/s^2) */
  INT16U position = 0; /* Value between 0 and 20000 (0.0 m and 2000.0 m)  */
  INT16S velocity = 0; /* Value between -200 and 700 (-20.0 m/s amd 70.0 m/s) */
  INT16S wind_factor;   /* Value between -10 and 20 (2.0 m/s^2 and -1.0 m/s^2) */

  printf("Vehicle task created!\n");

  while(1)
    {
      OSSemPend (Semaphore1, 0, &err);
      
     //printf("Vehicle Called.!");
      if (engine == on) //&& gas_pedal == on)
      {
      err = OSMboxPost(Mbox_Velocity, (void *) &velocity);      // Task wants to send a message to another task

      //OSTimeDlyHMSM(0,0,0,VEHICLE_PERIOD); 
      //OSTmrSignal();     

      /* Non-blocking read of mailbox: 
       - message in mailbox: update throttle
       - no message:         use old throttle
      */
      msg = OSMboxPend(Mbox_Throttle, 1, &err);     // Task expects to receive a message
      if (err == OS_NO_ERR) 
         throttle = (INT8U*) msg;

      /* Retardation : Factor of Terrain and Wind Resistance */
      if (velocity > 0)
         wind_factor = velocity * velocity / 10000 + 1;
      else 
         wind_factor = (-1) * velocity * velocity / 10000 + 1;
         
      if (position < 4000) 
         retardation = wind_factor; // even ground
      else if (position < 8000)
          retardation = wind_factor + 15; // traveling uphill
        else if (position < 12000)
            retardation = wind_factor + 25; // traveling steep uphill
          else if (position < 16000)
              retardation = wind_factor; // even ground
            else if (position < 20000)
                retardation = wind_factor - 10; //traveling downhill
              else
                  retardation = wind_factor - 5 ; // traveling steep downhill
                  
      acceleration = *throttle / 2 - retardation;     
      position = adjust_position(position, velocity, acceleration, 300); 
      velocity = adjust_velocity(velocity, acceleration, brake_pedal, 300); 
      printf("Position: %dm\n", position / 10);
      printf("Velocity: %4.1fm/s\n", velocity /10.0);
      printf("Throttle: %dV\n", *throttle / 10);
      show_position_on_sevenseg(position / 10);
      show_velocity_on_sevenseg((INT8S) (velocity / 10));
      chk = 2;
      vehiclet = 1;
      }
     // OSTmrSignal();
      //v = (velocity / 10.0);
      //show_position (position / 10);
    }
} 
 
/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

/*void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle = 40; // Value between 0 and 80, which is interpreted as between 0.0V and 8.0V 
  void* msg;
  INT16S* current_velocity;

  printf("Control Task created!\n");

  while(1)
    {
      OSSemPend (Semaphore2, 0, &err);
      if (engine == on)
      {
        msg = OSMboxPend(Mbox_Velocity, 0, &err);
        current_velocity = (INT16S*) msg;
      
      //if (current_velocity >= 20.0)
            //IOWR_ALTERA_AVALON_PIO_DATA (DE2_PIO_GREENLED9_BASE, LED_GREEN_0);
            
        err = OSMboxPost(Mbox_Throttle, (void *) &throttle);      // Task wants to send a message to another task
      }
      //OSTmrSignal();
      //OSSemPost (Semaphore);
      //OSTimeDlyHMSM(0,0,0, CONTROL_PERIOD);
    }
}*/
//////////////////////////////////////////////////////////////
void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle = 40; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
  void* msg;
  INT16S* current_velocity;
  INT16S VelToMaintian = 0;
  INT16S DiffInVel = 0;
  INT8U Flag = 1;
  INT8U Result = 0;

  printf("Control Task created!\n");

  while(1)
    {
        OSSemPend (Semaphore2, 0, &err);
        if (engine == on)
        {
            msg = OSMboxPend(Mbox_Velocity, 0, &err);
            current_velocity = (INT16S*) msg;
            Result = ((((((engine == on) && (cruise_control == on))&& (top_gear == on)) 
                && (gas_pedal == off)) && (brake_pedal == off)) && (*current_velocity >= 200));
            if(Result==1)
            {
   //     printf("D\n");
                if(Flag == 1)
                {
                    VelToMaintian = *current_velocity;
                    Flag = 0;
                }
      
                DiffInVel = (*current_velocity)-VelToMaintian;
  //    printf("VtoMain:%d\n",VelToMaintian);
  //    printf("CurrentV:%d\n",*current_velocity);
                if((DiffInVel < -2) || (DiffInVel > 2))
                {
                    if(DiffInVel< 0)
                    {
                        if((throttle+10) <= 80)
                            throttle = throttle + 10U;
                    }
                    else
                    {
                        if((throttle-10) >= 0)
                            throttle = throttle - 10U;
                    }
      
                 }   
            }
            else
            {
                Flag = 1;
                VelToMaintian = 0;
                throttle = 40;
            }
    
            //show_target_velocity (VelToMaintian / 10);
        }
        err = OSMboxPost(Mbox_Throttle, (void *) &throttle);
        chk = 1;
        controlt = 1;
        OSTmrSignal();
    }
}

/////////////////////////////////////////////////////////////
void ButtonIO(void* pdata)
{
    int value;
    //INT8U err;
    printf ("ButtonIO Task created!\n");
    
    while (1)
    {
        //OSSemPend (Semaphore3, 0, &err);
        value = buttons_pressed();
        value = value & 0xf;
        switch (value)
        {
            case CRUISE_CONTROL_FLAG:        // Key1 is pressed
                led_green = LED_GREEN_2; 
                cruise_control = on;
                gas_pedal = off;
                brake_pedal = off;
                break;
                
            case BRAKE_PEDAL_FLAG:      // Key2 is pressed
                led_green = LED_GREEN_4;
                brake_pedal = on;
                cruise_control = off;
                break;
                
            case GAS_PEDAL_FLAG:        // Key3 is pressed
                led_green = LED_GREEN_6;
                gas_pedal = on;
                brake_pedal = off;
                cruise_control = off;
                break;
                
            default:
                //led_green = 0;
                //cruise_control = off;
                //brake_pedal = off;
                //gas_pedal = off;
                break;
        }
        
        IOWR_ALTERA_AVALON_PIO_DATA (DE2_PIO_GREENLED9_BASE, led_green);        // LED is on
        buttont = 1;
        OSTimeDlyHMSM(0,0,0, 5);
    }   
}

void SwitchIO(void* pdata)
{
    int value;
    //INT8U err;
    printf ("SwitchIO Task created!\n");
    while (1)
    {
        
        //OSSemPend (Semaphore4, 0, &err);
        value = switches_pressed();
        switch_value = (unsigned char)(value >> 4);
        //printf("%d\n",value);
        if ((value & 0x0F) == ENGINE_FLAG)
        {
            led_red = (led_red & 0x3f000) | LED_RED_0;
            engine = on;
             
        }
        else if ((value & 0x0F) == TOP_GEAR_FLAG)
        {
            led_red = (led_red & 0x3f000) | LED_RED_1;
            top_gear = on;
            //printf("G\n");
        }
        else if ((value & 0x0F) == ENGINE_GEAR_FLAG)
        {
            led_red = (led_red & 0x3f000) | LED_RED_01;
            top_gear = on;
            engine = on;
        }
        else
        {
            led_red = (led_red & 0x3f000) | 0x00000;
            engine = off;
        }
        if(switch_value < 50)
        {
            utilization = switch_value * 2;
        }
        else if(switch_value >= 50)
        {
            utilization = 100;
        }
        else
        {
            utilization = 1;
        }
        IOWR_ALTERA_AVALON_PIO_DATA (DE2_PIO_REDLED18_BASE, led_red); 
        switcht = 1;          
        OSTimeDlyHMSM(0,0,0, 5);
    }   
}


void Watchdog (void *pdata)     // Watchdog Timer
{
    while(1)
    {
        if (engine == on)
        {
        if(signal == 1)
        {
            signal = 0;
            printf("Signal is OK!\n");
            check_signal = 1;
            vehiclet = 0; controlt = 0; buttont = 0; switcht = 0;
        }
        else if(signal == 2)
        {
            signal = 0;
            printf("Warning!\n");
            check_signal = 0;
            vehiclet = 0; controlt = 0; buttont = 0; switcht = 0;
        }
        }
        OSTimeDlyHMSM(0,0,0,5);
        
    }
}

void Overload (void *pdata)     // Overload_Detection 
{
    while(1)
    {
        if((check_signal == 0)&&((vehiclet == 1 && controlt == 1)&&(buttont == 1 && switcht == 1)))
        {
            //printf("Over\n");
            signal = 1;
        }   
        OSTimeDlyHMSM(0,0,0,1);
    } 
}


void Extraload (void* pdata)
{
    unsigned int loop = 0;
    printf("Extra_Loading Created\n");
    while(1)
    {
        loop = loading_effect * utilization;
        for(i = 0; i <= loop; i++);
        OSTimeDlyHMSM(0,0,0,20);
    }
}
/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 

void StartTask(void* pdata)
{
  INT8U err;
  INT8U perr;
  
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */
  
  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000; 
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
 
  if (alt_alarm_start (&alarm, delay, alarm_handler, context) < 0)
      {
          printf("No system clock available!n");
      }

  /* 
   * Create and start Software Timer 
   */
   
   Timer = OSTmrCreate (
                        0,          // Initial delay used by the timer
                        3,     // Period
                        OS_TMR_OPT_PERIODIC,        // Automatically reload
                        callbackVehicle,        // Callback function
                        NULL,      // Argument of callback function
                        NULL,     // Name of timer
                        &err);
                                
    overload_timer = OSTmrCreate (
                        0,          // Initial delay used by the timer
                        6,     // Period
                        OS_TMR_OPT_PERIODIC,        // Automatically reload
                        callbackOverload,        // Callback function
                        NULL,      // Argument of callback function
                        NULL,     // Name of timer
                        &err);
                        
    start = OSTmrStart (overload_timer, &perr);                                
    start = OSTmrStart (Timer, &perr);                 
                        

  /*
   * Creation of Kernel Objects
   */
  
  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  
  //Semaphores
  Semaphore1 = OSSemCreate(0);      // For controlling Vehicle task
  Semaphore2 = OSSemCreate(0);      // For controlling Control task
  
   
  /*
   * Create statistics task
   */

  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */


  err = OSTaskCreateExt(
       ControlTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       CONTROLTASK_PRIO,
       CONTROLTASK_PRIO,
       (void *)&ControlTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
       VehicleTask, // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       VEHICLETASK_PRIO,
       VEHICLETASK_PRIO,
       (void *)&VehicleTask_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);
  
  
  err = OSTaskCreateExt(
       ButtonIO,    // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &ButtonIO_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       BUTTONIO_PRIO,
       BUTTONIO_PRIO,
       (void *)&ButtonIO_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);
  
  
  err = OSTaskCreateExt(
       SwitchIO,    // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &SwitchIO_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       SWITCHIO_PRIO,
       SWITCHIO_PRIO,
       (void *)&SwitchIO_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);
  
  
  err = OSTaskCreateExt(
       Watchdog,    // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &Watchdog_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       WATCHDOG_PRIO,
       WATCHDOG_PRIO,
       (void *)&Watchdog_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);
       
       
  err = OSTaskCreateExt(
       Overload,    // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &Overload_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       OVERLOAD_PRIO,
       OVERLOAD_PRIO,
       (void *)&Overload_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);  
       
  err = OSTaskCreateExt(
       Extraload,    // Pointer to task code
       NULL,        // Pointer to argument that is
                    // passed to task
       &Extraload_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack
       EXTRALOAD_PRIO,
       EXTRALOAD_PRIO,
       (void *)&Extraload_Stack[0],
       TASK_STACKSIZE,
       (void *) 0,
       OS_TASK_OPT_STK_CHK);
  
  
  printf("All Tasks and Kernel Objects generated!\n");

  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void) {

  printf("Lab: Cruise Control\n");  
  
  //show_position_on_sevenseg(2);
 
  OSTaskCreateExt(
     StartTask, // Pointer to task code
         NULL,      // Pointer to argument that is
                    // passed to task
         (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
                             // of task stack 
         STARTTASK_PRIO,
         STARTTASK_PRIO,
         (void *)&StartTask_Stack[0],
         TASK_STACKSIZE,
         (void *) 0,  
         OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
         
  OSStart();
  
  return 0;
}
