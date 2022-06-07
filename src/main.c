/*
 * Paulo Pedreiras, 2022/03
 * Simple ADC example - nRF52840DK_nRF52840 board
 * 
 * 
 * Periodically reads ADC input and prints the corresponding raw and voltage value in the console
 * 
 * Adapted from: 
 *            https://devzone.nordicsemi.com/f/nordic-q-a/80685/using-saadc-in-nrf-connect-sdk/334204#334204
 *            https://github.com/simon-iversen/sdk-nrf/blob/light_controller/samples/light_controller/src/main.c
 * 
 *
 *      HW info
 *          https://infocenter.nordicsemi.com/topic/struct_nrf52/struct/nrf52840.html
 *          Section: nRF52840 Product Specification -> Peripherals -> GPIO / GPIOTE and SAADC 
 *          Board specific HW info can be found in the nRF52840_DK_User_Guide_20201203. I/O pins available at pg 27
 *
 *      Peripheral libs
 *          https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/reference/peripherals/index.html
 *          ADC     
 *
 *      NOTE 1:     **** NEVER APPLY MORE THAN 3 V (VDD) or negative voltages to ANx ****
 *
 *      NOTE 2:     In this board analog inputs are AIN{1,2,4,5,6,7} (see nRF52840_DK_User_Guide_20201203, page 28)
 *                  Some of the examples found in the internet are set to AIN0 and so do not work. The PCB also has a label "A0", which refers to Arduino and induces in error. 
 *    
 *      NOTE 3:     must add "CONFIG_ADC=y" to prj.conf
 * 
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <drivers/pwm.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>
#include <timing/timing.h>
#include <stdlib.h>
#include <stdio.h>

/*ADC definitions and includes*/                                    /*ADC STARTS*/
#include <hal/nrf_saadc.h>
#define ADC_NID DT_NODELABEL(adc) 
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1  

/* This is the actual nRF ANx input to use. Note that a channel can be assigned to any ANx. In fact a channel can */
/*    be assigned to two ANx, when differential reading is set (one ANx for the positive signal and the other one for the negative signal) */  
/* Note also that the configuration of differnt channels is completely independent (gain, resolution, ref voltage, ...) */
#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1 

#define BUFFER_SIZE 1
#define VECTOR_SIZE 10           /* Filter Local-Vector Size */


/* Other defines */
#define TIMER_INTERVAL_MSEC 1000 /* Interval between ADC samples */ 
                                                                    /*ADC ENDS*/

/* PWM !!! */
#define GPIO0_NID DT_NODELABEL(gpio0)                    /* PWM STARTS */
#define PWM0_NID DT_NODELABEL(pwm0) 
#define BOARDLED_OUT DT_PROP(PWM0_NID, ch0_pin)          
#define BOARDBUT1 0xB   /* P0.11  */
#define BOARDBUT2 0xC   /* P0.12  */
#define BOARDBUT3 0x18  /* P0.24  */
#define BOARDBUT4 0x19  /* P0.25  */
                                                        /* PWM ENDS   */

/* Thread periodicity (in ms)*/
#define SAMP_PERIOD_MS 20     //---------------------SAMPLING PERIOD----------------------   
                                    
/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

/* Thread scheduling priority */
#define thread_In_prio 1
#define thread_Filter_prio 1
#define thread_Out_prio 1

/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_In_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_Filter_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_Out_stack, STACK_SIZE);

/* Create variables for thread data */
struct k_thread thread_In_data;
struct k_thread thread_Filter_data;
struct k_thread thread_Out_data;

/* Create task IDs */
k_tid_t thread_In_tid;
k_tid_t thread_Filter_tid;
k_tid_t thread_Out_tid;

/* Global vars (shared memory) */
int sm_1 = 0;
int sm_2 = 0;
const struct device *gpio0_dev;         /* Pointer to GPIO device structure */
const struct device *pwm0_dev;          /* Pointer to PWM device structure */

/* Semaphores for task synch */
struct k_sem sem1;
struct k_sem sem2;

/* Thread code prototypes */
void Input(void *argA , void *argB, void *argC);
void Filter(void *argA , void *argB, void *argC);
void Output(void *argA , void *argB, void *argC);

/* ADC channel configuration */
static const struct adc_channel_cfg my_channel_cfg = {              /*ADC STARTS*/
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID,
	.input_positive = ADC_CHANNEL_INPUT
};

/* Global vars */
struct k_timer my_timer;
const struct device *adc_dev = NULL;
static uint16_t adc_sample_buffer[BUFFER_SIZE];                     /*ADC ENDS*/

/* Int related declarations */                          /* PWM STARTS */
static struct gpio_callback but1_cb_data; /* Callback structure */

/* Callback function and variables*/
volatile int dcToggleFlag1 = 0; /* Flag to signal a BUT1 press */ 

/* PWM configuration */
unsigned int pwmPeriod_us = 1000;       /* PWM priod in us */
unsigned int dcValue = 33;              /* Duty-cycle in % */
                                                        /* PWM ENDS   */



/* Takes one sample */
static int adc_sample(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	if (adc_dev == NULL) {
            printk("adc_sample(): error, must bind to adc first \n\r");
            return -1;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret) {
            printk("adc_read() failed with code %d\n", ret);
	}	

	return ret;
}

// ISR
void but1press_cbfunction(const struct device *dev, struct gpio_callback *cb, uint32_t pins){
    
    /* Update Flag*/
    dcToggleFlag1 = 1;
}

// CONFIG
void config(void){
    int err=0;

    /* ADC setup: bind and initialize */
    adc_dev = device_get_binding(DT_LABEL(ADC_NID));
	if (!adc_dev) {
        printk("ADC device_get_binding() failed\n");
    } 
    err = adc_channel_setup(adc_dev, &my_channel_cfg);
    if (err) {
        printk("adc_channel_setup() failed with error code %d\n", err);
    }
    
    /* It is recommended to calibrate the SAADC at least once before use, and whenever the ambient temperature has changed by more than 10 ?C */
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
    
    /* Bind to GPIO 0 and PWM0 */
    gpio0_dev = device_get_binding(DT_LABEL(GPIO0_NID));
    if (gpio0_dev == NULL) {
        printk("Error: Failed to bind to GPIO0\n\r");        
	return;
    }
    else {
        printk("Bind to GPIO0 successfull \n\r");        
    }

    pwm0_dev = device_get_binding(DT_LABEL(PWM0_NID));
    if (pwm0_dev == NULL) {
	printk("Error: Failed to bind to PWM0\n");
	return;
    }
    else  {
        printk("Bind to PWM0 successful\n");
    }
    int ret = 0;
    
    /* Configure PIN --------------------------------------------------------------------PIN-CONFIGS-------------------------------------------------*/
    ret = gpio_pin_configure(gpio0_dev, BOARDBUT1, GPIO_INPUT | GPIO_PULL_UP); // But 1
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 1 \n\r", ret);
        return;
    }

    ret = gpio_pin_configure(gpio0_dev, BOARDBUT2, GPIO_INPUT | GPIO_PULL_UP); // But 2
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 2 \n\r", ret);
        return;
    }

    ret = gpio_pin_configure(gpio0_dev, BOARDBUT3, GPIO_INPUT | GPIO_PULL_UP); // But 3
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 3 \n\r", ret);
        return;
    }

    ret = gpio_pin_configure(gpio0_dev, BOARDBUT4, GPIO_INPUT | GPIO_PULL_UP); // But 4
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 4 \n\r", ret);
        return;
    }
      /* Set interrupt HW - which pin and event generate interrupt -----------------------------INTERRUPT----------------------------------------------*/
    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT1, GPIO_INT_EDGE_TO_ACTIVE); // BUT 1
    if (ret != 0) {
        printk("Error %d: failed to configure interrupt on BUT1 pin \n\r", ret);
        return;
                
    }
    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT2, GPIO_INT_EDGE_TO_ACTIVE);  // BUT 2
    if (ret != 0) {
        printk("Error %d: failed to configure interrupt on BUT2 pin \n\r", ret);
        return;
    }

    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT3, GPIO_INT_EDGE_TO_ACTIVE);  // BUT 3
    if (ret != 0) {
        printk("Error %d: failed to configure interrupt on BUT3 pin \n\r", ret);
        return;
    }

    ret = gpio_pin_interrupt_configure(gpio0_dev, BOARDBUT4, GPIO_INT_EDGE_TO_ACTIVE); // BUT 4
    if (ret != 0) {
        printk("Error %d: failed to configure interrupt on BUT4 pin \n\r", ret);
        return;
    }
    /* Set callback -----------------------------------------------------------------------CALLBACK------------------------------------------------*/
    gpio_init_callback(&but1_cb_data, but1press_cbfunction, BIT(BOARDBUT1)| BIT(BOARDBUT2)| BIT(BOARDBUT3) | BIT(BOARDBUT4));
    gpio_add_callback(gpio0_dev, &but1_cb_data);
}

void main(void)
{
   config();

   /* Create and init semaphores */
    k_sem_init(&sem1, 0, 1);
    k_sem_init(&sem2, 0, 1);
    
    /* Create tasks */
    thread_In_tid = k_thread_create(&thread_In_data, thread_In_stack,
        K_THREAD_STACK_SIZEOF(thread_In_stack), Input,
        NULL, NULL, NULL, thread_In_prio, 0, K_NO_WAIT);

    thread_Filter_tid = k_thread_create(&thread_Filter_data, thread_Filter_stack,
        K_THREAD_STACK_SIZEOF(thread_Filter_stack), Filter,
        NULL, NULL, NULL, thread_Filter_prio, 0, K_NO_WAIT);

    thread_Out_tid = k_thread_create(&thread_Out_data, thread_Out_stack,
        K_THREAD_STACK_SIZEOF(thread_Out_stack), Output,
        NULL, NULL, NULL, thread_Out_prio, 0, K_NO_WAIT);

    /* Main loop
    while(true){
        // Get one sample, checks for errors and prints the values 
        err=adc_sample();
        if(err) {
            printk("adc_sample() failed with error code %d\n\r",err);
        }
        else {
            if(adc_sample_buffer[0] > 1023) {
                printk("adc reading out of range\n\r");
            }
            else {
                // ADC is set to use gain of 1/4 and reference VDD/4, so input range is 0...VDD (3 V), with 10 bit resolution 
                printk("adc reading: raw:%4u / %4u mV: \n\r",adc_sample_buffer[0],(uint16_t)(1000*adc_sample_buffer[0]*((float)3/1023)));
            }
        }

        // Sleep a while ... 
        k_msleep(TIMER_INTERVAL_MSEC);
        
    }*/
}

void Input(void *argA , void *argB, void *argC)
{
    /* Timing variables to control task periodicity */
    int64_t fin_time=0, release_time=0;

    /* Other variables */
    int16_t input;
    int err=0;
    
    printk("Input (task A) thread init (periodic)\n");
    printk("\x1b[2J"); /* Clear screen */
    /* Compute next release instant */
    release_time = k_uptime_get() + SAMP_PERIOD_MS;

    
    /* Thread loop */
    while(1) {
        
        printk("\x1b[H");   // Send cursor to home
        printf("\x1B[?25l");   // Hide cursor
        // printk("\x1b[2J");  /* Clear screen */
    
        /* Do the workload */
        err=adc_sample();
        if(err) {
            printk("adc_sample() failed with error code %d\n",err);
        }
        else {
            if(adc_sample_buffer[0] > 1023) {
                printk("adc reading out of range\n");
            }
            else {
                input = (uint16_t)(1000*adc_sample_buffer[0]*((float)3/1023));
                printk("Actual Sample: %d\x1b[0K\n", input);
                
                /* ADC is set to use gain of 1/4 and reference VDD/4, so input range is 0...VDD (3 V), with 10 bit resolution */
                //printk("adc reading: raw:%4u / %4u mV: \n\r",adc_sample_buffer[0],(uint16_t)(1000*adc_sample_buffer[0]*((float)3/1023)));
            }
        }
        
        sm_1= input;
        
        k_sem_give(&sem1);

        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += SAMP_PERIOD_MS;

        }
    }
}

void Filter(void *argA , void *argB, void *argC)
{
    /* Other variables */
    static int local_vect[VECTOR_SIZE] = {0,0,0,0,0};
    int avg_total;
    static int avg_rem;
    int sum;
    int remaining_samples;

    while(1) {
        k_sem_take(&sem1,  K_FOREVER);
       
        for (int i = 0; i<VECTOR_SIZE-1; i++){ //Local_Vector[] <- Shared memory 1
          local_vect[i] = local_vect[i+1];

        }
        local_vect[VECTOR_SIZE-1] = sm_1;
        
        sum = 0;
        for (int i = 0; i<VECTOR_SIZE; i++){ // Media com todas as amostras
          sum = sum + local_vect[i];

        }
        avg_total = sum /VECTOR_SIZE;
        printk("AVG_Total %d\x1b[0K\n",avg_total);

        sum = 0;
        remaining_samples = 0;
        for (int i = 0; i<VECTOR_SIZE; i++){ // Media sem outliars
          if (local_vect[i] < 300+avg_total && local_vect[i] > 300-avg_total){
            sum = sum + local_vect[i];

            remaining_samples++;
          }
        }
        printk("Remaining Samples Consideradas -> %d\x1b[0K\n",remaining_samples);

        if (remaining_samples>0){
          avg_rem = sum/remaining_samples;
        }
        else{
          printk("ALL OUTLIAR\x1b[0K\n");
        }
        sm_2=avg_rem;

        k_sem_give(&sem2);    
  }
}

void Output(void *argA , void *argB, void *argC)
{
    /* Other variables */
    int output;
    int ret = 0;


    while(1) {
        k_sem_take(&sem2,  K_FOREVER);
        output= sm_2;
        dcValue = (100*output)/3000;
        printk("Output = %d\x1b[0K\n", output);
        printk("PWM DC value set to %u %%\x1b[0K\n",dcValue);

        ret = pwm_pin_set_usec(pwm0_dev, BOARDLED_OUT,
                  pwmPeriod_us,(unsigned int)((pwmPeriod_us*dcValue)/100), PWM_POLARITY_NORMAL);
        if (ret) {
            printk("Error %d: failed to set pulse width\n", ret);
        }
        if (dcToggleFlag1){
            printk("But 1 pressed!");
            dcToggleFlag1 = 0;
        }
        printk("\x1b[0J"); // Erase from cursor to end of screen
        
  }
}