 /** \file main.c
* \brief Sistema de tempo real a simular controlo de iluminação.
*
*  Este código permite a leitura do valor em A0, utilizando a ADC.
*  Existe uma primeira thread responsavel apenas por servir de relogio
*  Uma outra thread apenas faz a leitura da ADC
*  Outra thread faz uma filtragem (Média total).  
*  Utiliza o valor final para dar feedback ao controlador.
*  O controlador atua em uma nova thread
*  A sincronização é feita através de semáforos.
*  Existe uma ultima thread que usa o valor dado pelo controlador para controlar o duty cycle a aplicar ao led.
*
* \author Daniel Barra de Almeida        85111
* \author Marco  Antonio da Silva Santos 83192
* \date 25/06/2022
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

#include <console/console.h>

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
#define VECTOR_SIZE 100           /* Filter Local-Vector Size */


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
#define thread_Clock_prio 1
#define thread_In_prio 1
#define thread_Filter_prio 1
#define thread_Control_prio 1
#define thread_Out_prio 1


#define MODE_MANUAL 1
#define MODE_AUTO 2
#define AUTO_WORKMODE_ON 1
#define AUTO_WORKMODE_OFF 0




/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_Clock_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_In_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_Filter_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_Control_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_Out_stack, STACK_SIZE);

/* Create variables for thread data */
struct k_thread thread_Clock_data;
struct k_thread thread_In_data;
struct k_thread thread_Filter_data;
struct k_thread thread_Control_data;
struct k_thread thread_Out_data;

/* Create task IDs */
k_tid_t thread_Clock_tid;
k_tid_t thread_In_tid;
k_tid_t thread_Filter_tid;
k_tid_t thread_Control_tid;
k_tid_t thread_Out_tid;

/* Global vars (shared memory) */
int sharedmemory_1 = 0;
int sharedmemory_2 = 0;
int sharedmemory_3 = 0;
const struct device *gpio0_dev;         /* Pointer to GPIO device structure */
const struct device *pwm0_dev;          /* Pointer to PWM device structure */

/* Semaphores for task synch */
struct k_sem sem1; //Filter
struct k_sem sem2; //Control
struct k_sem sem3; //Output

/* Thread code prototypes */
void Clock(void *argA , void *argB, void *argC);
void Input(void *argA , void *argB, void *argC);
void Filter(void *argA , void *argB, void *argC);
void Control(void *argA , void *argB, void *argC);
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
const struct device *adc_dev = NULL;
static uint16_t adc_sample_buffer[BUFFER_SIZE];                     /*ADC ENDS*/

/* Int related declarations */                          /* PWM STARTS */
static struct gpio_callback but1_cb_data; /* Callback structure */


/* PWM configuration */
unsigned int pwmPeriod_us = 10000;       /* PWM priod in us */
unsigned int dcValue = 0;              /* Duty-cycle in % */
                                                        /* PWM ENDS   */
int objective = 0;
int ambience_light = 0;

/* More Global Vars */
//Clock
int segundos = 0;
int minutos = 0;
int horas = 0;
//Control
int setpoint = 1500;
int mode = MODE_AUTO;
int auto_workmode = AUTO_WORKMODE_OFF;
int start_hour = 1;
int end_hour = 4;
int end_minute = 0;
int workmode_flip = 0;

/* Takes one sample */
/** \brief  Amostragem da ADC.
*  Faz a leitura de uma amostra pela ADC.
*
* Esta função faz a leitura de uma amostra do sinal analógico
* presente na porta A0 da placa.(Valor ente 0 e 1023 V)
*/
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
    
    /* Test each button ...*/
    if(BIT(BOARDBUT1) & pins) {
       if(mode == MODE_AUTO){
        mode = MODE_MANUAL;
       }
       else{
        mode = MODE_AUTO;
       }
    }

    if(BIT(BOARDBUT2) & pins) {
      if(workmode_flip){
        workmode_flip = 0;
      }
      else{
        workmode_flip = 1;
      }

    }

    if(BIT(BOARDBUT3) & pins) {
      if(mode == MODE_MANUAL){
        /* Update global var*/        
        if (dcValue <= 95){
          dcValue = dcValue + 5;
        }
        else{
          dcValue = 0;
        }
      }
    }

    if(BIT(BOARDBUT4) & pins) {
      if(mode == MODE_MANUAL){
        /* Update global var*/
        if (dcValue >= 5){
          dcValue = dcValue - 5;
        }
        else{
          dcValue = 100;
        }        
      } 
    }
}

// CONFIG
/** \brief  Configuração.
*  Configura a interface da placa.
*
* Esta função faz as configurações necessárias
* para a interface da placa ser utilizada adequadamente.
*/
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

/** \brief  Função principal.
*  Inicio das threads.
*
* Esta é a funcao principal que logo de incio chama a funcção de configuração.
* São inicializados semaforos e criadas as Threads/Tasks para o funcionamento
* 
*/
void main(void)
{
   printk("\x1b[2J");  /* Clear screen */
   printk("\x1b[H");   // Send cursor to home
   config();
   
    
   /* Create and init semaphores */
    k_sem_init(&sem1, 0, 1);
    k_sem_init(&sem2, 0, 1);
    k_sem_init(&sem3, 0, 1);

    
    /* Create tasks */
    thread_Clock_tid = k_thread_create(&thread_Clock_data, thread_Clock_stack,
        K_THREAD_STACK_SIZEOF(thread_Clock_stack), Clock,
        NULL, NULL, NULL, thread_Clock_prio, 0, K_NO_WAIT);

    thread_In_tid = k_thread_create(&thread_In_data, thread_In_stack,
        K_THREAD_STACK_SIZEOF(thread_In_stack), Input,
        NULL, NULL, NULL, thread_In_prio, 0, K_NO_WAIT);

    thread_Filter_tid = k_thread_create(&thread_Filter_data, thread_Filter_stack,
        K_THREAD_STACK_SIZEOF(thread_Filter_stack), Filter,
        NULL, NULL, NULL, thread_Filter_prio, 0, K_NO_WAIT);
    
    thread_Control_tid = k_thread_create(&thread_Control_data, thread_Control_stack,
        K_THREAD_STACK_SIZEOF(thread_Control_stack), Control,
        NULL, NULL, NULL, thread_Control_prio, 0, K_NO_WAIT);

    thread_Out_tid = k_thread_create(&thread_Out_data, thread_Out_stack,
        K_THREAD_STACK_SIZEOF(thread_Out_stack), Output,
        NULL, NULL, NULL, thread_Out_prio, 0, K_NO_WAIT);
}

/** \brief  Clock Thread/Task.
*  Relógio do sistema.
*
* Esta thread apenas é responsavel por manter um relogio funcional no sistema
* de modo a se poderem programar periodos de tempo com funcionamentos diferentes.
* 
*/
void Clock(void *argA , void *argB, void *argC)
{
  int64_t time_stamp;
  int64_t release_time;
  while(1)
  {
    printk("Clock thread init\n");
    segundos = 0;
    minutos = 0;
    horas = 0;
    release_time = k_uptime_get() + 10;
    while(horas <= 24)
    {
      time_stamp = k_uptime_get();
      if( time_stamp < release_time)
      {
        k_msleep(release_time - time_stamp);
        release_time += 10;
        segundos++;               // segundos        
        if (segundos >= 60)
        {
          minutos++;             // minutos
          segundos = 0;
        }                                              
        if (minutos >= 60)
        {
          horas++;                // horas
          minutos = 0;
        }                                                                  
      }
      if (horas >= start_hour && (horas <= end_hour && minutos <= end_minute)){
        auto_workmode = AUTO_WORKMODE_ON;
        if (workmode_flip){
          auto_workmode = AUTO_WORKMODE_OFF;
        }
      }
      else{
        auto_workmode = AUTO_WORKMODE_OFF;
        if (workmode_flip){
          auto_workmode = AUTO_WORKMODE_ON;
        }
      }
    }
  }
}

/** \brief  Thread Input.
*  Funcionamento de aquisição da informação
*
* Leitura da ADC.
* Conversão da escala de 0 a 1023 para 0 a 3000 (correspondente de 0 a 3 V).
* Inserir o resultao da conversão na primeira memória partilhada.
* Aumentar o valor do primeiro semáforo (k_sem_give).
* Esperar pelo fim do periodo de amostragem.
* Repete.
* 
*/
void Input(void *argA , void *argB, void *argC)
{
    /* Timing variables to control task periodicity */
    int64_t fin_time=0, release_time=0;

    /* Other variables */
    int16_t input;
    int err=0;
    
    printk("\x1b[2J"); /* Clear screen */
    /* Compute next release instant */
    release_time = k_uptime_get() + SAMP_PERIOD_MS;

    
    /* Thread loop */
    while(1)
    {   
      //printk("Read thread init\n");
      printk("\x1b[H");   // Send cursor to home
      printf("\x1B[?25l");   // Hide cursor
      // printk("\x1b[2J");  /* Clear screen */
      if (mode == MODE_AUTO){
        if (auto_workmode == AUTO_WORKMODE_OFF){
          printk("Mode: AUTO State: OFF\x1b[0K\n");
        }
        else{
          printk("Mode: AUTO State: ON\x1b[0K\n");
        }
      }
      else{
        printk("Mode: MANUAL\x1b[0K\n");
      }
      printk("Setpoint: %d\x1b[0K\n",setpoint);

      /* Do the workload */
      err=adc_sample();
      if(err)
      {
          printk("adc_sample() failed with error code %d\n",err);
      }
      else
      {
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
    
      sharedmemory_1= input;
    
      k_sem_give(&sem1);

      /* Wait for next release instant */ 
      fin_time = k_uptime_get();
      if( fin_time < release_time) {
          k_msleep(release_time - fin_time);
          release_time += SAMP_PERIOD_MS;

      }
    }
}

/** \brief  Filter Thread.
*  Funcionamento de filtragem da informação
*
* Esperar pelo levantamento do semáforo (k_sem_take).
* Retirar o valor presente na memoria partilhada e adicioná-lo a um vetor local, retirando o mais antigo.
* Filtragem (Média total).
* Inserir resultado numa segunda memória partilhada.
* Aumentar o valor do segundo semáforo (k_sem_give).
* 
*/
void Filter(void *argA , void *argB, void *argC)
{
    /* Other variables */
    static int local_vect[VECTOR_SIZE] = {0,0,0,0,0};
    int sum;

    while(1) {
        //printk("Filter thread init\n");
        k_sem_take(&sem1,  K_FOREVER);
       
        for (int i = 0; i<VECTOR_SIZE-1; i++){ //Local_Vector[] <- Shared memory 1
          local_vect[i] = local_vect[i+1];

        }
        local_vect[VECTOR_SIZE-1] = sharedmemory_1;
        
        sum = 0;
        for (int i = 0; i<VECTOR_SIZE; i++){ // Media com todas as amostras
          sum = sum + local_vect[i];

        }
        printk("AVG_Total %d\x1b[0K\n",sum /VECTOR_SIZE);
        
        if(mode == MODE_AUTO){
          if (auto_workmode == AUTO_WORKMODE_ON){
             sharedmemory_2 = sum /VECTOR_SIZE;
             k_sem_give(&sem2);
          }
          else{
            dcValue = 0;
            k_sem_give(&sem3);
          }
         
        }
        if(mode == MODE_MANUAL){        
          k_sem_give(&sem3);
        }
            
  }
}

/** \brief  Control Thread.
*  Funcionamento de controlador PI
*
* Esperar pelo levantamento do semáforo (k_sem_take).
* Retirar o valor presente na segunda memoria partilhada e usalo como valor de feedback do sistema.
* Calculo do erro e do novo valor de entrada correspondente atraves de um controlador PI
* Inserir resultado numa terceira memória partilhada.
* Aumentar o valor do terceiro semáforo (k_sem_give).
* 
*/
void Control(void *argA , void *argB, void *argC)
{
  float Ki=0.005;
  float Kp=0.5;
  int input=0;
  int error=0;
  int integral_error=0;
  int output=0;
   
  while(1)
  {
    //printk("Control thread init\n");
    k_sem_take(&sem2,  K_FOREVER);
    input = sharedmemory_2;
    error = (setpoint-input)/30;

    output = (Kp*error + Ki*integral_error);    // New Duty Cycle value to achieve the desired 

    // Anti-windup -> Limits the effect of I

    if(((output >= 100) || (output <= 0))  && (((error >= 0) && (integral_error >= 0)) || ((error < 0) && (integral_error < 0))))
    {
      integral_error = integral_error;
    }
    else
    {
      integral_error = integral_error + error;
    }
    
    // Limit output
    if (output > 100)
    {
        output = 100;
    }
    if (output < 0)
    {
        output = 0;
    }

    printk("Controlled Duty_Cycle %d\x1b[0K\n",output);
    sharedmemory_3 = output;

    k_sem_give(&sem3); 
    
  }
      

}

/** \brief  Thread Output.
*  Funcionamento de saída da informação.
*
* Esperar o levantamento do terceiro semáforo (k_sem_take).
* Ler o valor da terceira memória partilhada.
* Mostrar no terminal os valores atualizados de controlo e o tempo de funcionamento.
* Ajustar o duty-cycle do PWM aplicado ao LED1.
* Repete.
* 
*/
void Output(void *argA , void *argB, void *argC)
{
    /* Local variables */
    int ret = 0;
    while(1)
    {
        //printk("Output thread init\n");

        k_sem_take(&sem3,  K_FOREVER);
        if (mode == MODE_AUTO){
          if (auto_workmode == AUTO_WORKMODE_ON){
            dcValue = sharedmemory_3;
          }
          else{
            dcValue = 0;
          }
        }
        
        
        //dcValue = (100*output)/3000;
        //printk("Output = %d\x1b[0K\n", output);
        printk("PWM DC value set to %u %%\x1b[0K\n",dcValue);

        ret = pwm_pin_set_usec(pwm0_dev, BOARDLED_OUT, pwmPeriod_us,(unsigned int)((pwmPeriod_us*(100-dcValue))/100), PWM_POLARITY_NORMAL);

        if (ret)
        {
            printk("Error %d: failed to set pulse width\n", ret);
        }
        
        printk("Time-> %d h %d min %d sec \x1b[0K\n",horas, minutos, segundos);
        printk("\x1b[0J"); // Erase from cursor to end of screen
        
    }
}