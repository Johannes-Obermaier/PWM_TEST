
#include "driver/gpio.h" 
#include "esp_log.h"
#include "driver/mcpwm.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"   // Software timer related API prototypes.
#include "soc/mcpwm_periph.h"
#include "math.h"
#include "esp_timer.h"


//H=High= (P-kanal Mosfet)
//L=Low= (N-kanal Mosfet)
//Pins rechts: Phase A: IO39, IO40; Phase B: IO37, IO38; Phase C: IO35,IO36
//Pins links:  Phase A: IO7,IO6;    Phase B: IO9,IO8;    Phase C: IO11,IO10


#define MOTOR_LEFT_A_H 6        //6 = L_A2
#define MOTOR_LEFT_A_L 7        //7 = L_A1
#define MOTOR_LEFT_B_H 8        //8 = L_B2
#define MOTOR_LEFT_B_L 9        //9 = L_B1
#define MOTOR_LEFT_C_H 10       //10 = L_C2
#define MOTOR_LEFT_C_L 11       //11 = L_C1

#define MOTOR_RIGHT_A_H 40      //40 = R_A2
#define MOTOR_RIGHT_A_L 39      //39 = R_A1  
#define MOTOR_RIGHT_B_H 38      //38 = R_B2 
#define MOTOR_RIGHT_B_L 37      //37 = R_B1
#define MOTOR_RIGHT_C_H 36      //36 = R_C2
#define MOTOR_RIGHT_C_L 35      //35 = R_C1



#define LED_BLUE 4

#define MAIN_PERIOD_MS           pdMS_TO_TICKS(1)
#define PI 3.14159265358979323846
#define SIN_TABLE_SIZE 6
float sinusTable[SIN_TABLE_SIZE];
int tableIndex_A = 0;
int tableIndex_B = 0;
int tableIndex_C = 0;



#define PWM_FREQ_HZ_MOTOR 20000
#define PWM_DEADTIME_MOTOR 3


void pwm_init()


{   // ------------------- GPIO -------------------
    //Configure Motor left as output
    
    gpio_pad_select_gpio(MOTOR_LEFT_A_H);
    gpio_pad_select_gpio(MOTOR_LEFT_A_L);
    gpio_pad_select_gpio(MOTOR_LEFT_B_H);
    gpio_pad_select_gpio(MOTOR_LEFT_B_L);
    gpio_pad_select_gpio(MOTOR_LEFT_C_H);
    gpio_pad_select_gpio(MOTOR_LEFT_C_L);
    gpio_pad_select_gpio(LED_BLUE);
    
    gpio_set_direction(MOTOR_LEFT_A_H, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_LEFT_A_L, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_LEFT_B_H, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_LEFT_B_L, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_LEFT_C_H, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_LEFT_C_L, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE, GPIO_MODE_OUTPUT);



    //gpio_set_level(MOTOR_LEFT_A_H, 1);


     //Configure Pins for Motor left for MCPWM
    mcpwm_pin_config_t gpio_config_left = {
        .mcpwm0a_out_num = MOTOR_LEFT_A_H,
        .mcpwm0b_out_num = MOTOR_LEFT_A_L,
        .mcpwm1a_out_num = MOTOR_LEFT_B_H,
        .mcpwm1b_out_num = MOTOR_LEFT_B_L,
        .mcpwm2a_out_num = MOTOR_LEFT_C_H,
        .mcpwm2b_out_num = MOTOR_LEFT_C_L,
    };
    mcpwm_set_pin(MCPWM_UNIT_0,&gpio_config_left);
    
   
    //Configure Motor right as output
    gpio_pad_select_gpio(MOTOR_RIGHT_A_H);
    gpio_pad_select_gpio(MOTOR_RIGHT_A_L);
    gpio_pad_select_gpio(MOTOR_RIGHT_B_H);
    gpio_pad_select_gpio(MOTOR_RIGHT_B_L);
    gpio_pad_select_gpio(MOTOR_RIGHT_C_H);
    gpio_pad_select_gpio(MOTOR_RIGHT_C_L);
    
    gpio_set_direction(MOTOR_RIGHT_A_H, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_RIGHT_A_L, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_RIGHT_B_H, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_RIGHT_B_L, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_RIGHT_C_H, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_RIGHT_C_L, GPIO_MODE_OUTPUT);
    
    //Configure Pins for Motor right for MCPWM
    mcpwm_pin_config_t gpio_config_right = {
        .mcpwm0a_out_num = MOTOR_RIGHT_A_H,
        .mcpwm0b_out_num = MOTOR_RIGHT_A_L,
        .mcpwm1a_out_num = MOTOR_RIGHT_B_H,
        .mcpwm1b_out_num = MOTOR_RIGHT_B_L,
        .mcpwm2a_out_num = MOTOR_RIGHT_C_H,
        .mcpwm2b_out_num = MOTOR_RIGHT_C_L,
    };
    mcpwm_set_pin(MCPWM_UNIT_1,&gpio_config_right);   

    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LO1);    Use GPIO 12 for MCPWM0A ?????????
    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, HO1);    Use GPIO 13 for MCPWM0B ?????????

    
    // ------------------- Timer -------------------            //https://yopiediy.xyz/esp32-mcpwm-as-spwm-generator/
    // Configuration parameters of Motor PWM Timer
    // The same timer is used for both Motors


    mcpwm_config_t pwm_config;                              //Struktur mit Name pwm_config, in der Konfigurationen gespeichert sind
    pwm_config.frequency = PWM_FREQ_HZ_MOTOR;               //Frequenz des PWM Signals
    pwm_config.counter_mode = MCPWM_UP_COUNTER;        //Zähler ?
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;               //aktiv hohes PWM SIgnal
    pwm_config.cmpr_a = 0.0;                                  //Duty-Cycle zum Anfang auf 0% setzen
    pwm_config.cmpr_b = 0.0;                                  //Duty-Cycle zum Anfang auf 0% setzen

    mcpwm_init(MCPWM_UNIT_0,MCPWM_TIMER_0, &pwm_config);    //Initialisierung mit den oberen Parametern
    mcpwm_init(MCPWM_UNIT_0,MCPWM_TIMER_1, &pwm_config);                                                      //wo kommt MCPWM_TIMER_0 her?
    mcpwm_init(MCPWM_UNIT_0,MCPWM_TIMER_2, &pwm_config);    //Initialisierung mit den oberen Parameternnnn
    mcpwm_init(MCPWM_UNIT_1,MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1,MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1,MCPWM_TIMER_2, &pwm_config);


 mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_LOW_MODE, PWM_DEADTIME_MOTOR, PWM_DEADTIME_MOTOR);
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_ACTIVE_LOW_MODE, PWM_DEADTIME_MOTOR, PWM_DEADTIME_MOTOR);
   mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_ACTIVE_LOW_MODE, PWM_DEADTIME_MOTOR, PWM_DEADTIME_MOTOR);
    mcpwm_deadtime_enable(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_ACTIVE_LOW_MODE, PWM_DEADTIME_MOTOR, PWM_DEADTIME_MOTOR);
     mcpwm_deadtime_enable(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_ACTIVE_LOW_MODE, PWM_DEADTIME_MOTOR, PWM_DEADTIME_MOTOR);
      mcpwm_deadtime_enable(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_ACTIVE_LOW_MODE, PWM_DEADTIME_MOTOR, PWM_DEADTIME_MOTOR);

 //mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_MODE, 200, 200);
// mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_RED_FED_FROM_PWMXA, 200, 200);

   // ESP_LOGI(TAG, "Motor init done.");
    
    
    printf("Motor_init done\n");
    
}


void motor_control_left(float duty_cycle_left_A, float duty_cycle_left_B,float duty_cycle_left_C) {
    // PWM-Duty-Cycle-Werte für linken Motor
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle_left_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle_left_A);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_cycle_left_B);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, duty_cycle_left_B);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, duty_cycle_left_C);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, duty_cycle_left_C);
}
void motor_control_right(float duty_cycle_right_A, float duty_cycle_right_B, float duty_cycle_right_C) {
    // PWM-Duty-Cycle-Werte für rechten Motor
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle_right_A);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle_right_A);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, duty_cycle_right_B);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, duty_cycle_right_B);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A, duty_cycle_right_C);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_B, duty_cycle_right_C);
}


void generateSinusTable() {

    
    for (int i = 0; i < SIN_TABLE_SIZE; i++) {
        float angle = (2 * PI * i) / SIN_TABLE_SIZE;
        
        sinusTable[i] = 50.0 * (1.0 + sin(angle)); 
        //vTaskDelay(pdMS_TO_TICKS(2));
        //printf("%d angle: %f sinustable: %f \n", i, angle, sinusTable[i]);
    
    }
    printf("sinustable created\n");
}


// creat Main Task

static void vMainTaskCallback( TimerHandle_t xTimer )
{
     //Werte aus der Sinustabelle
        float left_motor_speed_A = sinusTable[tableIndex_A];
        float left_motor_speed_B = sinusTable[tableIndex_B];
        float left_motor_speed_C = sinusTable[tableIndex_C];
        
        float right_motor_speed_A = sinusTable[tableIndex_A];
    	float right_motor_speed_B = sinusTable[tableIndex_B];
        float right_motor_speed_C = sinusTable[tableIndex_C];

       // printf("Phase A: %d und %f \n", tableIndex_A, sinusTable[tableIndex_A]);
      //  printf("Phase B: %d und %f \n", tableIndex_B, sinusTable[tableIndex_B]);
       // printf("Phase C: %d und %f \n", tableIndex_C, sinusTable[tableIndex_C]);
        
        motor_control_left(left_motor_speed_A, left_motor_speed_B, left_motor_speed_C);
        motor_control_right(right_motor_speed_A , right_motor_speed_B, right_motor_speed_C);

        //motor_control_right(0 , 0, 0);
        //motor_control_left(0, 0, 0);
        //mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50);
        //mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 50);
        
        tableIndex_A = (tableIndex_A + 1);
        tableIndex_B = (tableIndex_B + 1);
        tableIndex_C = (tableIndex_C + 1);

       if (tableIndex_A > SIN_TABLE_SIZE) {
         tableIndex_A = 0;
        }
          if (tableIndex_B > SIN_TABLE_SIZE) {
         tableIndex_B = 0;
        }
          if (tableIndex_C > SIN_TABLE_SIZE) {
         tableIndex_C = 0;
        }



}




void app_main() {

    // Initialisierung PWM für Motoren
    pwm_init();
    generateSinusTable();
    gpio_set_level(LED_BLUE, 1);
    
     tableIndex_A = 0;
     tableIndex_B = SIN_TABLE_SIZE/3;
     tableIndex_C = tableIndex_B*2;

    TimerHandle_t xMainTimer = xTimerCreate(    ( const char * ) "MainTask",
                                                MAIN_PERIOD_MS,
                                                pdTRUE, // periodic timer, so xAutoReload is set to pdTRUE.
                                                ( void * ) 0,
                                                vMainTaskCallback
                                        );

    // Start the Task timer
    xTimerStart( xMainTimer, 0 );



    while(1){
        
       vTaskDelay(pdMS_TO_TICKS(1));
    }
        
    
    
}