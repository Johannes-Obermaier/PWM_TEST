
#include "driver/gpio.h" 
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "freertos/timers.h"   // Software timer related API prototypes.
#include "esp_timer.h"
// #include "soc/mcpwm_periph.h"
#include "math.h"

static const char *TAG = "MCPWM";

//H=High= (P-kanal Mosfet)
//L=Low= (N-kanal Mosfet)
//Pins rechts: Phase A: IO39, IO40; Phase B: IO37, IO38; Phase C: IO35,IO36
//Pins links:  Phase A: IO7,IO6;    Phase B: IO9,IO8;    Phase C: IO11,IO10


#define MOTOR_LEFT_A_PMOS 6        //6 = L_A2
#define MOTOR_LEFT_A_NMOS 7        //7 = L_A1
#define MOTOR_LEFT_B_PMOS 8        //8 = L_B2
#define MOTOR_LEFT_B_NMOS 9        //9 = L_B1
#define MOTOR_LEFT_C_PMOS 10       //10 = L_C2
#define MOTOR_LEFT_C_NMOS 11       //11 = L_C1

#define MOTOR_RIGHT_A_PMOS 40      //40 = R_A2
#define MOTOR_RIGHT_A_NMOS 39      //39 = R_A1  
#define MOTOR_RIGHT_B_PMOS 38      //38 = R_B2 
#define MOTOR_RIGHT_B_NMOS 37      //37 = R_B1
#define MOTOR_RIGHT_C_PMOS 36      //36 = R_C2
#define MOTOR_RIGHT_C_NMOS 35      //35 = R_C1

#define BLDC_MCPWM_OP_INDEX_A     0
#define BLDC_MCPWM_OP_INDEX_B     1
#define BLDC_MCPWM_OP_INDEX_C     2
#define BLDC_MCPWM_GEN_INDEX_PMOS 0
#define BLDC_MCPWM_GEN_INDEX_NMOS  1



#define LED_BLUE 4

#define MAIN_PERIOD_MS           pdMS_TO_TICKS( 1 )
#define PI 3.14159265358979323846
#define SIN_TABLE_SIZE 99
uint32_t sinusTable[SIN_TABLE_SIZE];
int tableIndex_A = 0;
int tableIndex_B = 0;
int tableIndex_C = 0;



#define BLDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BLDC_MCPWM_PERIOD              500      // 50us, 20KHz
#define BLDC_SPEED_UPDATE_PERIOD_US    250   // 250us, 4kHz


// comparators for motor PWM signal
mcpwm_cmpr_handle_t comparators_left[3];
mcpwm_cmpr_handle_t comparators_right[3];


static void update_motor_pwm_cb()
{
    //Werte aus der Sinustabelle
    float left_motor_speed_A = sinusTable[tableIndex_A];
    float left_motor_speed_B = sinusTable[tableIndex_B];
    float left_motor_speed_C = sinusTable[tableIndex_C];
        
    float right_motor_speed_A = sinusTable[tableIndex_A];
    float right_motor_speed_B = sinusTable[tableIndex_B];
    float right_motor_speed_C = sinusTable[tableIndex_C];

    // Führe eine Verzögerung aus (optional, um die Geschwindigkeit zu steuern)
       
    // vTaskDelay(pdMS_TO_TICKS(1));
    //vTaskDelay(pdMS_TO_TICKS(70));
        
    // printf("Phase A: %d und %f \n", tableIndex_A, sinusTable[tableIndex_A]);
    //printf("Phase B: %d und %f \n", tableIndex_B, sinusTable[tableIndex_B]);
    //printf("Phase C: %d und %f \n", tableIndex_C, sinusTable[tableIndex_C]);
    

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators_left[0], left_motor_speed_A));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators_left[1], left_motor_speed_B));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators_left[2], left_motor_speed_C));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators_right[0], right_motor_speed_A));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators_right[1], right_motor_speed_B));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators_right[2], right_motor_speed_C));
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50);
    //mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 50);
        

    tableIndex_A = (tableIndex_A + 1) % SIN_TABLE_SIZE;
    tableIndex_B = (tableIndex_B + 1) % SIN_TABLE_SIZE;
    tableIndex_C = (tableIndex_C + 1) % SIN_TABLE_SIZE;

}

void pwm_init(){   
    // ------------------- Timer -------------------            //https://yopiediy.xyz/esp32-mcpwm-as-spwm-generator/
    // Configuration parameters of Motor PWM Timer
    // The same timer is used for both Motors

    // TODO: PCB Design: It would be good if we could deactivate all MOSFETS with one pin

    ESP_LOGI(TAG, "Create MCPWM timer");
    mcpwm_timer_handle_t timer = NULL;
    // TODO: is it possible to use the timer of group_id 0 with a comparator of group_id 1?
    // TODO: if there is a problem with the motor on the right, this could be the reason!!!
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = BLDC_MCPWM_TIMER_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = BLDC_MCPWM_PERIOD,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));


    ESP_LOGI(TAG, "Create MCPWM operator");
    mcpwm_oper_handle_t operators_left[3];
    mcpwm_oper_handle_t operators_right[3];
    mcpwm_operator_config_t operator_config_left = {
        .group_id = 0,
    };
    mcpwm_operator_config_t operator_config_right = {
        .group_id = 1,
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config_left, &operators_left[i]));
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config_right, &operators_right[i]));
    }

    ESP_LOGI(TAG, "Connect operators to the same timer");
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators_left[i], timer));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators_right[i], timer));
    }


    ESP_LOGI(TAG, "Create comparators");
    mcpwm_comparator_config_t compare_config = {
        .flags.update_cmp_on_tez = true, // update the compare threshold when the timer counts to zero.
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_new_comparator(operators_left[i], &compare_config, &comparators_left[i]));
        ESP_ERROR_CHECK(mcpwm_new_comparator(operators_right[i], &compare_config, &comparators_right[i]));
        // set compare value to 0, we will adjust the speed in a period timer callback
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators_left[i], 0));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators_right[i], 0));
    }


    ESP_LOGI(TAG, "Create PWM generators");
    mcpwm_gen_handle_t generators_left[3][2] = {};
    mcpwm_gen_handle_t generators_right[3][2] = {};
    mcpwm_generator_config_t gen_config_left = {};
    mcpwm_generator_config_t gen_config_right = {};
    const int gen_gpios_left[3][2] = {
        {MOTOR_LEFT_A_PMOS, MOTOR_LEFT_A_NMOS},
        {MOTOR_LEFT_B_PMOS, MOTOR_LEFT_B_NMOS},
        {MOTOR_LEFT_C_PMOS, MOTOR_LEFT_C_NMOS},
    };
    const int gen_gpios_right[3][2] = {
        {MOTOR_RIGHT_A_PMOS, MOTOR_RIGHT_A_NMOS},
        {MOTOR_RIGHT_B_PMOS, MOTOR_RIGHT_B_NMOS},
        {MOTOR_RIGHT_C_PMOS, MOTOR_RIGHT_C_NMOS},
    };
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            gen_config_left.gen_gpio_num = gen_gpios_left[i][j];
            gen_config_right.gen_gpio_num = gen_gpios_right[i][j];
            ESP_ERROR_CHECK(mcpwm_new_generator(operators_left[i], &gen_config_left, &generators_left[i][j]));
            ESP_ERROR_CHECK(mcpwm_new_generator(operators_right[i], &gen_config_right, &generators_right[i][j]));
        }
    }


    ESP_LOGI(TAG, "Setup deadtime");
    // Active LOW https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/mcpwm.html?highlight=mcpwm#active-low
    // Configuration for NMOS Output
    mcpwm_dead_time_config_t dt_config_NMOS = {
        .posedge_delay_ticks = 0,
        .negedge_delay_ticks = 10, // delay the PWM waveform on the falling edge
        .flags.invert_output = true,
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generators_left[i][BLDC_MCPWM_GEN_INDEX_NMOS], generators_left[i][BLDC_MCPWM_GEN_INDEX_NMOS], &dt_config_NMOS));
        ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generators_right[i][BLDC_MCPWM_GEN_INDEX_NMOS], generators_right[i][BLDC_MCPWM_GEN_INDEX_NMOS], &dt_config_NMOS));
    }
    // Configuration for PMOS output
    mcpwm_dead_time_config_t dt_config_PMOS = {
        .posedge_delay_ticks = 10, // delay the PWM waveform on the rising edge
        .negedge_delay_ticks = 0,
        .flags.invert_output = true,
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generators_left[i][BLDC_MCPWM_GEN_INDEX_PMOS], generators_left[i][BLDC_MCPWM_GEN_INDEX_PMOS], &dt_config_PMOS));
        ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generators_right[i][BLDC_MCPWM_GEN_INDEX_PMOS], generators_right[i][BLDC_MCPWM_GEN_INDEX_PMOS], &dt_config_PMOS));
    }

    ESP_LOGI(TAG, "Turn off all the gates");
    // TODO: !!!ACHTUNG!!! Das hier muss erst getestet werden, und dann kann die funktion entfernt werden für den Betrieb muss set_force_level deaktiviert sein!!!
    // TODO: das Ergebniss von dieser Funktion sollte sein, dass alle PMOS / IRLM2244 HIGH sind (IO 6, 8, 10, 36, 38, 40)
    // TODO: das Ergebniss von dieser Funktion sollte sein, dass alle NMOS / IRLM6244 LOW sind (IO 7, 9, 11, 35, 37, 39)
    for (int i = 0; i < 3; i++) {
        // The force level set by this function can be inverted by GPIO matrix or dead-time module. So the level set here doesn’t equal to the final output level.
        // because PMOS generators a inverted by dead time module, we need to set force level to 0
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generators_left[i][BLDC_MCPWM_GEN_INDEX_PMOS], 0, true));
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generators_right[i][BLDC_MCPWM_GEN_INDEX_PMOS], 0, true));
        // because NMOS generators a inverted by dead time module, we need to set force level to 1
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generators_left[i][BLDC_MCPWM_GEN_INDEX_NMOS], 1, true));
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generators_right[i][BLDC_MCPWM_GEN_INDEX_NMOS], 1, true));
    }

    ESP_LOGI(TAG, "Start a timer to adjust motor pwm periodically");
    esp_timer_handle_t periodic_timer = NULL;
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = update_motor_pwm_cb,
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, BLDC_SPEED_UPDATE_PERIOD_US));


    ESP_LOGI(TAG, "Motor init done.");    
}



void generateSinusTable() {

    
    for (int i = 0; i < SIN_TABLE_SIZE; i++) {
        float angle = (2 * PI * i) / SIN_TABLE_SIZE;
        
        // TODO: Better use round instead of casting to uint32_t
        sinusTable[i] = (uint32_t)(BLDC_MCPWM_PERIOD * (1.0 + sin(angle))); 
       // vTaskDelay(pdMS_TO_TICKS(2));
        //printf("%d angle: %f sinustable: %f \n", i, angle, sinusTable[i]);
      
      
    }
    printf("sinustable created\n");
}

void app_main() {



    
    // Initialisierung PWM für Motoren
    pwm_init();
    gpio_set_level(LED_BLUE, 0);

    generateSinusTable();
    gpio_set_level(LED_BLUE, 1);
    
    tableIndex_A = 0;
    tableIndex_B = SIN_TABLE_SIZE/3;
    tableIndex_C = tableIndex_B*2;

    //vTaskStartScheduler();
    //xTaskCreate(?, "TaskName", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES, NULL);
    //vTaskStartScheduler();

    
    while(1){
        vTaskDelay(pdMS_TO_TICKS(10));
    }
        
    
    
}