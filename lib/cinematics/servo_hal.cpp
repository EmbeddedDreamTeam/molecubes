#include "servo_hal.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <cmath>
#include "servo_types.h"

void servo_timer_init(){
    servo_data.duty_res =ledc_find_suitable_duty_resolution(80000000, 50);//auto clock freq of 80MHz and timer freq of 50Hz, the most common servo frequency
    ledc_timer_config_t timer_config ={
        .speed_mode=LEDC_LOW_SPEED_MODE, //low speed is sufficient for controlling a servo
        .duty_resolution = static_cast<ledc_timer_bit_t>(servo_data.duty_res),
        .timer_num=LEDC_TIMER_0, //timer number
        .freq_hz=50,
        .clk_cfg=LEDC_AUTO_CLK,
        .deconfigure=false,
    };
    ledc_timer_config(&timer_config);
    ledc_channel_config_t channel_config={
        .gpio_num=servo_data.gpio, //signal pin of the servo
        .speed_mode=LEDC_LOW_SPEED_MODE,
        .channel=LEDC_CHANNEL_0,
        .intr_type=LEDC_INTR_DISABLE,
        .timer_sel=LEDC_TIMER_0,
        .duty=0,
        .hpoint=0,
    };
    ledc_channel_config(&channel_config);
}

//converting degrees to radians
float rad_from_deg(int32_t degrees){
    return ((float)degrees/360.0 * 2*M_PI);
}

//this type of servo has a frequency of 50Hz and, considering the trimming zone they accept a signal that
// ranges from ~351ms to ~2640 ms  that corresponde to a range of motion of nearly ~309 degrees 
/// NOTE this function isn't meant to be used alone, it is used by the move_servo_speed function to set the position of the servo, if you want to set the position directly use move_servo_speed with speed=1.0f
esp_err_t set_servo_pos(float rad){
    if (rad>=servo_data.min_pos && rad<=servo_data.max_pos){
        //ESP_LOGI("SERVO_API", "Posizione impostata: %.4f rad", rad);
        double mid_point=servo_data.sgnl_min_duty+(servo_data.sgnl_max_duty-servo_data.sgnl_min_duty)/2.0;
        //double time= mid_point+rad/servo_data.max_pos*(servo_data.sgnl_max_duty-servo_data.sgnl_min_duty)/2.0; //calculating the signal time
        double time= mid_point+(rad+trim)/(1.5*M_PI)*(servo_data.sgnl_max_duty-servo_data.sgnl_min_duty); //calculating the signal time
        uint32_t max_duty = (1 << servo_data.duty_res); // valore massimo a cui arriva il contatore del timer
        uint32_t duty= (uint32_t)(time/20000.0*max_duty); //fraction of the period in micro-seconds
        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_0,
            duty
        );
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        // Update the logical current position (this is the commanded position,
        // not a physical feedback sample). Use atomic store for clarity.
        servo_data.current_pos.store(rad);
        return ESP_OK;
    }
    else{
        return ESP_ERR_INVALID_ARG;
    }

}
