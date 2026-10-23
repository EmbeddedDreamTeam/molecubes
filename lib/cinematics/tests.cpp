#include "servo_controller.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "servo_types.h"
#include <cmath>
#include <utils_uart_comms.h>

static constexpr float POS_EPS = 0.01f; // tolerance for final position
static constexpr float SPEED_EPS = 0.01f;
static constexpr float ACC_EPS = 0.5f;
static constexpr uint32_t CHECK_STEP_MS = 100;

static bool check_limits(const char* context) {
    float pos = servo_data.current_pos.load();
    float spd = servo_data.current_speed.load();
    float acc = servo_data.current_acc.load();
    bool ok = true;
    if (pos < servo_data.min_pos - POS_EPS || pos > servo_data.max_pos + POS_EPS) {
        ESP_LOGE("TEST", "%s: POSITION OUT OF RANGE: %f not in [%f, %f]", context, pos, servo_data.min_pos, servo_data.max_pos);
        ok = false;
    }
    if (spd < -(servo_data.max_speed + SPEED_EPS) || spd > servo_data.max_speed + SPEED_EPS) {
        ESP_LOGE("TEST", "%s: SPEED OUT OF RANGE: %f not in [%f, %f]", context, spd, -servo_data.max_speed, servo_data.max_speed);
        ok = false;
    }
    if (fabsf(acc) > servo_data.max_acc + ACC_EPS) {
        ESP_LOGE("TEST", "%s: ACC OUT OF RANGE: %f, max=%f", context, acc, servo_data.max_acc);
        ok = false;
    }
    return ok;
}

static bool check_final_position(const char* context, float target) {
    float final = servo_data.current_pos.load();
    if (fabsf(final - target) > POS_EPS) {
        ESP_LOGE("TEST", "%s: Final position mismatch: expected %.4f, got %.4f", context, target, final);
        return false;
    }
    ESP_LOGI("TEST", "%s: Final position OK: %.4f", context, final);
    return true;
}

static void wait_and_check_ms(uint32_t ms, const char* context) {
    uint32_t elapsed = 0;
    while (elapsed < ms) {
        check_limits(context);
        vTaskDelay(pdMS_TO_TICKS(CHECK_STEP_MS));
        elapsed += CHECK_STEP_MS;
    }
}

void send_movement_command(float target_rad, float speed, float acc, float jerk) {
    Payload p = {};
    p.payload_servo.radians = target_rad;
    p.payload_servo.speed = speed;
    p.payload_servo.acceleration = acc;
    p.payload_servo.jerk = jerk;
    p.payload_servo.relative = false; // absolute position command
    p.payload_servo.send_ack = true; // request acknowledgment

    Msg* msg = create_msg(SELF_ID.load(), SELF_ID.load(), type_servo, p);
    xQueueSend(h_queue_servo, &msg, portMAX_DELAY);
}

void test_sweep() {
    ESP_LOGI("TEST", "Inizio Sweep: da MIN a MAX...");
    // Muove il servo da MIN a MAX a velocità moderata (1.5 rad/s)
    send_movement_command(servo_data.max_pos, 1.5f, servo_data.max_acc, servo_data.max_jerk);
    wait_and_check_ms(7000, "test_sweep -> MAX");
    check_final_position("test_sweep -> MAX", servo_data.max_pos);
    
    send_movement_command(servo_data.min_pos, 1.5f, servo_data.max_acc, servo_data.max_jerk);
    wait_and_check_ms(7000, "test_sweep -> MIN");
    check_final_position("test_sweep -> MIN", servo_data.min_pos);
}

void test_precision() {
    ESP_LOGI("TEST", "Test precisione: 0 -> 0.5rad -> 0 -> -0.5rad");
    
    send_movement_command(0.5f, 2.0f, servo_data.max_acc, servo_data.max_jerk);
    wait_and_check_ms(7000, "test_precision -> 0.5");
    check_final_position("test_precision -> 0.5", 0.5f);
    
    send_movement_command(0.0f, 2.0f, servo_data.max_acc, servo_data.max_jerk);
    wait_and_check_ms(7000, "test_precision -> 0.0");
    check_final_position("test_precision -> 0.0", 0.0f);
    
    send_movement_command(-0.5f, 2.0f, servo_data.max_acc, servo_data.max_jerk);
    wait_and_check_ms(7000, "test_precision -> -0.5");
    check_final_position("test_precision -> -0.5", -0.5f);
    
    send_movement_command(0.0f, 2.0f, servo_data.max_acc, servo_data.max_jerk);
    wait_and_check_ms(7000, "test_precision -> 0.0 (end)");
    check_final_position("test_precision -> 0.0 (end)", 0.0f);
}

void test_reactivity() {
    ESP_LOGI("TEST", "Test reattività: cambio direzione al volo");
    
    // Invia ordine di andare a +1.0 rad
    send_movement_command(1.0f, 1.0f, servo_data.max_acc, servo_data.max_jerk);
    vTaskDelay(pdMS_TO_TICKS(500)); // Aspetta mezzo secondo
    
    // Invia ordine contrario: dovrebbe ignorare il primo e invertire
    send_movement_command(-1.0f, 2.0f, servo_data.max_acc, servo_data.max_jerk);
    // attendi e controlla
    wait_and_check_ms(3000, "test_reactivity final check");
    check_final_position("test_reactivity", -1.0f);
}

void test_speed_ramp() {
    ESP_LOGI("TEST", "Test velocità crescente");
    
    float speeds[] = {0.1f, 1.5f, 3.0f}; // Rad/s
    float pos;
    for(int i=0; i<3; i++) {
        pos=servo_data.current_pos.load();
        ESP_LOGI("TEST", "Test velocità crescente: speed=%f", speeds[i]);
        ESP_LOGI("TEST", "Posizione attuale: %f rad, posizione target: %f rad", pos, servo_data.max_pos);
        send_movement_command(servo_data.max_pos, speeds[i], servo_data.max_acc, servo_data.max_jerk); //TODO qualche problema con le velocità e i tempi di attesa, inoltre sembrano esserci problemi di costanza nelle velocità
        uint32_t wait_ms = (uint32_t)((fabsf(pos - servo_data.max_pos) / (speeds[i] > 0.0f ? speeds[i] : servo_data.max_speed)) * 1000.0f) + 2000;
        wait_and_check_ms(wait_ms, "test_speed_ramp -> MAX");
        check_final_position("test_speed_ramp -> MAX", servo_data.max_pos);

        pos=servo_data.current_pos.load();
        ESP_LOGI("TEST", "Posizione attuale: %f rad, posizione target: %f rad", pos, servo_data.min_pos);
        send_movement_command(servo_data.min_pos, speeds[i], servo_data.max_acc, servo_data.max_jerk);
        wait_ms = (uint32_t)((fabsf(pos - servo_data.min_pos) / (speeds[i] > 0.0f ? speeds[i] : servo_data.max_speed)) * 1000.0f) + 2000;
        wait_and_check_ms(wait_ms, "test_speed_ramp -> MIN");
        check_final_position("test_speed_ramp -> MIN", servo_data.min_pos);
    }
}

void test_acceleration() {
    ESP_LOGI("TEST", "Test accelerazione crescente");
    
    float accs[] = {0.5f, 2.0f, 5.0f}; // Rad/s^2
    for(int i=0; i<3; i++) {
        ESP_LOGI("TEST", "Test accelerazione crescente: acc=%f", accs[i]);
        send_movement_command(servo_data.max_pos, servo_data.max_speed, accs[i], servo_data.max_jerk);
        wait_and_check_ms(7000, "test_acceleration -> MAX");
        check_final_position("test_acceleration -> MAX", servo_data.max_pos);

        send_movement_command(servo_data.min_pos, servo_data.max_speed, accs[i], servo_data.max_jerk);
        wait_and_check_ms(7000, "test_acceleration -> MIN");
        check_final_position("test_acceleration -> MIN", servo_data.min_pos);
    }
}   

void test_jerk() {
    ESP_LOGI("TEST", "Test jerk crescente");
    float jerks[] = {0.5f, 80.0f, 150.0f}; // Rad/s^3
    for(int i=0; i<3; i++) {
        ESP_LOGI("TEST", "Test jerk crescente: jerk=%f", jerks[i]);
        send_movement_command(servo_data.max_pos, servo_data.max_speed, servo_data.max_acc, jerks[i]);
        wait_and_check_ms(20000, "test_jerk -> MAX");
        check_final_position("test_jerk -> MAX", servo_data.max_pos);

        send_movement_command(servo_data.min_pos, servo_data.max_speed, servo_data.max_acc, jerks[i]);
        wait_and_check_ms(20000, "test_jerk -> MIN");
        check_final_position("test_jerk -> MIN", servo_data.min_pos);
    }
}
