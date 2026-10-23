#include "servo_controller.h"
#include "msg_structs.h"
#include "utils_uart_comms.h"
#include <freertos/queue.h>
#include "esp_log.h"
#include <cmath>
#include "buffer_header.h"
#include "task_handler.h"



void send_movement_ack(){
    PayloadServoAck ack;
    ack.sender_id=SELF_ID.load();
    Msg* msg = create_msg(SELF_ID.load(), ROOT_ID, type_servo_ack, Payload{.payload_servo_ack=ack});
    if (SELF_ID.load()==ROOT_ID){
        sort_new_msg(msg);
    }
    send_msg_to_master(msg);
    // Do NOT delete msg here: ownership is transferred to the send subsystem.
    // The message pointer is queued and will be deleted by the task that
    // actually sends UART bytes (task_send_uart). Deleting it here causes
    // a use-after-free and intermittent crashes.
}


void move_servo_speed_task_state_machine(void *pvParameters) {
    ServoTaskParams cmd;
    Msg* msg = nullptr;

    TickType_t xFrequency  = pdMS_TO_TICKS(20);

    bool backlash_compensation=false;
    bool restart;
    float appo;
    //flag che indica se è stato ricevuto un nuovo comando nella direzione opposta a quello in esecuzione
    bool reverse=false; 
    while (1) {
        // by passing the portMAX_DELAY to xQueueReceive, we ensure that the task will be blocked until
        // there is a new command in the queue
        // (the third parameter is xTicksToWait that specify the maximum amount of time the task should
        // be blocked waiting for a command)
        if (!xQueueReceive(h_queue_servo, &msg, portMAX_DELAY)) continue;
        ESP_LOGI("Servo", "Received new command: target=%.4f, speed=%.3f, acc=%.3f, jerk=%.3f", cmd.target_rad, cmd.speed, cmd.acc, cmd.jerk);
        cmd=sanitize_servo_command(msg);
        servo_data.moving.store(true);
        backlash_compensation=false;
        reverse=false;

        do {
            restart = false;
            backlash_compensation=false;
            // initial state
            float pos    = servo_data.current_pos.load();
            float target = cmd.target_rad;


            if (target < pos){
                //settiamo il flag per la compensazione del backlash
                target= max(target - backlash, servo_data.min_pos);
                backlash_compensation=true;
            }
            ESP_LOGI("Servo", "New command: target=%.4f, speed=%.3f, acc=%.3f, jerk=%.3f, backlash_comp=%s", target, cmd.speed, cmd.acc, cmd.jerk, backlash_compensation ? "YES" : "NO");

            // Clamping parameters 
            const float j = cmd.jerk > 0.0f ? cmd.jerk : servo_data.max_jerk;
            const float a = cmd.acc > 0.0f ? cmd.acc : servo_data.max_acc;
            const float v = cmd.speed > 0.0f ? cmd.speed : servo_data.max_speed;

            // skippa un comando di movimento se il target è già stato raggiunto
            if (fabsf(target - pos) < servo_deadzone) break;

            const float dir = (target > pos) ? 1.0f : -1.0f;
            float vel  = servo_data.current_speed.load();   // module of speed, always ≥ 0
            float acc  = servo_data.current_acc.load();   // acc with sign [rad/s²]: + accel, − decel
            float prev_acc = acc; // previous acceleration

            if (reverse){
                vel=-vel;
                acc=-acc;
            }

            MotionPhase phase = PH_ACCEL_JUP;
            MotionPhase prev_phase = PH_ACCEL_JUP;
            bool done = false; //flag che indica se il target è stato raggiunto
            // salvando il tick corrente per calcolare il dt in modo estremamente preciso
            TickType_t xLastWake = xTaskGetTickCount();
            TickType_t xPrevTick = xLastWake;
            
            TickType_t now;
            float dt;
            float rem;
            float d_stop_curr; // distanza necessaria per fermarsi
            float d_trig; // distanza di trigger per passare alla fase di decelerazione

            // main loop with state machine for motion profiling
            while (!done) {
                 // if there is a new command in the queue, preempt the current motion and start the new one immediately
                Msg* next=nullptr;
                if (xQueueReceive(h_queue_servo, &next, 0) == pdTRUE) {
                    servo_data.current_pos.store(pos);
                    cmd = sanitize_servo_command(next);
                    // seting the flag to restart the FSM
                    restart = true;

                    if ((cmd.target_rad > pos? 1.0f : -1.0f)*dir < 0.0f) {
                        // il nuovo comando è nella direzione opposta
                        reverse=true;
                    }
                    // breaking the loop to restart the FSM with the new command
                    break;
                    // we have to preserve the previous state
                }

                // calculating actual dt
                now = xTaskGetTickCount();
                if (now-xPrevTick ==0) {
                    dt = 0.02f; // valore di default
                }
                else{
                    dt = (float)(now - xPrevTick) * (portTICK_PERIOD_MS / 1000.0f);
                }
                xPrevTick = now;

                // remaining distance
                rem = fabsf(target - pos);

                // distance necessary to stop
                // facciamo una simulazione con i valori correnti
                // in caso di overshoot il servo si fermerà anche se la velocità non è 0
                // in caso di undershoot la velocità non scenderà mai sotto un valore minimo per raggiungere
                // in modo fluido il target
                if (vel < 0.0f) {
                    //in questo modo viene calcolata la distanza di arresto prima di poter invertire la direzione
                    //d_stop_curr è negativa, in questo modo rem <= d_trig non sarà mai vero
                    d_stop_curr = -decel_distance_with_acc(-vel, -acc, a, j, v);
                } else {
                    d_stop_curr = decel_distance_with_acc(vel, acc, a, j, v);
                }
                d_trig = d_stop_curr;
                // in questo modo se durante l'esecuzione la fase cambia, vengono calcolati subito i nuovi parametri
                do{
                    ESP_LOGI("SERVO_API", "Fase: %d", phase);
                    prev_phase = phase;
                    switch (phase) {

                    case PH_ACCEL_JUP:
                        //in base alla simulazione abbiamo appena lo spazio per fermarci
                        if (rem <= d_trig) { 
                            ESP_LOGI("Servo", "Switching to ACCEL_JDN phase (remaining distance: %f, trigger distance: %f, acc: %f, vel: %f)", rem, d_trig, acc, vel);
                            phase = PH_ACCEL_JDN;
                            break; 
                        }
                        appo=acc;
                        acc += j * dt; //updating acceleration

                        if (acc >= a) { //capping acceleration to a
                            acc = a;
                            // questa è la velocità calcolata alla fine della fase di accelerazione, se questa velocità è maggiore della velocità target,
                            // allora dobbiamo passare alla fase di decelerazione.
                            // La velocità finale sarà data dalla velocità attuale + l'area sottesa alla curva di accelerazione,
                            // che in questo caso è un triangolo, dunque v= h*b/2=a*(a/j)/2 = a^2/(2*j)
                            // v= velocità target, vel=velocità attuale
                            if (vel + (a * a) / (2.0f * j) >= v) {
                                ESP_LOGI("Servo", "Switching to ACCEL_JDN phase (vp: %f, v: %f, acc: %f, vel: %f)", vel + (a * a) / (2.0f * j), v, acc, vel);
                                acc=appo;
                                phase = PH_ACCEL_JDN;
                            } else {
                                ESP_LOGI("Servo", "Switching to ACCEL_CONST phase (vp: %f, v: %f, acc: %f, vel: %f)", vel + (a * a) / (2.0f * j), v, acc, vel);
                                phase = PH_ACCEL_CONST;
                            }
                        } else {
                            
                            // questa è la velocità calcolata alla fine della fase di accelerazione, se questa velocità è maggiore della velocità target,
                            // allora dobbiamo passare alla fase di decelerazione.
                            // La velocità finale sarà data dalla velocità attuale + l'area sottesa alla curva di accelerazione,
                            // che in questo caso è un triangolo, dunque v= h*b/2=a*(a/j)/2 = a^2/(2*j)
                            //in questo caso siamo in un profilo triangolare
                            if (vel + (acc * acc) / (2.0f * j) >= v) {
                                ESP_LOGI("Servo", "Switching to ACCEL_JDN phase (vp: %f, v: %f, acc: %f, vel: %f)", vel + (acc * acc) / (2.0f * j), v, acc, vel);
                                phase = PH_ACCEL_JDN;
                            }
                        }
                        break;

                    case PH_ACCEL_CONST:
                        // in questo caso siamo nella fase di accelerazione costante, quindi la velocità aumenta linearmente
                        // se la distanza rimanente è minore di quella calcolata dobbiamo ridurre l'accelerazione fino a farla diventare negativa,
                        // quindi entriamo in una fase con jerk negativo, quindi accel_jdn
                        if (rem <= d_trig) { 
                            ESP_LOGI("Servo", "Switching to ACCEL_JDN phase (remaining distance: %f, trigger distance: %f, acc: %f, vel: %f)", rem, d_trig, acc, vel);
                            phase = PH_ACCEL_JDN;
                            break;
                        }
                        // questa è la velocità calcolata alla fine della fase di accelerazione, se questa velocità è maggiore della velocità target,
                        // allora dobbiamo passare alla fase di decelerazione.
                        // La velocità finale sarà data dalla velocità attuale + l'area sottesa alla curva di accelerazione,
                        // che in questo caso è un triangolo, dunque v= h*b/2=a*(a/j)/2 = a^2/(2*j)
                        if (vel + (a * a) / (2.0f * j) >= v) {
                            ESP_LOGI("Servo", "Switching to ACCEL_JDN phase (vel: %f, v: %f, acc: %f, vel: %f)", vel + (a * a) / (2.0f * j), v, acc, vel);
                            phase = PH_ACCEL_JDN;
                        }
                        break;
                    case PH_ACCEL_JDN:
                        acc -= j * dt;
                        
                        if (acc <= 0.0f) {
                            // se lo spazio rimanente è minore di quello calcolato dobbiamo ridurre l'accelerazione fino a farla diventare negativa,
                            // quindi entriamo in una fase con jerk negativo, quindi decel_jup
                            // questo non cambia la cinematica rispetto a questa fase dato che stiamo già riducendo l'accelerazione
                            if (rem <= d_trig) { 
                                ESP_LOGI("Servo", "Switching to DECEL_JUP phase (remaining distance: %f, trigger distance: %f, acc: %f, vel: %f)", rem, d_trig, acc, vel);
                                acc += j * dt;
                                phase = PH_DECEL_JUP; 
                                break; 
                            }
                            acc = 0.0f;
                            vel = v;      // snap at the precise speed
                            ESP_LOGI("Servo", "Switching to CRUISE phase (acc: %f, vel: %f)", acc, vel);
                            phase = PH_CRUISE; //linear motion phase
                        }
                        break;

                    case PH_CRUISE:
                        // se lo spazio rimanente è minore di quello calcolato dobbiamo entrare nella fase di accelerazione negativa
                        if (rem <= d_trig) {
                            ESP_LOGI("Servo", "Switching to DECEL_JUP phase (remaining distance: %f, trigger distance: %f, acc: %f, vel: %f)", rem, d_trig, acc, vel);
                            phase = PH_DECEL_JUP;
                        }
                        break;

                    

                    case PH_DECEL_JUP:
                        // starting deceleration ramp: acc goes from 0 to -a'
                        appo=acc;
                        acc -= j * dt;
                        if (acc < -a) acc = -a;

                        // in questo caso la velocità è tale che per arrivare a velocità zero con accelerazione zero dobbiamo per forza
                        // iniziare a ridurre l'accelerazione, quindi passiamo alla fase di decelerazione costante
                        // questo perchè la velocità è in questo caso è data dall'area sottesa alla curva di accelerazione,
                        // che in questo caso è un triangolo, dunque v= h*b/2=a*(a/j)/2 = a^2/(2*j)
                        if (vel <= (acc * acc) / (2.0f * j)) {
                            ESP_LOGI("Servo", "Switching to DECEL_JDN phase (vel: %f, acc: %f, j: %f)", vel, acc, j);
                            acc=appo;
                            phase = PH_DECEL_JDN;       // triangular profile
                            break;
                        } else if (acc <= -a) { // l'accelerazione raggiunta è quella massima, quindi possiamo passare alla fase di decelerazione costante
                            ESP_LOGI("Servo", "Switching to DECEL_CONST phase (acc: %f, a: %f, vel: %f)", acc, a, vel);
                            phase = PH_DECEL_CONST;     // trapezoidal profile
                        }
                        break;

                    case PH_DECEL_CONST:
                        // in questo caso la velocità è tale che per arrivare a velocità zero con accelerazione zero dobbiamo per forza
                        // iniziare a ridurre l'accelerazione, quindi passiamo alla fase di decelerazione costante
                        // questo perchè la velocità è in questo caso è data dall'area sottesa alla curva di accelerazione,
                        // che in questo caso è un triangolo, dunque v= h*b/2=a*(a/j)/2 = a^2/(2*j)
                        if (vel <= (a * a) / (2.0f * j)) {
                            ESP_LOGI("Servo", "Switching to DECEL_JDN phase (vel: %f, a: %f, j: %f, acc: %f)", vel, a, j, acc);
                            phase = PH_DECEL_JDN;
                        }
                        break;

                    case PH_DECEL_JDN:
                        // deceleration ramp: acc goes from -a to 0
                        acc += j * dt;
                        if (acc >= 0.0f) {
                            acc = 0.0f;
                            //vel = 0.0f;
                            ESP_LOGI("Servo", "Switching to DONE phase (acc: %f, vel: %f)", acc, vel);
                            //done = true;    // now we can stop, we have reached the target
                        }
                        break;
                    }
                } while (phase != prev_phase);
                // in questo modo se si verifica un cambio di fase vengono aggiornati immediatamente i parametri, invece di aspettare il tick successivo
                // quindi la distanza di decelerazione sarà più precisa e non ci saranno overshoot

                // calculating new speed and position with protections
                
                //vel += acc * dt;


                //in caso di undershoot il servo si fermerebbe prima di aver raggiunto il target
                // in questo caso non si va mai sotto una velocità minima per poter raggiungere 
                // il target in modo fluido
                if (vel < min_speed) {
                    if (phase == PH_DECEL_JUP || phase == PH_DECEL_CONST || phase == PH_DECEL_JDN) {
                        if (rem>d_trig){
                            ESP_LOGI("Servo", "default min_speed applicata per raggiungere correttamente il target");
                            vel = min_speed;
                        }
                        else if (vel < 0.0f) {
                            vel = 0.0f;
                            acc = 0.0f;
                            done = true; // we have reached the target
                        }
                    }
                }

                //pos += dir * vel * dt;

                pos+= dir* (vel * dt+0.5f*prev_acc * dt*dt+(1.0f/6.0f)*(acc-prev_acc)*dt*dt); // integrazione della velocità

                
                vel +=prev_acc * dt+ 0.5f * (acc - prev_acc) * dt; // trapezoidal integration of acceleration
                if (vel > v)    vel = v;
                prev_acc = acc;

                // correcting overshoot
                if ((dir > 0.0f && pos >= target) || (dir < 0.0f && pos <= target)) {
                    pos = target;
                    vel = 0.0f;
                    acc = 0.0f;
                    done = true;
                }
                // updating servo state
                servo_data.current_speed.store(vel);
                servo_data.current_acc.store(acc);
                servo_data.current_pos.store(pos);
                //making sure that the servo accepts the new position command,
                // if the new position signal is different from the previous one less than the deadzone
                // the servo will drop that command and keep the previous one
                
                ESP_LOGI("Servo", "Setting servo position: pos=%.4f, target=%.4f, vel=%.4f, acc=%.4f, backlash_compensation=%s", pos, target, vel, acc, backlash_compensation ? "true" : "false");
                if (fabsf(pos - target) > servo_deadzone){
                    set_servo_pos(pos);
                }
                else{
                    set_servo_pos(target);
                    done = true;
                }
                //metodo preciso per risvegliare la task
                if (!done) vTaskDelayUntil(&xLastWake, xFrequency);
            }
        } while (restart);
        ESP_LOGI("Servo", "backlash_compensation: %d", backlash_compensation);
        if (backlash_compensation){
            ESP_LOGI("Servo", "Backlash compensation: moving to intermediate target=%.4f", cmd.target_rad - backlash);
            // if we have done a backlash compensation, we need to move the servo back to the original target position to compensate for the backlash
            servo_data.current_speed.store(0.0f);
            servo_data.current_acc.store(0.0f);
            servo_data.moving.store(false);
            Payload p={};
            p.payload_servo.radians=cmd.target_rad;
            p.payload_servo.speed=0.5f;
            p.payload_servo.acceleration=10.0f;
            p.payload_servo.jerk=30.0f;
            p.payload_servo.send_ack=true;
            Msg* backlash_cmd=create_msg(SELF_ID.load(), SELF_ID.load(), type_servo, p);
            ESP_LOGI("Servo", "Backlash compensation: moving back to target=%.4f", cmd.target_rad);
            xQueueSend(h_queue_servo, &backlash_cmd, 0); // we can send the command directly to the queue, the FSM will take care of executing it immediately
        }
        else{
            set_servo_pos(cmd.target_rad); // ensure we end up in the exact target position (corrections for numerical errors)
            // updating final servo state with speed and acc = 0
            servo_data.current_speed.store(0.0f);
            servo_data.current_acc.store(0.0f);
            servo_data.moving.store(false);
            if (cmd.send_ack){
                ESP_LOGI("Servo", "Sending movement ack");
                send_movement_ack();
            }
            else{
                ESP_LOGI("Servo", "No movement ack requested");
            }
        }
    }
}


ServoTaskParams sanitize_servo_command(Msg* msg) {
    ESP_LOGI("SERVO_API", "move_servo_speed called with rad=%.2f, speed=%.2f, acc=%.2f, jerk=%.2f", msg->payload.payload_servo.radians, msg->payload.payload_servo.speed, msg->payload.payload_servo.acceleration, msg->payload.payload_servo.jerk);
    ESP_LOGI("SERVO_API", "Current servo state: pos=%.2f, speed=%.2f, acc=%.2f", servo_data.current_pos.load(), servo_data.current_speed.load(), servo_data.current_acc.load());

    ServoTaskParams params;
    // gestisce i comandi di posizione relativa
    if (msg->payload.payload_servo.relative) {
        params.target_rad = servo_data.current_pos.load() + msg->payload.payload_servo.radians;
    } else {
        params.target_rad = msg->payload.payload_servo.radians;
    }
    // sanitizing input parameters to ensure they are within the servo limits
    params.target_rad = fmaxf(servo_data.min_pos, fminf(servo_data.max_pos, params.target_rad));
    params.speed = msg->payload.payload_servo.speed>servo_data.max_speed?servo_data.max_speed:msg->payload.payload_servo.speed;
    params.acc = msg->payload.payload_servo.acceleration>servo_data.max_acc?servo_data.max_acc:msg->payload.payload_servo.acceleration;
    params.jerk = msg->payload.payload_servo.jerk>servo_data.max_jerk?servo_data.max_jerk:msg->payload.payload_servo.jerk;
    params.send_ack = msg->payload.payload_servo.send_ack;
    free_msg(msg); // free message allocated by UART layer
    return params;
}


esp_err_t servo_init(){
    servo_timer_init();
    BaseType_t result;

    // ensure logical current position has a known value before task start
    servo_data.current_pos.store(-0.1f);

    // creating the persistent task
    result = xTaskCreate(
        move_servo_speed_task_state_machine,
        "ServoMotorTask",
        3072, // Stack size
        NULL, //parameters
        1,
        &move_servo_speed_task_handle
    );
    if (result != pdPASS) {
        ESP_LOGE("SERVO_INIT", "Failed to create ServoMotorTask");
        return ESP_FAIL;
    }
    ESP_LOGI("SERVO_INIT", "Servo deadzone %f", servo_deadzone);
    //random delay to avoid all the servos to start at the same time and cause a big current absorption peak that could reset the board
    vTaskDelay(pdMS_TO_TICKS(rand()%3000)); 
    Payload p={};
    p.payload_servo.radians=0.0f;
    p.payload_servo.speed=1.0f;
    p.payload_servo.acceleration=servo_data.max_acc;
    p.payload_servo.jerk=servo_data.max_jerk;
    p.payload_servo.send_ack=false;
    Msg* init_cmd=create_msg(SELF_ID.load(), SELF_ID.load(), type_servo, p);
    xQueueSend(h_queue_servo, &init_cmd, 0); //moving the servo to the initial position with max speed, acc and jerk to ensure a fast initialization
    vTaskDelay(pdMS_TO_TICKS(1000));
    return ESP_OK;
}
