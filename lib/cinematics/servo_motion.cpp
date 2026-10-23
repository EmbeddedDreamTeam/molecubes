#include "servo_motion.h"
#include <math.h>

/// @brief  Numerically estimate the stopping distance with jerk and acceleration limits.
/// This function simulates the deceleration with time steps as close as possible to the control loop frequency.
float decel_distance_sim(float v_init, float acc_init, float a_max, float j_max, float v_max) {
    if (v_init <= 0.0f) return 0.0f;

    const float dt = 0.020f; 
    // starting speed and acceleration
    float v = v_init;
    float a = acc_init;
    // distance covered so far
    float x = 0.0f;
    // capping the number of iterations to avoid infinite loops
    const uint32_t max_iters = 2000;
    bool j_dn=false;
    float j = 0.0f;
    //interrupting the simulation if the velocity is very low
    for (uint32_t i = 0; i < max_iters && v > 0.002f; ++i) {
        //target acceleration is the maximum allowed acceleration but is negative because we want to decelerate
        const float target_a = -a_max;
        // if the target acceleration is already reached, we don't need to apply jerk
        if (a > target_a) {
            //fase di jerk_up
            j = -j_max; 
        }
        else {
            // fase di accelerazione costante
            j = 0.0f;
        }
        float v_next;
        float a_next;
        if (a<=0.0f&&v <= (a * a) / (2.0f * j_max)) {
            //passaggio alla fase di jerk_down
            j_dn=true;
        }
        if (!j_dn){
            // updating the acceleration of the next step and clamping it to the target acceleration
            a_next = a + j * dt;
            if (a_next < target_a) a_next = target_a;
            //v_next = v + a_next * dt; // uso l'accelerazione a scaglioni perchè è quello che fa la task reale
            v_next = v + a * dt + 0.5f * (a_next - a) * dt; // trapezoidal integration of acceleration
            // clamping velocity to max
            // per sicurezza ma non dovrebbe succedere
            if (v_next > v_max) v_next = v_max;
        }
        else{
            //siamo nella fase di jerk_down, quindi l'accelerazione va da -a a 0
            if (a<0.0f){
                j=j_max;
            }
            else{
                j=0.0f;
            }
            // updating the acceleration of the next step and clamping it to the target acceleration
            a_next = a + j * dt;
            if (a_next > 0.0f) a_next = 0.0f;
            //v_next = v + a_next * dt; // uso l'accelerazione a scaglioni perchè è quello che fa la task reale
            v_next = v + a * dt + 0.5f * (a_next - a) * dt; // trapezoidal integration of acceleration

        }
        // updating distance with linear accelerated motion
        //x += v_next * dt;
        if (v_next <= 0.0f){
            float delta=a*a-2.0f*j*v;
            float delta_sqrt;
            float new_dt;
            if (delta >=0.0f){
                delta_sqrt=sqrtf(delta);
                if ((-a+delta_sqrt)/j>=0.0f){
                    if((-a-delta_sqrt)/j>=0.0f){
                        new_dt=fmin((-a+delta_sqrt)/j,(-a-delta_sqrt)/j);
                    }
                    else{
                        new_dt=(-a+delta_sqrt)/j;
                    }
                }
                else{
                    new_dt=(-a-delta_sqrt)/j;
                }
            }
            else{
                new_dt=fabs(a)/j_max; // tempo necessario per arrivare ad avere l'accelerazione pari a zero
            }
            x+= v * new_dt + 0.5f * a * new_dt * new_dt + (1.0f / 6.0f) * (0.0f - a) * new_dt * new_dt;
        }
        else{
            x+= v * dt + 0.5f * a * dt * dt + (1.0f / 6.0f) * (a_next - a) * dt * dt;
        }
        v = v_next;
        a = a_next;
    }
    return x;
}

/// @brief Calculates the distance required to decelerate from a given velocity to zero
float decel_distance(float v, float a_max, float j_max, float v_max) {
    return decel_distance_sim(v, 0.0f, a_max, j_max, v_max);
}

/// @brief Calculates the distance required to decelerate from a given velocity and acceleration to zero
float decel_distance_with_acc(float v, float a, float a_max, float j_max, float v_max) {
    // if we're not currently accelerating (a == 0) the fallback is the same
    // as decel_distance.
    if (a == 0.0f) return decel_distance(v, a_max, j_max, v_max);
    return decel_distance_sim(v, a, a_max, j_max, v_max);
}
