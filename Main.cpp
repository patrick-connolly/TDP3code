#include "mbed.h"
#include "colorsensor.hpp"
#include <cstdio>
#include <cstring>
#include <cstdarg>

// USB serial for debug output
BufferedSerial pc(USBTX, USBRX, 115200);

static void dbg_println(const char* s) {
    pc.write(s, strlen(s));
    pc.write("\r\n", 2);
}

static void dbg_printf(const char* fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    pc.write(buf, strlen(buf));
}
//Motors
//Pin Allocations
PwmOut Right_motor(PTD4);
PwmOut Left_motor(PTA12);
DigitalOut direction_left_1(PTA4);
DigitalOut direction_left_2(PTA5);
DigitalOut direction_right_1(PTC9);
DigitalOut direction_right_2(PTC8);

//Variables
float period_motor = 0.001f; //Period of motor
float motor_r_power = 0.0f; //Duty cycle of right motor
float motor_l_power = 0.0f; //Duty cycle of right motor
float motor_r_power_c = 0.0f;
float motor_l_power_c = 0.0f;
int d_l_1 = 0;
int d_l_2 = 0;
int d_r_1 = 0;
int d_r_2 = 0;
float movement_speed_ratio = 1.0f;
int OffLine_clock = 3000; //Unit: ms

//Timers
Timer Timer_movement_offline_checker;
int Timer_movement_offline_checker_read_ms = 0;

Timer Timer_movement_offline_recovery_movement;
int Timer_movement_offline_recovery_movement_read_ms = 0;

Timer Timer_movement_right_angle_control;
int Timer_movement_right_angle_control_read = 0;
//Mode Control
//Off Line Protocal
int mode_Timer_movement_offline_checker = 0;
int mode_Timer_movement_offline_recovery_movement = 0;
int mode_LineSensor_movement_fail = 0;
int mode_right_angle_movement_control_spin = 0;
int mode_right_angle_movement_control_spin_reverse = 0;

int mode_Timer_movement_offline_recovery_movement_spin = 0;
//Movement
int mode_movement_spin = 0;
int mode_movement_spin_left = 0;
int mode_movement_spin_right = 0;
int mode_movement_right_angle = 0;

//Line Following
//Pin Allocations
DigitalIn Sensor_1_line(PTD3);
DigitalIn Sensor_2_line(PTD2);
DigitalIn Sensor_3_line(PTD0);
DigitalIn Sensor_4_line(PTD5);
DigitalIn Sensor_5_line(PTA13);


//Variables
int s_1, s_2, s_3, s_4, s_5;

//Traffic Light
//Mode Control
int mode_traffic_light = 0; // 0: no red light detected, 1: red light detected

// Single shared color sensor instance (created in main)
// Set to nullptr until main() constructs the sensor.
tcs3472::TCS3472* color_sensor_ptr = nullptr;

//Ultrasonic
//Front
DigitalOut ultrasonic_front_trig(PTC10);
DigitalIn  ultrasonic_front_echo(PTC11);

//Variables
int ultrasonic_front_echo_state = 0;
float ultrasonic_front_distance_measured = 1.0f; //Unit: meter
float ultrasonic_speed_of_sound = 343.0f; //meter per second
int condition_ultrasonic_front_obstacle_detection = 0; // 0: no obstacle detected, 1: obstacle detected
int number_of_trig_send_front = 0;

//Timer
Timer Timer_ultrasonic_front_distance_measurement;
float Timer_ultrasonic_front_distance_measurement_read = 0.0f;

Timer Timer_ultrasonic_front_failing_recovery_protocal;
float Timer_ultrasonic_front_failing_recovery_protocal_read = 0.0f;

//Side
DigitalOut ultrasonic_side_trig(PTA17);
DigitalIn  ultrasonic_side_echo(PTE31);

//Variables
int ultrasonic_side_echo_state = 0;
float ultrasonic_side_distance_measured = 1.0f; //Unit: meter
int condition_ultrasonic_side_obstacle_detection = 0; // 0: no obstacle detected, 1: obstacle detected
int number_of_trig_send_side = 0; //Used for side ultrasonic to prevent infinite loop when no echo received

//Timer
Timer Timer_ultrasonic_side_distance_measurement;
float Timer_ultrasonic_side_distance_measurement_read = 0.0f;

Timer Timer_ultrasonic_side_failing_recovery_protocal;
float Timer_ultrasonic_side_failing_recovery_protocal_read = 0.0f;



//General
//Pin Allocations
DigitalIn Switch_Speed_Control(PTC7);
DigitalIn Switch_On_Off(PTC0);
//LED
DigitalOut LED_L_1(PTC4);
DigitalOut LED_L_2(PTC5);
DigitalOut LED_L_3(PTC6);

DigitalOut LED_R_1(PTB10);
DigitalOut LED_R_2(PTB11);
DigitalOut LED_R_3(PTE2);

//Mode Control
volatile int mode = 0;
volatile int last_mode = 0;
int mode_obstacle_avoidance = 0;

//void functions

//Movement_main
void movement(){
    if (Switch_Speed_Control == 0){
        movement_speed_ratio = 0.75f;
    }else{
        movement_speed_ratio = 1.0f;
    }
    switch (mode){
        case 1: //Stop
            motor_l_power_c = movement_speed_ratio * 0.8f;
            motor_r_power_c = movement_speed_ratio *  0.8f;
            d_l_1 = 1;
            d_l_2 = 1;
            d_r_1 = 1;
            d_r_2 = 1;
            break;
        case 2: //Forward
            motor_l_power_c = movement_speed_ratio * 0.4f;
            motor_r_power_c = movement_speed_ratio * 0.4f;
            d_l_1 = 1;
            d_l_2 = 0;
            d_r_1 = 1;
            d_r_2 = 0;
            break;
        case 3: //Backward
            motor_l_power_c = movement_speed_ratio * 0.4f;
            motor_r_power_c = movement_speed_ratio * 0.4f;
            d_l_1 = 0;
            d_l_2 = 1;
            d_r_1 = 0;
            d_r_2 = 1;
            break;
        case 4: //Left turn (curve)
            motor_l_power_c = movement_speed_ratio * 0.28f;
            motor_r_power_c = movement_speed_ratio * 0.56f;
            d_l_1 = 0;
            d_l_2 = 1;
            d_r_1 = 1;
            d_r_2 = 0;
            break;
        case 5: //Right turn (curve)
            motor_l_power_c = movement_speed_ratio * 0.56f;
            motor_r_power_c = movement_speed_ratio * 0.28f;
            d_l_1 = 1;
            d_l_2 = 0;
            d_r_1 = 0;
            d_r_2 = 1;
            break;
        case 6: //Left turn (still)
            motor_l_power_c = movement_speed_ratio * 0.75f;
            motor_r_power_c = movement_speed_ratio * 0.75f;
            d_l_1 = 0;
            d_l_2 = 1;
            d_r_1 = 1;
            d_r_2 = 0;
            break;
        case 7: //Right turn (still)
            motor_l_power_c = movement_speed_ratio * 0.75f;
            motor_r_power_c = movement_speed_ratio * 0.75f;
            d_l_1 = 1;
            d_l_2 = 0;
            d_r_1 = 0;
            d_r_2 = 1;
            break;
        case 0:
            motor_l_power_c = 0.0f;
            motor_r_power_c = 0.0f;
            d_l_1 = 0;
            d_l_2 = 0;
            d_r_1 = 0;
            d_r_2 = 0;
            break;
    }
}

void Output_motion(){
    motor_l_power = motor_l_power_c;
    motor_r_power = motor_r_power_c;
    Right_motor.write(motor_r_power);
    Left_motor.write(motor_l_power);
    direction_left_1 = d_l_1;
    direction_left_2 = d_l_2;
    direction_right_1 = d_r_1;
    direction_right_2 = d_r_2;
}

void movement_main(){
    if (mode != 0){
        if (last_mode != mode){
          last_mode = mode;
            mode = 0;
            movement();
            Output_motion();
            ThisThread::sleep_for(5ms);

            mode = 1;
            movement();
            Output_motion();
            ThisThread::sleep_for(75ms);

            mode = 0;
            movement();
            Output_motion();
            ThisThread::sleep_for(5ms);

            mode = last_mode;
            movement();
            Output_motion();
        }
        else{
            last_mode = mode;
            movement();
            Output_motion();
        }
    }
    else{
        mode = last_mode;
        movement();
        Output_motion();
    }
    
}

//Line Following
void sensor_in_to_s(){
    s_1 = !Sensor_1_line;
    s_2 = !Sensor_2_line;
    s_3 = !Sensor_3_line;
    s_4 = !Sensor_4_line;
    s_5 = !Sensor_5_line;
}

void movement_spin_left(){
    while((s_3 == 0)){
        sensor_in_to_s();
        mode = 6;
        movement();
        Output_motion();
        ThisThread::sleep_for(5ms);
    }
}

void movement_spin_right(){
    while((s_3 == 0)){
        sensor_in_to_s();
        mode = 7;
        movement();
        Output_motion();
        ThisThread::sleep_for(5ms);
    }
    
}

void sensor_movement_control_right_angle(){
    mode_movement_right_angle = 1;
    if (s_3 == 1 && s_4 == 1 && s_5 == 1 && s_1 == 0 && s_2 == 0){
        while (s_4 == 0 || s_5 == 0){
            sensor_in_to_s();
            mode = 7;
            movement();
            Output_motion();
            ThisThread::sleep_for(5ms);
        }
        movement_spin_right();
    }else if (s_1 == 1 && s_2 == 1 && s_3 == 1 && s_4 == 0 && s_5 == 0){
        while (s_1 == 0 || s_2 == 0){
            sensor_in_to_s();
            mode = 6;
            movement();
            Output_motion();
            ThisThread::sleep_for(5ms);

        }
        movement_spin_left();
    }
    mode_movement_right_angle = 0;

}

void sensor_movement_spin_control(){
    if (mode_movement_spin == 1)
    {
        if (mode_movement_spin_left == 1)
        {
            movement_spin_left();
            mode_movement_spin_left = 0;
            mode_movement_spin = 0;
        }else if (mode_movement_spin_right == 1)
        {
            movement_spin_right();
            mode_movement_spin_right = 0;
            mode_movement_spin = 0;
        }else {
            sensor_in_to_s();
            if (s_4 == 1){
                mode_movement_spin_right = 1;
            }
            else if (s_2 == 1){
                mode_movement_spin_left = 1;
            }else{
                mode_movement_spin = 0;
            }
        }

    }
}

void OffLine_checker_reset(){
    Timer_movement_offline_checker.stop();
    Timer_movement_offline_checker.reset();
    mode_Timer_movement_offline_checker = 0;
}

void LineSensor_movement_fail_protocal(){
    if (mode_LineSensor_movement_fail == 1)
    {
        movement_speed_ratio = 0.5f;
        mode = 1;
        movement();
        Output_motion();
        ThisThread::sleep_for(100ms);
        Timer_movement_offline_recovery_movement.reset();
        Timer_movement_offline_recovery_movement.start();
        mode_Timer_movement_offline_recovery_movement = 1;
        while(mode_Timer_movement_offline_recovery_movement == 1){
            sensor_in_to_s();
            Timer_movement_offline_recovery_movement_read_ms = Timer_movement_offline_recovery_movement.read_ms();
            if (s_1 == 1 || s_2 == 1 || s_3 == 1 || s_4 == 1 || s_5 == 1){
                mode_LineSensor_movement_fail = 0;
                Timer_movement_offline_recovery_movement.stop();
                Timer_movement_offline_recovery_movement.reset();
                mode_Timer_movement_offline_recovery_movement = 0;
                movement_speed_ratio = 1.0f;
                break;
            }else if (Timer_movement_offline_recovery_movement_read_ms > 5000)
            {
                mode = 1;
                movement();
                Output_motion();
                ThisThread::sleep_for(100ms);
                mode_Timer_movement_offline_recovery_movement = 0;
                mode_Timer_movement_offline_recovery_movement_spin = 1;
            }else{
                mode = 3;
                movement();
                Output_motion();
                ThisThread::sleep_for(10ms);
            }
        }
        Timer_movement_offline_recovery_movement.reset();
        Timer_movement_offline_recovery_movement.start();
        while(mode_Timer_movement_offline_recovery_movement_spin == 1){
            sensor_in_to_s();
            Timer_movement_offline_recovery_movement_read_ms = Timer_movement_offline_recovery_movement.read_ms();
            if (s_1 == 1 || s_2 == 1 || s_3 == 1 || s_4 == 1 || s_5 == 1){
                if (s_4 ==1 || s_5 == 1){
                    mode = 4;
                    movement();
                    Output_motion();
                    ThisThread::sleep_for(150ms);
                    mode_LineSensor_movement_fail = 0;
                    Timer_movement_offline_recovery_movement.stop();
                    mode_Timer_movement_offline_recovery_movement_spin = 0;
                    movement_speed_ratio = 1.0f;
                    break;
                }else{
                    mode_LineSensor_movement_fail = 0;
                    Timer_movement_offline_recovery_movement.stop();
                    mode_Timer_movement_offline_recovery_movement_spin = 0;
                    movement_speed_ratio = 1.0f;
                    break;
                }
            }else if (Timer_movement_offline_recovery_movement_read_ms > 1000)
            {
                mode = 1;
                movement();
                Output_motion();
                ThisThread::sleep_for(100ms);
                exit(0);
            }else{
                mode = 7;
                movement();
                Output_motion();
            }
        }
    }
}

void sensor_movement_control(){
    sensor_in_to_s();
    if ((s_3 == 1 && s_4 == 1 && s_5 == 1 && s_1 == 0) || (s_1 == 1 && s_2 == 1 && s_3 == 1 && s_4 == 0 && s_5 == 0)){
        movement_speed_ratio = 0.7f;
        mode_movement_right_angle = 1;
        sensor_movement_control_right_angle();
        OffLine_checker_reset();
        
    }else if (s_3 == 1 && s_1 == 0 && s_2 == 0 && s_4 == 0 && s_5 == 0){
        OffLine_checker_reset();
        mode = 2;

    }else if ((s_2 == 1 && s_1 == 0 && s_3 == 0 && s_4 == 0 && s_5 == 0) || (s_1 == 1 && s_2 == 0 && s_3 == 0 && s_4 == 0 && s_5 == 0) || (s_2 == 1 && s_1 == 1 && s_3 == 0 && s_4 == 0 && s_5 == 0)){
        OffLine_checker_reset();
        mode_movement_spin_left = 1;
        mode_movement_spin = 1;

        sensor_movement_spin_control();
    }else if ((s_4 == 1 && s_1 == 0 && s_2 == 0 && s_3 == 0 && s_5 == 0) || (s_5 == 1 && s_1 == 0 && s_2 == 0 && s_3 == 0 && s_4 == 0) || (s_5 == 1 && s_1 == 0 && s_2 == 0 && s_3 == 0 && s_4 == 1)){
        OffLine_checker_reset();
        mode_movement_spin_right = 1;
        mode_movement_spin = 1;

        sensor_movement_spin_control();
    }else{
        mode = 2;
        if (mode_Timer_movement_offline_checker == 0)
        {
            Timer_movement_offline_checker.reset();
            Timer_movement_offline_checker.start();
            mode_Timer_movement_offline_checker = 1;
        }
        Timer_movement_offline_checker_read_ms = Timer_movement_offline_checker.read_ms();

        if (Timer_movement_offline_checker_read_ms > OffLine_clock && mode_Timer_movement_offline_checker == 1)
        {
            mode_LineSensor_movement_fail = 1;
            OffLine_checker_reset();
            LineSensor_movement_fail_protocal();
            if (s_3 == 1){
                OffLine_clock = 6000;
            }else{
                OffLine_clock = 750;
            }

        }
    }

}

//Traffic Light
// Read the shared color sensor and block while a red traffic light is detected.
// This function samples the sensor for a short period, decides whether a red
// light is present, and if so puts the vehicle into STOP (mode=1) until the
// red condition clears or a safety timeout elapses.
void traffic_light_detection(){
    using namespace tcs3472;
    if (!color_sensor_ptr) return; // sensor not available

    const int sample_timeout_ms = 800; // how long to collect samples
    const int stop_max_ms = 10000;     // max time to wait while red

    Timer t; t.start();
    int samples = 0;
    float rn_acc = 0.0f, gn_acc = 0.0f, bn_acc = 0.0f;

    // Collect a few stable samples (non-blocking-ish; read_raw waits for data)
    while (t.read_ms() < sample_timeout_ms) {
        RGBC v = color_sensor_ptr->read_raw(true, 300);
        if (!v.valid || v.c == 0) { ThisThread::sleep_for(30ms); continue; }
        float rn = (float)v.r / (float)v.c;
        float gn = (float)v.g / (float)v.c;
        float bn = (float)v.b / (float)v.c;
        rn_acc += rn; gn_acc += gn; bn_acc += bn;
        samples++;
        ThisThread::sleep_for(30ms);
    }

    if (samples == 0) return; // no reliable reading

    float rn = rn_acc / samples;
    float gn = gn_acc / samples;
    float bn = bn_acc / samples;

    // Simple red classifier (tweak thresholds with real data)
    bool is_red = (rn > 0.45f && rn > gn + 0.12f && rn > bn + 0.12f);

    if (is_red) {
        // Stop the vehicle while a red light is present (but cap wait time)
        mode = 1;
        movement(); Output_motion();

        Timer tw; tw.start();
        while (tw.read_ms() < stop_max_ms) {
            RGBC v = color_sensor_ptr->read_raw(true, 300);
            if (!v.valid || v.c == 0) { ThisThread::sleep_for(50ms); continue; }
            float rnn = (float)v.r / (float)v.c;
            float gnn = (float)v.g / (float)v.c;
            // consider cleared if red fraction drops or green becomes comparable
            if (rnn < 0.35f || gnn > rnn - 0.08f) break;
            ThisThread::sleep_for(100ms);
        }
    }
}

//Ultrasonic 
//Front ultrasonic
void ultrasonic_front_detection(){
    if (ultrasonic_front_distance_measured < 0.2f){
        condition_ultrasonic_front_obstacle_detection = 1;
    }else if (ultrasonic_front_distance_measured >= 0.2f){
        condition_ultrasonic_front_obstacle_detection = 0;
    }
}

void ultrasonic_front_measurement(){
    ultrasonic_front_detection();
        while(number_of_trig_send_front < 1){
        if (ultrasonic_front_echo_state == 0){

            ultrasonic_front_trig = 1;
            wait_us(10);

            ultrasonic_front_trig = 0;
            ultrasonic_front_echo_state = 1;

            Timer_ultrasonic_front_failing_recovery_protocal.reset();
            Timer_ultrasonic_front_failing_recovery_protocal.start();

        }
            

        if (ultrasonic_front_echo == 1 && ultrasonic_front_echo_state == 1){
            
            Timer_ultrasonic_front_distance_measurement.reset();
            Timer_ultrasonic_front_distance_measurement.start();

            ultrasonic_front_echo_state = 2;
            
        }else if (ultrasonic_front_echo == 0 && ultrasonic_front_echo_state == 2){
            
            Timer_ultrasonic_front_distance_measurement.stop();
            Timer_ultrasonic_front_distance_measurement_read = Timer_ultrasonic_front_distance_measurement.read_us();

            ultrasonic_front_distance_measured = (Timer_ultrasonic_front_distance_measurement_read / 1e6f) * (ultrasonic_speed_of_sound / 2);
            
            ultrasonic_front_echo_state = 0;
            number_of_trig_send_front ++;

        }
        
        Timer_ultrasonic_front_failing_recovery_protocal_read = Timer_ultrasonic_front_failing_recovery_protocal.read_ms();
        if (Timer_ultrasonic_front_failing_recovery_protocal_read > 100.0f){
            
            ultrasonic_front_echo_state = 0;
            Timer_ultrasonic_front_failing_recovery_protocal.stop();

        }
    }
    number_of_trig_send_front = 0;
}

//Side ultrasonic
void ultrasonic_side_detection(){
    if (ultrasonic_side_distance_measured < 0.2f){
        condition_ultrasonic_side_obstacle_detection = 1;
    }else if (ultrasonic_side_distance_measured >= 0.25f){
        condition_ultrasonic_side_obstacle_detection = 0;
    }
}

void ultrasonic_side_measurement(){
    ultrasonic_side_detection();
    while(number_of_trig_send_side < 3){
        if (ultrasonic_side_echo_state == 0){

            ultrasonic_side_trig = 1;
            wait_us(10);

            ultrasonic_side_trig = 0;
            ultrasonic_side_echo_state = 1;

            Timer_ultrasonic_side_failing_recovery_protocal.reset();
            Timer_ultrasonic_side_failing_recovery_protocal.start();

        }
            

        if (ultrasonic_side_echo == 1 && ultrasonic_side_echo_state == 1){
            
            Timer_ultrasonic_side_distance_measurement.reset();
            Timer_ultrasonic_side_distance_measurement.start();

            ultrasonic_side_echo_state = 2;
            
        }else if (ultrasonic_side_echo == 0 && ultrasonic_side_echo_state == 2){
            
            Timer_ultrasonic_side_distance_measurement.stop();
            Timer_ultrasonic_side_distance_measurement_read = Timer_ultrasonic_side_distance_measurement.read_us();

            ultrasonic_side_distance_measured = (Timer_ultrasonic_side_distance_measurement_read / 1e6f) * (ultrasonic_speed_of_sound / 2);
            
            ultrasonic_side_echo_state = 0;
            number_of_trig_send_side ++;

        }
        
        Timer_ultrasonic_side_failing_recovery_protocal_read = Timer_ultrasonic_side_failing_recovery_protocal.read_ms();
        if (Timer_ultrasonic_side_failing_recovery_protocal_read > 100.0f){
            
            ultrasonic_side_echo_state = 0;
            Timer_ultrasonic_side_failing_recovery_protocal.stop();
            number_of_trig_send_side ++;
        }
    }
    number_of_trig_send_side = 0;
}

//obsticale avoidance
void obstacle_avoidance_main(){
    while (mode_obstacle_avoidance == 1){
        ultrasonic_front_measurement();
        ultrasonic_side_measurement();
        if (condition_ultrasonic_front_obstacle_detection == 1){
            while (condition_ultrasonic_side_obstacle_detection == 0){
                ultrasonic_front_measurement();
                ultrasonic_side_measurement();
                mode = 6;
                movement();
                Output_motion();
                ThisThread::sleep_for(5ms);
                
            }
        }
        if (ultrasonic_side_distance_measured < 0.15f){
            mode = 2;
            movement();
            Output_motion();
            ThisThread::sleep_for(5ms);
        }
        else if (0.25f >= ultrasonic_side_distance_measured && ultrasonic_side_distance_measured >= 0.30f){
            mode = 5;
            movement();
            Output_motion();
            ThisThread::sleep_for(5ms);
        }
        else if (0.35f > ultrasonic_side_distance_measured && ultrasonic_side_distance_measured > 0.30f){
            mode = 7;
            movement();
            Output_motion();
            ThisThread::sleep_for(5ms);
        }else if (ultrasonic_side_distance_measured >= 0.35f){
            mode = 2;
            movement();
            Output_motion();
            ThisThread::sleep_for(5ms);
        }
        if(s_1 == 1 || s_2 == 1 || s_3 == 1){
            mode_obstacle_avoidance = 0;
        }
        if(s_4 == 1 || s_5 == 1){
            while(s_1 == 0 || s_2 == 0 || s_3 == 0){
                sensor_in_to_s();
                mode = 4;
                movement();
                Output_motion();
                ThisThread::sleep_for(5ms);
            }
            mode = 1;
            movement();
            Output_motion();
            ThisThread::sleep_for(100ms);
            mode_obstacle_avoidance = 0;
        }
    }

}

//Movement_Control_Main
void movement_control_main(){
    // Debug: print basic state once per call (helps determine why motors don't start)
    dbg_printf("SW=%d s1..s5=%d%d%d%d%d uf=%.3f us=%.3f mode=%d sensor=%d\r\n",
               Switch_On_Off, s_1, s_2, s_3, s_4, s_5,
               ultrasonic_front_distance_measured, ultrasonic_side_distance_measured,
               mode, (color_sensor_ptr != nullptr));

    if (Switch_On_Off == 0){
        mode = 0;
    }else{
        traffic_light_detection();
        ultrasonic_front_measurement();
        if (ultrasonic_front_distance_measured < 0.3f){
            mode_obstacle_avoidance = 1;
            obstacle_avoidance_main();
        }else{
            sensor_movement_control();
        }
    }
    
    
}

//Main function
int main()
{
    Right_motor.period(period_motor);
    Right_motor.write(0);
    Left_motor.period(period_motor);
    Left_motor.write(0);
    
    // Create and initialize the shared color sensor once (used by traffic light logic)
    {
        using namespace tcs3472;
        static TCS3472 sensor(PTE0, PTE1, 100000);
        color_sensor_ptr = &sensor;
        // try to init; if init fails, clear pointer so traffic code won't run
        if (!sensor.init(50.0f, Gain::X4)) {
            color_sensor_ptr = nullptr;
        }
    }
    

    while (true) {
        movement_control_main();
        movement_main();
    }
}