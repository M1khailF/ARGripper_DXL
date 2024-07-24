#ifndef canmotorlib_h
#define canmotorlib_h

#include "Arduino.h"

class canmotorlib {
  private:
    uint8_t motor_id;
    String motor_type;

  public:
    bool getMotorInit();
    void setCurrentStep(float value);
    canmotorlib(uint8_t motorID, String motorType);
    void send_can_frame(uint8_t *data);
    float* enable_motor();
    void disable_motor();
    float getStep(float _current_pos, float _pos_end);
    float* set_zero_position();
    float* recv_motor_data();
    float radians_to_degrees(float radians);
    uint8_t* getCanMsg();
    void waitOhneSleep(int dt);
    int* decode_motor_status(uint8_t* data_frame);
    uint8_t* recv_can_frame();
    void setVelocity(float value);
    float* send_deg_command(float p_des_deg, float v_des_deg, float kp, float kd, float tau_ff);
    void send_rad_command(float p_des_rad, float v_des_rad, float kp, float kd, float tau_ff);
    int* convert_physical_rad_to_raw(float p_des_rad, float v_des_rad, float kp, float kd, float tau_ff);
    float* convert_raw_to_physical_rad(int positionRawValue, int velocityRawValue, int currentRawValue);
    void send_raw_command(int p_des, int v_des, int kp, int kd, int tau_ff);
};

class PIDController {
  public:
    PIDController(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd), integral_(0), prev_error_(0) {}
    float calculate(float setpoint, float process_variable);
    void set_parameters(float kp, float ki, float kd);
    void reset();

  private:
    float kp_;
    float ki_;
    float kd_;
    float integral_;
    float prev_error_;
};

#endif
