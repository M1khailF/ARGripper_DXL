#include "canmotorlib.h"
#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

CAN_device_t CAN_cfg;
CAN_frame_t rx_frame;
CAN_frame_t tx_frame;

const int rx_queue_size = 10;
int sizeMsg = 8;

uint8_t canMotorTx[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
//uint8_t canRx[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
bool debugMode = false;

float physicalArray[3] = {0.0, 0.0, 0.0};

const int maxRawKp = pow(2, 12) - 1;
const int maxRawKd = pow(2, 12) - 1;
bool motorInit = false;

const float maxAcceleration = 0.1;
float maxSpeed = 2.5;
float currentSpeed = 0;

struct MotorParams {
  float P_MIN;
  float P_MAX;
  float V_MIN;
  float V_MAX;
  float KP_MIN;
  float KP_MAX;
  float KD_MIN;
  float KD_MAX;
  float T_MIN;
  float T_MAX;
  int AXIS_DIRECTION;
};

const PROGMEM MotorParams AK80_6_V1_PARAMS = {
  -95.5,
  95.5,
  -45.0,
  45.0,
  0.0,
  500,
  0.0,
  5.0,
  -18.0,
  18.0,
  -1
};

const PROGMEM MotorParams AK80_6_V1p1_PARAMS = {
  -12.5,
  12.5,
  -22.5,
  22.5,
  0.0,
  500,
  0.0,
  5.0,
  -12.0,
  12.0,
  -1
};

MotorParams motorParams;

String uint12ToBinaryString(uint16_t val) {
  String result = "";
  for (int i = 11; i >= 0; i--) {
    result += ((val >> i) & 0x01) ? "1" : "0";
  }
  return result;
}

void canmotorlib::setVelocity(float value){
  maxSpeed = value;
}

bool canmotorlib::getMotorInit(){
  return motorInit;
}

String uint16ToBinaryString(uint16_t val) {
  String result = "";
  for (int i = 15; i >= 0; i--) {
    result += ((val >> i) & 0x01) ? "1" : "0";
  }
  return result;
}

String uint8ToBinaryString(uint8_t val) {
  String result = "";
  for (int i = 7; i >= 0; i--) {
    result += ((val >> i) & 0x01) ? "1" : "0";
  }
  return result;
}

int float_to_uint(float x, float x_min, float x_max, int numBit) {
  if (x < x_min) {
    x = x_min;
  } else if (x > x_max) {
    x = x_max;
  }

  float scaled = (x - x_min) / (x_max - x_min);
  int result = int(scaled * ((1 << numBit) - 1));

  return result;
}

float uint_to_float(int value, float x_min, float x_max, int numBit) {
  float scaled = float(value) / ((1 << numBit) - 1);
  return (scaled * (x_max - x_min)) + x_min;
}

union UInt32Converter {
  uint32_t uintValue;
  byte bytes[4];
};

void intToBytes(int val, byte bytes[2]) {
  bytes[0] = (val >> 8) & 0xFF;
  bytes[1] = val & 0xFF;
}

float canmotorlib::radians_to_degrees(float radians) {
  return radians * 180.0 / PI;
}

float degrees_to_radians(float degrees) {
  return degrees * PI / 180.0;
}

canmotorlib::canmotorlib(uint8_t motorID, String motorType) {
  if (debugMode) {
    Serial.begin(115200);
  }

  CAN_cfg.speed = CAN_SPEED_1000KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));

  ESP32Can.CANInit();

  motor_id = motorID;
  motor_type = motorType;

  if (motor_type == "AK80_6_V1") {
    motorParams = AK80_6_V1_PARAMS;
  } else if (motor_type == "AK80_6_V1p1") {
    motorParams = AK80_6_V1p1_PARAMS;
  }
}

uint8_t* canmotorlib::recv_can_frame() {
  static uint8_t canRead[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 1 * portTICK_PERIOD_MS) == pdTRUE) {
    for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
      canRead[i] = rx_frame.data.u8[i];
    }
    motorInit = true;
  }
  else {
    Serial.println(" ");
    Serial.println("Error receiving from the queue!");
  }

  return canRead;
}

void canmotorlib::send_can_frame(uint8_t *data) {
  if (debugMode) {
    Serial.println("");
    Serial.print("Send byte: ");

    for (int i = 0; i < sizeMsg; i++) {
      canMotorTx[i] = data[i];
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println("");
  }

  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.MsgID = motor_id;
  tx_frame.FIR.B.DLC = sizeMsg;
  tx_frame.data.u8[0] = data[0];
  tx_frame.data.u8[1] = data[1];
  tx_frame.data.u8[2] = data[2];
  tx_frame.data.u8[3] = data[3];
  tx_frame.data.u8[4] = data[4];
  tx_frame.data.u8[5] = data[5];
  tx_frame.data.u8[6] = data[6];
  tx_frame.data.u8[7] = data[7];
  ESP32Can.CANWriteFrame(&tx_frame);
}

uint8_t* canmotorlib::getCanMsg() {
  static uint8_t arr[8] = {canMotorTx[0], canMotorTx[1], canMotorTx[2], canMotorTx[3], canMotorTx[4], canMotorTx[5], canMotorTx[6], canMotorTx[7]};
  return arr;
}

void canmotorlib::waitOhneSleep(int dt) {
  delay(dt);
}

float* canmotorlib::recv_motor_data() {
  uint8_t* motorStatusData = recv_can_frame();
  int* rawData = decode_motor_status(motorStatusData);
  float* result = convert_raw_to_physical_rad(rawData[0], rawData[1], rawData[2]);
  delete[] rawData;

  return result;
}

float* canmotorlib::enable_motor() {
  uint8_t canTx[sizeMsg] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  send_can_frame(canTx);
  waitOhneSleep(1);
  float* result = recv_motor_data();

  if (debugMode == true) {
    Serial.print("Position: ");
    Serial.println(result[0]);
    Serial.print("Velocity: ");
    Serial.println(result[1]);
    Serial.print("Current: ");
    Serial.println(result[2]);
  }

  delay(100);
  return result;
}

int* canmotorlib::decode_motor_status(uint8_t* data_frame) {
  int* intArr = new int[3];
  if (debugMode == true) {
    Serial.print("Recv byte: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(data_frame[i], HEX);
      Serial.print(" ");
    }
    Serial.println("");
  }

  String dataBitString = "";
  for (int i = 0; i < 6; i++) {
    dataBitString += uint8ToBinaryString(data_frame[i]);
  }

  String positionBitString = dataBitString.substring(8, 24);
  String velocityBitString = dataBitString.substring(24, 36);
  String currentBitString = dataBitString.substring(36, 48);

  int positionRawValue = strtol(positionBitString.c_str(), NULL, 2);
  int velocityRawValue = strtol(velocityBitString.c_str(), NULL, 2);
  int currentRawValue = strtol(currentBitString.c_str(), NULL, 2);

  intArr[0] = positionRawValue;
  intArr[1] = velocityRawValue;
  intArr[2] = currentRawValue;

  return intArr;
}

void canmotorlib::disable_motor() {
  uint8_t canTx[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  send_can_frame(canTx);
}

float* canmotorlib::set_zero_position() {
  uint8_t canTx[sizeMsg] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  send_can_frame(canTx);

  waitOhneSleep(1);
  float* result = recv_motor_data();

  return result;
}

int* canmotorlib::convert_physical_rad_to_raw(float p_des_rad, float v_des_rad, float kp, float kd, float tau_ff) {
  int* arrRaw = new int[5];
  p_des_rad = p_des_rad * motorParams.AXIS_DIRECTION;
  v_des_rad = v_des_rad * motorParams.AXIS_DIRECTION;
  tau_ff = tau_ff * motorParams.AXIS_DIRECTION;

  int rawPosition = float_to_uint(p_des_rad, motorParams.P_MIN, motorParams.P_MAX, 16);
  int rawVelocity = float_to_uint(v_des_rad, motorParams.V_MIN, motorParams.V_MAX, 12);
  int rawTorque = float_to_uint(tau_ff, motorParams.T_MIN, motorParams.T_MAX, 12);
  int rawKp = (maxRawKp * kp) / motorParams.KP_MAX;
  int rawKd = (maxRawKd * kd) / motorParams.KD_MAX;

  arrRaw[0] = rawPosition;
  arrRaw[1] = rawVelocity;
  arrRaw[2] = rawKp;
  arrRaw[3] = rawKd;
  arrRaw[4] = rawTorque;

  return arrRaw;
}

float* canmotorlib::convert_raw_to_physical_rad(int positionRawValue, int velocityRawValue, int currentRawValue) {
  float physicalPositionRad = uint_to_float(positionRawValue, motorParams.P_MIN, motorParams.P_MAX, 16);
  float physicalVelocityRad = uint_to_float(velocityRawValue, motorParams.V_MIN, motorParams.V_MAX, 12);
  float physicalCurrent = uint_to_float(currentRawValue, motorParams.T_MIN, motorParams.T_MAX, 12);

  physicalArray[0] = physicalPositionRad * motorParams.AXIS_DIRECTION;
  physicalArray[1] = physicalVelocityRad * motorParams.AXIS_DIRECTION;
  physicalArray[2] = physicalCurrent * motorParams.AXIS_DIRECTION;

  return physicalArray;
}

void canmotorlib::send_rad_command(float p_des_rad, float v_des_rad, float kp, float kd, float tau_ff) {
  if (tau_ff < motorParams.T_MIN) {
    tau_ff = motorParams.T_MIN;
  }
  else if (tau_ff > motorParams.T_MAX) {
    tau_ff = motorParams.T_MAX;
  }

  p_des_rad = constrain(p_des_rad, motorParams.P_MIN, motorParams.P_MAX);
  v_des_rad = constrain(v_des_rad, motorParams.V_MIN, motorParams.V_MAX);
  kp = constrain(kp, motorParams.KP_MIN, motorParams.KP_MAX);
  kd = constrain(kd, motorParams.KD_MIN, motorParams.KD_MAX);

  int* result = convert_physical_rad_to_raw(p_des_rad, v_des_rad, kp, kd, tau_ff);
  send_raw_command(result[0], result[1], result[2], result[3], result[4]);
  delete[] result;
}

float* canmotorlib::send_deg_command(float p_des_deg, float v_des_deg, float kp, float kd, float tau_ff) {
  float p_des_rad = degrees_to_radians(p_des_deg);
  float v_des_rad = degrees_to_radians(v_des_deg);
  send_rad_command(p_des_rad, v_des_rad, kp, kd, tau_ff);
  float* result = recv_motor_data();

  return result;
}

void canmotorlib::send_raw_command(int rawPosition, int rawVelocity, int rawKp, int rawKd, int rawTorque) {
  byte resultBytes[sizeMsg];

  uint16_t _p_des_BitArray = rawPosition;
  uint16_t _v_des_BitArray = rawVelocity;
  uint16_t _kp_BitArray = rawKp;
  uint16_t _kd_BitArray = rawKd;
  uint16_t _tau_BitArray = rawTorque;

  uint16_t mask12 = 0b0000111111111111;
  uint16_t v_12bit = _v_des_BitArray & mask12;
  uint16_t kp_12bit = _kp_BitArray & mask12;
  uint16_t kd_12bit = _kd_BitArray & mask12;
  uint16_t tau_12bit = _tau_BitArray & mask12;

  String v_desBits = uint12ToBinaryString(v_12bit);
  String kpBits = uint12ToBinaryString(kp_12bit);
  String kdBits = uint12ToBinaryString(kd_12bit);
  String tauBits = uint12ToBinaryString(tau_12bit);
  String p_desBits = uint16ToBinaryString(_p_des_BitArray);

  String cmd_BitArray = p_desBits + v_desBits + kpBits + kdBits + tauBits;

  for (int i = 0; i < sizeMsg; i++) {
    resultBytes[i] = 0; // Инициализируем байт
    for (int j = 0; j < sizeMsg; j++) {
      resultBytes[i] |= (cmd_BitArray.charAt(i * 8 + j) - '0') << (7 - j);
    }
  }
  send_can_frame(resultBytes);
}

float canmotorlib::getStep(float _current_pos, float _pos_end) {
    const float distance = _pos_end - _current_pos;
    const float direction = (distance >= 0) ? 1 : -1;
    const float absDistance = abs(distance);
    
    if (absDistance > abs(9 * currentSpeed)) {
        currentSpeed += maxAcceleration * direction;
    } else if (absDistance <= 0.1) {
        currentSpeed = 0;
    } else {
        currentSpeed *= 0.9;
    }

    if (abs(currentSpeed) > maxSpeed) {
        currentSpeed = maxSpeed * direction;
    }

    return currentSpeed;
}

void canmotorlib::setCurrentStep(float value){
  currentSpeed = value;
}

float PIDController::calculate(float setpoint, float input) {
    float _dt = 1.00;
    float error = setpoint - input;
    integral_ += error * _dt;
    float derivative = (error - prev_error_) / _dt;
    prev_error_ = error;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

void PIDController::set_parameters(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::reset() {
    integral_ = 0;
    prev_error_ = 0;
}
