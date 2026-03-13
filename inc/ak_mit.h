#pragma once

typedef unsigned char uint8_t;
typedef unsigned int uint32_t;
typedef signed char int8_t;
typedef signed int int32_t;
typedef short int16_t;

float fminf(float a, float b);
float fmaxf(float a, float b);

// ── Per-model parameter ranges (Manual p. 63) ─────────────────────────────
typedef struct {
    const char* name;
    float v_max;   // rad/s (symmetric ±)
    float t_max;   // N·m  (symmetric ±)
    // Position is ±12.5 rad for all models
} MotorModel;

static const MotorModel MOTOR_MODELS[] = {
    {"AK10-9", 50.0f, 65.0f},  {"AK60-6", 45.0f, 15.0f},
    {"AK70-10", 50.0f, 25.0f}, {"AK80-6", 76.0f, 12.0f},
    {"AK80-9", 50.0f, 18.0f},  {"AK80-64", 8.0f, 144.0f},
    {"AK80-8", 37.5f, 32.0f},  {"AK45-36", 6.0f, 34.0f},
    {"AK45-10", 20.0f, 8.0f},  {"AK40-10", 45.5f, 5.0f},
};

static const int NUM_MODELS = (int)(sizeof(MOTOR_MODELS)/sizeof(MOTOR_MODELS[0]));

// Active motor model index – default AK40-10 (index 9)
static int ak_model_idx = 9;  // default to AK40-10

// Convert float to unsigned int with given bit-width
int float_to_uint(float x, float x_min, float x_max, unsigned int bits);

// Convert unsigned int back to float
float uint_to_float(int x_int, float x_min, float x_max, int bits);

typedef struct {
    uint8_t standard_id; // per manual, all MIT commands use StdId = 1
    uint8_t data[8];
    uint8_t len;
} AKMotorMITMessage;

// Pack an MIT mode command into 8 bytes using the active motor model's ranges
void pack_mit_cmd(uint8_t out[8], float p_des, float v_des, float kp, float kd, float t_ff);

AKMotorMITMessage generate_mit_enter_message(uint8_t id);

AKMotorMITMessage generate_mit_exit_message(uint8_t id);

AKMotorMITMessage generate_mit_set_zero_message(uint8_t id);

AKMotorMITMessage generate_mit_command_message(uint8_t id, float p_des, float v_des, float kp, float kd, float t_ff);