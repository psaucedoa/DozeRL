#ifndef DOZERL_H
#define DOZERL_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>

// GLOBAL PARAMS
#define SPATIAL_OBS_SIZE 50  // grid size for the map observations
#define GRID_SIZE 150
#define CELL_SIZE 0.2f
#define GRAVITY 9.81f
#define PI 3.14159265358979323846f
// for more global params, see env_reset(), since we'll be randomizing these values between runs

// Required PufferLib Log struct. Only use floats!
typedef struct {
    float perf; // Recommended 0-1 normalized. 0 = none of goal map built, 1 = all of goal map built
    float score; // Recommended unnormalized single real number perf metric
    float episode_return; // Recommended metric: sum of agent rewards over episode
    float episode_length; // Recommended metric: number of steps of agent episode

    float count_large_neg_rewards; // Custom metric: average large negative rewards per episode
    float count_off_map;  // Custom metric: average off-map steps per episode
    float count_jitter;   // Custom metric: average high-jitter steps per episode
    // max values
    float max_vel_arm;  // Custom metric: max arm angular velocity per episode
    float max_vel_blade_pitch;  // Custom metric: max blade pitch angular velocity per episode
    float max_vel_blade_roll;  // Custom metric: max blade roll velocity per episode
    float max_vel_linear;  // Custom metric: max linear vehicle velocity per episode
    // min values
    float min_vel_arm;  // Custom metric: min arm angular velocity per episode
    float min_vel_blade_pitch;  // Custom metric: min blade pitch angular velocity per episode
    float min_vel_blade_roll;  // Custom metric: min blade roll velocity per episode
    float min_vel_linear;  // Custom metric: min linear vehicle velocity per episode

    // reward signals per ep
    float r1_norm;
    float r2_norm;
    float r3_norm;
    float r4_norm;
    float r5_norm;

    float n; // Required as the last field
} Log;

typedef struct {
    // Spatial Origin (Center of Blade)
    // In vehicle frame, X = forward, Y = Lateral, Z = Up
    float angular_x;        // (rad)   Dozer angular position about X axis - roll
    float angular_y;        // (rad)   Dozer angular position about Y axis - pitch
    float angular_z;        // (rad)   Dozer angular position about Z axis - yaw
    float position_x;       // (m)     (hidden) global Longitudinal position
    float position_y;       // (m)     (hidden) global Lateral position (Y-axis in world coords)
    float position_z;       // (m)     (hidden) global Elevation (Z-axis in world coords)
    float twist_angular_x;  // (rad/s) Dozer angular velocity about X axis - roll
    float twist_angular_y;  // (rad/s) Dozer angular velocity about Y axis - pitch
    float twist_angular_z;  // (rad/s) Dozer angular velocity about Z axis - yaw
    float twist_linear_x;   // (m/s)   Dozer linear velocity about X axis
    float twist_linear_y;   // (m/s)   Dozer linear velocity about Y axis
    float twist_linear_z;   // (m/s)   Dozer linear velocity about Z axis

    // Effort Inputs (-1.0 to 1.0)
    float effort_lift;        // efort
    float effort_pitch;       // efort
    float effort_roll;        // efort
    float effort_yaw;         // efort
    float effort_linear;      // efort
    float effort_rotational;  // efort

    // Joint States POS
    float pos_tracks_rotational;
    float pos_tracks_linear;
    float pos_virtual_lift_arm;  // (rad) arm angle
    float pos_blade_pitch;       // (rad) blade pitch
    float pos_blade_roll;        // (rad) blade roll
    float pos_blade_yaw;         // (rad) blade yaw

    // Joint States VEL
    float vel_tracks_rotational;  // (rad/s) tracks rotational velocity
    float vel_tracks_linear;      // (m/s)   tracks linear velocity
    float vel_virtual_lift_arm;   // Current arm angular velocity (rad/s)
    float vel_blade_pitch;        // Current relative pitch velocity (rad/s)
    float vel_blade_roll;         // Current relative roll velocity (rad/s)
    float vel_blade_yaw;          // Current relative yaw velocity (rad/s)

    // Blade Geometry
    float blade_width;        // (m)
    float blade_rake_angle;   // (rad)
    float blade_surcharge_Q;  // (N)  TODO: CHECK UNITS
    float blade_x;            // (m)
    float blade_y;            // (m)
    float blade_z;            // (m)

    // joint limits
    float pos_virtual_lift_arm_min;  // (rad)
    float pos_virtual_lift_arm_max;  // (rad)
    float pos_blade_pitch_min;       // (rad)
    float pos_blade_pitch_max;       // (rad)
    float pos_blade_roll_min;        // (rad)
    float pos_blade_roll_max;        // (rad)
    float pos_blade_yaw_min;         // (rad)
    float pos_blade_yaw_max;         // (rad)

    // Dynamics
    float last_force;       // (N)
    float last_yaw_moment;  // (N*m)

    // Actuator Dynamics Constants
    float max_torque_list_arm;
    float max_torque_pitch;
    float max_torque_roll;
    float max_torque_rotational;
    float max_force_track;
    float virtual_lift_arm_damping;
    float blade_pitch_damping;
    float blade_roll_damping;
    float track_damping;
    float hydraulic_stiffness;

    // Masses
    float arm_mass;
    float pitch_mass;
    float blade_mass;
    float machine_mass;

    // Inertias
    float arm_inertia;
    float pitch_intertia;
    float roll_inertia;
    float machine_inertia;

} Dozer;

typedef struct {
    Log log; // Required field. Env binding code uses this to aggregate logs
    float* observations; // Required
    float* actions; // Required
    float* rewards; // Required
    float* terminals; // Required
    int num_agents;

    // maps
    float grid_H[GRID_SIZE][GRID_SIZE];     // (m) Hard Soil
    float grid_L[GRID_SIZE][GRID_SIZE];     // (m) Loose Soil
    float grid_G[GRID_SIZE][GRID_SIZE];     // (m) Goal Map
    float original_H[GRID_SIZE][GRID_SIZE]; // (m) Original Terrain Map (to prevent moving soil penalty)
    char map_region[GRID_SIZE][GRID_SIZE];  // ( ) 0=Neutral, 1=Cut, 2=Fill

    // height params (may change episode-to-episode)
    float H_cfp_max;  // max expected pile size / error
    float dH_max;     // expected maximum displacement per cell

    // soil params (may change episode-to-episode)
    float swell_ratio;         // ( )       Volumetric change when going from compact to loose soil
    float loose_soil_density;  // (kg/m^3 ) Density
    float soil_c;
    float soil_c_a;
    float soil_phi;
    float soil_delta;
    float soil_gamma;

    Dozer dozer;
    int step_num;
    int tick;
    unsigned int rng;

    // Thread-safe precomputed FEE factors
    float N_gamma;
    float N_Q;
    float N_c;
    float N_ca;

    // Log metrics
    float initial_error;
    float episode_return;
    float count_off_map;
    float count_jitter;
    float count_large_neg_rewards;
    float r1;
    float r2;
    float r3;
    float r4;
    float r5;
} SoilEnv;

typedef struct {
    float proprioceptive[11];
    float spatial[2][SPATIAL_OBS_SIZE][SPATIAL_OBS_SIZE];
} Observation;

// Helper: Thread-safe random float [0, 1]
static inline float rand_f(unsigned int* seed) {
    return (float)rand_r(seed) / (float)RAND_MAX;
}

// Helper: Clamp action to [-1.0f, 1.0f]
static inline float clamp_action(float x) {
    return x < -1.0f ? -1.0f : (x > 1.0f ? 1.0f : x);
}

// Helper: Clamp index to valid grid range
static inline int clamp_idx(int idx) {
    return idx < 0 ? 0 : (idx >= GRID_SIZE ? GRID_SIZE - 1 : idx);
}