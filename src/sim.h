#ifndef SIM_H
#define SIM_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define GRID_SIZE 150
#define PI 3.14159265358979323846f
#define CELL_SIZE 0.2f 

#define SWELL_RATIO 1.2f
#define LOOSE_SOIL_DENSITY 1300.0f
#define GRAVITY 9.81f

// Actuator Dynamics Constants
#define MAX_FORCE_LIFT 50000.0f
#define MAX_TORQUE_PITCH 15000.0f
#define MAX_TORQUE_ROLL 15000.0f
#define MAX_FORCE_LINEAR 40000.0f
#define MAX_TORQUE_ROTATIONAL 30000.0f

#define ARM_MASS 800.0f
#define PITCH_INERTIA 200.0f
#define ROLL_INERTIA 200.0f
#define MACHINE_MASS 4500.0f
#define MACHINE_INERTIA 6000.0f

// Machine Geometry Constants
#define TRACK_LENGTH 2.5f
#define TRACK_WIDTH 0.4f
#define TRACK_GAUGE 1.8f

#define ARM_DAMPING 40000.0f  // Reduced to allow arm to lift faster under effort
#define PITCH_DAMPING 50000.0f // Increased to slow down pitch
#define ROLL_DAMPING 50000.0f  // Increased to slow down roll
#define LINEAR_DAMPING 10000.0f
#define ROTATIONAL_DAMPING 15000.0f

#define HYDRAULIC_STIFFNESS 0.9998f // Increased to simulate extremely hard backdriving

#define ARM_MIN -0.1f
#define ARM_MAX 1.5f
#define PITCH_MIN (-30.0f * (PI / 180.0f))
#define PITCH_MAX (45.0f * (PI / 180.0f))
#define ROLL_MIN (-20.0f * (PI / 180.0f))
#define ROLL_MAX (20.0f * (PI / 180.0f))

#define SPATIAL_OBS_SIZE 50

typedef struct {
    // Spatial Origin (Center of Blade)
    float x;           
    float y; // Lateral position (Y-axis in world coords)
    float z; // Elevation (Z-axis in world coords)
    
    // Blade Geometry & State
    float width;       
    float rake_angle;  
    float surcharge_Q; 
    
    // 6DOF Kinematics
    float pitch;
    float roll;
    float yaw;
    
    // Actuators (Action Space)
    float v_linear;        // Current linear velocity
    float v_rotational;    // Current rotational velocity
    float arm_height;      // Current arm height (m)
    float blade_pitch_rel; // Current relative pitch (rad)
    float blade_roll_rel;  // Current relative roll (rad)
    float blade_yaw_rel;   // Current relative yaw (rad)
    
    // Effort Inputs (-1.0 to 1.0)
    float effort_lift;
    float effort_pitch;
    float effort_roll;
    float effort_yaw;      // Fixed at 0 for now
    float effort_linear;
    float effort_rotational;

    // Internal Actuator State (Velocities)
    float vel_arm_height;
    float vel_pitch_rel;
    float vel_roll_rel;
    float vel_yaw_rel;
    
    float loader_x;
    float loader_y;
    float loader_z;
    
    // Dynamics
    float last_force; 
    float last_yaw_moment;
} Blade;

typedef struct {
    float grid_H[GRID_SIZE][GRID_SIZE];
    float grid_L[GRID_SIZE][GRID_SIZE];
    float grid_G[GRID_SIZE][GRID_SIZE]; // Goal Map
    Blade blade;
    int step_num;
} SoilEnv;

typedef struct {
    float proprioceptive[11];
    float spatial[2][SPATIAL_OBS_SIZE][SPATIAL_OBS_SIZE];
} Observation;

// Global Soil Properties (declared extern here, defined in sim.c)
extern float env_c;
extern float env_c_a;
extern float env_phi;
extern float env_delta;
extern float env_gamma;

// Precomputed FEE Factors
extern float N_gamma, N_Q, N_c, N_ca;

extern FILE* outfile;

// Function Prototypes
void env_get_observation(SoilEnv* env, Observation* obs);
float env_get_reward(SoilEnv* env, float prev_error);
void precompute_FEE(float rake_angle, float alpha);
SoilEnv* env_init();
void env_free(SoilEnv* env);
void env_reset(SoilEnv* env, int seed);
float calculate_FEE_column(Blade* blade, float depth, float width);
float calculate_max_traction();
void simulate_erosion(SoilEnv* env);
void update_kinematics(SoilEnv* env);
void simulate_step(SoilEnv* env, float dt);

#endif
