#ifndef DOZERL_H
#define DOZERL_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>

#define GRID_SIZE 150
#define PI 3.14159265358979323846f
#define CELL_SIZE 0.2f 

#define SWELL_RATIO 1.2f
#define LOOSE_SOIL_DENSITY 1300.0f
#define GRAVITY 9.81f

static int global_episode_count = 0;

// Actuator Dynamics Constants
#define MAX_TORQUE_LIFT 100000.0f
#define MAX_TORQUE_PITCH 15000.0f
#define MAX_TORQUE_ROLL 15000.0f
#define MAX_FORCE_LINEAR 100000.0f
#define MAX_TORQUE_ROTATIONAL 80000.0f
#define ARM_INERTIA 5000.0f
#define ARM_MASS 800.0f
#define PITCH_INERTIA 200.0f
#define ROLL_INERTIA 200.0f
#define MACHINE_MASS 18000.0f
#define MACHINE_INERTIA 25000.0f

// Machine Geometry Constants
#define TRACK_LENGTH 2.5f
#define TRACK_WIDTH 0.4f
#define TRACK_GAUGE 1.5f

#define ARM_DAMPING 30000.0f  // Rotational damping on the loader arm
#define PITCH_DAMPING 50000.0f // Increased to slow down pitch
#define ROLL_DAMPING 50000.0f  // Increased to slow down roll
#define LINEAR_DAMPING 30000.0f
#define ROTATIONAL_DAMPING 40000.0f

#define HYDRAULIC_STIFFNESS 0.9998f // Increased to simulate extremely hard backdriving

#define ARM_MIN -0.5f  // (rad)
#define ARM_MAX 0.5f   // (rad)
#define PITCH_MIN (-30.0f * (PI / 180.0f)) // (rad)
#define PITCH_MAX (45.0f * (PI / 180.0f))  // (rad)
#define ROLL_MIN (-20.0f * (PI / 180.0f))  // (rad)
#define ROLL_MAX (20.0f * (PI / 180.0f))   // (rad)

#define SPATIAL_OBS_SIZE 50

// Soil parameters
static const float env_c = 500.0f;      // Reduced from 1000.0f for more realistic loose soil
static const float env_c_a = 500.0f;    
static const float env_phi = 30.0f * (PI / 180.0f); // Increased from 10.0f to 30.0f
static const float env_delta = 20.0f * (PI / 180.0f); 
static const float env_gamma = 1500.0f * GRAVITY; 

static FILE* outfile = NULL;

// Required PufferLib Log struct. Only use floats!
typedef struct {
    float perf; // Recommended 0-1 normalized single real number perf metric
    float score; // Recommended unnormalized single real number perf metric
    float episode_return; // Recommended metric: sum of agent rewards over episode
    float episode_length; // Recommended metric: number of steps of agent episode
    float count_large_neg_rewards; // Custom metric: average large negative rewards per episode
    float count_off_map;  // Custom metric: average off-map steps per episode
    float count_jitter;   // Custom metric: average high-jitter steps per episode
    float max_vel_arm;  // Custom metric: max arm angular velocity per episode
    float max_vel_blade_pitch;  // Custom metric: max blade pitch angular velocity per episode
    float max_vel_blade_roll;  // Custom metric: max blade roll velocity per episode
    float n; // Required as the last field
} Log;

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
    float v_linear;        // Current linear velocity (m/s) //velocity
    float v_rotational;    // Current rotational velocity (rad/s) //velocity
    float arm_height;      // Current arm angle (rad)
    float blade_pitch_rel; // Current relative pitch (rad)
    float blade_roll_rel;  // Current relative roll (rad)
    float blade_yaw_rel;   // Current relative yaw (rad)
    
    // Effort Inputs (-1.0 to 1.0)
    float effort_lift;       // //efort
    float effort_pitch;      // //efort
    float effort_roll;       // //efort
    float effort_yaw;        // //efort
    float effort_linear;     // //efort
    float effort_rotational; // //efort

    // Internal Actuator State (Velocities)
    float vel_arm_height;    // Current arm angular velocity (rad/s) //velocity
    float vel_pitch_rel;     // Current relative pitch velocity (rad/s) //velocity
    float vel_roll_rel;      // Current relative roll velocity (rad/s) //velocity
    float vel_yaw_rel;       // Current relative yaw velocity (rad/s) //velocity
    
    float loader_x;
    float loader_y;
    float loader_z;
    
    // Dynamics
    float last_force; 
    float last_yaw_moment;
} Blade;

typedef struct {
    Log log; // Required field. Env binding code uses this to aggregate logs
    float* observations; // Required
    float* actions; // Required
    float* rewards; // Required
    float* terminals; // Required
    int num_agents;

    float grid_H[GRID_SIZE][GRID_SIZE];
    float grid_L[GRID_SIZE][GRID_SIZE];
    float grid_G[GRID_SIZE][GRID_SIZE]; // Goal Map
    Blade blade;
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

// Function definitions

static inline void env_get_observation(SoilEnv* env, Observation* obs) {
    Blade* blade = &env->blade;
    
    // Proprioceptive observations
    obs->proprioceptive[0] = blade->pitch;               // chassis pitch (rad)
    obs->proprioceptive[1] = blade->roll;                // chassis roll (rad)
    obs->proprioceptive[2] = blade->surcharge_Q / 10000.0f; // surcharge
    obs->proprioceptive[3] = blade->arm_height;          // arm angle (rad)
    obs->proprioceptive[4] = blade->blade_pitch_rel;     // relative blade pitch (rad)
    obs->proprioceptive[5] = blade->blade_roll_rel;      // relative blade roll (rad)
    obs->proprioceptive[6] = blade->vel_arm_height;      // arm angular velocity (rad/s) //velocity
    obs->proprioceptive[7] = blade->vel_pitch_rel;       // blade relative pitch velocity (rad/s) //velocity
    obs->proprioceptive[8] = blade->vel_roll_rel;        // blade relative roll velocity (rad/s) //velocity
    obs->proprioceptive[9] = blade->v_linear;            // linear velocity (m/s) //velocity
    obs->proprioceptive[10] = blade->v_rotational;       // rotational velocity (rad/s) //velocity 

    // Spatial Observations (2-Channel Heightmap)
    float cos_y = cosf(blade->yaw);
    float sin_y = sinf(blade->yaw);

    float half_obs = SPATIAL_OBS_SIZE / 2.0f;
    float start_grid_x = (blade->x / CELL_SIZE) - half_obs * cos_y + half_obs * sin_y;
    float start_grid_y = (blade->y / CELL_SIZE) - half_obs * sin_y - half_obs * cos_y;

    for (int i = 0; i < SPATIAL_OBS_SIZE; i++) {
        float row_grid_x = start_grid_x + i * cos_y;
        float row_grid_y = start_grid_y + i * sin_y;
        
        for (int j = 0; j < SPATIAL_OBS_SIZE; j++) {
            int grid_i = (int)(row_grid_x - j * sin_y);
            int grid_j = (int)(row_grid_y + j * cos_y);

            if (grid_i >= 0 && grid_i < GRID_SIZE && grid_j >= 0 && grid_j < GRID_SIZE) {
                obs->spatial[0][i][j] = (env->grid_H[grid_i][grid_j] + env->grid_L[grid_i][grid_j]) - blade->loader_z;
                obs->spatial[1][i][j] = env->grid_G[grid_i][grid_j] - blade->loader_z;
            } else {
                obs->spatial[0][i][j] = 0.0f;
                obs->spatial[1][i][j] = 0.0f;
            }
        }
    }
}

static inline void precompute_FEE(SoilEnv* env, float rake_angle, float alpha) {
    float rho = rake_angle;
    float beta = (PI / 4.0f) - (env_phi / 2.0f);
    float eta = env_delta + rho + env_phi + beta;
    float sin_eta = sinf(eta);
    if (fabsf(sin_eta) < 1e-6f) sin_eta = 1e-6f;
    
    env->N_gamma = ((1.0f / tanf(rho)) + (1.0f / tanf(beta))) * sinf(alpha + env_phi + beta) / (2.0f * sin_eta);
    env->N_Q = sinf(alpha + env_phi + beta) / sin_eta;
    env->N_c = cosf(env_phi) / (sinf(beta) * sin_eta);
    env->N_ca = -cosf(rho + env_phi + beta) / (sinf(rho) * sin_eta);
}

static inline void env_reset(SoilEnv* env) {
    int episode = __atomic_fetch_add(&global_episode_count, 1, __ATOMIC_RELAXED);
    float phase = (float)episode / 50000000.0f;
    if (phase > 1.0f) phase = 1.0f;

    // 1. Initial Multi-scale Noise
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float fine = (rand_f(&env->rng) - 0.5f) * 0.1f * phase;
            float coarse = (rand_f(&env->rng) - 0.5f) * 0.3f * phase;
            env->grid_H[i][j] = 1.0f + fine + coarse;
            env->grid_L[i][j] = 0.0f;
        }
    }

    // 2. Multi-pass smoothing
    for (int iter = 0; iter < 2; iter++) {
        float smoothed[GRID_SIZE][GRID_SIZE];
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                float sum = 0; int count = 0;
                for (int di = -2; di <= 2; di++) {
                    for (int dj = -2; dj <= 2; dj++) {
                        int ni = i + di; int nj = j + dj;
                        if (ni >= 0 && ni < GRID_SIZE && nj >= 0 && nj < GRID_SIZE) {
                            sum += env->grid_H[ni][nj]; count++;
                        }
                    }
                }
                smoothed[i][j] = sum / count;
            }
        }
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                env->grid_H[i][j] = smoothed[i][j];
                env->grid_G[i][j] = smoothed[i][j];
            }
        }
    }

    // Blade initialization (before goal map to sync width)
    env->blade.width = 2.2f;

    // 3. Goal Map: Slot and Pile
    // Determine Pile Parameters First
    float k = 0.4f; 
    float pile_h = 0.5f + rand_f(&env->rng) * 1.0f; // 0.5m to 1.5m

    float tan_phi = tanf(env_phi);
    if (tan_phi < 0.1f) tan_phi = 0.1f; 

    float b_base = pile_h / ((1.0f - k) * tan_phi);
    
    // Clamp footprint to stay within realistic bounds
    float min_b = (env->blade.width * 1.1f) / 2.0f;
    float max_b = (GRID_SIZE * CELL_SIZE) * 0.25f; 
    if (b_base < min_b) b_base = min_b;
    if (b_base > max_b) b_base = max_b;

    float a_base = b_base * 0.6f; 

    float v_unit = (PI * a_base * b_base * (1.0f + k + k*k)) / 3.0f;
    float target_pile_vol = v_unit * pile_h;
    float required_slot_vol = target_pile_vol / SWELL_RATIO;

    float slot_W = env->blade.width;
    float slot_D = 0.2f; 
    float slot_L = required_slot_vol / (slot_W * slot_D);

    if (slot_L > 12.0f) {
        slot_L = 12.0f;
        slot_D = required_slot_vol / (slot_L * slot_W);
    }

    // Now that we have slot_L and b_base, calculate placement so it fits in the env
    // Curriculum: angle randomization grows from 0 to full 360 degrees
    float angle = (rand_f(&env->rng) - 0.5f) * (2.0f * PI * phase);
    
    // Curriculum: starting distance from slot grows from tight (3.0m) to far (up to 7.0m)
    float start_offset = 1.0f + (rand_f(&env->rng) * 2.0f) * phase;
    
    float dist_to_midpoint = (-start_offset + slot_L + 2.0f + b_base) / 2.0f;
    float total_length = start_offset + slot_L + 2.0f + b_base;
    
    float map_center = (GRID_SIZE * CELL_SIZE) / 2.0f;
    float slack = (GRID_SIZE * CELL_SIZE) - total_length;
    float jitter = slack > 4.0f ? 4.0f : (slack > 0.0f ? slack : 0.0f);
    
    // Curriculum: map centering jitter grows with phase
    float mid_x = map_center + (rand_f(&env->rng) - 0.5f) * jitter * phase;
    float mid_y = map_center + (rand_f(&env->rng) - 0.5f) * jitter * phase;
    
    float start_x = mid_x - dist_to_midpoint * cosf(angle);
    float start_y = mid_y - dist_to_midpoint * sinf(angle);

    float cos_a = cosf(angle);
    float sin_a = sinf(angle);

    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float dx = (i * CELL_SIZE) - start_x;
            float dy = (j * CELL_SIZE) - start_y;
            float lx = dx * cos_a + dy * sin_a;
            float ly = -dx * sin_a + dy * cos_a;
            if (lx >= 0 && lx <= slot_L && fabsf(ly) <= slot_W / 2.0f) {
                env->grid_G[i][j] -= slot_D;
            }
        }
    }

    float pile_center_x = start_x + (slot_L + 2.0f) * cos_a;
    float pile_center_y = start_y + (slot_L + 2.0f) * sin_a;

    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float dx = (i * CELL_SIZE) - pile_center_x;
            float dy = (j * CELL_SIZE) - pile_center_y;
            
            float lx = dx * cos_a + dy * sin_a;
            float ly = -dx * sin_a + dy * cos_a;
            
            float r = sqrtf((lx * lx) / (a_base * a_base) + (ly * ly) / (b_base * b_base));
            
            float h_add = 0;
            if (r <= k) {
                h_add = pile_h; 
            } else if (r < 1.0f) {
                h_add = pile_h * (1.0f - r) / (1.0f - k); 
            }
            
            if (h_add > 0.001f) {
                env->grid_G[i][j] += h_add;
            }
        }
    }

    // Blade State
    env->blade.loader_x = start_x - start_offset * cos_a;
    env->blade.loader_y = start_y - start_offset * sin_a;
    env->blade.x = start_x - (start_offset - 2.0f) * cos_a;
    env->blade.y = start_y - (start_offset - 2.0f) * sin_a;
    env->blade.z = 0.8f;
    env->blade.rake_angle = 45.0f * (PI / 180.0f);
    env->blade.surcharge_Q = 0.0f;
    env->blade.pitch = -0.35f;
    env->blade.roll = 0.0f;
    env->blade.yaw = angle;
    env->blade.v_linear = 0.0f;
    env->blade.v_rotational = 0.0f;
    env->blade.arm_height = 0.25f; // (rad)
    env->blade.blade_pitch_rel = 0.0f;
    env->blade.blade_roll_rel = 0.0f;
    env->blade.blade_yaw_rel = 0.0f;
    env->blade.effort_lift = 0.0f;
    env->blade.effort_pitch = 0.0f;
    env->blade.effort_roll = 0.0f;
    env->blade.effort_yaw = 0.0f;
    env->blade.effort_linear = 0.0f;
    env->blade.effort_rotational = 0.0f;
    env->blade.vel_arm_height = 0.0f;
    env->blade.vel_pitch_rel = 0.0f;
    env->blade.vel_roll_rel = 0.0f;
    env->blade.vel_yaw_rel = 0.0f;
    env->blade.loader_z = 0.0f;
    env->blade.last_force = 0.0f;
    env->blade.last_yaw_moment = 0.0f;
    env->step_num = 0;
}

static inline float calculate_FEE_column(SoilEnv* env, Blade* blade, float hard_depth, float total_depth, float width) {
    if (total_depth <= 0.0f) return 0.0f;
    float q = blade->surcharge_Q * (width / blade->width);
    float df = q * env->N_Q;
    if (hard_depth > 0.0f) {
        df += env_gamma * hard_depth * hard_depth * width * env->N_gamma +
              env_c * hard_depth * width * env->N_c +
              env_c_a * hard_depth * width * env->N_ca;
    }
    return df;
}

static inline float calculate_max_traction() {
    float track_area = 2.0f * TRACK_LENGTH * TRACK_WIDTH;
    float machine_weight = MACHINE_MASS * GRAVITY;
    return track_area * env_c + machine_weight * tanf(env_phi);
}

static inline float env_get_reward(SoilEnv* env, float prev_error) {
    float current_error = 0;
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float diff = (env->grid_H[i][j] + env->grid_L[i][j]) - env->grid_G[i][j];
            current_error += fabsf(diff);
        }
    }
    float error_scale = (env->initial_error > 1e-5f) ? env->initial_error : 1.0f;
    float terrain_reward = (prev_error - current_error) / error_scale;
    float reward = terrain_reward;
    
    // 1. Pushing Reward: Reward moving forward while carrying soil surcharge
    float push_reward = (env->blade.surcharge_Q / 10000.0f) * env->blade.v_linear * 0.01f;
    if (push_reward > 0.0f) {
        reward += push_reward;
    }

    float arm_penalty = 0.0f;
    if (env->blade.arm_height > 0.35f) { // (rad)
        arm_penalty = -0.01f;
        reward += arm_penalty;
    }

    float stationary_penalty = 0.0f;
    // 2. Stationary Penalty: Prevent agent from sitting still to avoid effort penalties
    if (fabsf(env->blade.effort_linear) < 0.05f) {
        stationary_penalty = -0.01f;
        reward += stationary_penalty;
    }

    // 3. Jitter Penalty: Penalize rapid movement/shaking of the blade actuators (arm lift, pitch, roll)
    float jitter_penalty = (env->blade.vel_arm_height * env->blade.vel_arm_height) * 1.0f +
                           (env->blade.vel_pitch_rel * env->blade.vel_pitch_rel) * 1.0f +
                           (env->blade.vel_roll_rel * env->blade.vel_roll_rel) * 1.0f;
    float jitter_term = -jitter_penalty * 0.05f;
    reward += jitter_term;

    // add huge min reward if the vehicle moves off the map
    // float off_map_penalty = 0.0f;
    // float max_coord = GRID_SIZE * CELL_SIZE;
    // if (env->blade.loader_x < 0.0f || env->blade.loader_x > max_coord ||
    //     env->blade.loader_y < 0.0f || env->blade.loader_y > max_coord ||
    //     env->blade.x < 0.0f || env->blade.x > max_coord ||
    //     env->blade.y < 0.0f || env->blade.y > max_coord) {
    //     off_map_penalty = -0.1f;
    //     reward += off_map_penalty;
    // }

    // if (off_map_penalty < 0.0f) {
    //     env->count_off_map += 1.0f;
    // }

    if (jitter_term < -0.05f) {
        env->count_jitter += 1.0f;
    }

    // if (reward < -0.5f) {
    //     env->count_large_neg_rewards += 1.0f;
    //     printf("[DozeRL Warning] Large negative reward detected (%.4f) at step %d:\n"
    //            "  - Terrain reward: %.4f (prev_err: %.2f, curr_err: %.2f, init_err: %.2f)\n"
    //            "  - Push reward: %.4f\n"
    //            "  - Arm penalty: %.4f (arm_height: %.2f)\n"
    //            "  - Stationary penalty: %.4f (effort_linear: %.2f)\n"
    //            "  - Jitter penalty: %.4f (vel_arm: %.2f, vel_pitch: %.2f, vel_roll: %.2f)\n"
    //            "  - Off-map penalty: %.4f (loader: %.2f, %.2f, blade: %.2f, %.2f)\n",
    //            reward, env->step_num,
    //            terrain_reward, prev_error, current_error, env->initial_error,
    //            push_reward > 0.0f ? push_reward : 0.0f,
    //            arm_penalty, env->blade.arm_height,
    //            stationary_penalty, env->blade.effort_linear,
    //            jitter_term, env->blade.vel_arm_height, env->blade.vel_pitch_rel, env->blade.vel_roll_rel,
    //            off_map_penalty, env->blade.loader_x, env->blade.loader_y, env->blade.x, env->blade.y);
    // }

    return reward;
}

static inline void simulate_erosion(SoilEnv* env) {
    Blade* blade = &env->blade;
    int margin = (int)(4.0f / CELL_SIZE);
    int center_i = (int)(blade->x / CELL_SIZE);
    int min_i = clamp_idx(center_i - margin);
    int max_i = clamp_idx(center_i + margin);
    int center_j = (int)(blade->y / CELL_SIZE);
    int min_j = clamp_idx(center_j - margin);
    int max_j = clamp_idx(center_j + margin);

    float loader_length = 2.0f;
    float tan_phi = tanf(env_phi);

    for (int iter = 0; iter < 3; iter++) {
        float tempL[GRID_SIZE][GRID_SIZE];
        for (int i = min_i; i <= max_i; i++) {
            for (int j = min_j; j <= max_j; j++) {
                tempL[i][j] = env->grid_L[i][j];
            }
        }
        
        int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
        int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
        for (int i = min_i + 1; i < max_i; i++) {
            for (int j = min_j + 1; j < max_j; j++) {
                if (env->grid_L[i][j] <= 1e-4f) continue;
                float total_h = env->grid_H[i][j] + env->grid_L[i][j];
                
                for(int d=0; d<8; d++) {
                    int ni = i + dx[d], nj = j + dy[d];
                    
                    float cell_x = ni * CELL_SIZE;
                    float cell_y = nj * CELL_SIZE;
                    float w = cell_y - blade->y;
                    if (fabsf(w) <= blade->width / 2.0f) {
                        float blade_front_x = blade->x - w * sinf(blade->yaw);
                        float loader_back_x = blade_front_x - loader_length * cosf(blade->yaw);
                        if (cell_x >= loader_back_x && cell_x <= blade_front_x) continue;
                    }
                    
                    float neighbor_total_h = env->grid_H[ni][nj] + env->grid_L[ni][nj];
                    float dH = total_h - neighbor_total_h;
                    float dist = (d < 4) ? CELL_SIZE : (CELL_SIZE * 1.41421356f);
                    
                    if (dH > 0.0f) {
                        float t = dH / dist;
                        float L_val = tempL[i][j];
                        if (L_val < 1e-5f) L_val = 1e-5f;
                        float K = env_c / (LOOSE_SOIL_DENSITY * GRAVITY * L_val);
                        float t_limit = tan_phi;
                        
                        if (K >= 1e-5f) {
                            float disc = 1.0f - 4.0f * K * (tan_phi + K);
                            if (disc > 0.0f) {
                                t_limit = (1.0f - sqrtf(disc)) / (2.0f * K);
                            } else {
                                t_limit = 1e9f;
                            }
                        }
                        
                        if (t > t_limit) {
                            float slip = (t - t_limit) * dist * 0.2f;
                            if (slip > tempL[i][j]) slip = tempL[i][j];
                            tempL[i][j] -= slip;
                            tempL[ni][nj] += slip;
                            total_h -= slip;
                        }
                    }
                }
            }
        }
        for (int i = min_i; i <= max_i; i++) {
            for (int j = min_j; j <= max_j; j++) {
                env->grid_L[i][j] = tempL[i][j];
            }
        }
    }
}

static inline void update_kinematics(SoilEnv* env) {
    Blade* blade = &env->blade;
    float cos_y = cosf(blade->yaw), sin_y = sinf(blade->yaw);
    
    float tan_phi = tanf(env_phi);
    if (tan_phi < 0.01f) tan_phi = 0.01f;
    
    float N_q_b = expf(PI * tan_phi) * powf(tanf((PI / 4.0f) + (env_phi / 2.0f)), 2.0f);
    float N_c_b = (N_q_b - 1.0f) / tan_phi;
    float N_gamma_b = 2.0f * (N_q_b + 1.0f) * tan_phi;
    
    float q_u = env_c * N_c_b + 0.5f * env_gamma * TRACK_WIDTH * N_gamma_b;
    float ground_pressure = (MACHINE_MASS * GRAVITY) / (2.0f * TRACK_LENGTH * TRACK_WIDTH);
    
    float compaction_rate = (ground_pressure / q_u) * 0.5f; 
    if (compaction_rate > 0.8f) compaction_rate = 0.8f;
    if (compaction_rate < 0.05f) compaction_rate = 0.05f;

    int margin = (int)((TRACK_LENGTH / 2.0f + 1.0f) / CELL_SIZE);
    int center_i = (int)(blade->loader_x / CELL_SIZE);
    int min_i = clamp_idx(center_i - margin);
    int max_i = clamp_idx(center_i + margin);
    int center_j = (int)(blade->loader_y / CELL_SIZE);
    int min_j = clamp_idx(center_j - margin);
    int max_j = clamp_idx(center_j + margin);

    for (int i = min_i; i <= max_i; i++) {
        for (int j = min_j; j <= max_j; j++) {
            if (env->grid_L[i][j] < 0.001f) continue;
            
            float dx = (i * CELL_SIZE) - blade->loader_x;
            float dy = (j * CELL_SIZE) - blade->loader_y;
            
            float lx = dx * cos_y + dy * sin_y;
            float ly = -dx * sin_y + dy * cos_y;
            
            if (fabsf(lx) <= TRACK_LENGTH / 2.0f) {
                if (fabsf(ly - TRACK_GAUGE / 2.0f) <= TRACK_WIDTH / 2.0f || 
                    fabsf(ly + TRACK_GAUGE / 2.0f) <= TRACK_WIDTH / 2.0f) {
                    
                    float compacted = env->grid_L[i][j] * compaction_rate;
                    env->grid_L[i][j] -= compacted;
                    env->grid_H[i][j] += compacted / SWELL_RATIO;
                }
            }
        }
    }

    float hL = TRACK_LENGTH / 2.0f, hW = TRACK_GAUGE / 2.0f;
    int num_samples = 11;
    float sum_z_left = 0, sum_z_right = 0, sum_xz_left = 0, sum_xz_right = 0, sum_x2 = 0;

    for (int i = 0; i < num_samples; i++) {
        float lx = -hL + (TRACK_LENGTH * i) / (num_samples - 1);
        sum_x2 += lx * lx;
        float p_lx = blade->loader_x + lx * cos_y - hW * sin_y, p_ly = blade->loader_y + lx * sin_y + hW * cos_y;
        float p_rx = blade->loader_x + lx * cos_y + hW * sin_y, p_ry = blade->loader_y + lx * sin_y - hW * cos_y;
        
        int ixl = (int)(p_lx/CELL_SIZE), iyl = (int)(p_ly/CELL_SIZE);
        int ixr = (int)(p_rx/CELL_SIZE), iyr = (int)(p_ry/CELL_SIZE);
        
        float zl = 1.0f, zr = 1.0f;
        if (ixl >= 0 && ixl < GRID_SIZE && iyl >= 0 && iyl < GRID_SIZE) {
            zl = env->grid_H[ixl][iyl] + env->grid_L[ixl][iyl];
        }
        if (ixr >= 0 && ixr < GRID_SIZE && iyr >= 0 && iyr < GRID_SIZE) {
            zr = env->grid_H[ixr][iyr] + env->grid_L[ixr][iyr];
        }
        
        sum_z_left += zl; sum_z_right += zr;
        sum_xz_left += lx * zl; sum_xz_right += lx * zr;
    }
    blade->loader_z = (sum_z_left + sum_z_right) / (2.0f * num_samples);
    blade->pitch = (atan2f(sum_xz_left/sum_x2, 1.0f) + atan2f(sum_xz_right/sum_x2, 1.0f)) / 2.0f;
    blade->roll = atan2f((sum_z_left - sum_z_right)/num_samples, TRACK_GAUGE);

    // Pivot arm kinematics
    float arm_r = 3.35f;
    float pivot_x = -1.0f;
    float pivot_z = 1.5f;

    // Local coordinates of the blade relative to chassis origin (loader_z is at track ground level)
    float theta = blade->arm_height; // arm_height represents the arm pivot angle (rad)
    float x_blade_local = pivot_x + arm_r * cosf(theta);
    float z_blade_local = pivot_z + arm_r * sinf(theta);

    // Apply chassis roll rotation
    float cos_r = cosf(blade->roll);
    float sin_r = sinf(blade->roll);
    float x1 = x_blade_local;
    float y1 = -z_blade_local * sin_r;
    float z1 = z_blade_local * cos_r;

    // Apply chassis pitch rotation (pitch is positive when pitching up)
    float cos_p = cosf(blade->pitch);
    float sin_p = sinf(blade->pitch);
    float x2 = x1 * cos_p - z1 * sin_p;
    float y2 = y1;
    float z2 = x1 * sin_p + z1 * cos_p;

    // Apply chassis yaw rotation
    blade->x = blade->loader_x + x2 * cos_y - y2 * sin_y;
    blade->y = blade->loader_y + x2 * sin_y + y2 * cos_y;
    blade->z = blade->loader_z + z2;
}

static inline void simulate_step(SoilEnv* env, float dt) {
    Blade* blade = &env->blade;
    float prev_x = blade->x;
    float prev_z = blade->z;

    // Actuator Dynamics
    blade->v_linear = (blade->v_linear + ((blade->effort_linear * MAX_FORCE_LINEAR - blade->last_force) / MACHINE_MASS) * dt) / (1.0f + (LINEAR_DAMPING / MACHINE_MASS) * dt);
    
    float mu_lat = 0.6f; 
    float scrub_torque = (mu_lat * MACHINE_MASS * GRAVITY * TRACK_LENGTH) / 4.0f;
    float net_yaw_torque = blade->effort_rotational * MAX_TORQUE_ROTATIONAL + blade->last_yaw_moment;
    
    if (fabsf(blade->v_rotational) < 0.05f) {
        if (fabsf(net_yaw_torque) <= scrub_torque) {
            net_yaw_torque = 0.0f;
            blade->v_rotational = 0.0f;
        } else {
            net_yaw_torque -= (net_yaw_torque > 0 ? scrub_torque : -scrub_torque);
        }
    } else {
        net_yaw_torque -= (blade->v_rotational > 0 ? scrub_torque : -scrub_torque);
    }
    
    blade->v_rotational = (blade->v_rotational + (net_yaw_torque / MACHINE_INERTIA) * dt) / (1.0f + (ROTATIONAL_DAMPING / MACHINE_INERTIA) * dt);
    
    float gravity_torque = ARM_MASS * GRAVITY * 1.5f * cosf(blade->arm_height);
    float external_resist_torque = blade->last_force * 1.5f; 
    float external_total_torque = -gravity_torque - external_resist_torque;
    if (blade->effort_lift * external_total_torque <= 0) {
        external_total_torque *= (1.0f - HYDRAULIC_STIFFNESS);
    }
    blade->vel_arm_height = (blade->vel_arm_height + ((blade->effort_lift * MAX_TORQUE_LIFT + external_total_torque) / ARM_INERTIA) * dt) / (1.0f + (ARM_DAMPING / ARM_INERTIA) * dt);
    blade->arm_height += blade->vel_arm_height * dt;
    if (blade->arm_height < ARM_MIN) blade->arm_height = ARM_MIN;
    if (blade->arm_height > ARM_MAX) blade->arm_height = ARM_MAX;

    float blade_stiffness = 0.98f;
    float ext_pitch = blade->last_force * 0.5f;
    if (blade->effort_pitch * ext_pitch <= 0) {
        ext_pitch *= (1.0f - blade_stiffness);
    }
    blade->vel_pitch_rel = (blade->vel_pitch_rel + ((blade->effort_pitch * MAX_TORQUE_PITCH + ext_pitch) / PITCH_INERTIA) * dt) / (1.0f + (PITCH_DAMPING / PITCH_INERTIA) * dt);
    blade->blade_pitch_rel += blade->vel_pitch_rel * dt;
    if (blade->blade_pitch_rel < PITCH_MIN) blade->blade_pitch_rel = PITCH_MIN;
    if (blade->blade_pitch_rel > PITCH_MAX) blade->blade_pitch_rel = PITCH_MAX;

    float ext_roll = blade->last_yaw_moment * 0.5f;
    if (blade->effort_roll * ext_roll <= 0) {
        ext_roll *= (1.0f - blade_stiffness);
    }
    blade->vel_roll_rel = (blade->vel_roll_rel + ((blade->effort_roll * MAX_TORQUE_ROLL + ext_roll) / ROLL_INERTIA) * dt) / (1.0f + (ROLL_DAMPING / ROLL_INERTIA) * dt);
    blade->blade_roll_rel += blade->vel_roll_rel * dt;
    if (blade->blade_roll_rel < ROLL_MIN) blade->blade_roll_rel = ROLL_MIN;
    if (blade->blade_roll_rel > ROLL_MAX) blade->blade_roll_rel = ROLL_MAX;

    blade->rake_angle = (45.0f * (PI/180.0f)) + blade->blade_pitch_rel + blade->pitch;
    precompute_FEE(env, blade->rake_angle, 0.0f);

    float slip = blade->last_force / calculate_max_traction(); 
    if (slip < 0) slip = 0;
    if (slip > 1) slip = 1;
    blade->yaw += blade->v_rotational * dt;
    blade->loader_x += blade->v_linear * (1.0f - slip) * cosf(blade->yaw) * dt;
    blade->loader_y += blade->v_linear * (1.0f - slip) * sinf(blade->yaw) * dt;

    update_kinematics(env);

    float total_vol_cut = 0, total_force = 0, total_yaw_moment = 0;
    for (int j = 0; j < GRID_SIZE; j++) {
        float w = (j * CELL_SIZE) - blade->y; 
        if (fabsf(w) > blade->width / 2.0f) {
            continue;
        }
        float curr_edge_x = blade->x - w * sinf(blade->yaw), prev_edge_x = prev_x - w * sinf(blade->yaw);
        int start = (int)(prev_edge_x / CELL_SIZE), end = (int)(curr_edge_x / CELL_SIZE);
        if (start > end) { 
            int t = start; start = end; end = t; 
        }
        if (start < 0) start = 0;
        if (end >= GRID_SIZE) end = GRID_SIZE - 1;

        for (int i = start; i <= end; i++) {
            float t = 1.0f;
            if (curr_edge_x > prev_edge_x) {
                t = (i * CELL_SIZE - prev_edge_x) / (curr_edge_x - prev_edge_x);
                if (t < 0.0f) t = 0.0f;
                if (t > 1.0f) t = 1.0f;
            }
            float interp_center_z = prev_z + t * (blade->z - prev_z);
            float blade_elev = interp_center_z + w * sinf(blade->roll + blade->blade_roll_rel);

            float total_h = env->grid_H[i][j] + env->grid_L[i][j];
            float depth = total_h - blade_elev;
            if (depth > 0) {
                if (env->grid_L[i][j] > 0) {
                    float l_cut = (depth < env->grid_L[i][j]) ? depth : env->grid_L[i][j];
                    env->grid_L[i][j] -= l_cut; depth -= l_cut; total_vol_cut += l_cut * CELL_SIZE * CELL_SIZE;
                }
                if (depth > 0) {
                    env->grid_H[i][j] -= depth; total_vol_cut += depth * CELL_SIZE * CELL_SIZE * SWELL_RATIO;
                }
            }
        }
        int front = end + 1;
        if (front >= 0 && front < GRID_SIZE) {
            float blade_elev = blade->z + w * sinf(blade->roll + blade->blade_roll_rel);
            float total_h = env->grid_H[front][j] + env->grid_L[front][j];
            float total_depth = total_h - blade_elev;
            float hard_depth = env->grid_H[front][j] - blade_elev;
            float df = calculate_FEE_column(env, blade, hard_depth, total_depth, CELL_SIZE);
            total_force += df; total_yaw_moment += df * w;
        }
    }
    blade->last_force = total_force; blade->last_yaw_moment = total_yaw_moment;

    if (total_vol_cut > 0) {
        float num_cols = blade->width / CELL_SIZE;
        float dh = (total_vol_cut / num_cols) / (CELL_SIZE * CELL_SIZE);
        for (int j = 0; j < GRID_SIZE; j++) {
            float w = (j * CELL_SIZE) - blade->y; 
            if (fabsf(w) > blade->width / 2.0f) {
                continue;
            }
            int dep = (int)((blade->x - w * sinf(blade->yaw)) / CELL_SIZE) + 1;
            if (dep >= 0 && dep < GRID_SIZE) {
                env->grid_L[dep][j] += dh;
            }
        }
    }

    simulate_erosion(env);

    // 4. Update Surcharge Q Dynamically
    float current_surcharge_vol = 0.0f;
    for (int j = 0; j < GRID_SIZE; j++) {
        float cell_y = j * CELL_SIZE;
        float w = cell_y - blade->y;
        if (fabsf(w) > blade->width / 2.0f) {
            continue;
        }

        float curr_edge_x = blade->x - w * sinf(blade->yaw);
        int start_i = (int)(curr_edge_x / CELL_SIZE) + 1;
        for (int i = start_i; i <= start_i + (int)(1.5f / CELL_SIZE); i++) {
            if (i >= 0 && i < GRID_SIZE) {
                current_surcharge_vol += env->grid_L[i][j] * CELL_SIZE * CELL_SIZE;
            }
        }
    }
    blade->surcharge_Q = current_surcharge_vol * LOOSE_SOIL_DENSITY * GRAVITY;
    if (outfile && env->step_num % 1 == 0) {
        struct { int step; float p[16]; int gs; float cs; } h = { env->step_num, {blade->loader_x, blade->loader_z, blade->loader_y, blade->yaw, blade->pitch, blade->roll, blade->arm_height, blade->vel_arm_height, blade->blade_pitch_rel, blade->vel_pitch_rel, blade->blade_roll_rel, blade->vel_roll_rel, blade->blade_yaw_rel, blade->vel_yaw_rel, blade->v_linear, blade->v_rotational}, GRID_SIZE, CELL_SIZE };
        fwrite(&h, sizeof(h), 1, outfile);
        float flat[GRID_SIZE * GRID_SIZE];
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                flat[i * GRID_SIZE + j] = env->grid_H[i][j] + env->grid_L[i][j];
            }
        }
        fwrite(flat, 4, GRID_SIZE * GRID_SIZE, outfile);
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                flat[i * GRID_SIZE + j] = env->grid_G[i][j];
            }
        }
        fwrite(flat, 4, GRID_SIZE * GRID_SIZE, outfile);
    }
    
    if (outfile && env->step_num % 10 == 0) {
        float slip = blade->last_force / calculate_max_traction();
        if (slip > 1.0f) slip = 1.0f;
        if (slip < 0.0f) slip = 0.0f;
        printf("Step: %d, Surcharge: %.2f N, Force: %.2f N, Slip: %.1f%%\n", env->step_num, blade->surcharge_Q, blade->last_force, slip * 100.0f);
    }

    env->step_num++;
}

// PufferLib C API implementation

static inline void add_log(SoilEnv* env) {
    float current_error = 0;
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float diff = (env->grid_H[i][j] + env->grid_L[i][j]) - env->grid_G[i][j];
            current_error += fabsf(diff);
        }
    }
    float error_reduction = env->initial_error - current_error;
    float perf = error_reduction / (env->initial_error + 1e-5f);
    if (perf < 0.0f) perf = 0.0f;
    if (perf > 1.0f) perf = 1.0f;
    if (env->blade.vel_arm_height > env->log.max_vel_arm) env->log.max_vel_arm = env->blade.vel_arm_height;
    if (env->blade.vel_pitch_rel > env->log.max_vel_blade_pitch) env->log.max_vel_blade_pitch = env->blade.vel_pitch_rel;
    if (env->blade.vel_roll_rel > env->log.max_vel_blade_roll) env->log.max_vel_blade_roll = env->blade.vel_roll_rel;

    env->log.perf += perf;
    env->log.score += env->episode_return;
    env->log.episode_length += env->tick;
    env->log.episode_return += env->episode_return;
    env->log.count_off_map += env->count_off_map;
    env->log.count_jitter += env->count_jitter;
    env->log.count_large_neg_rewards += env->count_large_neg_rewards;
    env->log.n++;
}

// Required function
void c_reset(SoilEnv* env) {
    env->tick = 0;
    env_reset(env);
    precompute_FEE(env, env->blade.rake_angle, 0.0f);
    update_kinematics(env);

    float current_error = 0;
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float diff = (env->grid_H[i][j] + env->grid_L[i][j]) - env->grid_G[i][j];
            current_error += fabsf(diff);
        }
    }
    env->initial_error = current_error;
    env->episode_return = 0.0f;
    env->count_off_map = 0.0f;
    env->count_jitter = 0.0f;
    env->count_large_neg_rewards = 0.0f;

    // Zero observations then write current obs
    memset(env->observations, 0, 5011 * sizeof(float));
    env_get_observation(env, (Observation*)env->observations);
}

// Required function
void c_step(SoilEnv* env) {
    env->tick += 1;
    env->terminals[0] = 0.0f;
    env->rewards[0] = 0.0f;

    float prev_error = 0;
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float diff = (env->grid_H[i][j] + env->grid_L[i][j]) - env->grid_G[i][j];
            prev_error += fabsf(diff);
        }
    }

    // Map effort control inputs (-1.0 to 1.0)
    // actions: effort_linear, effort_rotational, effort_lift, effort_pitch, effort_roll, effort_yaw
    env->blade.effort_linear     = clamp_action(env->actions[0]); // //efort
    env->blade.effort_rotational = clamp_action(env->actions[1]); // //efort
    env->blade.effort_lift       = clamp_action(env->actions[2]); // //efort
    env->blade.effort_pitch      = clamp_action(env->actions[3]); // //efort
    env->blade.effort_roll       = clamp_action(env->actions[4]); // //efort
    env->blade.effort_yaw        = clamp_action(env->actions[5]); // //efort

    // Sub-stepping: simulate 3 steps of 0.033333s for stability and control dynamics (10Hz control loop with 30Hz physics)
    for (int sub = 0; sub < 3; sub++) {
        simulate_step(env, 0.033333f);
    }

    env->rewards[0] = env_get_reward(env, prev_error);
    env->episode_return += env->rewards[0];

    env_get_observation(env, (Observation*)env->observations);

    // Calculate current terrain error reduction performance
    float current_error = 0;
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float diff = (env->grid_H[i][j] + env->grid_L[i][j]) - env->grid_G[i][j];
            current_error += fabsf(diff);
        }
    }
    float error_reduction = env->initial_error - current_error;
    float current_perf = error_reduction / (env->initial_error + 1e-5f);

    // Terminal condition (600 step limit, or 90% goal map built success)
    if (env->tick >= 600 || current_perf >= 0.90f) {
        env->terminals[0] = 1.0f;
        add_log(env);
        c_reset(env);
    }
}

// Required function
void c_render(SoilEnv* env) {
    // No-op. Rendering is handled externally via viz.py
}

// Required function
void c_close(SoilEnv* env) {
    // No-op
}

#endif // DOZERL_H
