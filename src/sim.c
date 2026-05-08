#include "sim.h"

// Global Soil Properties Definitions
float env_c = 200.0f;      // Reduced from 1000.0f for more realistic loose soil
float env_c_a = 500.0f;    
float env_phi = 30.0f * (PI / 180.0f); // Increased from 10.0f to 30.0f (typical sand)
float env_delta = 20.0f * (PI / 180.0f); 
float env_gamma = 1500.0f * GRAVITY; 

// Precomputed FEE Factors
float N_gamma, N_Q, N_c, N_ca;

FILE* outfile;

void env_get_observation(SoilEnv* env, Observation* obs) {
    Blade* blade = &env->blade;
    
    // Proprioceptive observations
    // Chassis Attitude: Pitch, Roll, Yaw rate
    obs->proprioceptive[0] = blade->pitch;
    obs->proprioceptive[1] = blade->roll;
    obs->proprioceptive[2] = blade->v_rotational; // Yaw rate
    
    // Joint Positions
    obs->proprioceptive[3] = blade->arm_height;
    obs->proprioceptive[4] = blade->blade_pitch_rel;
    obs->proprioceptive[5] = blade->blade_roll_rel;
    
    // Joint Velocities
    obs->proprioceptive[6] = blade->vel_arm_height;
    obs->proprioceptive[7] = blade->vel_pitch_rel;
    obs->proprioceptive[8] = blade->vel_roll_rel;
    
    // Track State
    obs->proprioceptive[9] = blade->v_linear;
    obs->proprioceptive[10] = blade->v_rotational; 

    // Spatial Observations (Heightmap Image)
    // 32x32 window centered in front of the blade.
    float cos_y = cosf(blade->yaw);
    float sin_y = sinf(blade->yaw);

    for (int i = 0; i < SPATIAL_OBS_SIZE; i++) {
        for (int j = 0; j < SPATIAL_OBS_SIZE; j++) {
            // Local coordinates relative to blade center
            float local_x = i * CELL_SIZE; 
            float local_y = (j - SPATIAL_OBS_SIZE / 2.0f) * CELL_SIZE;

            // Transform to global
            float global_x = blade->x + local_x * cos_y - local_y * sin_y;
            float global_y = blade->y + local_x * sin_y + local_y * cos_y;

            int grid_i = (int)(global_x / CELL_SIZE);
            int grid_j = (int)(global_y / CELL_SIZE);

            if (grid_i >= 0 && grid_i < GRID_SIZE && grid_j >= 0 && grid_j < GRID_SIZE) {
                // Height relative to chassis (loader_z)
                obs->spatial[i][j] = env->grid_H[grid_i][grid_j] + env->grid_L[grid_i][grid_j] - blade->loader_z;
            } else {
                obs->spatial[i][j] = 0.0f; // Out of bounds
            }
        }
    }
}

void precompute_FEE(float rake_angle, float alpha) {
    float rho = rake_angle;
    float beta = (PI / 4.0f) - (env_phi / 2.0f);
    float eta = env_delta + rho + env_phi + beta;
    float sin_eta = sinf(eta);
    if (fabsf(sin_eta) < 1e-6f) sin_eta = 1e-6f;
    
    N_gamma = ((1.0f / tanf(rho)) + (1.0f / tanf(beta))) * sinf(alpha + env_phi + beta) / (2.0f * sin_eta);
    N_Q = sinf(alpha + env_phi + beta) / sin_eta;
    N_c = cosf(env_phi) / (sinf(beta) * sin_eta);
    N_ca = -cosf(rho + env_phi + beta) / (sinf(rho) * sin_eta);
}

SoilEnv* env_init() {
    SoilEnv* env = (SoilEnv*)malloc(sizeof(SoilEnv));
    return env;
}

void env_free(SoilEnv* env) {
    if (env) {
        free(env);
    }
}

void env_reset(SoilEnv* env, int seed) {
    srand(seed); 
    
    // 1. Initial multi-scale random noise
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float fine = ((float)rand() / RAND_MAX - 0.5f) * 0.2f;
            float coarse = ((float)rand() / RAND_MAX - 0.5f) * 0.6f;
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
            }
        }
    }

    // 3. Add a berm x
    // float berm_x = 8.0f;
    // float berm_height = 0.6f; 
    // float berm_width = 3.9f;  
    // for (int i = 0; i < GRID_SIZE; i++) {
    //     float x_m = i * CELL_SIZE;
    //     float dist = fabsf(x_m - berm_x);
    //     if (dist < berm_width / 2.0f) {
    //         float h_add = berm_height * 0.5f * (1.0f + cosf(2.0f * PI * dist / berm_width));
    //         for (int j = 0; j < GRID_SIZE; j++) {
    //             env->grid_H[i][j] += h_add;
    //         }
    //     }
    // }

    // berm_x = 12.0f;
    // berm_height = 0.6f; 
    // berm_width = 3.9f;  
    // for (int i = 0; i < GRID_SIZE; i++) {
    //     float x_m = i * CELL_SIZE;
    //     float dist = fabsf(x_m - berm_x);
    //     if (dist < berm_width / 2.0f) {
    //         float h_add = berm_height * 0.5f * (1.0f + cosf(2.0f * PI * dist / berm_width));
    //         for (int j = 0; j < GRID_SIZE; j++) {
    //             env->grid_H[i][j] += h_add;
    //         }
    //     }
    // }

    // Initialize blade state
    env->blade.x = 3.0f;
    env->blade.y = 10.0f;
    env->blade.z = 0.8f;
    env->blade.width = 2.2f;
    env->blade.rake_angle = 45.0f * (PI / 180.0f);
    env->blade.surcharge_Q = 0.0f;
    env->blade.pitch = -0.35f;
    env->blade.roll = 0.0f;
    env->blade.yaw = 0.0f;
    
    env->blade.v_linear = 0.0f;
    env->blade.v_rotational = 0.0f;
    env->blade.arm_height = 0.25f;
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
    
    env->blade.loader_x = 0.0f;
    env->blade.loader_y = 0.0f;
    env->blade.loader_z = 0.0f;
    env->blade.last_force = 0.0f;
    env->blade.last_yaw_moment = 0.0f;

    env->step_num = 0;
}

float calculate_FEE_column(Blade* blade, float depth, float width) {
    if (depth <= 0.0f) return 0.0f;
    // Surcharge is proportionally distributed across columns
    float q = blade->surcharge_Q * (width / blade->width);
    return env_gamma * depth * depth * width * N_gamma +
           env_c * depth * width * N_c +
           q * N_Q +
           env_c_a * depth * width * N_ca;
}

// 3D Erosion simulation 
void simulate_erosion(SoilEnv* env) {
    Blade* blade = &env->blade;
    int margin_cells = (int)(4.0f / CELL_SIZE);
    int min_i = (int)(blade->x / CELL_SIZE) - margin_cells;
    int max_i = (int)(blade->x / CELL_SIZE) + margin_cells;
    int min_j = (int)(blade->y / CELL_SIZE) - margin_cells;
    int max_j = (int)(blade->y / CELL_SIZE) + margin_cells;

    if (min_i < 0) min_i = 0;
    if (max_i >= GRID_SIZE) max_i = GRID_SIZE - 1;
    if (min_j < 0) min_j = 0;
    if (max_j >= GRID_SIZE) max_j = GRID_SIZE - 1;

    for (int iter = 0; iter < 3; iter++) {
        float tempL[GRID_SIZE][GRID_SIZE];
        
        int min_i_temp = (min_i > 0) ? min_i - 1 : 0;
        int max_i_temp = (max_i < GRID_SIZE - 1) ? max_i + 1 : GRID_SIZE - 1;
        int min_j_temp = (min_j > 0) ? min_j - 1 : 0;
        int max_j_temp = (max_j < GRID_SIZE - 1) ? max_j + 1 : GRID_SIZE - 1;

        for (int i = min_i_temp; i <= max_i_temp; i++) {
            for (int j = min_j_temp; j <= max_j_temp; j++) {
                tempL[i][j] = env->grid_L[i][j];
            }
        }
        
        int dx[] = {1, -1, 0, 0};
        int dy[] = {0, 0, 1, -1};
        float loader_length = 2.0f;

        for (int i = min_i; i <= max_i; i++) {
            for (int j = min_j; j <= max_j; j++) {
                if (env->grid_L[i][j] <= 1e-4f) continue;
                float total_h = env->grid_H[i][j] + env->grid_L[i][j];
                
                for(int d=0; d<4; d++) {
                    int ni = i + dx[d];
                    int nj = j + dy[d];
                    if (ni >= min_i_temp && ni <= max_i_temp && nj >= min_j_temp && nj <= max_j_temp) {
                        
                        // Rotated/Translated rigid machine collision box
                        float cell_x = ni * CELL_SIZE;
                        float cell_y = nj * CELL_SIZE;
                        float w = cell_y - blade->y; // Lateral distance from machine center
                        if (fabsf(w) <= blade->width / 2.0f) {
                            float blade_front_x = blade->x - w * sinf(blade->yaw);
                            float loader_back_x = blade_front_x - loader_length * cosf(blade->yaw);
                            if (cell_x >= loader_back_x && cell_x <= blade_front_x) {
                                continue; // Blocked by machine chassis
                            }
                        }
                        
                        float neighbor_total_h = env->grid_H[ni][nj] + env->grid_L[ni][nj];
                        float dH = total_h - neighbor_total_h;
                        
                        if (dH > 0.0f) {
                            float alpha = atan2f(dH, CELL_SIZE);
                            float W = env->grid_L[i][j] * CELL_SIZE * CELL_SIZE * LOOSE_SOIL_DENSITY * GRAVITY;
                            
                            if (W > 1e-4f) {
                                // Dimensional fix: c * Area_slip, where Area_slip = CELL_SIZE^2 / cos(alpha)
                                float area_slip = (CELL_SIZE * CELL_SIZE) / cosf(alpha);
                                float F_s = (env_c * area_slip + W * cosf(alpha) * tanf(env_phi)) / (W * sinf(alpha));
                                
                                if (F_s < 1.0f) {
                                    // Approximate the target dH where F_s = 1
                                    float target_dH = CELL_SIZE * (env_c * area_slip + W * cosf(alpha) * tanf(env_phi)) / W;
                                    float min_target_dH = CELL_SIZE * tanf(env_phi);
                                    if (target_dH < min_target_dH) target_dH = min_target_dH;

                                    if (dH > target_dH) {
                                        float slip = (dH - target_dH) / 2.0f;
                                        slip *= 0.4f; 
                                        if (slip > tempL[i][j]) slip = tempL[i][j]; 
                                        tempL[i][j] -= slip;
                                        tempL[ni][nj] += slip;
                                        total_h -= slip; 
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        for (int i = min_i_temp; i <= max_i_temp; i++) {
            for (int j = min_j_temp; j <= max_j_temp; j++) {
                env->grid_L[i][j] = tempL[i][j];
            }
        }
    }
}

// Vehicle Kinematics Model with Yaw and Y Position
void update_kinematics(SoilEnv* env) {
    Blade* blade = &env->blade;
    float track_length = 2.5f;
    float track_gauge = 1.8f;
    float blade_offset_x = 2.0f; 

    // Chassis center (behind the blade)
    blade->loader_x = blade->x - blade_offset_x * cosf(blade->yaw);
    blade->loader_y = blade->y - blade_offset_x * sinf(blade->yaw);
    
    float cos_y = cosf(blade->yaw);
    float sin_y = sinf(blade->yaw);
    
    float hL = track_length / 2.0f;
    float hW = track_gauge / 2.0f;
    
    int num_samples = 11; // Sample every ~0.1m along 2.0m track
    float sum_z_left = 0.0f;
    float sum_z_right = 0.0f;
    float sum_xz_left = 0.0f;
    float sum_xz_right = 0.0f;
    float sum_x2 = 0.0f;

    for (int i = 0; i < num_samples; i++) {
        // Local x goes from -hL to +hL (rear to front)
        float local_x = -hL + (track_length * i) / (num_samples - 1);
        sum_x2 += local_x * local_x;
        
        // Left track point
        float l_x = blade->loader_x + local_x * cos_y - hW * sin_y;
        float l_y = blade->loader_y + local_x * sin_y + hW * cos_y;
        
        // Right track point
        float r_x = blade->loader_x + local_x * cos_y + hW * sin_y;
        float r_y = blade->loader_y + local_x * sin_y - hW * cos_y;
        
        float track_z_L = 1.0f; // Default height
        float track_z_R = 1.0f;
        

        float compaction_rate = 0.15f; // Tracks compress 15% of loose soil per sample pass

        // Left track sampling and compaction
        int ix_L = (int)(l_x / CELL_SIZE); 
        int iy_L = (int)(l_y / CELL_SIZE);
        if (ix_L >= 0 && ix_L < GRID_SIZE && iy_L >= 0 && iy_L < GRID_SIZE) {
            if (env->grid_L[ix_L][iy_L] > 0.001f) {
                float compacted_amount = env->grid_L[ix_L][iy_L] * compaction_rate;
                env->grid_L[ix_L][iy_L] -= compacted_amount;
                // Revert the swell ratio when moving back to compacted state
                env->grid_H[ix_L][iy_L] += compacted_amount / SWELL_RATIO; 
            }
            track_z_L = env->grid_H[ix_L][iy_L] + env->grid_L[ix_L][iy_L];
        }

        // Right track sampling and compaction
        int ix_R = (int)(r_x / CELL_SIZE); 
        int iy_R = (int)(r_y / CELL_SIZE);
        if (ix_R >= 0 && ix_R < GRID_SIZE && iy_R >= 0 && iy_R < GRID_SIZE) {
            if (env->grid_L[ix_R][iy_R] > 0.001f) {
                float compacted_amount = env->grid_L[ix_R][iy_R] * compaction_rate;
                env->grid_L[ix_R][iy_R] -= compacted_amount;
                // Revert the swell ratio when moving back to compacted state
                env->grid_H[ix_R][iy_R] += compacted_amount / SWELL_RATIO;
            }
            track_z_R = env->grid_H[ix_R][iy_R] + env->grid_L[ix_R][iy_R];
        }
        
        sum_z_left += track_z_L;
        sum_z_right += track_z_R;
        sum_xz_left += local_x * track_z_L;
        sum_xz_right += local_x * track_z_R;
    }
    
    float avg_z_left = sum_z_left / num_samples;
    float avg_z_right = sum_z_right / num_samples;
    
    // Z is the average of both track averages
    blade->loader_z = (avg_z_left + avg_z_right) / 2.0f;
    
    // Linear regression for pitch: slope = sum(x*z) / sum(x^2)
    // Avoid division by zero (though sum_x2 is > 0 for num_samples > 1)
    if (sum_x2 > 0.0001f) {
        float pitch_left = atan2f(sum_xz_left / sum_x2, 1.0f);
        float pitch_right = atan2f(sum_xz_right / sum_x2, 1.0f);
        blade->pitch = (pitch_left + pitch_right) / 2.0f;
    } else {
        blade->pitch = 0.0f;
    }
    
    // Roll is difference between average left and right heights over the track gauge
    blade->roll = atan2f(avg_z_left - avg_z_right, track_gauge);
    
    // Blade origin (z is global elevation of blade center hinge)
    blade->z = blade->loader_z + blade_offset_x * sinf(blade->pitch) + blade->arm_height;
}

void simulate_step(SoilEnv* env, float dt) {
    Blade* blade = &env->blade;
    float prev_x = blade->x;
    float prev_z = blade->z; 

    // 0. Actuator Dynamics (Effort to Velocity/Position)
    
    // Linear Tracks
    float net_force_linear = blade->effort_linear * MAX_FORCE_LINEAR - blade->last_force;
    blade->v_linear = (blade->v_linear + (net_force_linear / MACHINE_MASS) * dt) / (1.0f + (LINEAR_DAMPING / MACHINE_MASS) * dt);
    
    // Rotational Tracks (Machine Yaw)
    float net_torque_rot = blade->effort_rotational * MAX_TORQUE_ROTATIONAL + blade->last_yaw_moment;
    blade->v_rotational = (blade->v_rotational + (net_torque_rot / MACHINE_INERTIA) * dt) / (1.0f + (ROTATIONAL_DAMPING / MACHINE_INERTIA) * dt);

    // External load backdrive
    float external_lift_force = blade->last_force * 0.2f;
    float external_pitch_torque = blade->last_force * 0.5f;
    float external_roll_torque = blade->last_yaw_moment * 0.5f;
    float blade_stiffness = 0.98f; // Easier to backdrive than arm (HYDRAULIC_STIFFNESS)

    // Arm Lift
    float gravity_force_arm = ARM_MASS * GRAVITY;
    float applied_force_lift = blade->effort_lift * MAX_FORCE_LIFT;
    float external_total_lift = -gravity_force_arm - external_lift_force;
    
    // Hydraulic check valves prevent external forces from backdriving unless assisting the command
    if (blade->effort_lift * external_total_lift <= 0) {
        external_total_lift *= (1.0f - HYDRAULIC_STIFFNESS); 
    }
    float net_force_lift = applied_force_lift + external_total_lift;
    
    if (fabsf(blade->effort_lift) < 0.05f) {
        blade->vel_arm_height *= 0.9f; // Active braking
    }
    blade->vel_arm_height = (blade->vel_arm_height + (net_force_lift / ARM_MASS) * dt) / (1.0f + (ARM_DAMPING / ARM_MASS) * dt);
    blade->arm_height += blade->vel_arm_height * dt;
    if (blade->arm_height < ARM_MIN) { blade->arm_height = ARM_MIN; blade->vel_arm_height = 0; }
    if (blade->arm_height > ARM_MAX) { blade->arm_height = ARM_MAX; blade->vel_arm_height = 0; }

    // Blade Pitch
    float applied_torque_pitch = blade->effort_pitch * MAX_TORQUE_PITCH;
    float external_total_pitch = external_pitch_torque;
    if (blade->effort_pitch * external_total_pitch <= 0) {
        external_total_pitch *= (1.0f - blade_stiffness);
    }
    float net_torque_pitch = applied_torque_pitch + external_total_pitch;

    if (fabsf(blade->effort_pitch) < 0.05f) {
        blade->vel_pitch_rel *= 0.9f;
    }
    blade->vel_pitch_rel = (blade->vel_pitch_rel + (net_torque_pitch / PITCH_INERTIA) * dt) / (1.0f + (PITCH_DAMPING / PITCH_INERTIA) * dt);
    blade->blade_pitch_rel += blade->vel_pitch_rel * dt;
    if (blade->blade_pitch_rel < PITCH_MIN) { blade->blade_pitch_rel = PITCH_MIN; blade->vel_pitch_rel = 0; }
    if (blade->blade_pitch_rel > PITCH_MAX) { blade->blade_pitch_rel = PITCH_MAX; blade->vel_pitch_rel = 0; }

    // Blade Roll
    float applied_torque_roll = blade->effort_roll * MAX_TORQUE_ROLL;
    float external_total_roll = external_roll_torque;
    if (blade->effort_roll * external_total_roll <= 0) {
        external_total_roll *= (1.0f - blade_stiffness);
    }
    float net_torque_roll = applied_torque_roll + external_total_roll;

    if (fabsf(blade->effort_roll) < 0.05f) {
        blade->vel_roll_rel *= 0.9f;
    }
    blade->vel_roll_rel = (blade->vel_roll_rel + (net_torque_roll / ROLL_INERTIA) * dt) / (1.0f + (ROLL_DAMPING / ROLL_INERTIA) * dt);
    blade->blade_roll_rel += blade->vel_roll_rel * dt;
    if (blade->blade_roll_rel < ROLL_MIN) { blade->blade_roll_rel = ROLL_MIN; blade->vel_roll_rel = 0; }
    if (blade->blade_roll_rel > ROLL_MAX) { blade->blade_roll_rel = ROLL_MAX; blade->vel_roll_rel = 0; }

    // Apply dynamic pitch to rake angle
    float base_rake_angle = 45.0f * (PI / 180.0f);
    blade->rake_angle = base_rake_angle + blade->blade_pitch_rel + blade->pitch;
    precompute_FEE(blade->rake_angle, 0.0f);

    // Track Slip Dynamics
    float max_traction = 35000.0f; 
    float slip_ratio = blade->last_force / max_traction;
    if (slip_ratio < 0.0f) slip_ratio = 0.0f;
    if (slip_ratio > 1.0f) slip_ratio = 1.0f; 
    
    float v_actual = blade->v_linear * (1.0f - slip_ratio);

    // Update Heading
    blade->yaw += blade->v_rotational * dt;

    // Move Forward in 2D
    blade->x += v_actual * cosf(blade->yaw) * dt; 
    blade->y += v_actual * sinf(blade->yaw) * dt;
    
    float curr_x = blade->x;
    float curr_y = blade->y;
    
    update_kinematics(env);
    float curr_z = blade->z;
    
    float total_vol_cut = 0.0f;
    float total_force = 0.0f;
    float total_yaw_moment = 0.0f;
    
    // 1. Per-Column Cut & Force Calculation
    for (int j = 0; j < GRID_SIZE; j++) {
        float cell_y = j * CELL_SIZE;
        float w = cell_y - curr_y; // Offset from blade center
        if (fabsf(w) > blade->width / 2.0f) continue;
        
        // Blade edge position at this specific column
        float curr_edge_x = curr_x - w * sinf(blade->yaw);
        float prev_edge_x = prev_x - w * sinf(blade->yaw);
        
        int start_idx = (int)((prev_edge_x + 1e-4f) / CELL_SIZE);
        int end_idx = (int)((curr_edge_x + 1e-4f) / CELL_SIZE);
        if (start_idx > end_idx) { int tmp = start_idx; start_idx = end_idx; end_idx = tmp; }
        
        if (start_idx < 0) start_idx = 0;
        if (end_idx >= GRID_SIZE) end_idx = GRID_SIZE - 1;

        for (int i = start_idx; i <= end_idx; i++) {
            float cell_x = i * CELL_SIZE;
            float t = 1.0f;
            if (curr_edge_x > prev_edge_x) {
                t = (cell_x - prev_edge_x) / (curr_edge_x - prev_edge_x);
                if (t < 0.0f) t = 0.0f;
                if (t > 1.0f) t = 1.0f;
            }
            
            float interp_center_z = prev_z + t * (curr_z - prev_z);
            // Apply blade roll relative to chassis
            float blade_elev = interp_center_z + w * sinf(blade->roll + blade->blade_roll_rel);
            
            float total_h = env->grid_H[i][j] + env->grid_L[i][j];
            float depth = total_h - blade_elev;
            
            if (depth > 0) {
                if (env->grid_L[i][j] > 0) {
                    float l_cut = (depth < env->grid_L[i][j]) ? depth : env->grid_L[i][j];
                    env->grid_L[i][j] -= l_cut;
                    depth -= l_cut;
                    total_vol_cut += l_cut * CELL_SIZE * CELL_SIZE; 
                }
                if (depth > 0) {
                    env->grid_H[i][j] -= depth;
                    total_vol_cut += depth * CELL_SIZE * CELL_SIZE * SWELL_RATIO; 
                }
            }
        }
        
        // Calculate physics force specific to this column hitting the undisturbed wall
        int front_idx = end_idx + 1;
        if (front_idx >= 0 && front_idx < GRID_SIZE) {
            float blade_elev = curr_z + w * sinf(blade->roll + blade->blade_roll_rel);
            float static_depth = env->grid_H[front_idx][j] - blade_elev;
            if (static_depth > 0) {
                float df = calculate_FEE_column(blade, static_depth, CELL_SIZE);
                total_force += df;
                // Force opposes forward motion. Force on Right (w<0 if Y-Left) pushes Right Side Back -> Turns Left (-Yaw)
                total_yaw_moment -= df * w; 
            }
        }
    }
    
    blade->last_force = total_force;
    blade->last_yaw_moment = total_yaw_moment;

    // 2. Transport Cut Soil dynamically in front of yawed blade
    if (total_vol_cut > 0) {
        float num_cols = blade->width / CELL_SIZE;
        float dh = (total_vol_cut / num_cols) / (CELL_SIZE * CELL_SIZE);
        
        for (int j = 0; j < GRID_SIZE; j++) {
            float cell_y = j * CELL_SIZE;
            float w = cell_y - curr_y;
            if (fabsf(w) > blade->width / 2.0f) continue;
            
            float curr_edge_x = curr_x - w * sinf(blade->yaw);
            int deposit_idx = (int)(curr_edge_x / CELL_SIZE) + 1;
            
            if (deposit_idx >= 0 && deposit_idx < GRID_SIZE) {
                env->grid_L[deposit_idx][j] += dh;
            }
        }
    }

    // 3. Stability check
    simulate_erosion(env);

    // 4. Update Surcharge Q Dynamically
    float current_surcharge_vol = 0;
    for (int j = 0; j < GRID_SIZE; j++) {
        float cell_y = j * CELL_SIZE;
        float w = cell_y - curr_y;
        if (fabsf(w) > blade->width / 2.0f) continue;
        
        float curr_edge_x = curr_x - w * sinf(blade->yaw);
        int start_i = (int)(curr_edge_x / CELL_SIZE) + 1;
        
        for (int i = start_i; i <= start_i + (int)(1.5f / CELL_SIZE); i++) {
            if (i >= 0 && i < GRID_SIZE) {
                current_surcharge_vol += env->grid_L[i][j] * CELL_SIZE * CELL_SIZE;
            }
        }
    }
    blade->surcharge_Q = current_surcharge_vol * LOOSE_SOIL_DENSITY * GRAVITY;

#ifndef BENCHMARK
    if (env->step_num % 5 == 0) {
        printf("Step %d [X: %.2fm, Y: %.2fm, Z: %.2fm]: Yaw: %5.2f° | Roll: %4.1f° | Slip: %3.0f%% | Q: %6.0f N | M_yaw: %6.0f Nm\n", 
               env->step_num, blade->x, blade->y, blade->z, blade->yaw * (180.0f/PI), 
               blade->roll * (180.0f/PI), slip_ratio * 100.0f, blade->surcharge_Q, blade->last_yaw_moment);
    }

    // Write to Rerun visualizer in binary format for high performance
    if (outfile) {
        struct {
            int step;
            // Chassis Pose
            float l_x, l_z, l_y, yaw, pitch, roll;
            // Joint States (Position, Velocity)
            float arm_pos, arm_vel;
            float pitch_pos, pitch_vel;
            float roll_pos, roll_vel;
            float yaw_pos, yaw_vel;
            // Track Velocities
            float track_v_lin, track_v_rot;
            
            int grid_size;
            float cell_size;
        } header = {
            env->step_num,
            blade->loader_x, blade->loader_z, blade->loader_y, blade->yaw, blade->pitch, blade->roll,
            blade->arm_height, blade->vel_arm_height,
            blade->blade_pitch_rel, blade->vel_pitch_rel,
            blade->blade_roll_rel, blade->vel_roll_rel,
            blade->blade_yaw_rel, blade->vel_yaw_rel,
            blade->v_linear, blade->v_rotational,
            GRID_SIZE, CELL_SIZE
        };
        
        fwrite(&header, sizeof(header), 1, outfile);

        float flat_grid[GRID_SIZE * GRID_SIZE];
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                flat_grid[i * GRID_SIZE + j] = env->grid_H[i][j] + env->grid_L[i][j];
            }
        }
        fwrite(flat_grid, sizeof(float), GRID_SIZE * GRID_SIZE, outfile);
    }
#endif
    env->step_num++;
}

int main() {
#ifndef BENCHMARK
    outfile = fopen("out/sim_out.bin", "wb");
    if (!outfile) return 1;
#endif

    SoilEnv* env = env_init();
    env_reset(env, 42);
    
    precompute_FEE(env->blade.rake_angle, 0.0f);
    update_kinematics(env); 
    
#ifdef BENCHMARK
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    int steps = 1000;
    for (int t = 0; t < steps; t++) {
        simulate_step(env, 0.01f);
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    double elapsed = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    printf("Benchmarking RL-Optimized Sim: %d steps in %.4f seconds (%.2f Hz)\n", steps, elapsed, steps / elapsed);
#else
    printf("Starting 6DOF 3D Soil Simulation with Effort Control...\n");
    float dt = 0.01f; 
    
    // Simulate some movement and actuation
    for (int t = 0; t < 1000; t++) {
        // Default forward effort
        
        // Change actions periodically to test them out
        if (t < 200) {
            env->blade.effort_linear = 0.8f;
            env->blade.effort_rotational = 0.0f;
            env->blade.effort_lift = -0.01f; 
            env->blade.effort_pitch = 0.1f;
            env->blade.effort_roll = 0.0f;
        } else if (t >= 200 && t < 400) {
            env->blade.effort_rotational = 0.0f;
            env->blade.effort_lift = 0.1f; 
            env->blade.effort_pitch = 0.0f;
            env->blade.effort_roll = 0.0f;
        } else if (t >= 400 && t < 600) {
            env->blade.effort_lift = 0.1f; 
            env->blade.effort_pitch = 0.0f;
            env->blade.effort_roll = 0.0f;
        } else if (t >= 600 && t < 800) {
            env->blade.effort_lift = 0.1f; 
            env->blade.effort_pitch = 0.0f;
            env->blade.effort_roll = 0.0f;
        } else if (t >= 800 && t < 1000) {
            env->blade.effort_lift = 0.0f; 
            env->blade.effort_pitch = -0.2f;
            env->blade.effort_roll = 0.0f;
        }
        
        simulate_step(env, dt);
    }
    
    if (outfile) fclose(outfile);
    printf("Simulation Complete. Data written to sim_out.bin.\n");
#endif

    env_free(env);
    return 0;
}
