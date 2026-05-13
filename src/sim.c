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
    obs->proprioceptive[0] = blade->pitch;
    obs->proprioceptive[1] = blade->roll;
    obs->proprioceptive[2] = blade->v_rotational;
    obs->proprioceptive[3] = blade->arm_height;
    obs->proprioceptive[4] = blade->blade_pitch_rel;
    obs->proprioceptive[5] = blade->blade_roll_rel;
    obs->proprioceptive[6] = blade->vel_arm_height;
    obs->proprioceptive[7] = blade->vel_pitch_rel;
    obs->proprioceptive[8] = blade->vel_roll_rel;
    obs->proprioceptive[9] = blade->v_linear;
    obs->proprioceptive[10] = blade->v_rotational; 

    // Spatial Observations (2-Channel Heightmap)
    float cos_y = cosf(blade->yaw);
    float sin_y = sinf(blade->yaw);

    for (int i = 0; i < SPATIAL_OBS_SIZE; i++) {
        for (int j = 0; j < SPATIAL_OBS_SIZE; j++) {
            float local_x = i * CELL_SIZE; 
            float local_y = (j - SPATIAL_OBS_SIZE / 2.0f) * CELL_SIZE;

            float global_x = blade->x + local_x * cos_y - local_y * sin_y;
            float global_y = blade->y + local_x * sin_y + local_y * cos_y;

            int grid_i = (int)(global_x / CELL_SIZE);
            int grid_j = (int)(global_y / CELL_SIZE);

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
    if (env) free(env);
}

void env_reset(SoilEnv* env, int seed) {
    srand(seed); 
    
    // 1. Initial Multi-scale Noise
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float fine = ((float)rand() / RAND_MAX - 0.5f) * 0.1f;
            float coarse = ((float)rand() / RAND_MAX - 0.5f) * 0.3f;
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
    float start_x = 5.0f + ((float)rand() / RAND_MAX) * 5.0f;
    float start_y = 5.0f + ((float)rand() / RAND_MAX) * 15.0f;
    float angle = ((float)rand() / RAND_MAX - 0.5f) * (PI / 4.0f);
    float slot_L = 4.0f + ((float)rand() / RAND_MAX) * 4.0f; // 4.0m to 8.0m
    float slot_W = env->blade.width; // Match blade width exactly
    float slot_D = 0.1f + ((float)rand() / RAND_MAX) * 0.4f; // 0.1m to 0.5m

    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    float total_slot_vol = 0;

    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float dx = (i * CELL_SIZE) - start_x;
            float dy = (j * CELL_SIZE) - start_y;
            float lx = dx * cos_a + dy * sin_a;
            float ly = -dx * sin_a + dy * cos_a;
            if (lx >= 0 && lx <= slot_L && fabsf(ly) <= slot_W / 2.0f) {
                env->grid_G[i][j] -= slot_D;
                total_slot_vol += slot_D * CELL_SIZE * CELL_SIZE;
            }
        }
    }

    float pile_center_x = start_x + (slot_L + 2.0f) * cos_a;
    float pile_center_y = start_y + (slot_L + 2.0f) * sin_a;
    
    // Oval / Gaussian shaped pile, longer perpendicular to the slot
    float target_pile_vol = total_slot_vol * SWELL_RATIO;
    
    // Target a visible width of ~1.3x blade width. 
    // Visible Gaussian width is ~6*sigma, so sigma_y = (1.3 * blade.width) / 6.0
    float base_sigma_y = (env->blade.width * 1.3f) / 6.0f; 
    // Make the pile shorter along the pushing direction (e.g. 0.6x the perpendicular spread)
    float base_sigma_x = base_sigma_y * 0.6f; 
    
    float pile_amplitude = target_pile_vol / (2.0f * PI * base_sigma_x * base_sigma_y);
    float sigma_x = base_sigma_x;
    float sigma_y = base_sigma_y;
    
    // Clamp pile height between 0.5m and 2.0m to keep it realistic.
    // If it violates bounds, scale the footprint (both sigmas) equally to conserve volume.
    if (pile_amplitude > 2.0f) {
        float scale = sqrtf(pile_amplitude / 2.0f);
        sigma_x *= scale;
        sigma_y *= scale;
        pile_amplitude = 2.0f;
    } else if (pile_amplitude < 0.5f) {
        float scale = sqrtf(pile_amplitude / 0.5f);
        sigma_x *= scale;
        sigma_y *= scale;
        pile_amplitude = 0.5f;
    }

    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float dx = (i * CELL_SIZE) - pile_center_x;
            float dy = (j * CELL_SIZE) - pile_center_y;
            
            // Transform to local pile coords
            float lx = dx * cos_a + dy * sin_a;
            float ly = -dx * sin_a + dy * cos_a;
            
            float exponent = -((lx * lx) / (2.0f * sigma_x * sigma_x) + (ly * ly) / (2.0f * sigma_y * sigma_y));
            float h_add = pile_amplitude * expf(exponent);
            
            // Prevent infinite tails
            if (h_add > 0.01f) {
                env->grid_G[i][j] += h_add;
            }
        }
    }

    // Blade State
    env->blade.x = start_x - 2.0f * cos_a;
    env->blade.y = start_y - 2.0f * sin_a;
    env->blade.z = 0.8f;
    env->blade.rake_angle = 45.0f * (PI / 180.0f);
    env->blade.surcharge_Q = 0.0f;
    env->blade.pitch = -0.35f;
    env->blade.roll = 0.0f;
    env->blade.yaw = angle;
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
    float q = blade->surcharge_Q * (width / blade->width);
    return env_gamma * depth * depth * width * N_gamma +
           env_c * depth * width * N_c +
           q * N_Q +
           env_c_a * depth * width * N_ca;
}

float env_get_reward(SoilEnv* env, float prev_error) {
    float current_error = 0;
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float diff = (env->grid_H[i][j] + env->grid_L[i][j]) - env->grid_G[i][j];
            current_error += fabsf(diff);
        }
    }
    float reward = (prev_error - current_error);
    float max_traction = 35000.0f;
    float slip_ratio = env->blade.last_force / max_traction;
    if (slip_ratio > 0.8f) reward -= (slip_ratio - 0.8f) * 10.0f;
    reward -= (env->blade.effort_linear * env->blade.effort_linear + 
               env->blade.effort_lift * env->blade.effort_lift +
               env->blade.effort_pitch * env->blade.effort_pitch) * 0.01f;
    return reward;
}

void simulate_erosion(SoilEnv* env) {
    Blade* blade = &env->blade;
    int margin = (int)(4.0f / CELL_SIZE);
    int min_i = (int)(blade->x / CELL_SIZE) - margin; 
    if (min_i < 0) min_i = 0;
    int max_i = (int)(blade->x / CELL_SIZE) + margin; 
    if (max_i >= GRID_SIZE) max_i = GRID_SIZE - 1;
    int min_j = (int)(blade->y / CELL_SIZE) - margin; 
    if (min_j < 0) min_j = 0;
    int max_j = (int)(blade->y / CELL_SIZE) + margin; 
    if (max_j >= GRID_SIZE) max_j = GRID_SIZE - 1;

    float loader_length = 2.0f;

    for (int iter = 0; iter < 3; iter++) {
        float tempL[GRID_SIZE][GRID_SIZE];
        for (int i = min_i; i <= max_i; i++) {
            for (int j = min_j; j <= max_j; j++) {
                tempL[i][j] = env->grid_L[i][j];
            }
        }
        
        int dx[] = {1, -1, 0, 0}, dy[] = {0, 0, 1, -1};
        for (int i = min_i + 1; i < max_i; i++) {
            for (int j = min_j + 1; j < max_j; j++) {
                if (env->grid_L[i][j] <= 1e-4f) continue;
                float total_h = env->grid_H[i][j] + env->grid_L[i][j];
                
                for(int d=0; d<4; d++) {
                    int ni = i + dx[d], nj = j + dy[d];
                    
                    // Rigid machine collision box
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
                    if (dH > CELL_SIZE * tanf(env_phi)) {
                        float slip = (dH - CELL_SIZE * tanf(env_phi)) * 0.2f;
                        if (slip > tempL[i][j]) slip = tempL[i][j];
                        tempL[i][j] -= slip; tempL[ni][nj] += slip; total_h -= slip;
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

void update_kinematics(SoilEnv* env) {
    Blade* blade = &env->blade;
    float track_length = 2.5f, track_gauge = 1.8f, blade_offset_x = 2.0f; 
    blade->loader_x = blade->x - blade_offset_x * cosf(blade->yaw);
    blade->loader_y = blade->y - blade_offset_x * sinf(blade->yaw);
    
    float cos_y = cosf(blade->yaw), sin_y = sinf(blade->yaw);
    float hL = track_length / 2.0f, hW = track_gauge / 2.0f;
    int num_samples = 11;
    float sum_z_left = 0, sum_z_right = 0, sum_xz_left = 0, sum_xz_right = 0, sum_x2 = 0;
    float compaction_rate = 0.15f;

    for (int i = 0; i < num_samples; i++) {
        float lx = -hL + (track_length * i) / (num_samples - 1);
        sum_x2 += lx * lx;
        float p_lx = blade->loader_x + lx * cos_y - hW * sin_y, p_ly = blade->loader_y + lx * sin_y + hW * cos_y;
        float p_rx = blade->loader_x + lx * cos_y + hW * sin_y, p_ry = blade->loader_y + lx * sin_y - hW * cos_y;
        
        int ixl = (int)(p_lx/CELL_SIZE), iyl = (int)(p_ly/CELL_SIZE);
        int ixr = (int)(p_rx/CELL_SIZE), iyr = (int)(p_ry/CELL_SIZE);
        
        float zl = 1.0f, zr = 1.0f;
        if (ixl >= 0 && ixl < GRID_SIZE && iyl >= 0 && iyl < GRID_SIZE) {
            if (env->grid_L[ixl][iyl] > 0.001f) {
                float compacted = env->grid_L[ixl][iyl] * compaction_rate;
                env->grid_L[ixl][iyl] -= compacted;
                env->grid_H[ixl][iyl] += compacted / SWELL_RATIO;
            }
            zl = env->grid_H[ixl][iyl] + env->grid_L[ixl][iyl];
        }
        if (ixr >= 0 && ixr < GRID_SIZE && iyr >= 0 && iyr < GRID_SIZE) {
            if (env->grid_L[ixr][iyr] > 0.001f) {
                float compacted = env->grid_L[ixr][iyr] * compaction_rate;
                env->grid_L[ixr][iyr] -= compacted;
                env->grid_H[ixr][iyr] += compacted / SWELL_RATIO;
            }
            zr = env->grid_H[ixr][iyr] + env->grid_L[ixr][iyr];
        }
        
        sum_z_left += zl; sum_z_right += zr;
        sum_xz_left += lx * zl; sum_xz_right += lx * zr;
    }
    blade->loader_z = (sum_z_left + sum_z_right) / (2.0f * num_samples);
    blade->pitch = (atan2f(sum_xz_left/sum_x2, 1.0f) + atan2f(sum_xz_right/sum_x2, 1.0f)) / 2.0f;
    blade->roll = atan2f((sum_z_left - sum_z_right)/num_samples, track_gauge);
    blade->z = blade->loader_z + blade_offset_x * sinf(blade->pitch) + blade->arm_height;
}

void simulate_step(SoilEnv* env, float dt) {
    Blade* blade = &env->blade;
    float prev_x = blade->x;
    float prev_z = blade->z;

    // Actuator Dynamics
    blade->v_linear = (blade->v_linear + ((blade->effort_linear * MAX_FORCE_LINEAR - blade->last_force) / MACHINE_MASS) * dt) / (1.0f + (LINEAR_DAMPING / MACHINE_MASS) * dt);
    blade->v_rotational = (blade->v_rotational + ((blade->effort_rotational * MAX_TORQUE_ROTATIONAL + blade->last_yaw_moment) / MACHINE_INERTIA) * dt) / (1.0f + (ROTATIONAL_DAMPING / MACHINE_INERTIA) * dt);
    
    // Arm Lift with Hydraulic Stiffness
    float external_lift_force = blade->last_force * 0.2f;
    float external_total_lift = -ARM_MASS * GRAVITY - external_lift_force;
    if (blade->effort_lift * external_total_lift <= 0) {
        external_total_lift *= (1.0f - HYDRAULIC_STIFFNESS);
    }
    blade->vel_arm_height = (blade->vel_arm_height + ((blade->effort_lift * MAX_FORCE_LIFT + external_total_lift) / ARM_MASS) * dt) / (1.0f + (ARM_DAMPING / ARM_MASS) * dt);
    blade->arm_height += blade->vel_arm_height * dt;
    if (blade->arm_height < ARM_MIN) {
        blade->arm_height = ARM_MIN;
    }
    if (blade->arm_height > ARM_MAX) {
        blade->arm_height = ARM_MAX;
    }

    // Blade Pitch and Roll
    float blade_stiffness = 0.98f;
    float ext_pitch = blade->last_force * 0.5f;
    if (blade->effort_pitch * ext_pitch <= 0) {
        ext_pitch *= (1.0f - blade_stiffness);
    }
    blade->vel_pitch_rel = (blade->vel_pitch_rel + ((blade->effort_pitch * MAX_TORQUE_PITCH + ext_pitch) / PITCH_INERTIA) * dt) / (1.0f + (PITCH_DAMPING / PITCH_INERTIA) * dt);
    blade->blade_pitch_rel += blade->vel_pitch_rel * dt;
    if (blade->blade_pitch_rel < PITCH_MIN) {
        blade->blade_pitch_rel = PITCH_MIN;
    }
    if (blade->blade_pitch_rel > PITCH_MAX) {
        blade->blade_pitch_rel = PITCH_MAX;
    }

    float ext_roll = blade->last_yaw_moment * 0.5f;
    if (blade->effort_roll * ext_roll <= 0) {
        ext_roll *= (1.0f - blade_stiffness);
    }
    blade->vel_roll_rel = (blade->vel_roll_rel + ((blade->effort_roll * MAX_TORQUE_ROLL + ext_roll) / ROLL_INERTIA) * dt) / (1.0f + (ROLL_DAMPING / ROLL_INERTIA) * dt);
    blade->blade_roll_rel += blade->vel_roll_rel * dt;
    if (blade->blade_roll_rel < ROLL_MIN) {
        blade->blade_roll_rel = ROLL_MIN;
    }
    if (blade->blade_roll_rel > ROLL_MAX) {
        blade->blade_roll_rel = ROLL_MAX;
    }

    blade->rake_angle = (45.0f * (PI/180.0f)) + blade->blade_pitch_rel + blade->pitch;
    precompute_FEE(blade->rake_angle, 0.0f);

    float slip = blade->last_force / 35000.0f; 
    if (slip < 0) {
        slip = 0;
    }
    if (slip > 1) {
        slip = 1;
    }
    blade->yaw += blade->v_rotational * dt;
    blade->x += blade->v_linear * (1.0f - slip) * cosf(blade->yaw) * dt;
    blade->y += blade->v_linear * (1.0f - slip) * sinf(blade->yaw) * dt;
    
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
        if (start < 0) {
            start = 0;
        }
        if (end >= GRID_SIZE) {
            end = GRID_SIZE - 1;
        }

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
        if (front < GRID_SIZE) {
            float depth = env->grid_H[front][j] - (blade->z + w * sinf(blade->roll + blade->blade_roll_rel));
            if (depth > 0) {
                float df = calculate_FEE_column(blade, depth, CELL_SIZE);
                total_force += df; total_yaw_moment -= df * w;
            }
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
    
    if (outfile && env->step_num % 5 == 0) {
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
    env->step_num++;
}

int main() {
#ifndef BENCHMARK
    outfile = fopen("out/sim_out.bin", "wb");
    if (!outfile) return 1;
#endif
    SoilEnv* env = env_init(); env_reset(env, 42);
    precompute_FEE(env->blade.rake_angle, 0.0f); update_kinematics(env);
    
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
    for (int t = 0; t < 1000; t++) {
        if (t < 200) {
            env->blade.effort_linear = 0.8f;
        }
        if (t < 100) {
            env->blade.effort_lift = -0.1f;
        }
        if (t > 100) {
            env->blade.effort_lift = 0.0f;
        }
        simulate_step(env, 0.01f);
    }
    if (outfile) {
        fclose(outfile);
    }
    printf("Simulation Complete. Data written to sim_out.bin.\n");
#endif
    env_free(env);
    return 0;
}
