#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define GRID_SIZE 200
#define PI 3.14159265358979323846f
#define CELL_SIZE 0.1f 

#define SWELL_RATIO 1.2f
#define LOOSE_SOIL_DENSITY 1300.0f 
#define GRAVITY 9.81f

// Global Soil Properties
float env_c = 1000.0f;     
float env_c_a = 500.0f;    
float env_phi = 30.0f * (PI / 180.0f); 
float env_delta = 20.0f * (PI / 180.0f); 
float env_gamma = 1500.0f * GRAVITY; 

// Precomputed FEE Factors
float N_gamma, N_Q, N_c, N_ca;

// Struct of Arrays (SoA) for cache coherency
float grid_H[GRID_SIZE][GRID_SIZE];
float grid_L[GRID_SIZE][GRID_SIZE];

typedef struct {
    // Spatial Origin (Center of Blade)
    float x;           
    float y;           
    float lat_pos; // Lateral position (Y-axis in world coords)
    
    // Blade Geometry & State
    float width;       
    float rake_angle;  
    float surcharge_Q; 
    
    // 6DOF Kinematics
    float pitch;
    float roll;
    float yaw;
    float blade_roll_rel; // Operator command: blade roll relative to chassis
    
    float loader_x;
    float loader_z;
    
    // Dynamics
    float last_force; 
    float last_yaw_moment;
} Blade;

FILE* outfile;
int step_num = 0;

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

void init_grid() {
    srand(42); 
    
    // 1. Initial multi-scale random noise
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float fine = ((float)rand() / RAND_MAX - 0.5f) * 0.1f;
            float coarse = ((float)rand() / RAND_MAX - 0.5f) * 0.4f;
            grid_H[i][j] = 1.0f + fine + coarse;
            grid_L[i][j] = 0.0f;
        }
    }

    // 2. Multi-pass smoothing
    for (int iter = 0; iter < 3; iter++) {
        float smoothed[GRID_SIZE][GRID_SIZE];
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                float sum = 0; int count = 0;
                for (int di = -2; di <= 2; di++) {
                    for (int dj = -2; dj <= 2; dj++) {
                        int ni = i + di; int nj = j + dj;
                        if (ni >= 0 && ni < GRID_SIZE && nj >= 0 && nj < GRID_SIZE) {
                            sum += grid_H[ni][nj]; count++;
                        }
                    }
                }
                smoothed[i][j] = sum / count;
            }
        }
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                grid_H[i][j] = smoothed[i][j];
            }
        }
    }

    // 3. Add an ASYMMETRIC berm on the right side to test yaw forces
    // float berm_x = 4.0f;
    // float berm_height = 0.75f; 
    // float berm_width = 2.0f;  
    
    // for (int i = 0; i < GRID_SIZE; i++) {
    //     float x_m = i * CELL_SIZE;
    //     float dist = fabsf(x_m - berm_x);
    //     if (dist < berm_width / 2.0f) {
    //         float h_add = berm_height * 0.5f * (1.0f + cosf(2.0f * PI * dist / berm_width));
    //         // ONLY ADD TO RIGHT HALF OF THE MAP (j >= center)
    //         for (int j = 0; j < GRID_SIZE/2.0f - 5; j++) {
    //             grid_H[i][j] += h_add;
    //         }
    //     }
    // }
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
void simulate_erosion(Blade* blade) {
    int margin_cells = (int)(4.0f / CELL_SIZE);
    int min_i = (int)(blade->x / CELL_SIZE) - margin_cells;
    int max_i = (int)(blade->x / CELL_SIZE) + margin_cells;
    int min_j = (int)(blade->lat_pos / CELL_SIZE) - margin_cells;
    int max_j = (int)(blade->lat_pos / CELL_SIZE) + margin_cells;

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
                tempL[i][j] = grid_L[i][j];
            }
        }
        
        int dx[] = {1, -1, 0, 0};
        int dy[] = {0, 0, 1, -1};
        float max_slope = tanf(env_phi);
        float max_dH = CELL_SIZE * max_slope;
        float loader_length = 2.0f;

        for (int i = min_i; i <= max_i; i++) {
            for (int j = min_j; j <= max_j; j++) {
                if (grid_L[i][j] <= 1e-4f) continue;
                float total_h = grid_H[i][j] + grid_L[i][j];
                
                for(int d=0; d<4; d++) {
                    int ni = i + dx[d];
                    int nj = j + dy[d];
                    if (ni >= min_i_temp && ni <= max_i_temp && nj >= min_j_temp && nj <= max_j_temp) {
                        
                        // Rotated/Translated rigid machine collision box
                        float cell_x = ni * CELL_SIZE;
                        float cell_lat = nj * CELL_SIZE;
                        float w = cell_lat - blade->lat_pos; // Lateral distance from machine center
                        if (fabsf(w) <= blade->width / 2.0f) {
                            float blade_front_x = blade->x - w * sinf(blade->yaw);
                            float loader_back_x = blade_front_x - loader_length * cosf(blade->yaw);
                            if (cell_x >= loader_back_x && cell_x <= blade_front_x) {
                                continue; // Blocked by machine chassis
                            }
                        }
                        
                        float neighbor_total_h = grid_H[ni][nj] + grid_L[ni][nj];
                        float dH = total_h - neighbor_total_h;
                        
                        if (dH > max_dH) {
                            float slip = (dH - max_dH) / 2.0f;
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
        
        for (int i = min_i_temp; i <= max_i_temp; i++) {
            for (int j = min_j_temp; j <= max_j_temp; j++) {
                grid_L[i][j] = tempL[i][j];
            }
        }
    }
}

// Vehicle Kinematics Model with Yaw and Lat Pos
void update_kinematics(Blade* blade) {
    float track_length = 2.0f;
    float track_gauge = 1.5f;
    float blade_offset_x = 1.5f; 
    float target_cut_depth = -0.05f; 

    // Chassis center (behind the blade)
    blade->loader_x = blade->x - blade_offset_x * cosf(blade->yaw);
    float loader_lat = blade->lat_pos - blade_offset_x * sinf(blade->yaw);
    
    float cos_y = cosf(blade->yaw);
    float sin_y = sinf(blade->yaw);
    
    // 4 Sampling corners rotated by yaw
    float hL = track_length / 2.0f;
    float hW = track_gauge / 2.0f;
    
    float fl_x = blade->loader_x + hL * cos_y - hW * sin_y;
    float fl_lat = loader_lat + hL * sin_y + hW * cos_y;
    
    float fr_x = blade->loader_x + hL * cos_y + hW * sin_y;
    float fr_lat = loader_lat + hL * sin_y - hW * cos_y;
    
    float rl_x = blade->loader_x - hL * cos_y - hW * sin_y;
    float rl_lat = loader_lat - hL * sin_y + hW * cos_y;
    
    float rr_x = blade->loader_x - hL * cos_y + hW * sin_y;
    float rr_lat = loader_lat - hL * sin_y - hW * cos_y;
    
    float z_FL = 1.0f, z_FR = 1.0f, z_RL = 1.0f, z_RR = 1.0f;
    
    int ix, iy;
    ix = (int)(fl_x / CELL_SIZE); iy = (int)(fl_lat / CELL_SIZE);
    if(ix>=0 && ix<GRID_SIZE && iy>=0 && iy<GRID_SIZE) z_FL = grid_H[ix][iy] + grid_L[ix][iy];
    
    ix = (int)(fr_x / CELL_SIZE); iy = (int)(fr_lat / CELL_SIZE);
    if(ix>=0 && ix<GRID_SIZE && iy>=0 && iy<GRID_SIZE) z_FR = grid_H[ix][iy] + grid_L[ix][iy];
    
    ix = (int)(rl_x / CELL_SIZE); iy = (int)(rl_lat / CELL_SIZE);
    if(ix>=0 && ix<GRID_SIZE && iy>=0 && iy<GRID_SIZE) z_RL = grid_H[ix][iy] + grid_L[ix][iy];
    
    ix = (int)(rr_x / CELL_SIZE); iy = (int)(rr_lat / CELL_SIZE);
    if(ix>=0 && ix<GRID_SIZE && iy>=0 && iy<GRID_SIZE) z_RR = grid_H[ix][iy] + grid_L[ix][iy];
    
    float z_front = (z_FL + z_FR) / 2.0f;
    float z_rear = (z_RL + z_RR) / 2.0f;
    float z_left = (z_FL + z_RL) / 2.0f;
    float z_right = (z_FR + z_RR) / 2.0f;
    
    blade->loader_z = (z_front + z_rear) / 2.0f;
    blade->pitch = atan2f(z_front - z_rear, track_length);
    blade->roll = atan2f(z_left - z_right, track_gauge);
    
    blade->y = blade->loader_z + blade_offset_x * sinf(blade->pitch) - target_cut_depth;
}

void simulate_step(Blade* blade, float dt) {
    float prev_x = blade->x;
    float prev_y = blade->y; 
    float prev_lat = blade->lat_pos;

    // Track Slip Dynamics
    float max_traction = 40000.0f; 
    float slip_ratio = blade->last_force / max_traction;
    if (slip_ratio < 0.0f) slip_ratio = 0.0f;
    if (slip_ratio > 1.0f) slip_ratio = 1.0f; 
    
    float v_cmd = 1.0f; 
    float v_actual = v_cmd * (1.0f - slip_ratio);

    // Yaw Drift Dynamics
    // Last yaw moment twists the machine. Tracks have high friction (stiffness).
    float yaw_stiffness = 50000.0f; // Nm/rad roughly
    blade->yaw -= (blade->last_yaw_moment / yaw_stiffness) * dt;

    // Move Forward in 2D
    blade->x += v_actual * cosf(blade->yaw) * dt; 
    blade->lat_pos += v_actual * sinf(blade->yaw) * dt;
    
    float curr_x = blade->x;
    float curr_lat = blade->lat_pos;
    
    update_kinematics(blade);
    float curr_y = blade->y;
    
    float total_vol_cut = 0.0f;
    float total_force = 0.0f;
    float total_yaw_moment = 0.0f;
    
    // 1. Per-Column Cut & Force Calculation
    for (int j = 0; j < GRID_SIZE; j++) {
        float cell_lat = j * CELL_SIZE;
        float w = cell_lat - curr_lat; // Offset from blade center
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
            
            float interp_center_y = prev_y + t * (curr_y - prev_y);
            // Apply blade roll relative to chassis
            float blade_elev = interp_center_y + w * sinf(blade->roll + blade->blade_roll_rel);
            
            float total_h = grid_H[i][j] + grid_L[i][j];
            float depth = total_h - blade_elev;
            
            if (depth > 0) {
                if (grid_L[i][j] > 0) {
                    float l_cut = (depth < grid_L[i][j]) ? depth : grid_L[i][j];
                    grid_L[i][j] -= l_cut;
                    depth -= l_cut;
                    total_vol_cut += l_cut * CELL_SIZE * CELL_SIZE; 
                }
                if (depth > 0) {
                    grid_H[i][j] -= depth;
                    total_vol_cut += depth * CELL_SIZE * CELL_SIZE * SWELL_RATIO; 
                }
            }
        }
        
        // Calculate physics force specific to this column hitting the undisturbed wall
        int front_idx = end_idx + 1;
        if (front_idx < GRID_SIZE) {
            float blade_elev = curr_y + w * sinf(blade->roll + blade->blade_roll_rel);
            float static_depth = grid_H[front_idx][j] - blade_elev;
            if (static_depth > 0) {
                float df = calculate_FEE_column(blade, static_depth, CELL_SIZE);
                total_force += df;
                // Force opposes forward motion. Force on Right (w>0) pushes Right Side Back -> Turns Left (-Yaw)
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
            float cell_lat = j * CELL_SIZE;
            float w = cell_lat - curr_lat;
            if (fabsf(w) > blade->width / 2.0f) continue;
            
            float curr_edge_x = curr_x - w * sinf(blade->yaw);
            int deposit_idx = (int)(curr_edge_x / CELL_SIZE) + 1;
            
            if (deposit_idx >= 0 && deposit_idx < GRID_SIZE) {
                grid_L[deposit_idx][j] += dh;
            }
        }
    }

    // 3. Stability check
    simulate_erosion(blade);

    // 4. Update Surcharge Q Dynamically
    float current_surcharge_vol = 0;
    for (int j = 0; j < GRID_SIZE; j++) {
        float cell_lat = j * CELL_SIZE;
        float w = cell_lat - curr_lat;
        if (fabsf(w) > blade->width / 2.0f) continue;
        
        float curr_edge_x = curr_x - w * sinf(blade->yaw);
        int start_i = (int)(curr_edge_x / CELL_SIZE) + 1;
        
        for (int i = start_i; i <= start_i + 15; i++) {
            if (i >= 0 && i < GRID_SIZE) {
                current_surcharge_vol += grid_L[i][j] * CELL_SIZE * CELL_SIZE;
            }
        }
    }
    blade->surcharge_Q = current_surcharge_vol * LOOSE_SOIL_DENSITY * GRAVITY;

#ifndef BENCHMARK
    if (step_num % 5 == 0) {
        printf("Step %d [X: %.2fm, Y: %.2fm]: Yaw: %5.2f° | Roll: %4.1f° | Slip: %3.0f%% | M_yaw: %6.0f Nm\n", 
               step_num, blade->x, blade->lat_pos, blade->yaw * (180.0f/PI), 
               blade->roll * (180.0f/PI), slip_ratio * 100.0f, blade->last_yaw_moment);
    }

    // Write to Rerun visualizer in binary format for high performance
    if (outfile) {
        struct {
            int step;
            float bx, by, bw, pitch, roll, lx, lz, lat, yaw, blade_roll_rel;
            int grid_size;
            float cell_size;
        } header = {
            step_num, blade->x, blade->y, blade->width, 
            blade->pitch, blade->roll, blade->loader_x, blade->loader_z,
            blade->lat_pos, blade->yaw, blade->blade_roll_rel,
            GRID_SIZE, CELL_SIZE
        };
        
        fwrite(&header, sizeof(header), 1, outfile);

        float flat_grid[GRID_SIZE * GRID_SIZE];
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                flat_grid[i * GRID_SIZE + j] = grid_H[i][j] + grid_L[i][j];
            }
        }
        fwrite(flat_grid, sizeof(float), GRID_SIZE * GRID_SIZE, outfile);
    }
#endif
    step_num++;
}

int main() {
#ifndef BENCHMARK
    outfile = fopen("sim_out.bin", "wb");
    if (!outfile) return 1;
#endif

    init_grid();
    
    Blade blade = {0.0f, 0.8f, 10.0f, 3.0f, 45.0f * (PI / 180.0f), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    precompute_FEE(blade.rake_angle, 0.0f);
    update_kinematics(&blade); 
    
#ifdef BENCHMARK
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    int steps = 1000;
    for (int t = 0; t < steps; t++) {
        simulate_step(&blade, 0.01f);
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    double elapsed = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    printf("Benchmarking RL-Optimized Sim: %d steps in %.4f seconds (%.2f Hz)\n", steps, elapsed, steps / elapsed);
#else
    printf("Starting 6DOF 3D Soil Simulation for RL...\n");
    float dt = 0.02f; 
    for (int t = 0; t < 1000; t++) {
        // Let's oscillate the blade roll command dynamically to test it!
        // blade.blade_roll_rel = sinf(t * 0.05f) * 0.1f;
        blade.blade_roll_rel = 0.1f;
        simulate_step(&blade, dt);
    }
    
    if (outfile) fclose(outfile);
    printf("Simulation Complete. Data written to sim_out.bin.\n");
#endif

    return 0;
}