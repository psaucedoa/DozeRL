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
#define EROSION_MARGIN 20  // cells buffered around the blade for slumping (~4.0m at CELL_SIZE=0.2f); keep in sync with CELL_SIZE
#define EROSION_WIN (2 * EROSION_MARGIN + 1)  // side length of the local erosion scratch window
#define GRAVITY 9.81f
#define PI 3.14159265358979323846f
// for more global params, see env_reset(), since we'll be randomizing these values between runs

// Required PufferLib Log struct. Only use floats!
typedef struct {
    float perf; // Recommended 0-1 normalized. 0 = none of goal map built, 1 = all of goal map built
    float score; // Recommended unnormalized single real number perf metric
    float episode_return; // Recommended metric: sum of agent rewards over episode
    float episode_length; // Recommended metric: number of steps of agent episode

    // float count_large_neg_rewards; // Custom metric: average large negative rewards per episode
    // float count_off_map;  // Custom metric: average off-map steps per episode
    // float count_jitter;   // Custom metric: average high-jitter steps per episode

    // max values
    float max_vel_arm;  // Custom metric: max arm angular velocity per episode
    float max_vel_blade_pitch;  // Custom metric: max blade pitch angular velocity per episode
    // float max_vel_blade_roll;  // Custom metric: max blade roll velocity per episode
    float max_vel_linear;  // Custom metric: max linear vehicle velocity per episode

    // min values
    float min_vel_arm;  // Custom metric: min arm angular velocity per episode
    float min_vel_blade_pitch;  // Custom metric: min blade pitch angular velocity per episode
    // float min_vel_blade_roll;  // Custom metric: min blade roll velocity per episode
    float min_vel_linear;  // Custom metric: min linear vehicle velocity per episode

    // reward signals per ep
    // float total_reward;
    float r_shaping;
    float r_off_map;
    float r_time;

    float n; // Required as the last field
} Log;

typedef struct
{
  // Spatial Origin (Center of Vehicle Base)
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
  float q[4];             // [w, x, y, z] quaternion orientation of the chassis

  // Effort Inputs (-1.0 to 1.0)
  float effort_lift;        // efort
  float effort_pitch;       // efort
  float effort_roll;        // efort
  float effort_yaw;         // efort
  float effort_linear;      // efort
  float effort_rotational;  // efort

  // Joint global coords [x, y, z, r, p, y]
  float _lift_arm_joint_pose[6];
  float _pitch_joint_pose[6];
  float _u_joint_pose[6];
  float _blade_edge_pose[6];

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
  float blade_height;       // (m)
  float blade_mount_pitch;  // (rad)
  float blade_rake_angle;   // (rad)
  float blade_surcharge_Q;  // (N)  TODO: CHECK UNITS
  float blade_x;            // (m)
  float blade_y;            // (m)
  float blade_z;            // (m)

  // Track Geometry
  float track_width;  // width of individual track
  float track_length; // length of track that makes contact with soil on flat plane
  float track_gauge;  // spacing between track centerpoints
  float track_contact_area;

  // Arm Geometry
  float arm_pivot_x;
  float arm_pivot_z;
  float arm_length;
  float pitch_length;

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
  float last_force;       // (N) Previous Reaction Force?
  float last_yaw_moment;  // (N*m)
  float last_roll_moment; // (N*m)
  float last_push_sign;   // (+1 forward / -1 backdrag) soil-push direction, held across steps for hysteresis

  // Actuator Dynamics Constants
  float max_torque_lift_arm;
  float max_torque_pitch;
  float max_torque_roll;
  float max_force_track;  // (N) max hydraulic thrust one track can produce; linear & yaw both derive from this
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

typedef struct
{
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
  float soil_c;  // soil cohesion
  float soil_c_a;  // adhesion
  float soil_phi;  // soil internal friction angle
  float soil_delta;
  float soil_gamma;  // moist? unit weight of soil
  float soil_q_u; // soil ultimate bearing capacity | NOTE: Since we're dealing with 'homogenous' soil properties, we can effectively just calculate this once!

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
  float cur_error; // incremental volume error over cut/fill zone
  float prev_progress;
  float prev_error;
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

typedef struct
{
  float proprioceptive[11];
  float spatial[2][SPATIAL_OBS_SIZE][SPATIAL_OBS_SIZE];
} Observation;

// Quaternion helpers
static inline void euler_to_quat(float r, float p, float y, float q[4])
{
  float cx = cosf(r * 0.5f), sx = sinf(r * 0.5f);
  float cy = cosf(p * 0.5f), sy = sinf(p * 0.5f);
  float cz = cosf(y * 0.5f), sz = sinf(y * 0.5f);

  q[0] = cx * cy * cz + sx * sy * sz; // w
  q[1] = sx * cy * cz - cx * sy * sz; // x
  q[2] = cx * sy * cz + sx * cy * sz; // y
  q[3] = cx * cy * sz - sx * sy * cz; // z
}

static inline void quat_to_euler(const float q[4], float rpy[3])
{
  float w = q[0], x = q[1], y = q[2], z = q[3];

  float r20 = 2.0f * (w * y - x * z);
  float r21 = 2.0f * (y * z + w * x);
  float r22 = 1.0f - 2.0f * (x * x + y * y);
  float r10 = 2.0f * (x * y + w * z);
  float r00 = 1.0f - 2.0f * (y * y + z * z);

  rpy[1] = atan2f(r20, sqrtf(r21 * r21 + r22 * r22)); // Pitch
  rpy[0] = atan2f(r21, r22); // Roll
  rpy[2] = atan2f(r10, r00); // Yaw
}

static inline void quat_multiply(const float q1[4], const float q2[4], float out[4])
{
  float w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
  float w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];

  out[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
  out[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
  out[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
  out[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}

static inline void quat_rotate(const float q[4], const float v[3], float out[3])
{
  float w = q[0], qx = q[1], qy = q[2], qz = q[3];
  float vx = v[0], vy = v[1], vz = v[2];

  // t = 2 * (q_xyz x v)
  float tx = 2.0f * (qy * vz - qz * vy);
  float ty = 2.0f * (qz * vx - qx * vz);
  float tz = 2.0f * (qx * vy - qy * vx);

  // v' = v + w * t + q_xyz x t
  out[0] = vx + w * tx + (qy * tz - qz * ty);
  out[1] = vy + w * ty + (qz * tx - qx * tz);
  out[2] = vz + w * tz + (qx * ty - qy * tx);
}

// Helper: Thread-safe random float [0, 1]
static inline float rand_f(unsigned int* seed)
{
  return (float)rand_r(seed) / (float)RAND_MAX;
}

// Helper: Clamp action to [-1.0f, 1.0f]
static inline float clamp_action(float x)
{
  return x < -1.0f ? -1.0f : (x > 1.0f ? 1.0f : x);
}

// Helper: Clamp index to valid grid range
static inline int clamp_idx(int idx)
{
  return idx < 0 ? 0 : (idx >= GRID_SIZE ? GRID_SIZE - 1 : idx);
}

static inline void get_obs(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;

  // get observations. First [0-10] are proprioceptive (10 + noisy surcharge), rest are map (2x50x50)
  env->observations[0]  = dozer->pos_virtual_lift_arm;
  env->observations[1]  = dozer->pos_blade_pitch;
  env->observations[2]  = dozer->pos_blade_roll;
  env->observations[3]  = dozer->pos_blade_yaw;  // we still observe this, even without direct control
  env->observations[4]  = dozer->vel_tracks_rotational;
  env->observations[5]  = dozer->vel_tracks_linear;
  env->observations[6]  = dozer->vel_virtual_lift_arm;
  env->observations[7]  = dozer->vel_blade_pitch;
  env->observations[8]  = dozer->vel_blade_roll;
  env->observations[9]  = dozer->vel_blade_yaw;  // this one, however, may not be necessary..?
  // Noisy surcharge: real platform would estimate from cylinder pressure,
  // so model as ±20% multiplicative + ±500N additive, normalized to ~1.0 ≈ 15kN loaded
  {
    float q = dozer->blade_surcharge_Q;
    float q_noisy = q * (1.0f + (rand_f(&env->rng) - 0.5f) * 0.4f) + (rand_f(&env->rng) - 0.5f) * 1000.0f;
    if (q_noisy < 0.0f) q_noisy = 0.0f;
    env->observations[10] = q_noisy / 15000.0f; // 0=empty, ~1=full blade
  }
  int obs_offset = 11;  // 11 proprio (10 + noisy surcharge) before spatial
  // onto map observations
  // first we compute cos & sin for the vehicle's current yaw
  float cos_y = cosf(dozer->angular_z);
  float sin_y = sinf(dozer->angular_z);

  // next we get the dozer's cell position and shift by half the obs grid size to get to the starting corner
  float half_obs_grid = SPATIAL_OBS_SIZE * 0.5f;
  float obs_start_x = (dozer->position_x / CELL_SIZE) - half_obs_grid * cos_y + half_obs_grid * sin_y;
  float obs_start_y = (dozer->position_y / CELL_SIZE) - half_obs_grid * sin_y - half_obs_grid * cos_y;

  // then we iterate to get all cells in the obs space
  for (int i = 0; i < SPATIAL_OBS_SIZE; i++)
  {
    float row_grid_x = obs_start_x + i * cos_y;
    float row_grid_y = obs_start_y + i * sin_y;

    for (int j = 0; j < SPATIAL_OBS_SIZE; j++)
    {
      int grid_i = (int)floorf(row_grid_x - j * sin_y);
      int grid_j = (int)floorf(row_grid_y + j * cos_y);

      // here we do some index mapping from 2d grid coords -> 1d obs array shape
      int h_obs_index = (i * SPATIAL_OBS_SIZE) + j + obs_offset;
      int g_obs_index = h_obs_index + (SPATIAL_OBS_SIZE * SPATIAL_OBS_SIZE);  // we offset this by the amount of cells in the obs to concatenate to 1d array

      // if the selected env grid is within bounds of the sim region, grab the underlying map data
      if ((unsigned int)grid_i < GRID_SIZE && (unsigned int)grid_j < GRID_SIZE)
      {
        env->observations[h_obs_index] = (env->grid_H[grid_i][grid_j] + env->grid_L[grid_i][grid_j]) - dozer->position_z;
        env->observations[g_obs_index] = env->grid_G[grid_i][grid_j] - dozer->position_z;
      }
      else  // if it's outside the sim region, just fill with 0s
      {
        env->observations[h_obs_index] = 0.0f;
        env->observations[g_obs_index] = 0.0f;
      }
    }
  }
}

static inline float compute_terrain_error(SoilEnv* env)
{
  float err = 0.0f;
  float cell_area = CELL_SIZE * CELL_SIZE;
  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      char region = env->map_region[i][j];
      if (region == 0) continue; // neutral: free transit (option A)
      float cur_h = env->grid_H[i][j] + env->grid_L[i][j];
      float goal = env->grid_G[i][j];
      float contrib;
      if (region == 1) { // CUT: need cur <= goal, penalize under-cut fully, over-cut 0.2x
        float over = cur_h - goal; // >0 means still above goal, need more cut
        contrib = (over > 0.0f) ? over : -over * 0.2f;
      } else { // region==2 FILL: need cur >= goal
        float under = goal - cur_h; // >0 means still below goal, need more fill
        contrib = (under > 0.0f) ? under : -under * 0.2f;
      }
      err += contrib * cell_area;
    }
  }
  return err;
}

static inline float cell_error_contrib(SoilEnv* env, int i, int j)
{
  char region = env->map_region[i][j];
  if (region == 0) return 0.0f;
  float cur_h = env->grid_H[i][j] + env->grid_L[i][j];
  float goal = env->grid_G[i][j];
  if (region == 1) {
    float over = cur_h - goal;
    float contrib = (over > 0.0f) ? over : -over * 0.2f;
    return contrib * CELL_SIZE * CELL_SIZE;
  } else {
    float under = goal - cur_h;
    float contrib = (under > 0.0f) ? under : -under * 0.2f;
    return contrib * CELL_SIZE * CELL_SIZE;
  }
}

static inline void update_reward_and_terminal(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;
  float cur_error = env->cur_error;
  float prev_error = env->prev_error;

  float init = env->initial_error;
  float progress = (init - cur_error) / init;
  if (progress < 0.0f) progress = 0.0f;
  if (progress > 1.0f) progress = 1.0f;
  // dense shaping: delta progress scaled
  // float r_shaping = (cur_error - prev_error) * 1000 / init;

  // penalties
  // float r_time = -0.001f;
  float r_off_map = 0.0f;

  // off-map tracking (for logs)
  if (dozer->position_x < 0.0f || dozer->position_x > GRID_SIZE * CELL_SIZE ||
      dozer->position_y < 0.0f || dozer->position_y > GRID_SIZE * CELL_SIZE) {
    env->count_off_map += 1.0f;
    r_off_map += -5.0;
  }

  // float r_motion = dozer->vel_tracks_linear * 0.005;
  float r_rotation = -1.0 * fabs(dozer->vel_tracks_rotational) * 0.05;

  // float r_surcharge = 0;
  // if (dozer->blade_surcharge_Q > 0)
  // {
    // r_surcharge = 0.01;
  // }

  // float reward = r_shaping + r_time + r_off_map + r_motion;
  // float reward = r_shaping + r_time + r_off_map;
  // float reward = r_off_map + r_motion + r_rotation + r_surcharge + (progress - env->prev_progress) * 100;
  float reward = r_off_map + r_rotation + (progress - env->prev_progress) * 1000;

  // success bonus + terminal
  int done = 0;
  if (cur_error < 0.02f * init) {
    reward += 5.0f;
    done = 1;
  }

  // time limit (1 min at 60Hz control = 3600 steps)
  if (env->step_num >= 3600) done = 1;
  // if (env->count_off_map > 100.0f) done = 1; // persistent off-map

  env->rewards[0] = reward;
  env->terminals[0] = done ? 1.0f : 0.0f;

  // velocity extremes for debugging — track true max/min per episode
  if (dozer->vel_virtual_lift_arm > env->log.max_vel_arm) env->log.max_vel_arm = dozer->vel_virtual_lift_arm;
  if (dozer->vel_virtual_lift_arm < env->log.min_vel_arm) env->log.min_vel_arm = dozer->vel_virtual_lift_arm;
  if (dozer->vel_blade_pitch > env->log.max_vel_blade_pitch) env->log.max_vel_blade_pitch = dozer->vel_blade_pitch;
  if (dozer->vel_blade_pitch < env->log.min_vel_blade_pitch) env->log.min_vel_blade_pitch = dozer->vel_blade_pitch;
  // if (dozer->vel_blade_roll > env->log.max_vel_blade_roll) env->log.max_vel_blade_roll = dozer->vel_blade_roll;
  // if (dozer->vel_blade_roll < env->log.min_vel_blade_roll) env->log.min_vel_blade_roll = dozer->vel_blade_roll;
  if (dozer->twist_linear_x > env->log.max_vel_linear) env->log.max_vel_linear = dozer->twist_linear_x;
  if (dozer->twist_linear_x < env->log.min_vel_linear) env->log.min_vel_linear = dozer->twist_linear_x;

  // logs for pufferlib — only counted when n=1 (episode done)
  // env->log.r_shaping += r_shaping;
  // env->log.r_time += r_time;
  env->log.r_off_map += r_off_map;
  env->episode_return += reward;

  env->prev_error = cur_error;
  env->prev_progress = progress;

  env->log.perf = progress;
  env->log.score = -cur_error; // lower error = higher score
  // env->log.total_reward += reward;
  env->log.episode_return = env->episode_return;
  env->log.episode_length = (float)env->step_num;
  // env->log.count_large_neg_rewards = env->count_large_neg_rewards;
  // env->log.count_off_map = env->count_off_map;
  // env->log.count_jitter = env->count_jitter;
  env->log.n = done ? 1.0f : 0.0f;
}

static inline void precompute_FEE(SoilEnv* env, float alpha)
{
  Dozer * dozer = &env->dozer;

  float rho = dozer->blade_rake_angle;
  float beta = (PI / 4.0f) - (env->soil_phi / 2.0f);
  // clamp beta/rho to avoid sin/tan singularities when phi→90° or rake→0°
  float rho_eff = fmaxf(rho, 5.0f * PI / 180.0f);
  float beta_eff = fmaxf(beta, 5.0f * PI / 180.0f);
  float eta = env->soil_delta + rho_eff + env->soil_phi + beta_eff;
  float sin_eta = sinf(eta);
  if (fabsf(sin_eta) < 1e-6f) sin_eta = 1e-6f;
  float sin_beta = sinf(beta_eff);
  if (fabsf(sin_beta) < 1e-6f) sin_beta = 1e-6f;
  float sin_rho = sinf(rho_eff);
  if (fabsf(sin_rho) < 1e-6f) sin_rho = 1e-6f;
  float tan_rho = tanf(rho_eff);
  if (fabsf(tan_rho) < 1e-6f) tan_rho = 1e-6f;
  float tan_beta = tanf(beta_eff);
  if (fabsf(tan_beta) < 1e-6f) tan_beta = 1e-6f;

  env->N_gamma = ((1.0f / tan_rho) + (1.0f / tan_beta)) * sinf(alpha + env->soil_phi + beta_eff) / (2.0f * sin_eta);
  env->N_Q = sinf(alpha + env->soil_phi + beta_eff) / sin_eta;
  env->N_c = cosf(env->soil_phi) / (sin_beta * sin_eta);
  env->N_ca = -cosf(rho_eff + env->soil_phi + beta_eff) / (sin_rho * sin_eta);
}

static inline float calculate_max_traction(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;

  float track_area = 2.0f * dozer->track_length * dozer->track_width;
  float machine_weight = dozer->machine_mass * GRAVITY;
  return track_area * env->soil_c + machine_weight * tanf(env->soil_phi);
}

static inline void precompute_soil_bearing_capacity(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;

  // precompute the trig terms for the given step — clamp phi before trig so tan/cos stay consistent at low phi
  float phi = env->soil_phi;
  float phi_eff = fmaxf(phi, 5.0f * PI / 180.0f);
  float tan_phi = tanf(phi_eff);
  float cos_phi = cosf((PI * 0.25f) + (phi_eff * 0.5f));

  // Using Terzaghi's soil bearing capacity theory.
  // Bearing capacity factors are additionally subscripted with '_b' to distinguish them from the blade FEE factors
  // Q_u = c * N_c + gamma * D * N_q + 0.5 * gamma * B * N_gamma
  //                                    ^ this value is because we assume a "strip footing"
  // Q_u = [cohesion] + [footing depth & overburden pressure] + [footing width & length of shear stress area]
  float N_q_b = expf( (3.0f * PI * 0.5f * tan_phi) - phi_eff * tan_phi ) / ( 2.0f *  cos_phi * cos_phi);
  float N_c_b = (N_q_b - 1.0f) / tan_phi;
  if (phi < 5.0f * PI / 180.0f) N_c_b = 5.7f; // Terzaghi phi=0 limit
  float N_gamma_b = 2.0f * (N_q_b + 1.0f) * tan_phi;

  // given that out 'footing' is at the surface, our foundation depth is 0, and the second term goes to 0, thus
  // Q_u = [cohesion] + [footing width & length of shear stress area]
  env->soil_q_u = env->soil_c * N_c_b + 0.5 * env->soil_gamma * dozer->track_width * N_gamma_b;
}

static inline void update_chassis_pose(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;

  // precalculate trigs and track half-dimensions
  float cos_y = cosf(dozer->angular_z);
  float sin_y = sinf(dozer->angular_z);
  float half_track_length = dozer->track_length * 0.5f;
  float half_track_gauge = dozer->track_gauge * 0.5f;

  int num_samples = 11;
  float sum_height_left = 0.0f;
  float sum_height_right = 0.0f;
  float sum_moment_left = 0.0f;
  float sum_moment_right = 0.0f;
  float sum_length_squared = 0.0f;

  for (int i = 0; i < num_samples; i++)
  {
    float local_x = -half_track_length + (dozer->track_length * i) / (num_samples - 1);
    sum_length_squared += local_x * local_x;

    // Left track is at +half_track_gauge in local Y, Right track is at -half_track_gauge in local Y
    float point_left_x = dozer->position_x + local_x * cos_y - half_track_gauge * sin_y;
    float point_left_y = dozer->position_y + local_x * sin_y + half_track_gauge * cos_y;
    float point_right_x = dozer->position_x + local_x * cos_y + half_track_gauge * sin_y;
    float point_right_y = dozer->position_y + local_x * sin_y - half_track_gauge * cos_y;

    int grid_index_left_x = (int)floorf(point_left_x / CELL_SIZE);
    int grid_index_left_y = (int)floorf(point_left_y / CELL_SIZE);
    int grid_index_right_x = (int)floorf(point_right_x / CELL_SIZE);
    int grid_index_right_y = (int)floorf(point_right_y / CELL_SIZE);

    float soil_height_left = 1.0f;
    float soil_height_right = 1.0f;

    if ((unsigned int)grid_index_left_x < GRID_SIZE && (unsigned int)grid_index_left_y < GRID_SIZE)
    {
      soil_height_left = env->grid_H[grid_index_left_x][grid_index_left_y] + env->grid_L[grid_index_left_x][grid_index_left_y];
    }
    if ((unsigned int)grid_index_right_x < GRID_SIZE && (unsigned int)grid_index_right_y < GRID_SIZE)
    {
      soil_height_right = env->grid_H[grid_index_right_x][grid_index_right_y] + env->grid_L[grid_index_right_x][grid_index_right_y];
    }

    sum_height_left += soil_height_left;
    sum_height_right += soil_height_right;
    sum_moment_left += local_x * soil_height_left;
    sum_moment_right += local_x * soil_height_right;
  }

  dozer->position_z = (sum_height_left + sum_height_right) / (2.0f * num_samples);
  dozer->angular_y = (atan2f(sum_moment_left / sum_length_squared, 1.0f) + atan2f(sum_moment_right / sum_length_squared, 1.0f)) / 2.0f; // Pitch
  dozer->angular_x = atan2f((sum_height_left - sum_height_right) / num_samples, dozer->track_gauge); // Roll

  // Keep chassis orientation quaternion updated
  euler_to_quat(dozer->angular_x, -dozer->angular_y, dozer->angular_z, dozer->q);
}

static inline void forward_kinematics(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;

  float theta_arm = dozer->pos_virtual_lift_arm;
  float theta_pitch = dozer->pos_blade_pitch;
  float theta_rake = dozer->blade_mount_pitch;

  // 1. Lift arm joint (local in chassis frame)
  float q_arm_local[4];
  euler_to_quat(0.0f, -theta_arm, 0.0f, q_arm_local);

  float lift_arm_local_pos[3] = {dozer->arm_pivot_x, 0.0f, dozer->arm_pivot_z};

  // 2. Pitch joint (local in chassis frame)
  float arm_vector[3] = {dozer->arm_length, 0.0f, 0.0f};
  float arm_offset[3];
  quat_rotate(q_arm_local, arm_vector, arm_offset);

  float pitch_joint_local_pos[3] = {
    lift_arm_local_pos[0] + arm_offset[0],
    lift_arm_local_pos[1] + arm_offset[1],
    lift_arm_local_pos[2] + arm_offset[2]
  };

  float q_pitch_rel[4];
  euler_to_quat(0.0f, -theta_pitch, 0.0f, q_pitch_rel);

  float q_pitch_local[4];
  quat_multiply(q_arm_local, q_pitch_rel, q_pitch_local);

  // 3. Universal joint (local in chassis frame)
  float pitch_vector[3] = {dozer->pitch_length, 0.0f, 0.0f};
  float pitch_offset[3];
  quat_rotate(q_pitch_local, pitch_vector, pitch_offset);

  float u_joint_local_pos[3] = {
    pitch_joint_local_pos[0] + pitch_offset[0],
    pitch_joint_local_pos[1] + pitch_offset[1],
    pitch_joint_local_pos[2] + pitch_offset[2]
  };

  float q_u_joint_rel[4];
  euler_to_quat(dozer->pos_blade_roll, 0.0f, dozer->pos_blade_yaw, q_u_joint_rel);

  float q_u_joint_local[4];
  quat_multiply(q_pitch_local, q_u_joint_rel, q_u_joint_local);

  // 4. Blade edge (local in chassis frame) — rake is a fixed pitch of the
  // blade relative to the u-joint, so apply roll+yaw first (q_u_joint) then rake
  float q_rake_rel[4];
  euler_to_quat(0.0f, -theta_rake, 0.0f, q_rake_rel);

  float q_blade_local[4];
  quat_multiply(q_u_joint_local, q_rake_rel, q_blade_local);

  float blade_edge_vector[3] = {0.0f, 0.0f, dozer->blade_height * -0.5f};
  float blade_edge_offset[3];
  quat_rotate(q_blade_local, blade_edge_vector, blade_edge_offset);

  float blade_edge_local_pos[3] = {
    u_joint_local_pos[0] + blade_edge_offset[0],
    u_joint_local_pos[1] + blade_edge_offset[1],
    u_joint_local_pos[2] + blade_edge_offset[2]
  };

  // 5. Transform blade edge from chassis local frame to world frame using dozer->q
  float chassis_pos[3] = {dozer->position_x, dozer->position_y, dozer->position_z};

  // Blade edge world pose
  float blade_edge_world_pos[3];
  quat_rotate(dozer->q, blade_edge_local_pos, blade_edge_world_pos);
  dozer->_blade_edge_pose[0] = blade_edge_world_pos[0] + chassis_pos[0];
  dozer->_blade_edge_pose[1] = blade_edge_world_pos[1] + chassis_pos[1];
  dozer->_blade_edge_pose[2] = blade_edge_world_pos[2] + chassis_pos[2];

  float q_blade_world[4];
  quat_multiply(dozer->q, q_blade_local, q_blade_world);
  float rpy_blade[3];
  quat_to_euler(q_blade_world, rpy_blade);
  dozer->_blade_edge_pose[3] = rpy_blade[0];
  dozer->_blade_edge_pose[4] = rpy_blade[1];
  dozer->_blade_edge_pose[5] = rpy_blade[2];

  // Get global blade edge coordinates
  dozer->blade_x = dozer->_blade_edge_pose[0];
  dozer->blade_y = dozer->_blade_edge_pose[1];
  dozer->blade_z = dozer->_blade_edge_pose[2];
}

static inline void update_kinematics(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;

  // precalculate trigs, track dims, and cell area
  float cos_y = cosf(dozer->angular_z);
  float sin_y = sinf(dozer->angular_z);

  float half_track_length = dozer->track_length * 0.5f;
  float half_track_width = dozer->track_width * 0.5f;
  float half_track_gauge = dozer->track_gauge * 0.5f;
  float cell_area = CELL_SIZE * CELL_SIZE;

  // 1. Initial pose update before compaction iterations
  update_chassis_pose(env);

  // Contact-footprint bounds are invariant across the compaction iterations (position_x/y don't change
  // here -- only z/pitch/roll do), so compute them once instead of every iteration.
  int margin = (int)((half_track_length + 1.0f) / CELL_SIZE);
  int center_i = (int)floorf(dozer->position_x / CELL_SIZE);
  int min_i = clamp_idx(center_i - margin);
  int max_i = clamp_idx(center_i + margin);
  int center_j = (int)floorf(dozer->position_y / CELL_SIZE);
  int min_j = clamp_idx(center_j - margin);
  int max_j = clamp_idx(center_j + margin);

  struct {
    int i, j;
    float local_x, local_y;
  } track_cells[200];
  int num_track_cells = 0;

  for (int i = min_i; i <= max_i; i++)
  {
    for (int j = min_j; j <= max_j; j++)
    {
      float dx = ((i + 0.5f) * CELL_SIZE) - dozer->position_x;
      float dy = ((j + 0.5f) * CELL_SIZE) - dozer->position_y;
      float local_x = dx * cos_y + dy * sin_y;
      float local_y = -dx * sin_y + dy * cos_y;

      if (fabsf(local_x) <= half_track_length &&
          (fabsf(local_y - half_track_gauge) <= half_track_width ||
           fabsf(local_y + half_track_gauge) <= half_track_width))
      {
        if (num_track_cells < 200) {
          track_cells[num_track_cells].i = i;
          track_cells[num_track_cells].j = j;
          track_cells[num_track_cells].local_x = local_x;
          track_cells[num_track_cells].local_y = local_y;
          num_track_cells++;
        }
      }
    }
  }

  for (int iter = 0; iter < 3; iter++)
  {
    // 2. Determine Contact Area
    float marked_area = 0.0f;
    float tan_pitch = tanf(dozer->angular_y);
    float tan_roll = tanf(dozer->angular_x);

    for (int k = 0; k < num_track_cells; k++)
    {
      int i = track_cells[k].i;
      int j = track_cells[k].j;
      float track_z = dozer->position_z + track_cells[k].local_x * tan_pitch + track_cells[k].local_y * tan_roll;
      float soil_z = env->grid_H[i][j] + env->grid_L[i][j];
      if (soil_z >= track_z)
      {
        marked_area += cell_area;
      }
    }

    dozer->track_contact_area = marked_area;

    // 3. Calculate Ground Pressure and Compaction Rate
    float ground_pressure = 0.0f;
    float compaction_rate = 0.0f;
    if (marked_area > 0.01f)
    {
      ground_pressure = (dozer->machine_mass * GRAVITY) / marked_area;
      // Prevent divide-by-zero if soil_q_u is zero or very small
      if (env->soil_q_u > 0.001f)
      {
        compaction_rate = (ground_pressure / env->soil_q_u) * 0.5f;
        if (compaction_rate > 0.8f) compaction_rate = 0.8f;
        if (compaction_rate < 0.05f) compaction_rate = 0.05f;
      }
    }

    // 4. Apply compaction to marked cells (incremental error tracking)
    if (compaction_rate > 0.0f)
    {
      for (int k = 0; k < num_track_cells; k++)
      {
        int i = track_cells[k].i;
        int j = track_cells[k].j;
        if (env->grid_L[i][j] < 0.001f) continue;
        
        float track_z = dozer->position_z + track_cells[k].local_x * tan_pitch + track_cells[k].local_y * tan_roll;
        float soil_z = env->grid_H[i][j] + env->grid_L[i][j];

        if (soil_z >= track_z)
        {
          float old_c = cell_error_contrib(env, i, j);
          float compacted = env->grid_L[i][j] * compaction_rate;
          env->grid_L[i][j] -= compacted;
          env->grid_H[i][j] += compacted / env->swell_ratio;
          env->cur_error += cell_error_contrib(env, i, j) - old_c;
        }
      }
    }

    // 5. Update vehicle pose at the end of the compaction iteration
    update_chassis_pose(env);
  }

  // NOTE: blade forward_kinematics is intentionally NOT called here. simulate_step() updates the joint
  // positions after this function and then calls forward_kinematics() once, so computing the blade pose
  // here (from stale joint angles) would just be overwritten -- it was redundant work every step.
}

static inline void update_chassis_velocity(SoilEnv* env, float dt) {
  Dozer * dozer = &env->dozer;

  // Differential (skid-steer) mixing: a single hydraulic system feeds both tracks, so linear and
  // yaw commands share the same per-track force budget. Each track command saturates at +/-1.
  float effort_left  = clamp_action(dozer->effort_linear - dozer->effort_rotational);
  float effort_right = clamp_action(dozer->effort_linear + dozer->effort_rotational);

  // Commanded thrust per track from the single hydraulic force limit.
  float f_cmd_left  = effort_left  * dozer->max_force_track;
  float f_cmd_right = effort_right * dozer->max_force_track;

  // Traction ceiling per track (Mohr-Coulomb, even weight split between the two tracks).
  float f_trac_track = calculate_max_traction(env) * 0.5f;

  // A track slips when its commanded thrust exceeds available traction; clamp to what the soil holds.
  int slip_left  = fabsf(f_cmd_left)  > f_trac_track;
  int slip_right = fabsf(f_cmd_right) > f_trac_track;
  float f_left  = fmaxf(-f_trac_track, fminf(f_cmd_left,  f_trac_track));
  float f_right = fmaxf(-f_trac_track, fminf(f_cmd_right, f_trac_track));

  // Couple the two tracks: net forward thrust is the sum, yaw torque is the differential thrust about
  // the track center (moment arm = half the gauge). Both now scale off the one max_force_track param.
  float f_push = f_left + f_right;
  float drive_torque = (f_right - f_left) * (dozer->track_gauge * 0.5f);

  float f_resist = dozer->last_force;
  float f_net = f_push;
  float actual_f_resist = 0.0f;

  // Passive soil resistance ALWAYS opposes the direction of motion/effort.
  if (f_push > 0.0f || (f_push == 0.0f && dozer->twist_linear_x > 0.01f)) {
      if (f_resist > f_push) {
          actual_f_resist = f_push;
          f_net = 0.0f; // Stall
          if (dozer->twist_linear_x > 0.0f) {
              dozer->twist_linear_x = 0.0f;
          }
      } else {
          actual_f_resist = f_resist;
          f_net = f_push - f_resist;
      }
  } else if (f_push < 0.0f || (f_push == 0.0f && dozer->twist_linear_x < -0.01f)) {
      if (f_resist > fabsf(f_push)) {
          actual_f_resist = fabsf(f_push);
          f_net = 0.0f; // Stall
          if (dozer->twist_linear_x < 0.0f) {
              dozer->twist_linear_x = 0.0f;
          }
      } else {
          actual_f_resist = f_resist;
          f_net = f_push + f_resist; // f_push is negative, f_resist is positive, so addition reduces magnitude
      }
  }

  dozer->twist_linear_x += (f_net / dozer->machine_mass) * dt;
  dozer->twist_linear_x *= (1.0f - dozer->track_damping * dt);

  // Yaw dynamics: differential track torque plus the (scaled) soil reaction moment.
  float actual_yaw_moment = 0.0f;
  if (f_resist > 0.001f) {
      actual_yaw_moment = dozer->last_yaw_moment * (actual_f_resist / f_resist);
  }
  float torque_net = drive_torque + actual_yaw_moment;
  dozer->twist_angular_z += (torque_net / dozer->machine_inertia) * dt;
  dozer->twist_angular_z *= (1.0f - dozer->track_damping * dt);

  // Report track surface speeds. A slipping track spins at its commanded speed; a gripping track
  // matches the ground speed at its own contact line. Convert the two into the linear (mean) and
  // rotational (differential) components the observation expects.
  float max_track_speed = 3.0f;
  float half_gauge = dozer->track_gauge * 0.5f;
  float v_ground_left  = dozer->twist_linear_x - dozer->twist_angular_z * half_gauge;
  float v_ground_right = dozer->twist_linear_x + dozer->twist_angular_z * half_gauge;
  float v_track_left  = slip_left  ? effort_left  * max_track_speed : v_ground_left;
  float v_track_right = slip_right ? effort_right * max_track_speed : v_ground_right;
  dozer->vel_tracks_linear     = 0.5f * (v_track_left + v_track_right);
  dozer->vel_tracks_rotational = (v_track_right - v_track_left) / dozer->track_gauge;
}

static inline void update_chassis_2d_position(SoilEnv* env, float dt)
{
  Dozer * dozer = &env->dozer;

  // 1. Update orientation quaternion by integrating local angular velocity around Z axis
  float theta = dozer->twist_angular_z * dt;
  float q_rot[4] = {cosf(theta * 0.5f), 0.0f, 0.0f, sinf(theta * 0.5f)};
  
  float q_new[4];
  quat_multiply(dozer->q, q_rot, q_new);
  
  // Normalize the quaternion
  float len = sqrtf(q_new[0]*q_new[0] + q_new[1]*q_new[1] + q_new[2]*q_new[2] + q_new[3]*q_new[3]);
  if (len > 1e-6f)
  {
    dozer->q[0] = q_new[0] / len;
    dozer->q[1] = q_new[1] / len;
    dozer->q[2] = q_new[2] / len;
    dozer->q[3] = q_new[3] / len;
  }

  // 2. Rotate local velocity [twist_linear_x, 0, 0] to world frame using chassis quaternion
  float v_local[3] = {dozer->twist_linear_x, 0.0f, 0.0f};
  float v_world[3];
  quat_rotate(dozer->q, v_local, v_world);

  dozer->position_x += v_world[0] * dt;
  dozer->position_y += v_world[1] * dt;
  
  // 3. Keep Euler angles in sync for logging/rendering/coordinate projection
  float rpy[3];
  quat_to_euler(dozer->q, rpy);
  dozer->angular_x = rpy[0];
  dozer->angular_y = -rpy[1]; // pitch is negative for nose-up in right-handed coordinates
  dozer->angular_z = rpy[2];
}

static inline float calculate_FEE_column(SoilEnv* env, float hard_depth, float total_depth, float width)
{
  if (total_depth <= 0.0f) return 0.0f;
  Dozer* dozer = &env->dozer;
  float q = dozer->blade_surcharge_Q * (width / dozer->blade_width);
  float df = q * env->N_Q;

  if (hard_depth > 0.0f)
  {
    df += env->soil_gamma * hard_depth * hard_depth * width * env->N_gamma +
          env->soil_c * hard_depth * width * env->N_c +
          env->soil_c_a * hard_depth * width * env->N_ca;
  }
  return df;
}


static inline void interact_with_soil(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;

  float total_force = 0.0f;
  float total_yaw_moment = 0.0f;
  float total_roll_moment = 0.0f;

  float half_w = dozer->blade_width / 2.0f;

  // 1. Get blade orientation quaternion
  float q_blade[4];
  euler_to_quat(dozer->_blade_edge_pose[3], dozer->_blade_edge_pose[4], dozer->_blade_edge_pose[5], q_blade);

  // 2. Compute unit vectors of the blade frame in the world frame
  float x_axis_local[3] = {1.0f, 0.0f, 0.0f};
  float y_axis_local[3] = {0.0f, 1.0f, 0.0f};
  float x_axis_world[3];
  float y_axis_world[3];
  quat_rotate(q_blade, x_axis_local, x_axis_world);
  quat_rotate(q_blade, y_axis_local, y_axis_world);

  // Find start (left) and end (right) coordinates of the blade in meters
  float start_m_x = dozer->blade_x + y_axis_world[0] * half_w;
  float start_m_y = dozer->blade_y + y_axis_world[1] * half_w;
  float end_m_x = dozer->blade_x - y_axis_world[0] * half_w;
  float end_m_y = dozer->blade_y - y_axis_world[1] * half_w;

  // 3. Convert to grid indices
  int x0 = (int)floorf(start_m_x / CELL_SIZE);
  int y0 = (int)floorf(start_m_y / CELL_SIZE);
  int x1 = (int)floorf(end_m_x / CELL_SIZE);
  int y1 = (int)floorf(end_m_y / CELL_SIZE);

  int dx_i = (x1 > x0) ? (x1 - x0) : (x0 - x1);
  int dy_i = (y1 > y0) ? (y1 - y0) : (y0 - y1);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx_i - dy_i;

  // Direction of the blade's forward vector projected on the horizontal (X-Y) plane
  float x_denom = sqrtf(x_axis_world[0] * x_axis_world[0] + x_axis_world[1] * x_axis_world[1]);
  if (x_denom < 1e-6f) x_denom = 1e-6f;
  
  // Soil-push direction. Follow motion when it's clear; fall back to commanded effort when
  // nearly stopped; otherwise hold the previous direction (hysteresis) so we don't flip-flop
  // while creeping through zero velocity (e.g. coasting out of a backdrag). Stored on the dozer
  // so simulate_erosion() and update_surcharge() reuse the same direction this step.
  float push_sign = dozer->last_push_sign;
  if (dozer->twist_linear_x > 0.01f) {
      push_sign = 1.0f;
  } else if (dozer->twist_linear_x < -0.01f) {
      push_sign = -1.0f;
  } else if (dozer->effort_linear > 0.01f) {
      push_sign = 1.0f;
  } else if (dozer->effort_linear < -0.01f) {
      push_sign = -1.0f;
  }
  dozer->last_push_sign = push_sign;

  float fwd_dir_x = (x_axis_world[0] / x_denom) * push_sign;
  float fwd_dir_y = (x_axis_world[1] / x_denom) * push_sign;

  // Effective width of blade per intersected cell to conserve mass/force geometry.
  float effective_width = CELL_SIZE / fmaxf(fabsf(fwd_dir_x), fabsf(fwd_dir_y));

  // Arrays to store exactly which cells we visited and how much we cut
  int visited_x[100];
  int visited_y[100];
  float cut_vol[100];
  int num_cells = 0;

  while (1)
  {
    if ((unsigned int)x0 < GRID_SIZE && (unsigned int)y0 < GRID_SIZE)
    {
      // Calculate the moment arm (local_y) and blade elevation at this specific grid cell
      float cell_m_x = (x0 + 0.5f) * CELL_SIZE;
      float cell_m_y = (y0 + 0.5f) * CELL_SIZE;
      float dx = cell_m_x - dozer->blade_x;
      float dy = cell_m_y - dozer->blade_y;

      // Project the offset onto the 2D projected y_axis_world
      float denom = y_axis_world[0] * y_axis_world[0] + y_axis_world[1] * y_axis_world[1];
      if (denom < 1e-6f) denom = 1e-6f;
      float local_y = (dx * y_axis_world[0] + dy * y_axis_world[1]) / denom;

      // The exact 3D height of the blade at this lateral position
      float section_blade_z = dozer->blade_z + local_y * y_axis_world[2];

      // 3. Cut the soil (remove from heightmap, convert to loose) — track error incrementally
      float total_h = env->grid_H[x0][y0] + env->grid_L[x0][y0];
      float depth = total_h - section_blade_z;
      float cell_cut_vol = 0.0f;

      if (depth > 0.0f)
      {
        float old_c = cell_error_contrib(env, x0, y0);
        if (env->grid_L[x0][y0] > 0.0f)
        {
          float l_cut = (depth < env->grid_L[x0][y0]) ? depth : env->grid_L[x0][y0];
          env->grid_L[x0][y0] -= l_cut; 
          depth -= l_cut; 
          cell_cut_vol += l_cut * CELL_SIZE * CELL_SIZE;
        }
        if (depth > 0.0f)
        {
          env->grid_H[x0][y0] -= depth; 
          cell_cut_vol += depth * CELL_SIZE * CELL_SIZE * env->swell_ratio;
        }
        env->cur_error += cell_error_contrib(env, x0, y0) - old_c;
      }

      if (num_cells < 100)
      {
        visited_x[num_cells] = x0;
        visited_y[num_cells] = y0;
        cut_vol[num_cells] = cell_cut_vol;
        num_cells++;
      }

      // 4. Calculate FEE force on the unyielding soil 1 cell directly in front
      float front_x = cell_m_x + CELL_SIZE * fwd_dir_x;
      float front_y = cell_m_y + CELL_SIZE * fwd_dir_y;
      int f_grid_x = (int)floorf(front_x / CELL_SIZE);
      int f_grid_y = (int)floorf(front_y / CELL_SIZE);

      if ((unsigned int)f_grid_x < GRID_SIZE && (unsigned int)f_grid_y < GRID_SIZE)
      {
        float f_total_h = env->grid_H[f_grid_x][f_grid_y] + env->grid_L[f_grid_x][f_grid_y];
        float f_total_depth = f_total_h - section_blade_z;
        float f_hard_depth = env->grid_H[f_grid_x][f_grid_y] - section_blade_z;

        float df = calculate_FEE_column(env, f_hard_depth, f_total_depth, effective_width);
        total_force += df;
        total_yaw_moment += df * local_y * push_sign;
        total_roll_moment += df * local_y * push_sign;
      }
    }

    // Bresenham step
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= -dy_i) { err -= dy_i; x0 += sx; }
    if (e2 <= dx_i) { err += dx_i; y0 += sy; }
  }

  dozer->last_force = total_force;
  dozer->last_yaw_moment = total_yaw_moment;
  dozer->last_roll_moment = total_roll_moment;

  // 5. Add the cut volume to the loose soil layer directly in front of the specific cell it was cut from
  for (int s = 0; s < num_cells; s++)
  {
    if (cut_vol[s] > 0.0f)
    {
      float dh = cut_vol[s] / (CELL_SIZE * CELL_SIZE);
      int cx = visited_x[s];
      int cy = visited_y[s];

      // Push dirt forward 1 cell length in the direction of blade forward travel
      float dep_global_x = (cx + 0.5f) * CELL_SIZE + CELL_SIZE * fwd_dir_x;
      float dep_global_y = (cy + 0.5f) * CELL_SIZE + CELL_SIZE * fwd_dir_y;

      int dep_x = (int)floorf(dep_global_x / CELL_SIZE);
      int dep_y = (int)floorf(dep_global_y / CELL_SIZE);
      if ((unsigned int)dep_x < GRID_SIZE && (unsigned int)dep_y < GRID_SIZE)
      {
        float old_c = cell_error_contrib(env, dep_x, dep_y);
        env->grid_L[dep_x][dep_y] += dh;
        env->cur_error += cell_error_contrib(env, dep_x, dep_y) - old_c;
      }
    }
  }
}

static inline void update_joint_vel(SoilEnv* env, float dt)
{
  Dozer * dozer = &env->dozer;

  float LIFT_ARM_CG = 0.5f;
  float TOTAL_ARM_LEN = dozer->arm_length + cosf(dozer->pos_blade_pitch + 0.5f) * dozer->pitch_length;
  float arm_gravity_torque = (dozer->arm_mass + dozer->pitch_mass + dozer->blade_mass) * GRAVITY * TOTAL_ARM_LEN * LIFT_ARM_CG * cosf(dozer->pos_virtual_lift_arm);
  // Horizontal soil reaction reverses with travel direction (last_push_sign): its generalized torque
  // on the arm flips between forward pushing and backdragging. last_force is an unsigned magnitude.
  float arm_resist_torque = sinf(dozer->pos_virtual_lift_arm) * dozer->last_force * TOTAL_ARM_LEN * dozer->last_push_sign;
  float arm_total_extern_torque = -arm_gravity_torque - arm_resist_torque;

  if (dozer->effort_lift * arm_total_extern_torque <= 0)
  {
    arm_total_extern_torque *= (1.0f - dozer->hydraulic_stiffness);
  }

  dozer->vel_virtual_lift_arm = (dozer->vel_virtual_lift_arm + ((dozer->effort_lift * dozer->max_torque_lift_arm + arm_total_extern_torque) / dozer->arm_inertia) * dt) / (1.0f + (dozer->virtual_lift_arm_damping / dozer->arm_inertia) * dt);

  float PITCH_LINK_CG = 0.75f;
  float pitch_gravity_torque = (dozer->pitch_mass + dozer->blade_mass) * GRAVITY * dozer->pitch_length * PITCH_LINK_CG * cosf(dozer->pos_virtual_lift_arm + dozer->pos_blade_pitch);
  float pitch_resist_torque = sinf(dozer->pos_virtual_lift_arm + dozer->pos_blade_pitch) * dozer->last_force * dozer->pitch_length * dozer->last_push_sign;
  float pitch_total_extern_torque = -pitch_gravity_torque - pitch_resist_torque;
  if (dozer->effort_pitch * pitch_total_extern_torque <= 0)
  {
    pitch_total_extern_torque *= (1.0f - dozer->hydraulic_stiffness); 
  }

  dozer->vel_blade_pitch = (dozer->vel_blade_pitch + ((dozer->effort_pitch * dozer->max_torque_pitch + pitch_total_extern_torque) / dozer->pitch_intertia) * dt) / (1.0f + (dozer->blade_pitch_damping / dozer->pitch_intertia) * dt);

  float roll_resist_torque = dozer->last_roll_moment; 
  float roll_total_extern_torque = -roll_resist_torque;
  if (dozer->effort_roll * roll_total_extern_torque <= 0)
  {
    roll_total_extern_torque *= (1.0f - dozer->hydraulic_stiffness);
  }
  dozer->vel_blade_roll = (dozer->vel_blade_roll + ((dozer->effort_roll * dozer->max_torque_roll + roll_total_extern_torque) / dozer->roll_inertia) * dt) / (1.0f + (dozer->blade_roll_damping / dozer->roll_inertia) * dt);
}

static inline void update_joint_pos(SoilEnv* env, float dt)
{
  Dozer * dozer = &env->dozer;

  dozer->pos_virtual_lift_arm += dozer->vel_virtual_lift_arm * dt;
  if (dozer->pos_virtual_lift_arm < dozer->pos_virtual_lift_arm_min) dozer->pos_virtual_lift_arm = dozer->pos_virtual_lift_arm_min;
  if (dozer->pos_virtual_lift_arm > dozer->pos_virtual_lift_arm_max) dozer->pos_virtual_lift_arm = dozer->pos_virtual_lift_arm_max;

  dozer->pos_blade_pitch += dozer->vel_blade_pitch * dt;
  if (dozer->pos_blade_pitch < dozer->pos_blade_pitch_min) dozer->pos_blade_pitch = dozer->pos_blade_pitch_min;
  if (dozer->pos_blade_pitch > dozer->pos_blade_pitch_max) dozer->pos_blade_pitch = dozer->pos_blade_pitch_max;

  dozer->pos_blade_roll += dozer->vel_blade_roll * dt;
  if (dozer->pos_blade_roll < dozer->pos_blade_roll_min) dozer->pos_blade_roll = dozer->pos_blade_roll_min;
  if (dozer->pos_blade_roll > dozer->pos_blade_roll_max) dozer->pos_blade_roll = dozer->pos_blade_roll_max;
}

static inline void simulate_erosion(SoilEnv* env, const int num_loops)
{
  Dozer* dozer = &env->dozer;

  int margin = EROSION_MARGIN;  // number of cells we buffer about the blade (~4.0m at CELL_SIZE=0.2)
  int center_i = (int)floorf(dozer->blade_x / CELL_SIZE);  // get the ith cell location of the blade
  int min_i = clamp_idx(center_i - margin);  // use our buffer to find the min ith cell
  int max_i = clamp_idx(center_i + margin);  // same but for max
  int center_j = (int)floorf(dozer->blade_y / CELL_SIZE);  // repeat last three but for jth cells
  int min_j = clamp_idx(center_j - margin);
  int max_j = clamp_idx(center_j + margin);

  // Local scratch buffers are indexed relative to the clamped window origin (base_i=min_i, base_j=min_j),
  // so the [min,max] range always fits in [0, EROSION_WIN) even when blade is off-map (center far outside).
  int base_i = min_i;
  int base_j = min_j;

  float loader_length = dozer->track_length; // TODO: determine which is better here
  float tan_phi = tanf(env->soil_phi);
  float half_bw = dozer->blade_width * 0.5f;
  float cos_y = cosf(dozer->angular_z);
  float sin_y = sinf(dozer->angular_z);
  float push_sign = dozer->last_push_sign;  // set in interact_with_soil() earlier this step

  float temp_L[EROSION_WIN][EROSION_WIN];   // loose-soil snapshot for the blade window
  float temp_T[EROSION_WIN][EROSION_WIN];   // total-height snapshot (grid_H + loose); avoids re-reading grid_H
  float delta_L[EROSION_WIN][EROSION_WIN];  // accumulates symmetric slumping changes
  char  frozen[EROSION_WIN][EROSION_WIN];   // cells the machine sits on that must not slump

  memset(frozen, 0, sizeof(frozen));

  float min_lx = (push_sign > 0.0f) ? -loader_length : 0.0f;
  float max_lx = (push_sign > 0.0f) ? 0.0f : loader_length;
  
  float cx[4] = {min_lx, max_lx, max_lx, min_lx};
  float cy[4] = {-half_bw, -half_bw, half_bw, half_bw};
  
  int frozen_min_i = max_i;
  int frozen_max_i = min_i;
  int frozen_min_j = max_j;
  int frozen_max_j = min_j;

  for (int c = 0; c < 4; c++) {
    float wx = dozer->blade_x + cx[c] * cos_y - cy[c] * sin_y;
    float wy = dozer->blade_y + cx[c] * sin_y + cy[c] * cos_y;
    int gi = (int)floorf(wx / CELL_SIZE);
    int gj = (int)floorf(wy / CELL_SIZE);
    if (gi < frozen_min_i) frozen_min_i = gi;
    if (gi > frozen_max_i) frozen_max_i = gi;
    if (gj < frozen_min_j) frozen_min_j = gj;
    if (gj > frozen_max_j) frozen_max_j = gj;
  }
  
  frozen_min_i = clamp_idx(frozen_min_i - 1);
  frozen_max_i = clamp_idx(frozen_max_i + 1);
  frozen_min_j = clamp_idx(frozen_min_j - 1);
  frozen_max_j = clamp_idx(frozen_max_j + 1);

  if (frozen_min_i < min_i) frozen_min_i = min_i;
  if (frozen_max_i > max_i) frozen_max_i = max_i;
  if (frozen_min_j < min_j) frozen_min_j = min_j;
  if (frozen_max_j > max_j) frozen_max_j = max_j;

  for (int i = frozen_min_i; i <= frozen_max_i; i++)
  {
    for (int j = frozen_min_j; j <= frozen_max_j; j++)
    {
      float dx_cell = (i + 0.5f) * CELL_SIZE - dozer->blade_x;
      float dy_cell = (j + 0.5f) * CELL_SIZE - dozer->blade_y;
      float local_x = dx_cell * cos_y + dy_cell * sin_y;
      float local_y = -dx_cell * sin_y + dy_cell * cos_y;
      float lx = local_x * push_sign;
      if (fabsf(local_y) <= half_bw && lx >= -loader_length && lx <= 0.0f)
      {
        frozen[i - base_i][j - base_j] = 1;
      }
    }
  }

  static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};  // 8 surrounding pixel locations
  static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  static const float dists[8] = {
    CELL_SIZE, CELL_SIZE, CELL_SIZE, CELL_SIZE,
    CELL_SIZE * 1.41421356f, CELL_SIZE * 1.41421356f, CELL_SIZE * 1.41421356f, CELL_SIZE * 1.41421356f
  };

  int scan_min_i = min_i, scan_max_i = max_i;
  int scan_min_j = min_j, scan_max_j = max_j;

  for (int iter = 0; iter < num_loops; iter++)  // loop 3 times
  {
    int active_min_i = max_i;
    int active_max_i = min_i;
    int active_min_j = max_j;
    int active_max_j = min_j;
    int any_loose = 0;

    for (int i = scan_min_i; i <= scan_max_i; i++)  // snapshot loose + total height within the scan area
    {
      for (int j = scan_min_j; j <= scan_max_j; j++)
      {
        int li = i - base_i, lj = j - base_j;
        float L = env->grid_L[i][j];
        temp_L[li][lj] = L;
        temp_T[li][lj] = env->grid_H[i][j] + L;
        delta_L[li][lj] = 0.0f;
        if (L > 1e-4f) {
           any_loose = 1;
           if (i < active_min_i) active_min_i = i;
           if (i > active_max_i) active_max_i = i;
           if (j < active_min_j) active_min_j = j;
           if (j > active_max_j) active_max_j = j;
        }
      }
    }

    if (!any_loose) break;  // nothing to slump, and nothing will slump in future iterations

    // Slump only from active cells
    int slump_min_i = (active_min_i > min_i) ? active_min_i : min_i + 1;
    int slump_max_i = (active_max_i < max_i) ? active_max_i : max_i - 1;
    int slump_min_j = (active_min_j > min_j) ? active_min_j : min_j + 1;
    int slump_max_j = (active_max_j < max_j) ? active_max_j : max_j - 1;

    for (int i = slump_min_i; i <= slump_max_i; i++)
    {
      for (int j = slump_min_j; j <= slump_max_j; j++)
      {
        int li = i - base_i, lj = j - base_j;
        float L_ij = temp_L[li][lj];
        if (L_ij <= 1e-4f) continue;  // if loose soil at this pixel is 0, skip
        if (frozen[li][lj]) continue;  // don't let soil slump *from* the machine footprint
        float total_h = temp_T[li][lj];  // get total height at this pixel

        float L_val = L_ij;
        if (L_val < 1e-5f) L_val = 1e-5f;
        float K = env->soil_c / (env->soil_gamma * L_val);
        float t_limit = tan_phi;

        if (K >= 1e-5f)
        {
          float disc = 1.0f - 4.0f * K * (tan_phi + K);
          if (disc > 0.0f)
          {
            t_limit = (1.0f - sqrtf(disc)) / (2.0f * K);
          }
          else
          {
            t_limit = 1e9f;  // basically infinity
          }
        }

        for (int d = 0; d < 8; d++)  // look through the 8 surrounding pixels
        {
          int lni = li + dx[d], lnj = lj + dy[d];
          if (frozen[lni][lnj]) continue;  // don't let soil slump into the machine footprint

          float dH = total_h - temp_T[lni][lnj];
          if (dH <= 0.0f) continue;  // only shed to lower neighbors

          float dist = dists[d];
          float t = dH / dist;

          if (t > t_limit)
          {
            float slip = (t - t_limit) * dist * 0.2f;
            // Limit slip to avoid over-drawing from a single cell if multiple neighbors demand soil
            if (slip > L_ij / 8.0f) slip = L_ij / 8.0f;
            delta_L[li][lj] -= slip;
            delta_L[lni][lnj] += slip;
            // We DO NOT update total_h or temp_L here to maintain symmetry
          }
        }
      }
    }

    // Apply delta_L only in the region that could have been modified (active bounds + 1)
    int apply_min_i = (active_min_i > min_i) ? active_min_i - 1 : min_i;
    int apply_max_i = (active_max_i < max_i) ? active_max_i + 1 : max_i;
    int apply_min_j = (active_min_j > min_j) ? active_min_j - 1 : min_j;
    int apply_max_j = (active_max_j < max_j) ? active_max_j + 1 : max_j;

    for (int i = apply_min_i; i <= apply_max_i; i++)
    {
      for (int j = apply_min_j; j <= apply_max_j; j++)
      {
        float d = delta_L[i - base_i][j - base_j];
        if (fabsf(d) < 1e-9f) continue;
        float old_c = cell_error_contrib(env, i, j);
        env->grid_L[i][j] += d;
        if (env->grid_L[i][j] < 0.0f) env->grid_L[i][j] = 0.0f;
        env->cur_error += cell_error_contrib(env, i, j) - old_c;
      }
    }

    // Shrink the scan window for the next iteration to the apply window.
    scan_min_i = apply_min_i;
    scan_max_i = apply_max_i;
    scan_min_j = apply_min_j;
    scan_max_j = apply_max_j;
  }
}

static inline void update_surcharge(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;

  // Create a local bounding box to search for loose soil (1.5m forward, plus width of blade)
  float search_radius = 2.0f; // rough max extent
  int center_i = (int)floorf(dozer->blade_x / CELL_SIZE);
  int center_j = (int)floorf(dozer->blade_y / CELL_SIZE);
  int margin = (int)(search_radius / CELL_SIZE) + 1;

  int min_i = clamp_idx(center_i - margin);
  int max_i = clamp_idx(center_i + margin);
  int min_j = clamp_idx(center_j - margin);
  int max_j = clamp_idx(center_j + margin);

  float current_surcharge_vol = 0.0f;

  float q_blade[4];
  euler_to_quat(dozer->_blade_edge_pose[3], dozer->_blade_edge_pose[4], dozer->_blade_edge_pose[5], q_blade);

  float x_axis_local[3] = {1.0f, 0.0f, 0.0f};
  float y_axis_local[3] = {0.0f, 1.0f, 0.0f};
  float x_axis_world[3];
  float y_axis_world[3];
  quat_rotate(q_blade, x_axis_local, x_axis_world);
  quat_rotate(q_blade, y_axis_local, y_axis_world);

  // Normalize x_axis_world and y_axis_world on the XY plane for horizontal distance checks
  float x_len = sqrtf(x_axis_world[0]*x_axis_world[0] + x_axis_world[1]*x_axis_world[1]);
  float y_len = sqrtf(y_axis_world[0]*y_axis_world[0] + y_axis_world[1]*y_axis_world[1]);
  if (x_len < 1e-6f) x_len = 1e-6f;
  if (y_len < 1e-6f) y_len = 1e-6f;
  
  float fwd_x = x_axis_world[0] / x_len;
  float fwd_y = x_axis_world[1] / x_len;
  float left_x = y_axis_world[0] / y_len;
  float left_y = y_axis_world[1] / y_len;

  float half_width = dozer->blade_width / 2.0f;
  float lookahead = 1.5f;
  float cell_area = CELL_SIZE * CELL_SIZE;
  float push_sign = dozer->last_push_sign;  // set in interact_with_soil() earlier this step

  for (int i = min_i; i <= max_i; i++)
  {
    for (int j = min_j; j <= max_j; j++)
    {
      if (env->grid_L[i][j] <= 0.0f) continue;

      float cell_x = (i + 0.5f) * CELL_SIZE;
      float cell_y = (j + 0.5f) * CELL_SIZE;

      float dx = cell_x - dozer->blade_x;
      float dy = cell_y - dozer->blade_y;

      // Transform cell coordinates into the blade's local horizontal frame.
      // Scale local_x by push_sign so the window follows the working direction: it captures the
      // pile in front when pushing forward, and the dragged soil behind the blade when backdragging.
      float local_x = (dx * fwd_x + dy * fwd_y) * push_sign;
      float local_y = dx * left_x + dy * left_y;

      // Check if the cell is inside the 1.5m box on the working side of the blade
      if (local_x >= 0.0f && local_x <= lookahead && fabsf(local_y) <= half_width)
      {
        current_surcharge_vol += env->grid_L[i][j] * cell_area;
      }
    }
  }

  // Calculate weight of the soil (Volume * Density * Gravity)
  dozer->blade_surcharge_Q = current_surcharge_vol * env->loose_soil_density * GRAVITY;
}

static inline void simulate_step(SoilEnv* env, float dt)
{
  // 1. Calculate Track Forces, Slip, and update Chassis 2D Velocity (X, Y, Yaw)
  update_chassis_velocity(env, dt);

  // 2. Update Chassis 2D Position based on velocities
  update_chassis_2d_position(env, dt);

  // 3. Project Chassis onto 3D Terrain and run compaction
  update_kinematics(env);  

  // 4. Actuator Dynamics (Arm, Pitch, Roll)
  update_joint_vel(env, dt);
  update_joint_pos(env, dt);

  // 5. Update Blade global 3D coordinates based on new joint positions
  forward_kinematics(env);

  // 6. Soil Interaction (Calculate reactive forces for next step)
  interact_with_soil(env);

  // 7. Simulate Soil Erosion (Slumping)
  simulate_erosion(env, 3);

  // 8. Update Blade Surcharge
  update_surcharge(env);

  // precompute soil forces based on the updated blade rake angle
  precompute_FEE(env, 0.0f); // TODO: ALPHA argument is currently 0.0f

  env->step_num++;
}

// Build the target heightmap: the starting terrain, minus a cut "slot" and plus a "pile" made of the
// soil that slot yields. Volume is kept (roughly) consistent across the compact->loose swell, i.e.
// loose pile volume = swell_ratio * compact slot volume. The pile is a triangular ridge spanning one
// blade width at the far end of the slot (~1 m tall); the slot is 2.5-5 m long with varied depth.
static inline void generate_goal_map(SoilEnv* env)
{
  Dozer* dozer = &env->dozer;

  // Goal starts equal to the current terrain everywhere; region defaults to neutral.
  for (int i = 0; i < GRID_SIZE; i++)
  {
    for (int j = 0; j < GRID_SIZE; j++)
    {
      env->grid_G[i][j] = env->grid_H[i][j] + env->grid_L[i][j];
      env->map_region[i][j] = 0; // 0 = Neutral
    }
  }

  float slot_width = dozer->blade_width;            // slot as wide as blade
  float pile_width = dozer->blade_width * 1.75f;      // pile ~1.75× wider for slumping
  float half_slot_w = slot_width * 0.5f;
  float half_pile_w = pile_width * 0.5f;

  // Randomized geometry (per-episode)
  float slot_len    = 5.5f + rand_f(&env->rng) * 1.5f;   // [5.5, 7.0] m
  float slot_depth  = 0.15f + rand_f(&env->rng) * 0.10f; // [0.15, 0.25] m
  float pile_height = 0.8f + rand_f(&env->rng) * 0.4f;   // [0.8, 1.2] m (~1 m)

  // Volume balance: loose pile = swell_ratio * compacted cut (slot width vs pile width)
  float cut_volume  = slot_len * slot_width * slot_depth;
  float pile_volume = env->swell_ratio * cut_volume;

  // Gaussian pile across pile_width: V = ∫h dA  ->  base length from volume
  float pile_len = (3.0f * pile_volume) / (pile_height * pile_width);

  // Random heading; the slot mouth begins at the dozer and runs outward, pile at the far end.
  // float theta = rand_f(&env->rng) * 2.0f * PI;
  float theta = 0;
  float dir_x = cosf(theta),  dir_y = sinf(theta);
  float perp_x = -dir_y,      perp_y = dir_x;
  float start_x = dozer->position_x + 2.0f;
  float start_y = dozer->position_y;

  float total_len = slot_len + pile_len;
  float half_pile = pile_len * 0.5f;

  // Gaussian pile: bell-shaped mound with ~30° repose, volume-matched to cut*swell
  // Two-pass to preserve exact volume: first pass sum raw Gaussian, second apply scaled heights
  float sigma_p = pile_len * 0.25f; // along-pile std (≈ pile_len/4 gives ~30° side slope)
  float sigma_v = half_pile_w * 0.5f;    // across-pile std (≈ pile_width/4)
  if (sigma_p < 0.2f) sigma_p = 0.2f;
  if (sigma_v < 0.2f) sigma_v = 0.2f;
  float raw_vol = 0.0f;
  for (int i = 0; i < GRID_SIZE; i++)
  {
    for (int j = 0; j < GRID_SIZE; j++)
    {
      float cell_x = (i + 0.5f) * CELL_SIZE;
      float cell_y = (j + 0.5f) * CELL_SIZE;
      float rx = cell_x - start_x;
      float ry = cell_y - start_y;
      float u = rx * dir_x + ry * dir_y;
      float v = rx * perp_x + ry * perp_y;
      if (fabsf(v) > half_pile_w) continue;
      if (u < 0.0f || u > total_len) continue;
      if (u <= slot_len) continue; // slot not part of pile volume
      float p = u - slot_len;
      float dp = p - half_pile;
      float h_raw = expf(-0.5f * ((dp*dp)/(sigma_p*sigma_p) + (v*v)/(sigma_v*sigma_v)));
      raw_vol += h_raw * CELL_SIZE * CELL_SIZE;
    }
  }
  float pile_scale = 1.0f;
  if (raw_vol > 1e-6f) pile_scale = pile_volume / raw_vol;

  for (int i = 0; i < GRID_SIZE; i++)
  {
    for (int j = 0; j < GRID_SIZE; j++)
    {
      float cell_x = (i + 0.5f) * CELL_SIZE;
      float cell_y = (j + 0.5f) * CELL_SIZE;
      float rx = cell_x - start_x;
      float ry = cell_y - start_y;

      float u = rx * dir_x + ry * dir_y;    // along the slot->pile axis
      float v = rx * perp_x + ry * perp_y;  // across the width

      if (u < 0.0f || u > total_len) continue;

      if (u <= slot_len)
      {
        if (fabsf(v) > half_slot_w) continue; // slot width = blade width
        env->grid_G[i][j] -= slot_depth;    // dig the slot
        env->map_region[i][j] = 1;          // 1 = Cut
      }
      else
      {
        if (fabsf(v) > half_pile_w) continue; // pile ~1.75× wider for slumping
        float p = u - slot_len;             // 0..pile_len within the pile
        float dp = p - half_pile;
        float h_raw = expf(-0.5f * ((dp*dp)/(sigma_p*sigma_p) + (v*v)/(sigma_v*sigma_v)));
        float h = h_raw * pile_scale;
        if (h < 0.0f) h = 0.0f;
        env->grid_G[i][j] += h;             // heap the Gaussian pile
        env->map_region[i][j] = 2;          // 2 = Fill
      }

      if (env->grid_G[i][j] < 0.0f) env->grid_G[i][j] = 0.0f;
    }
  }
}

static inline void env_reset(SoilEnv* env)
{
  env->loose_soil_density = 1200.0f;
  env->soil_gamma = 15000.0f;
  env->soil_c = 300.0f;   // (Pa) soil cohesion
  env->soil_c_a = 0.0f;   // (Pa) soil-blade adhesion
  env->soil_phi = 45.0f * M_PI / 180.0f;
  env->soil_delta = 10.0f * M_PI / 180.0f;
  env->swell_ratio = 1.2f;

  // rewards
  // env->log.total_reward = 0;
  env->log.r_shaping = 0;
  env->log.r_off_map = 0;
  env->log.r_time = 0;

  Dozer* dozer = &env->dozer;

  // dimensions | m
  dozer->track_length = 1.7112f;
  dozer->track_width = 0.4572f; 
  dozer->track_gauge = 1.5494f;
  dozer->blade_width = 1.85f;
  dozer->blade_height = 0.76f;
  dozer->blade_rake_angle = 30.0f * M_PI / 180.0f;
  dozer->blade_mount_pitch = 0.0f;
  dozer->arm_length = 3.8f;
  dozer->arm_pivot_x = -2.2f;
  dozer->arm_pivot_z = 1.8987f;
  dozer->pitch_length = 0.83f;

  // max track force | N (single hydraulic thrust budget per track; linear & yaw both derive from this)
  dozer->max_force_track = 12500.0f;

  // max torques | Nm
  dozer->max_torque_pitch = 5000.0f;
  dozer->max_torque_roll = 2500.0f;
  dozer->max_torque_lift_arm = 9000.0f;

  // masses | kg
  dozer->machine_mass = 5200.0f;
  dozer->arm_mass = 490.0f;
  dozer->pitch_mass = 200.0f;
  dozer->blade_mass = 635.0f;

  // inertia
  dozer->machine_inertia = 5071.0f;
  dozer->arm_inertia = 5000.0f;
  dozer->roll_inertia = 80.0f;
  dozer->pitch_intertia = 19.0f;

  // damping
  dozer->hydraulic_stiffness = 0.9998f;
  dozer->track_damping = 3.0f; // (30000 / 8570)
  dozer->virtual_lift_arm_damping = 30000.0f;
  dozer->blade_pitch_damping = 5000.0f;
  dozer->blade_roll_damping = 5000.0f;

  // limits | rad
  dozer->pos_virtual_lift_arm_min = -0.5f;
  dozer->pos_virtual_lift_arm_max = 0.5f;
  dozer->pos_blade_pitch_min = -0.5f;
  dozer->pos_blade_pitch_max = 0.5f;
  dozer->pos_blade_roll_min = -0.5f;
  dozer->pos_blade_roll_max = 0.5f;

  dozer->effort_lift = 0.0f;        // efort
  dozer->effort_pitch = 0.0f;       // efort
  dozer->effort_roll = 0.0f;        // efort
  dozer->effort_yaw = 0.0f;         // efort
  dozer->effort_linear = 0.0f;      // efort
  dozer->effort_rotational = 0.0f;  // efort

  // Joint States POS — start with blade slightly above ground plane (~0.1-0.2m)
  dozer->pos_tracks_rotational = 0.0f;
  dozer->pos_tracks_linear = 0.0f;
  dozer->pos_virtual_lift_arm = -0.43f;  // (rad) arm angle
  dozer->pos_blade_pitch = 0.5f;        // (rad) blade pitch
  dozer->pos_blade_roll = 0.0f;        // (rad) blade roll
  dozer->pos_blade_yaw = 0.0f;         // (rad) blade yaw

  // Joint States VEL
  dozer->vel_tracks_rotational = 0.0f;  // (rad/s) tracks rotational velocity
  dozer->vel_tracks_linear = 0.0f;      // (m/s)   tracks linear velocity
  dozer->vel_virtual_lift_arm = 0.0f;   // Current arm angular velocity (rad/s)
  dozer->vel_blade_pitch = 0.0f;        // Current relative pitch velocity (rad/s)
  dozer->vel_blade_roll = 0.0f;         // Current relative roll velocity (rad/s)
  dozer->vel_blade_yaw = 0.0f;          // Current relative yaw velocity (rad/s)

  dozer->position_x = (GRID_SIZE * CELL_SIZE) / 2.0f - 10.0f;
  dozer->position_y = (GRID_SIZE * CELL_SIZE) / 2.0f;
  dozer->position_z = 1.0f;

  dozer->q[0] = 1.0f;
  dozer->q[1] = 0.0f;
  dozer->q[2] = 0.0f;
  dozer->q[3] = 0.0f;

  dozer->last_push_sign = 1.0f;  // default to forward until motion/effort says otherwise

  for(int i = 0; i < GRID_SIZE; i++) {
    for(int j = 0; j < GRID_SIZE; j++) {
      env->grid_H[i][j] = 1.0f;
      env->grid_L[i][j] = 0.0f;
      env->original_H[i][j] = env->grid_H[i][j] + env->grid_L[i][j];  // snapshot of starting terrain
    }
  }

  generate_goal_map(env);

  precompute_soil_bearing_capacity(env);

  // reward bookkeeping — init from terrain error (volume error over cut/fill zone)
  env->initial_error = compute_terrain_error(env);
  env->cur_error = env->initial_error;
  env->prev_error = 0.0f;
  env->prev_progress = 0.0f;
  env->episode_return = 0.0f;
  env->count_off_map = 0.0f;
  env->count_jitter = 0.0f;
  env->count_large_neg_rewards = 0.0f;
  memset(&env->log, 0, sizeof(Log));
  env->log.perf = 0.0f;
  env->log.n = 0.0f;
  env->log.max_vel_arm = -1e9f;
  env->log.max_vel_blade_pitch = -1e9f;
  // env->log.max_vel_blade_roll = -1e9f;
  env->log.max_vel_linear = -1e9f;
  env->log.min_vel_arm = 1e9f;
  env->log.min_vel_blade_pitch = 1e9f;
  // env->log.min_vel_blade_roll = 1e9f;
  env->log.min_vel_linear = 1e9f;
}

void c_reset(SoilEnv* env)
{
  env->tick = 0;
  env->step_num = 0;
  memset(&env->dozer, 0, sizeof(Dozer));
  env_reset(env);
}

// Required function
void c_step(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;
  env->tick +=1;

  // get inputs
  // Continuous acitons: Clamp to [-1, 1] and then threshold
  dozer->effort_linear     = clamp_action(env->actions[0]);
  dozer->effort_rotational = clamp_action(env->actions[1]);
  dozer->effort_lift       = clamp_action(env->actions[2]);
  dozer->effort_pitch      = clamp_action(env->actions[3]);
  // dozer->effort_roll       = clamp_action(env->actions[4]);
  dozer->effort_roll       = 0;
  dozer->effort_yaw        = 0;  // this should just get zero'd (since we don't have control over this)

  env->terminals[0] = 0;  // zero these guys just in case
  env->rewards[0]   = 0;  // zero these guys just in case

  // Single physics step per control action (no top-level sub-stepping for now).
  // Soil erosion still sub-loops internally (see simulate_erosion, num_loops=3).
  // Revisit if the main loop proves unstable once we run/test the sim.
  simulate_step(env, 0.0167);

  // get observations (rewards and terminals also seen here, since we're already doing some loops!)
  get_obs(env);

  // reward: init-relative volume error over cut/fill zone [55,299]
  update_reward_and_terminal(env);
}

#include "raylib_render.h"

static inline void forward_kinematics_visual(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;

  float theta_arm = dozer->pos_virtual_lift_arm;
  float theta_pitch = dozer->pos_blade_pitch;

  // 1. Lift arm joint (local in chassis frame)
  float q_arm_local[4];
  euler_to_quat(0.0f, -theta_arm, 0.0f, q_arm_local);
  float lift_arm_local_pos[3] = {dozer->arm_pivot_x, 0.0f, dozer->arm_pivot_z};

  // 2. Pitch joint (local in chassis frame)
  float arm_vector[3] = {dozer->arm_length, 0.0f, 0.0f};
  float arm_offset[3];
  quat_rotate(q_arm_local, arm_vector, arm_offset);

  float pitch_joint_local_pos[3] = {
    lift_arm_local_pos[0] + arm_offset[0],
    lift_arm_local_pos[1] + arm_offset[1],
    lift_arm_local_pos[2] + arm_offset[2]
  };

  float q_pitch_rel[4];
  euler_to_quat(0.0f, -theta_pitch, 0.0f, q_pitch_rel);
  float q_pitch_local[4];
  quat_multiply(q_arm_local, q_pitch_rel, q_pitch_local);

  // 3. Universal joint (local in chassis frame)
  float pitch_vector[3] = {dozer->pitch_length, 0.0f, 0.0f};
  float pitch_offset[3];
  quat_rotate(q_pitch_local, pitch_vector, pitch_offset);

  float u_joint_local_pos[3] = {
    pitch_joint_local_pos[0] + pitch_offset[0],
    pitch_joint_local_pos[1] + pitch_offset[1],
    pitch_joint_local_pos[2] + pitch_offset[2]
  };

  float q_u_joint_rel[4];
  euler_to_quat(dozer->pos_blade_roll, 0.0f, dozer->pos_blade_yaw, q_u_joint_rel);
  float q_u_joint_local[4];
  quat_multiply(q_pitch_local, q_u_joint_rel, q_u_joint_local);

  // Transform intermediate joints from chassis local frame to world frame using dozer->q
  float chassis_pos[3] = {dozer->position_x, dozer->position_y, dozer->position_z};

  // Lift arm joint world pose
  float lift_arm_world_pos[3];
  quat_rotate(dozer->q, lift_arm_local_pos, lift_arm_world_pos);
  dozer->_lift_arm_joint_pose[0] = lift_arm_world_pos[0] + chassis_pos[0];
  dozer->_lift_arm_joint_pose[1] = lift_arm_world_pos[1] + chassis_pos[1];
  dozer->_lift_arm_joint_pose[2] = lift_arm_world_pos[2] + chassis_pos[2];

  float q_arm_world[4];
  quat_multiply(dozer->q, q_arm_local, q_arm_world);
  float rpy_arm[3];
  quat_to_euler(q_arm_world, rpy_arm);
  dozer->_lift_arm_joint_pose[3] = rpy_arm[0];
  dozer->_lift_arm_joint_pose[4] = rpy_arm[1];
  dozer->_lift_arm_joint_pose[5] = rpy_arm[2];

  // Pitch joint world pose
  float pitch_joint_world_pos[3];
  quat_rotate(dozer->q, pitch_joint_local_pos, pitch_joint_world_pos);
  dozer->_pitch_joint_pose[0] = pitch_joint_world_pos[0] + chassis_pos[0];
  dozer->_pitch_joint_pose[1] = pitch_joint_world_pos[1] + chassis_pos[1];
  dozer->_pitch_joint_pose[2] = pitch_joint_world_pos[2] + chassis_pos[2];

  float q_pitch_world[4];
  quat_multiply(dozer->q, q_pitch_local, q_pitch_world);
  float rpy_pitch[3];
  quat_to_euler(q_pitch_world, rpy_pitch);
  dozer->_pitch_joint_pose[3] = rpy_pitch[0];
  dozer->_pitch_joint_pose[4] = rpy_pitch[1];
  dozer->_pitch_joint_pose[5] = rpy_pitch[2];

  // U-joint world pose
  float u_joint_world_pos[3];
  quat_rotate(dozer->q, u_joint_local_pos, u_joint_world_pos);
  dozer->_u_joint_pose[0] = u_joint_world_pos[0] + chassis_pos[0];
  dozer->_u_joint_pose[1] = u_joint_world_pos[1] + chassis_pos[1];
  dozer->_u_joint_pose[2] = u_joint_world_pos[2] + chassis_pos[2];

  float q_u_joint_world[4];
  quat_multiply(dozer->q, q_u_joint_local, q_u_joint_world);
  float rpy_u_joint[3];
  quat_to_euler(q_u_joint_world, rpy_u_joint);
  dozer->_u_joint_pose[3] = rpy_u_joint[0];
  dozer->_u_joint_pose[4] = rpy_u_joint[1];
  dozer->_u_joint_pose[5] = rpy_u_joint[2];
}

void c_render(SoilEnv* env)
{
  if (!IsWindowReady())
  {
    init_render();
  }

  if (IsKeyDown(KEY_ESCAPE))
  {
    exit(0);
  }

  forward_kinematics_visual(env);
  render_step(env);
}

void c_close(SoilEnv* env)
{
  if (IsWindowReady())
  {
    close_render();
  }
}

#endif