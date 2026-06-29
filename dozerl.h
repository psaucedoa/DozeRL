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

  // force limits
  float max_force_linear;
  float max_force_rotational;

  // Dynamics
  float last_force;       // (N) Previous Reaction Force?
  float last_yaw_moment;  // (N*m)
  float last_roll_moment; // (N*m)

  // Actuator Dynamics Constants
  float max_torque_list_arm;
  float max_torque_pitch;
  float max_torque_roll;
  float max_torque_rotational;
  float max_force_track;  // this or max_force_linear? Probably this, just determine max force per track, and calc stuff by summing track forces?
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

  // get observations. First [0-9] are proprioceptive, rest are map (2x250)
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
  int obs_offset = 10;  // for index mapping later on
  // onto map observations
  // first we compute cos & sin for the vehicle's current yaw
  float cos_y = cosf(dozer->angular_z);
  float sin_y = sinf(dozer->angular_z);

  // next we get the dozer's cell position and shift by half the obs grid size to get to the starting corner
  float half_obs_grid = SPATIAL_OBS_SIZE * 0.5f;
  float obs_start_x = (dozer->position_x / CELL_SIZE) - half_obs_grid * cos_y + half_obs_grid * sin_y;
  float obs_start_y = (dozer->position_x / CELL_SIZE) - half_obs_grid * sin_y - half_obs_grid * cos_y;

  // then we iterate to get all cells in the obs space
  for (int i = 0; i < SPATIAL_OBS_SIZE; i++)
  {
    float row_grid_x = obs_start_x + i * cos_y;
    float row_grid_y = obs_start_y + i * sin_y;

    for (int j = 0; j < SPATIAL_OBS_SIZE; j++)
    {
      int grid_i = (int)(row_grid_x - j * sin_y);
      int grid_j = (int)(row_grid_y + j * cos_y);

      // here we do some index mapping from 2d grid coords -> 1d obs array shape
      int h_obs_index = (i * SPATIAL_OBS_SIZE) + j + obs_offset;
      int g_obs_index = h_obs_index + (SPATIAL_OBS_SIZE * SPATIAL_OBS_SIZE);  // we offset this by the amount of cells in the obs to concatenate to 1d array

      // if the selected env grid is within bounds of the sim region, grab the underlying map data
      if (grid_i >= 0 && grid_i < GRID_SIZE && grid_j >= 0 && grid_j < GRID_SIZE)
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

static inline void precompute_FEE(SoilEnv* env, float alpha)
{
  Dozer * dozer = &env->dozer;

  float rho = dozer->blade_rake_angle;
  float beta = (PI / 4.0f) - (env->soil_phi / 2.0f);
  float eta = env->soil_delta + rho + env->soil_phi + beta;
  float sin_eta = sinf(eta);
  if (fabsf(sin_eta) < 1e-6f) sin_eta = 1e-6f;

  env->N_gamma = ((1.0f / tanf(rho)) + (1.0f / tanf(beta))) * sinf(alpha + env->soil_phi + beta) / (2.0f * sin_eta);
  env->N_Q = sinf(alpha + env->soil_phi + beta) / sin_eta;
  env->N_c = cosf(env->soil_phi) / (sinf(beta) * sin_eta);
  env->N_ca = -cosf(rho + env->soil_phi + beta) / (sinf(rho) * sin_eta);
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

  // precompute the trig terms for the given step
  float cos_y = cosf(dozer->angular_z);
  float sin_y = sinf(dozer->angular_z);
  float phi = env->soil_phi;
  float tan_phi = tanf(phi);
  float cos_phi = cosf((PI * 0.25f) + (phi * 0.5f));

  if (tan_phi < 0.01f) tan_phi = 0.01f;  // set min bound for tan_phi

  // Using Terzaghi's soil bearing capacity theory.
  // Bearing capacity factors are additionally subscripted with '_b' to distinguish them from the blade FEE factors
  // Q_u = c * N_c + gamma * D * N_q + 0.5 * gamma * B * N_gamma
  //                                    ^ this value is because we assume a "strip footing"
  // Q_u = [cohesion] + [footing depth & overburden pressure] + [footing width & length of shear stress area]
  float N_q_b = expf( (3.0f * PI * 0.5f * tan_phi) - phi* tan_phi ) / ( 2.0f *  cos_phi * cos_phi);
  float N_c_b = (N_q_b - 1.0f) / tan_phi;
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

    int grid_index_left_x = (int)(point_left_x / CELL_SIZE);
    int grid_index_left_y = (int)(point_left_y / CELL_SIZE);
    int grid_index_right_x = (int)(point_right_x / CELL_SIZE);
    int grid_index_right_y = (int)(point_right_y / CELL_SIZE);

    float soil_height_left = 1.0f;
    float soil_height_right = 1.0f;

    if (grid_index_left_x >= 0 && grid_index_left_x < GRID_SIZE && grid_index_left_y >= 0 && grid_index_left_y < GRID_SIZE)
    {
      soil_height_left = env->grid_H[grid_index_left_x][grid_index_left_y] + env->grid_L[grid_index_left_x][grid_index_left_y];
    }
    if (grid_index_right_x >= 0 && grid_index_right_x < GRID_SIZE && grid_index_right_y >= 0 && grid_index_right_y < GRID_SIZE)
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
}

static inline void forward_kinematics(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;

  // Local coordinates of the blade joint relative to chassis origin
  float theta_arm = dozer->pos_virtual_lift_arm;
  float theta_pitch = dozer->pos_virtual_lift_arm;
  float theta_rake = dozer->blade_mount_pitch;

  float x_pitch_joint = dozer->arm_pivot_x + dozer->arm_length * cosf(theta_arm);
  float z_pitch_joint = dozer->arm_pivot_z + dozer->arm_length * sinf(theta_arm);

  float x_u_joint = x_pitch_joint + dozer->pitch_length * cosf(theta_arm + theta_pitch);
  float z_u_joint = z_pitch_joint + dozer->pitch_length * sinf(theta_arm + theta_pitch);

  float x_blade_edge = x_u_joint + dozer->blade_height * 0.5 * cosf(theta_arm + theta_pitch + theta_rake);
  float z_blade_edge = z_u_joint + dozer->pitch_length * 0.5 * sinf(theta_arm + theta_pitch + theta_rake);

  // Apply chassis roll rotation to the edge
  float x1 = x_blade_edge;
  float y1 = z_blade_edge * -sinf(dozer->angular_x);
  float z1 = z_blade_edge * cosf(dozer->angular_x);

  // Apply chassis pitch rotation (pitch is positive when pitching up)
  float cos_p = cosf(dozer->angular_y);
  float sin_p = sinf(dozer->angular_y);
  float x2 = x1 * cos_p - z1 * sin_p;
  float y2 = y1;
  float z2 = x1 * sin_p + z1 * cos_p;

  // Apply chassis yaw rotation
  float cos_y = cosf(dozer->angular_z);
  float sin_y = sinf(dozer->angular_z);
  float x3 = x2 * cos_y - y2 * sin_y;
  float y3 = x2 * sin_y + y2 * cos_y;
  float z3 = z2;

  // get global blade edge coordinates
  dozer->blade_x = dozer->position_x + x3;
  dozer->blade_y = dozer->position_y + y3;
  dozer->blade_z = dozer->position_z + z3;
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

  for (int iter = 0; iter < 3; iter++)
  {
    // 2. Determine Contact Area
    int margin = (int)((half_track_length + 1.0f) / CELL_SIZE);
    int center_i = (int)(dozer->position_x / CELL_SIZE);
    int min_i = clamp_idx(center_i - margin);
    int max_i = clamp_idx(center_i + margin);
    int center_j = (int)(dozer->position_y / CELL_SIZE);
    int min_j = clamp_idx(center_j - margin);
    int max_j = clamp_idx(center_j + margin);

    float marked_area = 0.0f;
    float tan_pitch = tanf(dozer->angular_y);
    float tan_roll = tanf(dozer->angular_x);

    for (int i = min_i; i <= max_i; i++)
    {
      for (int j = min_j; j <= max_j; j++)
      {
        float dx = (i * CELL_SIZE) - dozer->position_x;
        float dy = (j * CELL_SIZE) - dozer->position_y;
        float local_x = dx * cos_y + dy * sin_y;
        float local_y = -dx * sin_y + dy * cos_y;

        if (fabsf(local_x) <= half_track_length)
        {
          if (fabsf(local_y - half_track_gauge) <= half_track_width ||
              fabsf(local_y + half_track_gauge) <= half_track_width)
          {
            // z-height of the track at this local position
            float track_z = dozer->position_z + local_x * tan_pitch + local_y * tan_roll;
            float soil_z = env->grid_H[i][j] + env->grid_L[i][j];
            if (soil_z >= track_z)
            {
              marked_area += cell_area;
            }
          }
        }
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

    // 4. Apply compaction to marked cells
    if (compaction_rate > 0.0f)
    {
      for (int i = min_i; i <= max_i; i++)
      {
        for (int j = min_j; j <= max_j; j++)
        {
          if (env->grid_L[i][j] < 0.001f) continue;
          float dx = (i * CELL_SIZE) - dozer->position_x;
          float dy = (j * CELL_SIZE) - dozer->position_y;
          float local_x = dx * cos_y + dy * sin_y;
          float local_y = -dx * sin_y + dy * cos_y;

          if (fabsf(local_x) <= half_track_length)
          {
            if (fabsf(local_y - half_track_gauge) <= half_track_width ||
                fabsf(local_y + half_track_gauge) <= half_track_width)
            {
              float track_z = dozer->position_z + local_x * tan_pitch + local_y * tan_roll;
              float soil_z = env->grid_H[i][j] + env->grid_L[i][j];

              if (soil_z >= track_z)
              {
                float compacted = env->grid_L[i][j] * compaction_rate;
                env->grid_L[i][j] -= compacted;
                env->grid_H[i][j] += compacted / env->swell_ratio;
              }
            }
          }
        }
      }
    }

    // 5. Update vehicle pose at the end of the compaction iteration
    update_chassis_pose(env);
  }

  // update global blade coordinates
  forward_kinematics(env);
}

static inline void update_chassis_velocity(SoilEnv* env, float dt) {
  Dozer * dozer = &env->dozer;

  float effort_left = clamp_action(dozer->effort_linear - dozer->effort_rotational);
  float effort_right = clamp_action(dozer->effort_linear + dozer->effort_rotational);

  float f_hyd_left = effort_left * dozer->max_force_linear / 2.0f;
  float f_hyd_right = effort_right * dozer->max_force_linear / 2.0f;
  float f_hyd_total = f_hyd_left + f_hyd_right;

  float f_trac_max = calculate_max_traction(env);
  float f_resist = dozer->last_force; 

  float f_push = fminf(fabsf(f_hyd_total), f_trac_max);
  if (f_hyd_total < 0) f_push = -f_push;

  float f_net = f_push - f_resist;

  // if resistive forces are greater than applied force, stall
  if (fabsf(f_resist) > fabsf(f_hyd_total) && (f_hyd_total * f_resist > 0))
  {
    dozer->twist_linear_x = 0.0f;
    dozer->vel_tracks_linear = 0.0f; 
  }
  else
  {
    dozer->twist_linear_x += (f_net / dozer->machine_mass) * dt;
    dozer->twist_linear_x *= (1.0f - dozer->track_damping * dt); 

    // if our applied force is greater than our available traction
    if (fabsf(f_hyd_total) > f_trac_max)
    {
      float max_track_speed = 3.0f; 

      // we're slipping, so track velocities just depend on commanded effort
      dozer->vel_tracks_linear = dozer->effort_linear * max_track_speed;
    }
    else
    {
      // otherwise, track velocities are equal to actual vehicle movement
      dozer->vel_tracks_linear = dozer->twist_linear_x;
    }
  }

  dozer->twist_angular_z += ((dozer->effort_rotational * dozer->max_force_rotational) / dozer->machine_inertia) * dt;
  dozer->twist_angular_z *= (1.0f - dozer->track_damping * dt);
}

static inline void update_chassis_2d_position(SoilEnv* env, float dt)
{
  Dozer * dozer = &env->dozer;
  dozer->angular_z += dozer->twist_angular_z * dt;
  dozer->position_x += dozer->twist_linear_x * cosf(dozer->angular_z) * dt;
  dozer->position_y += dozer->twist_linear_x * sinf(dozer->angular_z) * dt;
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

static inline void interact_with_soil(SoilEnv* env, float dt)
{
  Dozer * dozer = &env->dozer;

  float total_force = 0.0f;
  float total_yaw_moment = 0.0f;
  float total_roll_moment = 0.0f;
  float total_vol_cut = 0.0f;

  // precalculate some trigs
  float cos_y = cosf(dozer->angular_z);
  float sin_y = sinf(dozer->angular_z);

  float total_roll = dozer->angular_x + dozer->pos_blade_roll;

  // Bresenham's Line Algorithm to discretize blade over the grid
  // 1. Find start and end coordinates of the blade in meters
  float half_w = dozer->blade_width / 2.0f;
  float start_m_x = dozer->blade_x - (-half_w) * sin_y; // Left edge
  float start_m_y = dozer->blade_y + (-half_w) * cos_y;
  float end_m_x = dozer->blade_x - (half_w) * sin_y;   // Right edge
  float end_m_y = dozer->blade_y + (half_w) * cos_y;

  // 2. Convert to grid indices
  int x0 = (int)(start_m_x / CELL_SIZE);
  int y0 = (int)(start_m_y / CELL_SIZE);
  int x1 = (int)(end_m_x / CELL_SIZE);
  int y1 = (int)(end_m_y / CELL_SIZE);

  int dx_i = (x1 > x0) ? (x1 - x0) : (x0 - x1);
  int dy_i = (y1 > y0) ? (y1 - y0) : (y0 - y1);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx_i - dy_i;

  // Effective width of blade per intersected cell to conserve mass/force geometry.
  // When diagonal, this properly scales up to CELL_SIZE * sqrt(2) to prevent force loss.
  float effective_width = CELL_SIZE / fmaxf(fabsf(cos_y), fabsf(sin_y));

  // Arrays to store exactly which cells we visited to deposit cut volume later
  int visited_x[100];
  int visited_y[100];
  int num_cells = 0;

  while (1)
  {
    if (x0 >= 0 && x0 < GRID_SIZE && y0 >= 0 && y0 < GRID_SIZE)
    {
      if (num_cells < 100)
      {
        visited_x[num_cells] = x0;
        visited_y[num_cells] = y0;
        num_cells++;
      }

      // Calculate the moment arm (local_y) and blade elevation at this specific grid cell
      float cell_m_x = x0 * CELL_SIZE;
      float cell_m_y = y0 * CELL_SIZE;
      float dx = cell_m_x - dozer->blade_x;
      float dy = cell_m_y - dozer->blade_y;
      float local_y = -dx * sin_y + dy * cos_y;

      float section_blade_z = dozer->blade_z + local_y * sinf(total_roll);

      // 3. Cut the soil (remove from heightmap, convert to loose)
      float total_h = env->grid_H[x0][y0] + env->grid_L[x0][y0];
      float depth = total_h - section_blade_z;

      if (depth > 0.0f)
      {
        if (env->grid_L[x0][y0] > 0.0f)
        {
          float l_cut = (depth < env->grid_L[x0][y0]) ? depth : env->grid_L[x0][y0];
          env->grid_L[x0][y0] -= l_cut; 
          depth -= l_cut; 
          total_vol_cut += l_cut * CELL_SIZE * CELL_SIZE;
        }
        if (depth > 0.0f)
        {
          env->grid_H[x0][y0] -= depth; 
          total_vol_cut += depth * CELL_SIZE * CELL_SIZE * env->swell_ratio;
        }
      }

      // 4. Calculate FEE force on the unyielding soil 1 cell directly in front
      float front_x = cell_m_x + CELL_SIZE * cos_y;
      float front_y = cell_m_y + CELL_SIZE * sin_y;
      int f_grid_x = (int)(front_x / CELL_SIZE);
      int f_grid_y = (int)(front_y / CELL_SIZE);

      if (f_grid_x >= 0 && f_grid_x < GRID_SIZE && f_grid_y >= 0 && f_grid_y < GRID_SIZE)
      {
        float f_total_h = env->grid_H[f_grid_x][f_grid_y] + env->grid_L[f_grid_x][f_grid_y];
        float f_total_depth = f_total_h - section_blade_z;
        float f_hard_depth = env->grid_H[f_grid_x][f_grid_y] - section_blade_z;

        float df = calculate_FEE_column(env, f_hard_depth, f_total_depth, effective_width);
        total_force += df;
        total_yaw_moment += df * local_y; 
        total_roll_moment += df * local_y; 
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

  // 5. Add the cut volume to the loose soil layer directly in front of the visited cells
  if (total_vol_cut > 0.0f && num_cells > 0)
  {
    float dh = (total_vol_cut / num_cells) / (CELL_SIZE * CELL_SIZE);
    for (int s = 0; s < num_cells; s++)
    {
      int cx = visited_x[s];
      int cy = visited_y[s];

      // Push dirt forward 1 cell length in the direction of travel
      float dep_global_x = cx * CELL_SIZE + CELL_SIZE * cos_y;
      float dep_global_y = cy * CELL_SIZE + CELL_SIZE * sin_y;

      int dep_x = (int)(dep_global_x / CELL_SIZE);
      int dep_y = (int)(dep_global_y / CELL_SIZE);
      if (dep_x >= 0 && dep_x < GRID_SIZE && dep_y >= 0 && dep_y < GRID_SIZE)
      {
        env->grid_L[dep_x][dep_y] += dh;
      }
    }
  }
}

static inline void update_joint_vel(SoilEnv* env, float dt)
{
  Dozer * dozer = &env->dozer;

  float LIFT_ARM_CG = 0.5f;
  float TOTAL_ARM_LEN = dozer->arm_length + cosf(dozer->pos_blade_pitch + 0.5f) * dozer->pitch_length;
  float arm_gravity_torque = dozer->arm_mass * GRAVITY * TOTAL_ARM_LEN * LIFT_ARM_CG * cosf(dozer->pos_virtual_lift_arm);
  float arm_resist_torque = sinf(dozer->pos_virtual_lift_arm) * dozer->last_force * TOTAL_ARM_LEN;
  float arm_total_extern_torque = -arm_gravity_torque - arm_resist_torque;

  if (dozer->effort_lift * arm_total_extern_torque <= 0)
  {
    arm_total_extern_torque *= (1.0f - dozer->hydraulic_stiffness);
  }

  dozer->vel_virtual_lift_arm = (dozer->vel_virtual_lift_arm + ((dozer->effort_lift * dozer->max_torque_list_arm + arm_total_extern_torque) / dozer->arm_inertia) * dt) / (1.0f + (dozer->virtual_lift_arm_damping / dozer->arm_inertia) * dt);

  float PITCH_LINK_CG = 0.75f;
  float pitch_gravity_torque = dozer->pitch_mass * GRAVITY * dozer->pitch_length * PITCH_LINK_CG * cosf(dozer->pos_virtual_lift_arm + dozer->pos_blade_pitch);
  float pitch_resist_torque = sinf(dozer->pos_virtual_lift_arm + dozer->pos_blade_pitch) * dozer->last_force * dozer->pitch_length;
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

static inline void simulate_erosion(SoilEnv* env, float dt, const int num_loops)
{
  Dozer* dozer = &env->dozer;

  int margin = (int)(4.0f / CELL_SIZE);  // number of cells we buffer about the blade
  int center_i = (int)(dozer->blade_x / CELL_SIZE);  // get the ith cell location of the blade
  int min_i = clamp_idx(center_i - margin);  // use our buffer to find the min ith cell
  int max_i = clamp_idx(center_i + margin);  // same but for max
  int center_j = (int)(dozer->blade_y / CELL_SIZE);  // repeat last three but for jth cells
  int min_j = clamp_idx(center_j - margin);
  int max_j = clamp_idx(center_j + margin);

  // float loader_length = dozer->track_length + 1.0f; // rough approximation
  float loader_length = dozer->track_length; // TODO: determine which is better here
  float tan_phi = tanf(env->soil_phi);

  for (int iter = 0; iter < num_loops; iter++)  // loop 3 times
  {
    float temp_L[GRID_SIZE][GRID_SIZE];  // get temp loose soil grid (TODO: lots of memory allocation... maybe reserve this earlier)
    for (int i = min_i; i <= max_i; i++)  // copy over values within out buffered area
    {
      for (int j = min_j; j <= max_j; j++)
      {
        temp_L[i][j] = env->grid_L[i][j];
      }
    }

    int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};  // 8 surrounding pixel locations
    int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};

    for (int i = min_i + 1; i < max_i; i++)  // start looking at our buffered pixels
    {
      for (int j = min_j + 1; j < max_j; j++)
      {
        if (env->grid_L[i][j] <= 1e-4f) continue;  // if loose soil at this pixel is 0, skip
        float total_h = env->grid_H[i][j] + env->grid_L[i][j];  // get total height at this pixel

        for(int d=0; d<8; d++)  // look through the 8 surrounding pixels
        {
          int ni = i + dx[d], nj = j + dy[d];  // get the surrounding pixel index relative to the current pixel

          float cell_x = ni * CELL_SIZE;  // get the surrounding pixel diff location in meters
          float cell_y = nj * CELL_SIZE;

          float dx_cell = cell_x - dozer->blade_x;
          float dy_cell = cell_y - dozer->blade_y;
          float local_x = dx_cell * cosf(dozer->angular_z) + dy_cell * sinf(dozer->angular_z);  // get the surrounding pixel in local coords
          float local_y = -dx_cell * sinf(dozer->angular_z) + dy_cell * cosf(dozer->angular_z);

          if (fabsf(local_y) <= dozer->blade_width / 2.0f)  // if this pixel is under the vehicle, don't bother
          {
            if (local_x >= -loader_length && local_x <= 0.0f) continue; // prevent soil falling into the dozer
          }

          float neighbor_total_h = env->grid_H[ni][nj] + env->grid_L[ni][nj];  // get total height of surrounding pixel
          float dH = total_h - neighbor_total_h;
          float dist = (d < 4) ? CELL_SIZE : (CELL_SIZE * 1.41421356f);  // if surrounding pixel is diagnoal, mult by sqrt(2)

          if (dH > 0.0f)  // if current pixel is higher than neighbor
          {
            float t = dH / dist;
            float L_val = temp_L[i][j];
            if (L_val < 1e-5f) L_val = 1e-5f;
            float K = env->soil_c / (env->loose_soil_density * GRAVITY * L_val);
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

            if (t > t_limit)
            {
              float slip = (t - t_limit) * dist * 0.2f;
              if (slip > temp_L[i][j]) slip = temp_L[i][j];
              temp_L[i][j] -= slip;
              temp_L[ni][nj] += slip;
              total_h -= slip;
            }
          }
        }
      }
    }
    for (int i = min_i; i <= max_i; i++)
    {
      for (int j = min_j; j <= max_j; j++)
      {
        env->grid_L[i][j] = temp_L[i][j];
      }
    }
  }
}

static inline void update_surcharge(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;

  // Create a local bounding box to search for loose soil (1.5m forward, plus width of blade)
  float search_radius = 2.0f; // rough max extent
  int center_i = (int)(dozer->blade_x / CELL_SIZE);
  int center_j = (int)(dozer->blade_y / CELL_SIZE);
  int margin = (int)(search_radius / CELL_SIZE) + 1;

  int min_i = clamp_idx(center_i - margin);
  int max_i = clamp_idx(center_i + margin);
  int min_j = clamp_idx(center_j - margin);
  int max_j = clamp_idx(center_j + margin);

  float current_surcharge_vol = 0.0f;

  float cos_y = cosf(dozer->angular_z);
  float sin_y = sinf(dozer->angular_z);
  float half_width = dozer->blade_width / 2.0f;
  float lookahead = 1.5f;
  float cell_area = CELL_SIZE * CELL_SIZE;

  for (int i = min_i; i <= max_i; i++)
  {
    for (int j = min_j; j <= max_j; j++)
    {
      if (env->grid_L[i][j] <= 0.0f) continue;

      float cell_x = i * CELL_SIZE;
      float cell_y = j * CELL_SIZE;

      float dx = cell_x - dozer->blade_x;
      float dy = cell_y - dozer->blade_y;

      // Transform cell coordinates into the blade's local frame
      float local_x = dx * cos_y + dy * sin_y;
      float local_y = -dx * sin_y + dy * cos_y;

      // Check if the cell is inside the 1.5m box directly in front of the blade
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
  interact_with_soil(env, dt);

  // 7. Simulate Soil Erosion (Slumping)
  simulate_erosion(env, dt, 3);

  // 8. Update Blade Surcharge
  update_surcharge(env);

  // precompute soil forces based on the updated blade rake angle
  precompute_FEE(env, 0.0f); // TODO: ALPHA argument is currently 0.0f

  env->step_num++;
}

static inline void env_reset(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;

  precompute_soil_bearing_capacity(env);
}

void c_reset(SoilEnv* env)
{
  env->tick = 0;
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
  dozer->effort_roll       = clamp_action(env->actions[4]);
  dozer->effort_yaw        = 0;  // this should just get zero'd (since we don't have control over this)

  env->terminals[0] = 0;  // zero these guys just in case
  env->rewards[0]   = 0;  // zero these guys just in case

  // run the sim given inputs
  simulate_step(env, 0.1);

  // get observations (reawards and terminals also seen here, since we're already doing some loops!)
  get_obs(env);
}

#endif