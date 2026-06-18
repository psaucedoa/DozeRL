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
  float blade_rake_angle;   // (rad)
  float blade_surcharge_Q;  // (N)  TODO: CHECK UNITS
  float blade_x;            // (m)
  float blade_y;            // (m)
  float blade_z;            // (m)

  // Track Geometry
  float track_width;
  float track_length;

  // Arm Geometry
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
  float last_force;       // (N)
  float last_yaw_moment;  // (N*m)

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

static inline void update_kinematics(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;

  // precompute the trig terms for the given step
  float cos_y = cosf(dozer->angular_z);
  float sin_y = sinf(dozer->angular_z);
  float tan_phi = tanf(env->soil_phi);

  if (tan_phi < 0.01f) tan_phi = 0.01f;  // set min bound for tan_phi


}

static inline void simulate_step(SoilEnv* env, float dt)
{
  Dozer * dozer = &env->dozer;
  // Linear track speed
  dozer->vel_tracks_linear = (dozer->vel_tracks_linear + ((dozer->effort_linear * MAX_FORCE_LINEAR - dozer->last_force) / MACHINE_MASS) * dt) / (1.0f + (LINEAR_DAMPING / MACHINE_MASS) * dt);

  // net yaw torque
  float mu_lat = 0.6f; 
  float scrub_torque = (mu_lat * dozer->machine_mass * GRAVITY * dozer->track_length) / 4.0f;
  float net_yaw_torque = dozer->effort_rotational * MAX_TORQUE_ROTATIONAL + dozer->last_yaw_moment;

  // determine yaw torque
  if (fabsf(dozer->vel_tracks_rotational) < 0.05f)
  {
    if (fabsf(net_yaw_torque) <= scrub_torque)
    {
      net_yaw_torque = 0.0f;
      dozer->vel_tracks_rotational = 0.0f;
    }
    else
    {
      net_yaw_torque -= (net_yaw_torque > 0 ? scrub_torque : -scrub_torque);
    }
  }
  else
  {
    net_yaw_torque -= (dozer->vel_tracks_rotational > 0 ? scrub_torque : -scrub_torque);
  }

  // Rotational Track Speed
  dozer->vel_tracks_rotational = (dozer->vel_tracks_rotational + (net_yaw_torque / MACHINE_INERTIA) * dt) / (1.0f + (ROTATIONAL_DAMPING / MACHINE_INERTIA) * dt);

  // Lift arm external torques
  float LIFT_ARM_CG = 0.5f;
  float TOTAL_ARM_LEN = dozer->arm_length + cosf(dozer->pos_blade_pitch + 0.5) * dozer->pitch_length;  // offset by 0.5rad since it's basically inline at ~-0.5rad
  float arm_gravity_torque = dozer->arm_mass * GRAVITY * TOTAL_ARM_LEN * LIFT_ARM_CG * cosf(dozer->pos_virtual_lift_arm);  // TODO: this should be simplified with total arm inertia
  float arm_resist_torque = sinf(dozer->pos_virtual_lift_arm) * dozer->last_force * TOTAL_ARM_LEN;
  float arm_total_extern_torque = -arm_gravity_torque - arm_resist_torque;

  // why does this check exist?
  if (dozer->effort_lift * arm_total_extern_torque <= 0)
  {
    // this gets triggered if effort (+) && extern (-) OR
    // effort (-) && extern (+)
    arm_total_extern_torque *= (1.0f - HYDRAULIC_STIFFNESS);
  }

  // get true arm lift vel & integrate for pos
  dozer->vel_virtual_lift_arm = (dozer->vel_virtual_lift_arm + ((dozer->effort_lift * MAX_TORQUE_LIFT + arm_total_extern_torque) / ARM_INERTIA) * dt) / (1.0f + (ARM_DAMPING / ARM_INERTIA) * dt);
  dozer->pos_virtual_lift_arm += dozer->vel_virtual_lift_arm * dt;

  // check for arm limits
  float arm_min = dozer->pos_virtual_lift_arm_min;
  float arm_max = dozer->pos_virtual_lift_arm_max;
  if (dozer->pos_virtual_lift_arm < arm_min) dozer->pos_virtual_lift_arm = arm_min;
  if (dozer->pos_virtual_lift_arm > arm_max) dozer->pos_virtual_lift_arm = arm_max;

  // TODO: GLOABLIZE -> float blade_stiffness = 0.98f;

  // Pitch External Torques
  float PITCH_LINK_CG = 0.75f;
  float pitch_gravity_torque = PITCH_MASS * GRAVITY * dozer->pitch_length * PITCH_LINK_CG * cosf(dozer->pos_virtual_lift_arm + dozer->pos_blade_pitch) * dozer->last_force * dozer->pitch_length;
  float pitch_resist_torque = sinf(dozer->pos_virtual_lift_arm + dozer->pos_blade_pitch) * dozer->last_force * dozer->pitch_length;
  float pitch_total_extern_torque = -pitch_gravity_torque - pitch_resist_torque;
  if (dozer->effort_pitch * pitch_total_extern_torque <= 0)
  {
    pitch_total_extern_torque *= (1.0f - PITCH_STIFFNESS);  // TODO: Maybe the same as HYDRAULIC_STIFFNESS
  }

  // get true pitch joint vel and pos
  dozer->vel_blade_pitch = (dozer->vel_blade_pitch + ((dozer->effort_pitch * MAX_TORQUE_PITCH + pitch_total_extern_torque) / PITCH_INERTIA) * dt) / (1.0f + (PITCH_DAMPING / PITCH_INERTIA) * dt);
  dozer->pos_blade_pitch += dozer->vel_blade_pitch * dt;

  // check limits
  float PITCH_MIN = dozer->pos_blade_pitch_min;
  float PITCH_MAX = dozer->pos_blade_pitch_max;
  if (dozer->pos_blade_pitch < PITCH_MIN) dozer->pos_blade_pitch = PITCH_MIN;
  if (dozer->pos_blade_pitch > PITCH_MAX) dozer->pos_blade_pitch = PITCH_MAX;

  // roll external torques
  // TODO: This one's different since the resisting forces are moreso dependent on the pressure exerted
  // by the soil, resisting penetration by the edges of the blade (when in contact with soil)
  // Otherwise, it's just internal fricitons of the hydraulics and joint

  // TODO: why the 45 deg here?
  dozer->blade_rake_angle = (45.0f * (PI/180.0f)) + dozer->pos_blade_pitch + dozer->pos_virtual_lift_arm + dozer->angular_y;
  float ALPHA = 0.0;  // TODO: globalize? Describe this
  precompute_FEE(env, 0.0f);  // TODO: Is ALPHA a necessary arg here, or is the global param fine?

  // find slip
  float slip = dozer->last_force / calculate_max_traction();  // TODO: We might just be able to calculate this value once, with the assumption that we'll ~always have full track-on-soil contact... which is wrong. So alternatively, calculate contact area?
  if (slip < 0) slip = 0;
  if (slip > 1) slip = 1;

  // get updated positions
  dozer->angular_z += dozer->vel_tracks_rotational * (1.0f - slip) * dt;
  dozer->position_x += dozer->vel_tracks_linear * (1.0f - slip) * cosf(dozer->angular_z) * dt;
  dozer->position_y += dozer->vel_tracks_linear * (1.0f - slip) * sinf(dozer->angular_z) * dt;

  // get updated true velocities (odom)
  // maybe by adding up the track speeds and accountring for slip?
  // but then we'd have to keep track of individual track speeds internally
  // which would require some reqork in earlier sections...
  dozer->twist_linear_x = dozer->vel_tracks_linear * (1.0f - slip);
  dozer->twist_angular_z = dozer->vel_tracks_rotational * (1.0f - slip);

  update_kinematics(env);  // TODO: should this happen before or after we get updated positions?

}

// Required function
void c_step(SoilEnv* env)
{
  Dozer * dozer = &env->dozer;
  env->tick +=1;

  // get inputs
  // Continuous acitons: Clamp to [-1, 1] and then threshold
  dozer->effort_linear     = clamp_action(env->actions[0]);
  dozer->effort_rotational = clamp_action(env->actions[0]);
  dozer->effort_lift       = clamp_action(env->actions[0]);
  dozer->effort_pitch      = clamp_action(env->actions[0]);
  dozer->effort_roll       = clamp_action(env->actions[0]);
  dozer->effort_yaw        = 0;  // this should just get zero'd (since we don't have control over this)

  env->terminals[0] = 0;  // zero these guys just in case
  env->rewards[0]   = 0;  // zero these guys just in case

  // run the sim given inputs
  simulate_step(env, 0.1);

  // get observations (reawards and terminals also seen here, since we're already doing some loops!)
  get_obs(env);
}