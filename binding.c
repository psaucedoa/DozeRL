#include "dozerl.h"
#define OBS_SIZE 5011
#define NUM_ATNS 4
#define ACT_SIZES {1, 1, 1, 1}
#define OBS_TENSOR_T FloatTensor

#define Env SoilEnv
#include "vecenv.h"

void my_init(Env* env, Dict* kwargs) {
    env->num_agents = 1;
}

void my_log(Log* log, Dict* out) {
    dict_set(out, "perf", log->perf);
    dict_set(out, "score", log->score);
    dict_set(out, "episode_return", log->episode_return);
    dict_set(out, "episode_length", log->episode_length);
    dict_set(out, "r_push", log->r_push);
    dict_set(out, "r_progress", log->r_progress);
    dict_set(out, "r_off_map", log->r_off_map);
    dict_set(out, "r_stationary", log->r_stationary);

    dict_set(out, "max_obs_vel_arm",         log->max_vel_arm);
    dict_set(out, "max_obs_vel_blade_pitch", log->max_vel_blade_pitch);
    dict_set(out, "max_obs_vel_blade_roll",  log->max_vel_blade_roll);
    dict_set(out, "max_obs_vel_linear",      log->max_vel_linear);
    dict_set(out, "max_obs_vel_rotational",  log->max_vel_rotational);
    dict_set(out, "max_obs_vel_blade_yaw",   log->max_vel_blade_yaw);

    dict_set(out, "min_obs_vel_arm",         log->min_vel_arm);
    dict_set(out, "min_obs_vel_blade_pitch", log->min_vel_blade_pitch);
    dict_set(out, "min_obs_vel_blade_roll",  log->min_vel_blade_roll);
    dict_set(out, "min_obs_vel_linear",      log->min_vel_linear);
    dict_set(out, "min_obs_vel_rotational",  log->min_vel_rotational);
    dict_set(out, "min_obs_vel_blade_yaw",   log->min_vel_blade_yaw);

    dict_set(out, "max_obs_height", log->max_height);
    dict_set(out, "min_obs_height", log->min_height);

    dict_set(out, "n", log->n);
}
