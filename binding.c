#include "dozerl.h"
#define OBS_SIZE OBS_SIZE_FLAT
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
    dict_set(out, "max arm vel", log->max_vel_arm);
    dict_set(out, "max blade pitch vel", log->max_vel_blade_pitch);
    dict_set(out, "max linear vel", log->max_vel_linear);
    dict_set(out, "min arm vel", log->min_vel_arm);
    dict_set(out, "min blade pitch vel", log->min_vel_blade_pitch);
    dict_set(out, "min linear vel", log->min_vel_linear);
    dict_set(out, "r_shaping", log->r_shaping);
    dict_set(out, "r_off_map", log->r_off_map);
    dict_set(out, "r_time", log->r_time);
    dict_set(out, "n", log->n);
}
