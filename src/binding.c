#include "dozerl.h"
#define OBS_SIZE 5011
#define NUM_ATNS 6
#define ACT_SIZES {1, 1, 1, 1, 1, 1}
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
    dict_set(out, "count_off_map", log->count_off_map);
    dict_set(out, "count_jitter", log->count_jitter);
    dict_set(out, "count_large_neg_rewards", log->count_large_neg_rewards);
}
