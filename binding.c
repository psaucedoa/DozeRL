#include "dozerl.h"
// Flat for vecenv.h (11 proprio incl. noisy surcharge + 2×50×50 spatial)
// Keep flat for maximum 3.0/4.0 compat — Python model reshapes to image:
//   proprio = obs[0:11]                          // 11-d
//   spatial = obs[11:].reshape(2,50,50) // CHW: ch0 = H+L-z, ch1 = G-z  [dozerl.h:339]
// For 4.0 static allocator use the same 5011 contiguous buffer (Allocator sums 11+5000)
#define OBS_SIZE 5011
#define OBS_PROP_SIZE 11
#define OBS_SPATIAL_CH 2
#define OBS_SPATIAL_H 50
#define OBS_SPATIAL_W 50
#define NUM_ATNS 6
#define ACT_SIZES {1, 1, 1, 1, 1, 1}
#define OBS_TENSOR_T FloatTensor
// PufferLib 4.0: if you switch to Dict obs, expose as {proprio:Box(11,), spatial:Box(2,50,50)}
// and register two tensors instead of one flat — no env step change, just Python wrapper.

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
    dict_set(out, "count_large_neg_rewards", log->count_large_neg_rewards);
    dict_set(out, "count_off_map", log->count_off_map);
    dict_set(out, "count_jitter", log->count_jitter);
    dict_set(out, "max arm vel", log->max_vel_arm);
    dict_set(out, "max blade pitch vel", log->max_vel_blade_pitch);
    dict_set(out, "max blade roll vel", log->max_vel_blade_roll);
}
