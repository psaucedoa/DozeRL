#include "dozerl.h"
#include "puffernet.h"
#include "raylib_render.h"
#include <stdio.h>
#include <time.h>

void demo(const char* checkpoint_path)
{
  SoilEnv* env = (SoilEnv*)malloc(sizeof(SoilEnv));
  env->observations = (float*)calloc(5011, sizeof(float));
  env->actions = (float*)calloc(6, sizeof(float));
  env->rewards = (float*)calloc(1, sizeof(float));
  env->terminals = (float*)calloc(1, sizeof(float));
  env->rng = 42;

  Weights* weights = NULL;
  PufferNet* net = NULL;

  c_reset(env);
  c_render(env);

  while (!WindowShouldClose())
  {
    env->actions[0] = 0.0f;
    env->actions[1] = 0.0f;
    env->actions[2] = 0.0f;
    env->actions[3] = 0.0f;
    env->actions[4] = 0.0f;
    if (IsKeyDown(KEY_W)) env->actions[0] = 1.0f;
    if (IsKeyDown(KEY_S)) env->actions[0] = -1.0f;
    if (IsKeyDown(KEY_A)) env->actions[1] = -1.0f;
    if (IsKeyDown(KEY_D)) env->actions[1] = 1.0f;
    if (IsKeyDown(KEY_UP)) env->actions[2] = 1.0f;
    if (IsKeyDown(KEY_DOWN)) env->actions[2] = -1.0f;

    c_step(env);
    c_render(env);
  }

  if (net != NULL) free_puffernet(net);
  if (weights != NULL) free(weights);

  free(env->observations);
  free(env->actions);
  free(env->rewards);
  free(env->terminals);
  c_close(env);
}

int main(int argc, char** argv) {
    const char* checkpoint_path = NULL;
    if (argc > 1) {
        checkpoint_path = argv[1];
    }
    demo(checkpoint_path);
    return 0;
}
