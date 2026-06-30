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

    if (IsKeyPressed(KEY_R))
    {
      c_reset(env);
    }

    if (!IsKeyDown(KEY_LEFT_SHIFT) && IsGamepadAvailable(0))
    {
      float left_y = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y);
      env->actions[0] = -left_y;

      float left_x = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X);
      env->actions[1] = left_x;

      float right_y = GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_Y);
      env->actions[2] = -right_y;

      float right_x = GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_X);
      env->actions[3] = right_x;

      float lt = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_TRIGGER);
      float rt = GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_TRIGGER);
      float lt_val = (lt + 1.0f) / 2.0f;
      float rt_val = (rt + 1.0f) / 2.0f;
      env->actions[4] = rt_val - lt_val;
    }
    else
    {
      if (IsKeyDown(KEY_W)) env->actions[0] = 1.0f;
      if (IsKeyDown(KEY_S)) env->actions[0] = -1.0f;
      if (IsKeyDown(KEY_A)) env->actions[1] = -1.0f;
      if (IsKeyDown(KEY_D)) env->actions[1] = 1.0f;
      if (IsKeyDown(KEY_UP)) env->actions[2] = 1.0f;
      if (IsKeyDown(KEY_DOWN)) env->actions[2] = -1.0f;
    }


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
