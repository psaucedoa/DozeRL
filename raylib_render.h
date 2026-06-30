#ifndef RAYLIB_RENDER_H
#define RAYLIB_RENDER_H

#include "dozerl.h"
#include <raylib.h>
#include <rlgl.h>

static Camera3D camera = { 0 };

static inline void init_render()
{
  InitWindow(1280, 720, "DozeRL Simulator");
  SetTargetFPS(60);

  camera.position = (Vector3){ 15.0f, 15.0f, 15.0f };
  camera.target = (Vector3){ 5.0f, 0.0f, 5.0f };
  camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;
}

static inline void draw_heightmap_fast(SoilEnv* env)
{
  float cw = CELL_SIZE;

  // Draw hard soil (base terrain)
  rlBegin(RL_QUADS);
  rlColor4ub(101, 67, 33, 255); // Dark Brown
  for (int i = 0; i < GRID_SIZE - 1; i++)
  {
    for (int j = 0; j < GRID_SIZE - 1; j++)
    {
      float h00 = env->grid_H[i][j];
      float h10 = env->grid_H[i+1][j];
      float h01 = env->grid_H[i][j+1];
      float h11 = env->grid_H[i+1][j+1];
      float x0 = i * cw, x1 = (i+1) * cw;
      float z0 = j * cw, z1 = (j+1) * cw;
      rlVertex3f(x0, h00, z0);
      rlVertex3f(x0, h01, z1);
      rlVertex3f(x1, h11, z1);
      rlVertex3f(x1, h10, z0);
    }
  }
  rlEnd();

  // Draw loose soil on top
  rlBegin(RL_QUADS);
  rlColor4ub(139, 90, 43, 255); // Lighter Brown
  for (int i = 0; i < GRID_SIZE - 1; i++)
  {
    for (int j = 0; j < GRID_SIZE - 1; j++)
    {
      if (env->grid_L[i][j] <= 0.01f && env->grid_L[i+1][j] <= 0.01f && env->grid_L[i][j+1] <= 0.01f && env->grid_L[i+1][j+1] <= 0.01f) continue;

      float h00 = env->grid_H[i][j] + env->grid_L[i][j];
      float h10 = env->grid_H[i+1][j] + env->grid_L[i+1][j];
      float h01 = env->grid_H[i][j+1] + env->grid_L[i][j+1];
      float h11 = env->grid_H[i+1][j+1] + env->grid_L[i+1][j+1];

      float x0 = i * cw, x1 = (i+1) * cw;
      float z0 = j * cw, z1 = (j+1) * cw;

      rlVertex3f(x0, h00, z0);
      rlVertex3f(x0, h01, z1);
      rlVertex3f(x1, h11, z1);
      rlVertex3f(x1, h10, z0);
    }
  }
  rlEnd();
}

static inline void draw_rectangular_prism(Vector3 center, Vector3 size, float yaw, float pitch, float roll, Color color)
{
  rlPushMatrix();
  rlTranslatef(center.x, center.y, center.z);
  rlRotatef(yaw * RAD2DEG, 0.0f, 1.0f, 0.0f);   // Yaw around Y
  rlRotatef(pitch * RAD2DEG, 1.0f, 0.0f, 0.0f); // Pitch around X
  rlRotatef(roll * RAD2DEG, 0.0f, 0.0f, 1.0f);  // Roll around Z

  DrawCube((Vector3){0,0,0}, size.x, size.y, size.z, color);
  DrawCubeWires((Vector3){0,0,0}, size.x, size.y, size.z, BLACK);

  rlPopMatrix();
}

static inline void draw_dozer(SoilEnv* env)
{
  Dozer* dozer = &env->dozer;

  Color chassis_color = (Color){210, 180, 140, 255};      // Desert Tan
  Color arm_color = (Color){180, 150, 110, 255};          // Darker Desert Tan
  Color track_color = (Color){60, 60, 60, 255};           // Dark Grey
  Color blade_color = (Color){80, 80, 80, 255};           // Dark Grey

  float raylib_yaw = -dozer->angular_z; 
  float raylib_pitch = dozer->angular_y;
  float raylib_roll = dozer->angular_x;

  // Chassis
  Vector3 chassis_pos = { dozer->position_x, dozer->position_z + 0.9f, dozer->position_y };
  Vector3 chassis_size = { 2.54f, 1.8669f, 0.889f };
  draw_rectangular_prism(chassis_pos, chassis_size, raylib_yaw, raylib_pitch, raylib_roll, chassis_color);

  // Tracks (Left & Right)
  Vector3 track_size = { dozer->track_length, 0.4064f, dozer->track_width };
  float gauge_offset = dozer->track_gauge / 2.0f;

  Vector3 track_pos_l =
  {
    dozer->position_x - gauge_offset * sinf(dozer->angular_z),
    dozer->position_z + 0.2f, 
    dozer->position_y + gauge_offset * cosf(dozer->angular_z) 
  };

  draw_rectangular_prism(track_pos_l, track_size, raylib_yaw, raylib_pitch, raylib_roll, track_color);

  Vector3 track_pos_r = {
    dozer->position_x + gauge_offset * sinf(dozer->angular_z),
    dozer->position_z + 0.2f, 
    dozer->position_y - gauge_offset * cosf(dozer->angular_z) 
  };
  draw_rectangular_prism(track_pos_r, track_size, raylib_yaw, raylib_pitch, raylib_roll, track_color);

  // Virtual Arms (Left & Right)
  float theta_arm = dozer->pos_virtual_lift_arm;
  float arm_x = dozer->position_x + (dozer->arm_pivot_x + dozer->arm_length * 0.5f * cosf(theta_arm)) * cosf(dozer->angular_z);
  float arm_y = dozer->position_y + (dozer->arm_pivot_x + dozer->arm_length * 0.5f * cosf(theta_arm)) * sinf(dozer->angular_z);
  float arm_z = dozer->position_z + dozer->arm_pivot_z + dozer->arm_length * 0.5f * sinf(theta_arm);

  Vector3 arm_size = { dozer->arm_length, 0.1f, 0.1f };
  float arm_offset = 0.5f; // From viz.py arm_y_left/right

  Vector3 arm_pos_l = { arm_x - arm_offset * sinf(dozer->angular_z), arm_z, arm_y + arm_offset * cosf(dozer->angular_z) };
  draw_rectangular_prism(arm_pos_l, arm_size, raylib_yaw, raylib_pitch - theta_arm, raylib_roll, arm_color);

  Vector3 arm_pos_r = { arm_x + arm_offset * sinf(dozer->angular_z), arm_z, arm_y - arm_offset * cosf(dozer->angular_z) };
  draw_rectangular_prism(arm_pos_r, arm_size, raylib_yaw, raylib_pitch - theta_arm, raylib_roll, arm_color);

  // Blade
  Vector3 blade_pos = { dozer->blade_x, dozer->blade_z, dozer->blade_y };
  Vector3 blade_size = { 0.1f, dozer->blade_height, dozer->blade_width };

  float blade_yaw = -dozer->angular_z; 
  float blade_pitch = dozer->angular_y + dozer->pos_blade_pitch;
  float blade_roll = dozer->angular_x + dozer->pos_blade_roll;

  draw_rectangular_prism(blade_pos, blade_size, blade_yaw, blade_pitch, blade_roll, blade_color);
}

static inline void handle_input(SoilEnv* env)
{
  Dozer* dozer = &env->dozer;

  if (IsKeyPressed(KEY_R))
  {
    c_reset(env);
  }

  // Only Keyboard (Discard gamepad for now)
  env->actions[0] = 0;
  env->actions[1] = 0;
  env->actions[2] = 0;
  env->actions[3] = 0;
  env->actions[4] = 0;

  // Keyboard always acts as an active override
  // Map both WASD and Arrow Keys to movement just in case X11 is messing up the W keycode
  if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) env->actions[0] = 1.0f;
  if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) env->actions[0] = -1.0f;
  if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) env->actions[1] = 1.0f;
  if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) env->actions[1] = -1.0f;

  // Use I/K/J/L for the blade since Arrows are now used for movement fallback
  if (IsKeyDown(KEY_I)) env->actions[2] = 1.0f;
  if (IsKeyDown(KEY_K)) env->actions[2] = -1.0f;
  if (IsKeyDown(KEY_J)) env->actions[3] = 1.0f;
  if (IsKeyDown(KEY_L)) env->actions[3] = -1.0f;

  if (IsKeyDown(KEY_Q)) env->actions[4] = 1.0f;
  if (IsKeyDown(KEY_E)) env->actions[4] = -1.0f;
}

static inline void render_step(SoilEnv* env)
{
  Dozer* dozer = &env->dozer;

  camera.target = (Vector3){ dozer->position_x, dozer->position_z, dozer->position_y };
  float cam_dist = 12.0f;
  float cam_height = 8.0f;
  camera.position = (Vector3){
    dozer->position_x - cam_dist * cosf(dozer->angular_z),
    dozer->position_z + cam_height,
    dozer->position_y - cam_dist * sinf(dozer->angular_z)
  };
  BeginDrawing();
  ClearBackground(RAYWHITE);

  BeginMode3D(camera);
  draw_heightmap_fast(env);
  draw_dozer(env);
  EndMode3D();

  DrawFPS(10, 10);
  DrawText("R: Reset | WASD/Arrows: Move | I/K/J/L: Blade | QE: Roll", 10, 40, 20, DARKGRAY);

  DrawText("Keyboard Only Mode (Devcontainer)", 10, 70, 20, ORANGE);
  DrawText(TextFormat("Lin Vel: %.2f m/s | Yaw Vel: %.2f rad/s", dozer->twist_linear_x, dozer->twist_angular_z), 10, 100, 20, BLACK);
  DrawText(TextFormat("Arm Pos: %.2f | Vel: %.2f", dozer->pos_virtual_lift_arm, dozer->vel_virtual_lift_arm), 10, 130, 20, BLACK);
  DrawText(TextFormat("Pitch Pos: %.2f | Vel: %.2f", dozer->pos_blade_pitch, dozer->vel_blade_pitch), 10, 150, 20, BLACK);
  DrawText(TextFormat("Roll Pos: %.2f | Vel: %.2f", dozer->pos_blade_roll, dozer->vel_blade_roll), 10, 170, 20, BLACK);

  DrawText(TextFormat("Effort Lin: %.2f | Rot: %.2f", dozer->effort_linear, dozer->effort_rotational), 10, 200, 20, BLUE);
  DrawText(TextFormat("Effort Lift: %.2f | Pitch: %.2f | Roll: %.2f", dozer->effort_lift, dozer->effort_pitch, dozer->effort_roll), 10, 220, 20, BLUE);

  EndDrawing();
}

static inline void close_render()
{
  CloseWindow();
}

#endif
