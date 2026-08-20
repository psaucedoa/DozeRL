#ifndef RAYLIB_RENDER_H
#define RAYLIB_RENDER_H

#include "dozerl.h"
#include <raylib.h>
#include <rlgl.h>

static Camera3D camera = { 0 };
static bool show_goal = false;  // toggled with 'G': translucent goal-map overlay
static bool show_obs = false;  // toggled with 'O': translucent goal-map overlay
static int camera_view = 0; // 0: x-y top, 1: y-z side, 2: x-z front — cycled with 'C'
Font ibm_mono;

static inline void init_render()
{
  InitWindow(1920, 1080, "DozeRL Simulator");
  ibm_mono = LoadFontEx("/workspaces/puffertank/pufferlib/resources/fonts/IBM_Plex_Mono/IBMPlexMono-Medium.ttf", 64, 0, 250);
  SetTargetFPS(50);

  camera.position = (Vector3){ 15.0f, 15.0f, 15.0f };
  camera.target = (Vector3){ 5.0f, 0.0f, 5.0f };
  camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;
}

static inline void set_hard_soil_color(float h)
{
  unsigned char r = (unsigned char)(130.0f + h * 20.0f);
  unsigned char g = (unsigned char)(85.0f +  h * 20.0f);
  unsigned char b = (unsigned char)(45.0f +  h * 20.0f);
  rlColor4ub(r, g, b, 255);
}

static inline void set_loose_soil_color(float h)
{
  unsigned char r = (unsigned char)(70.0f + h * 30.0f);
  unsigned char g = (unsigned char)(40.0f + h * 30.0f);
  unsigned char b = (unsigned char)(20.0f + h * 30.0f);
  rlColor4ub(r, g, b, 255);
}

static inline void draw_heightmap_fast(SoilEnv* env)
{
  float cw = CELL_SIZE;

  // Draw hard soil (base terrain)
  rlBegin(RL_QUADS);
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

      set_hard_soil_color(h00);
      rlVertex3f(x0, h00, z0);

      set_hard_soil_color(h01);
      rlVertex3f(x0, h01, z1);

      set_hard_soil_color(h11);
      rlVertex3f(x1, h11, z1);

      set_hard_soil_color(h10);
      rlVertex3f(x1, h10, z0);
    }
  }
  rlEnd();

  // Draw loose soil on top
  rlBegin(RL_QUADS);
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

      set_loose_soil_color(h00);
      rlVertex3f(x0, h00, z0);

      set_loose_soil_color(h01);
      rlVertex3f(x0, h01, z1);

      set_loose_soil_color(h11);
      rlVertex3f(x1, h11, z1);

      set_loose_soil_color(h10);
      rlVertex3f(x1, h10, z0);
    }
  }
  rlEnd();
}

static inline void set_goal_color(char region)
{
  if (region == 1)      rlColor4ub(40, 120, 255, 120);   // 1 = Cut (slot)  -> translucent blue
  else if (region == 2) rlColor4ub(40, 220, 120, 120);   // 2 = Fill (pile) -> translucent green
  else                  rlColor4ub(200, 200, 200, 60);    // neutral (rarely drawn)
}

// Translucent "ghost" of the target heightmap (grid_G). Rendered with depth test disabled so the
// slot (which sits below the flat starting terrain) is visible through the ground as an x-ray.
// Only the worked cells (map_region != 0) are drawn, so the flat neutral goal doesn't clutter.
static inline void draw_goal_map(SoilEnv* env)
{
  float cw = CELL_SIZE;

  rlDisableDepthTest();   // x-ray: draw over the terrain regardless of occlusion
  rlDisableDepthMask();   // don't write depth; keep it a pure overlay
  rlSetBlendMode(RL_BLEND_ALPHA);

  rlBegin(RL_QUADS);
  for (int i = 0; i < GRID_SIZE - 1; i++)
  {
    for (int j = 0; j < GRID_SIZE - 1; j++)
    {
      char r00 = env->map_region[i][j];
      char r10 = env->map_region[i+1][j];
      char r01 = env->map_region[i][j+1];
      char r11 = env->map_region[i+1][j+1];
      if (r00 == 0 && r10 == 0 && r01 == 0 && r11 == 0) continue;  // skip untouched terrain

      float h00 = env->grid_G[i][j];
      float h10 = env->grid_G[i+1][j];
      float h01 = env->grid_G[i][j+1];
      float h11 = env->grid_G[i+1][j+1];

      float x0 = i * cw, x1 = (i+1) * cw;
      float z0 = j * cw, z1 = (j+1) * cw;

      set_goal_color(r00); rlVertex3f(x0, h00, z0);
      set_goal_color(r01); rlVertex3f(x0, h01, z1);
      set_goal_color(r11); rlVertex3f(x1, h11, z1);
      set_goal_color(r10); rlVertex3f(x1, h10, z0);
    }
  }
  rlEnd();

  rlSetBlendMode(RL_BLEND_ALPHA);
  rlEnableDepthMask();
  rlEnableDepthTest();
}

// raylib has a weird coordinate system (x, -z, y)
static inline void draw_rectangular_prism(Vector3 position, Vector3 rotation, Vector3 size, Color color)
{
  rlPushMatrix();
  rlTranslatef(position.x, position.z, position.y);
  rlRotatef(-rotation.z * RAD2DEG, 0.0f, 1.0f, 0.0f);  // Yaw around Y
  rlRotatef(-rotation.y * RAD2DEG, 0.0f, 0.0f, 1.0f);  // Pitch around Z
  rlRotatef(-rotation.x * RAD2DEG, 1.0f, 0.0f, 0.0f);  // Roll around X

  DrawCube((Vector3){0,0,0}, size.x, size.z, size.y, color);
  DrawCubeWires((Vector3){0,0,0}, size.x, size.z, size.y, BLACK);

  rlPopMatrix();
}

static inline void draw_obs(SoilEnv* env)
{
  Dozer* dozer = &env->dozer;
  Color obs = (Color){120, 80, 255, 50};  //see thru

  // obs viz
  Vector3 obs_pos = {dozer->position_x, dozer->position_y, 2.0f};
  Vector3 obs_rot = {0.0f, 0.0f, dozer->angular_z};
  Vector3 obs_siz = {20.0f, 20.0f, 0.1f};
  draw_rectangular_prism(obs_pos, obs_rot, obs_siz, obs);
}

static inline void draw_dozer(SoilEnv* env)
{
  Dozer* dozer = &env->dozer;

  Color yellow = (Color){255, 255, 0, 255};  //yellow

  Vector3 joint_size = {0.5f, 0.5f, 0.5f};

  // Chassis
  Vector3 chassis_pos = {dozer->position_x, dozer->position_y, dozer->position_z};
  Vector3 chassis_rot = {dozer->angular_x, -dozer->angular_y, dozer->angular_z};
  draw_rectangular_prism(chassis_pos, chassis_rot, joint_size, yellow);

  // arm lift joint
  Vector3 arm_joint_pos = {dozer->_lift_arm_joint_pose[0], dozer->_lift_arm_joint_pose[1], dozer->_lift_arm_joint_pose[2]};
  Vector3 arm_joint_rot = {dozer->_lift_arm_joint_pose[3], dozer->_lift_arm_joint_pose[4], dozer->_lift_arm_joint_pose[5]};
  draw_rectangular_prism(arm_joint_pos, arm_joint_rot, joint_size, yellow);

  // arm
  // Vector3 arm_size = {0.5f, 0.5f, 0.5f};
  // Vector3 arm_joint_pos = {dozer->_lift_arm_joint_pose[0], dozer->_lift_arm_joint_pose[1], dozer->_lift_arm_joint_pose[2]};
  // Vector3 arm_joint_rot = {dozer->_lift_arm_joint_pose[3], dozer->_lift_arm_joint_pose[4], dozer->_lift_arm_joint_pose[5]};
  // draw_rectangular_prism(arm_joint_pos, arm_joint_rot, joint_size, yellow);

  // pitch joint
  Vector3 pitch_joint_pos = {dozer->_pitch_joint_pose[0], dozer->_pitch_joint_pose[1], dozer->_pitch_joint_pose[2]};
  Vector3 pitch_joint_rot = {dozer->_pitch_joint_pose[3], dozer->_pitch_joint_pose[4], dozer->_pitch_joint_pose[5]};
  draw_rectangular_prism(pitch_joint_pos, pitch_joint_rot, joint_size, yellow);

  // u_joint
  Vector3 u_joint_pos = {dozer->_u_joint_pose[0], dozer->_u_joint_pose[1], dozer->_u_joint_pose[2]};
  Vector3 u_joint_rot = {dozer->_u_joint_pose[3], dozer->_u_joint_pose[4], dozer->_u_joint_pose[5]};
  Vector3 blade_size = {0.2, dozer->blade_width, dozer->blade_height};
  draw_rectangular_prism(u_joint_pos, u_joint_rot, blade_size, yellow);

  // blade_edge
  // Vector3 blade_edge_pos = {dozer->_blade_edge_pose[0], dozer->_blade_edge_pose[1], dozer->_blade_edge_pose[2]};
  // Vector3 blade_edge_rot = {dozer->_blade_edge_pose[3], dozer->_blade_edge_pose[4], dozer->_blade_edge_pose[5]};
  // draw_rectangular_prism(blade_edge_pos, blade_edge_rot, joint_size, yellow);
}

static inline void render_step(SoilEnv* env)
{
  Dozer* dozer = &env->dozer;

  if (IsKeyPressed(KEY_G)) show_goal = !show_goal;
  if (IsKeyPressed(KEY_O)) show_obs = !show_obs;
  if (IsKeyPressed(KEY_C)) camera_view = (camera_view + 1) % 3;

  // 3 views, all centered on middle of grid world (30x30m, center 15,15)
  float center = GRID_SIZE * CELL_SIZE * 0.5f;
  if (camera_view == 0) { // x-y plane — top-down (look down Y / world Z)
    camera.target = (Vector3){ center, 0.0f, center };
    camera.position = (Vector3){ center, 40.0f, center };
    camera.up = (Vector3){ 0.0f, 0.0f, -1.0f };
  } else if (camera_view == 1) { // y-z plane — side view along +X
    camera.target = (Vector3){ center, 1.0f, center };
    camera.position = (Vector3){ center - 50.0f, 2.0f, center };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
  } else { // x-z plane — front view along +Z (Y world)
    camera.target = (Vector3){ center, 1.0f, center };
    camera.position = (Vector3){ center, 2.0f, center - 50.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
  }
  BeginDrawing();
  ClearBackground(RAYWHITE);

  BeginMode3D(camera);
  draw_heightmap_fast(env);
  draw_dozer(env);
  if (show_goal) draw_goal_map(env);
  if (show_obs) draw_obs(env);
  EndMode3D();

  DrawFPS(10, 10);

  const float font_size = 20.0f;
  const float font_spacing = 1.0f;

  DrawTextEx(ibm_mono, "R: Reset | WASD/Arrows: Move | I/K/J/L: Blade | QE: Roll | G: Goal | C: View", (Vector2){ 10.0f, 20.0f }, font_size, font_spacing, DARKGRAY);
  DrawTextEx(ibm_mono, TextFormat("Global Error : %.2f / %.2f", env->map_error_absolute, env->initial_error_absolute), (Vector2){ 10.0f, 60.0f }, font_size, font_spacing, MAROON);
  DrawTextEx(ibm_mono, TextFormat("Goal   Error : %.2f / %.2f", env->map_error_goal, env->initial_error_absolute), (Vector2){ 10.0f, 80.0f }, font_size, font_spacing, MAROON);
  DrawTextEx(ibm_mono, TextFormat("Perf         : %.1f%%", env->log.perf*100.0f), (Vector2){ 10.0f, 100.0f }, font_size, font_spacing, MAROON);
  DrawTextEx(ibm_mono, TextFormat("Return       : %.1f", env->episode_return), (Vector2){ 10.0f, 120.0f }, font_size, font_spacing, MAROON);

  DrawTextEx(ibm_mono, TextFormat("Pos Arm   : %.2f | Vel : %.2f", dozer->pos_virtual_lift_arm, dozer->vel_virtual_lift_arm), (Vector2){ 10.0f, 160.0f }, font_size, font_spacing, BLACK);
  DrawTextEx(ibm_mono, TextFormat("Pos Pitch : %.2f | Vel : %.2f", dozer->pos_blade_pitch, dozer->vel_blade_pitch), (Vector2){ 10.0f, 180.0f }, font_size, font_spacing, BLACK);
  DrawTextEx(ibm_mono, TextFormat("Pos Roll  : %.2f | Vel : %.2f", dozer->pos_blade_roll, dozer->vel_blade_roll), (Vector2){ 10.0f, 200.0f }, font_size, font_spacing, BLACK);
  DrawTextEx(ibm_mono, TextFormat("X: %.2f | Y: %.2f", dozer->position_x, dozer->position_y), (Vector2){ 10.0f, 220.0f }, font_size, font_spacing, BLACK);

  DrawTextEx(ibm_mono, TextFormat("Effort Lin   : %.2f", dozer->effort_linear), (Vector2){ 10.0f, 260.0f }, font_size, font_spacing, BLUE);
  DrawTextEx(ibm_mono, TextFormat("Effort Rot   : %.2f", dozer->effort_rotational), (Vector2){ 10.0f, 280.0f }, font_size, font_spacing, BLUE);
  DrawTextEx(ibm_mono, TextFormat("Effort Lift  : %.2f", dozer->effort_lift), (Vector2){ 10.0f, 300.0f }, font_size, font_spacing, BLUE);
  DrawTextEx(ibm_mono, TextFormat("Effort Pitch : %.2f", dozer->effort_pitch), (Vector2){ 10.0f, 320.0f }, font_size, font_spacing, BLUE);
  DrawTextEx(ibm_mono, TextFormat("Effort Roll  : %.2f", dozer->effort_roll), (Vector2){ 10.0f, 340.0f }, font_size, font_spacing, BLUE);

  DrawTextEx(ibm_mono, TextFormat("Surcharge_q : %.2f", dozer->blade_surcharge_Q), (Vector2){ 10.0f, 380.0f }, font_size, font_spacing, BLACK);
  DrawTextEx(ibm_mono, TextFormat("Tracks Lin  : %.2f m/s", dozer->vel_tracks_linear), (Vector2){ 10.0f, 400.0f }, font_size, font_spacing, BLACK);
  DrawTextEx(ibm_mono, TextFormat("Tracks Rot  : %.2f rad/s", dozer->vel_tracks_rotational), (Vector2){ 10.0f, 420.0f }, font_size, font_spacing, BLACK);

  DrawTextEx(ibm_mono, TextFormat("r_progress   : %.2f", env->log.r_progress), (Vector2){ 10.0f, 460.0f }, font_size, font_spacing, BLACK);
  DrawTextEx(ibm_mono, TextFormat("r_goal_obs   : %.2f", env->log.r_goal_obs), (Vector2){ 10.0f, 480.0f }, font_size, font_spacing, BLACK);
  DrawTextEx(ibm_mono, TextFormat("r_push       : %.2f", env->log.r_push), (Vector2){ 10.0f, 500.0f }, font_size, font_spacing, BLACK);
  DrawTextEx(ibm_mono, TextFormat("r_stationary : %.2f", env->log.r_stationary), (Vector2){ 10.0f, 520.0f }, font_size, font_spacing, BLACK);

  DrawTextEx(ibm_mono, TextFormat("Goal overlay (G): %s", show_goal ? "ON" : "OFF"), (Vector2){ 10.0f, 1000.0f }, font_size, font_spacing, show_goal ? GREEN : GRAY);
  DrawTextEx(ibm_mono, TextFormat("View (C): %s", camera_view==0 ? "X-Y TOP" : camera_view==1 ? "Y-Z SIDE" : "X-Z FRONT"), (Vector2){ 10.0f, 1020.0f }, font_size, font_spacing, DARKGRAY);

  EndDrawing();
}

static inline void close_render()
{
  CloseWindow();
}

#endif
