#ifndef RAYLIB_RENDER_H
#define RAYLIB_RENDER_H

#include "dozerl.h"
#include <raylib.h>
#include <rlgl.h>

static Camera3D camera = { 0 };
static bool show_goal = false;  // toggled with 'G': translucent goal-map overlay
static int camera_view = 0; // 0: x-y top, 1: y-z side, 2: x-z front — cycled with 'C'

static inline void init_render()
{
  InitWindow(1920, 1080, "DozeRL Simulator");
  SetTargetFPS(60);

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
  EndMode3D();

  DrawFPS(10, 10);
  DrawText("R: Reset | WASD/Arrows: Move | I/K/J/L: Blade | QE: Roll | G: Goal | C: View", 10, 40, 20, DARKGRAY);
  DrawText(TextFormat("Terrain Error: %.2f / %.2f  Perf: %.1f%%  Return: %.1f", env->cur_error, env->initial_error, env->log.perf*100.0f, env->episode_return), 10, 70, 20, MAROON);

  DrawText(TextFormat("Lin Vel: %.2f m/s | Yaw Vel: %.2f rad/s", dozer->twist_linear_x, dozer->twist_angular_z), 10, 100, 20, BLACK);
  DrawText(TextFormat("Arm Pos: %.2f | Vel: %.2f", dozer->pos_virtual_lift_arm, dozer->vel_virtual_lift_arm), 10, 130, 20, BLACK);
  DrawText(TextFormat("Pitch Pos: %.2f | Vel: %.2f", dozer->pos_blade_pitch, dozer->vel_blade_pitch), 10, 150, 20, BLACK);
  DrawText(TextFormat("Roll Pos: %.2f | Vel: %.2f", dozer->pos_blade_roll, dozer->vel_blade_roll), 10, 170, 20, BLACK);
  DrawText(TextFormat("X: %.2f | Y: %.2f", dozer->position_x, dozer->position_y), 10, 190, 20, BLACK);

  DrawText(TextFormat("Effort Lin: %.2f | Rot: %.2f", dozer->effort_linear, dozer->effort_rotational), 10, 220, 20, BLUE);
  DrawText(TextFormat("Effort Lift: %.2f | Pitch: %.2f | Roll: %.2f", dozer->effort_lift, dozer->effort_pitch, dozer->effort_roll), 10, 240, 20, BLUE);

  DrawText(TextFormat("Surcharge_q: %.2f", dozer->blade_surcharge_Q), 10, 290, 20, BLACK);
  DrawText(TextFormat("Tracks Lin: %.2f m/s | Tracks Rot: %.2f rad/s", dozer->vel_tracks_linear, dozer->vel_tracks_rotational), 10, 310, 20, BLACK);

  DrawText(TextFormat("Goal overlay (G): %s", show_goal ? "ON" : "OFF"), 10, 360, 20, show_goal ? GREEN : GRAY);
  DrawText(TextFormat("View (C): %s", camera_view==0 ? "X-Y TOP" : camera_view==1 ? "Y-Z SIDE" : "X-Z FRONT"), 10, 380, 20, DARKGRAY);

  EndDrawing();
}

static inline void close_render()
{
  CloseWindow();
}

#endif
