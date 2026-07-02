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

static inline void set_hard_soil_color(float h)
{
  unsigned char r = (unsigned char)(130.0f + h * 20.0f);
  unsigned char g = (unsigned char)(85.0f +  h * 20.0f);
  unsigned char b = (unsigned char)(45.0f +  h * 20.0f);
  rlColor4ub(r, g, b, 255);
}

static inline void set_loose_soil_color(float h)
{
  unsigned char r = (unsigned char)(70.0f + h * 20.0f);
  unsigned char g = (unsigned char)(40.0f + h * 20.0f);
  unsigned char b = (unsigned char)(20.0f + h * 20.0f);
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

// raylib has a weird coordinate system (x, -z, y)
static inline void draw_rectangular_prism(Vector3 position, Vector3 rotation, Vector3 size, Color color)
{
  rlPushMatrix();
  rlTranslatef(position.x, position.z, position.y);
  rlRotatef(rotation.x * RAD2DEG, 1.0f, 0.0f, 0.0f);  // Pitch around X
  rlRotatef(-rotation.z * RAD2DEG, 0.0f, 1.0f, 0.0f);  // Yaw around Y
  rlRotatef(rotation.y * RAD2DEG, 0.0f, 0.0f, 1.0f);  // Roll around Z

  DrawCube((Vector3){0,0,0}, size.x, size.z, size.y, color);
  DrawCubeWires((Vector3){0,0,0}, size.x, size.z, size.y, BLACK);

  rlPopMatrix();
}

static inline void draw_dozer(SoilEnv* env)
{
  Dozer* dozer = &env->dozer;

  Color chassis_color = (Color){210, 180, 140, 055};      // Desert Tan
  Color arm_color = (Color){180, 150, 110, 255};          // Darker Desert Tan
  Color track_color = (Color){60, 60, 60, 255};           // Dark Grey
  Color blade_color = (Color){80, 80, 80, 255};           // Dark Grey
  Color yellow = (Color){255, 255, 0, 255};  //yellow

  Vector3 pitch_joint = {0.0f, 0.0f, 0.0f};
  Vector3 u_joint     = {0.0f, 0.0f, 0.0f};
  Vector3 blade_edge  = {0.0f, 0.0f, 0.0f};

  float track_height   = 0.41f;  // m
  float chassis_length = 2.54f;  // m
  float chassis_width  = 1.00f;  // m
  float chassis_height = 1.87f;  // m
  float gauge_offset = dozer->track_gauge * 0.5f;
  float track_offset = dozer->track_width * 0.5f;

  // Chassis
  // Vector3 chassis_size = {chassis_length, chassis_width, chassis_height};  // x, y, z
  Vector3 chassis_size = {1.5f, 1.5f, 1.5f};
  Vector3 chassis_pos = {dozer->position_x, dozer->position_y, dozer->position_z + 0.75f};
  Vector3 chassis_rot = {dozer->angular_x, dozer->angular_y, dozer->angular_z};
  draw_rectangular_prism(chassis_pos, chassis_rot, chassis_size, yellow);

  // arm lift joint
  Vector3 joint_size = {0.5f, 0.5f, 0.5f};
  Vector3 arm_joint_pos = {dozer->_lift_arm_joint_pose[0], dozer->_lift_arm_joint_pose[1], dozer->_lift_arm_joint_pose[2]};
  Vector3 arm_joint_rot = {dozer->_lift_arm_joint_pose[3], dozer->_lift_arm_joint_pose[4], dozer->_lift_arm_joint_pose[5]};
  draw_rectangular_prism(arm_joint_pos, arm_joint_rot, joint_size, yellow);

  // pitch joint
  Vector3 pitch_joint_pos = {dozer->_pitch_joint_pose[0], dozer->_pitch_joint_pose[1], dozer->_pitch_joint_pose[2]};
  Vector3 pitch_joint_rot = {dozer->_pitch_joint_pose[3], dozer->_pitch_joint_pose[4], dozer->_pitch_joint_pose[5]};
  draw_rectangular_prism(pitch_joint, pitch_joint_rot, joint_size, yellow);

  // u_joint
  Vector3 u_joint_pos = {dozer->_u_joint_pose[0], dozer->_u_joint_pose[1], dozer->_u_joint_pose[2]};
  Vector3 u_joint_rot = {dozer->_u_joint_pose[3], dozer->_u_joint_pose[4], dozer->_u_joint_pose[5]};
  draw_rectangular_prism(u_joint_pos, u_joint_rot, joint_size, yellow);

  // blade_edge
  Vector3 blade_edge_pos = {dozer->_blade_edge_pose[0], dozer->_blade_edge_pose[1], dozer->_blade_edge_pose[2]};
  Vector3 blade_edge_rot = {dozer->_blade_edge_pose[3], dozer->_blade_edge_pose[4], dozer->_blade_edge_pose[5]};
  draw_rectangular_prism(blade_edge_pos, blade_edge_rot, joint_size, yellow);

//   draw_rectangular_prism(track_pos_r_local, track_size, global_translation, global_rotation, track_color);
}

static inline void render_step(SoilEnv* env)
{
  Dozer* dozer = &env->dozer;

  camera.target = (Vector3){ dozer->position_x, dozer->position_z, dozer->position_y };
  float cam_dist = 12.0f;
  float cam_height = 8.0f;
  camera.position = (Vector3){
    0, //dozer->position_x - cam_dist * cosf(dozer->angular_z),
    8, //dozer->position_z + cam_height,
    0, //dozer->position_y - cam_dist * sinf(dozer->angular_z)
  };
  BeginDrawing();
  ClearBackground(RAYWHITE);

  BeginMode3D(camera);
  draw_heightmap_fast(env);
  draw_dozer(env);
  EndMode3D();

  DrawFPS(10, 10);
  DrawText("R: Reset | WASD/Arrows: Move | I/K/J/L: Blade | QE: Roll", 10, 40, 20, DARKGRAY);

  if (IsGamepadAvailable(0)) 
  {
    DrawText("Gamepad", 10, 70, 20, GREEN);
  }
  else
  {
    DrawText("Keyboard", 10, 70, 20, ORANGE);
  }

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
