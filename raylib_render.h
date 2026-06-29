#ifndef RAYLIB_RENDER_H
#define RAYLIB_RENDER_H

#include "dozerl.h"
#include <raylib.h>
#include <rlgl.h>

static Camera3D camera = { 0 };

static inline void init_render() {
    InitWindow(1280, 720, "DozeRL Simulator");
    SetTargetFPS(60);

    camera.position = (Vector3){ 15.0f, 15.0f, 15.0f }; // Camera position
    camera.target = (Vector3){ 5.0f, 0.0f, 5.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (Y is up in raylib)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera mode type
}

static inline void draw_heightmap(SoilEnv* env) {
    float cell_w = CELL_SIZE;
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            float h_hard = env->grid_H[i][j];
            float h_loose = env->grid_L[i][j];
            
            // Env coords: X, Y (horizontal), Z (up)
            // Raylib coords: X, Z (horizontal), Y (up)
            float x = i * cell_w;
            float z = j * cell_w;
            
            if (h_hard > 0.0f) {
                Vector3 posH = { x, h_hard / 2.0f, z };
                DrawCube(posH, cell_w, h_hard, cell_w, DARKBROWN);
            }
            
            if (h_loose > 0.0f) {
                Vector3 posL = { x, h_hard + h_loose / 2.0f, z };
                DrawCube(posL, cell_w, h_loose, cell_w, ORANGE);
            }
        }
    }
}

static inline void draw_rectangular_prism(Vector3 center, Vector3 size, float yaw, float pitch, float roll, Color color) {
    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);
    // Raylib rotations (assuming yaw->pitch->roll)
    rlRotatef(yaw * RAD2DEG, 0.0f, 1.0f, 0.0f);   // Yaw around Y
    rlRotatef(pitch * RAD2DEG, 1.0f, 0.0f, 0.0f); // Pitch around X
    rlRotatef(roll * RAD2DEG, 0.0f, 0.0f, 1.0f);  // Roll around Z
    
    DrawCube((Vector3){0,0,0}, size.x, size.y, size.z, color);
    DrawCubeWires((Vector3){0,0,0}, size.x, size.y, size.z, BLACK);
    
    rlPopMatrix();
}

static inline void draw_dozer(SoilEnv* env) {
    Dozer* dozer = &env->dozer;
    
    // Chassis
    Vector3 chassis_pos = { dozer->position_x, dozer->position_z + 0.5f, dozer->position_y };
    Vector3 chassis_size = { dozer->track_length, 1.0f, dozer->track_width };
    
    float raylib_yaw = -dozer->angular_z; 
    float raylib_pitch = dozer->angular_y;
    float raylib_roll = dozer->angular_x;

    draw_rectangular_prism(chassis_pos, chassis_size, raylib_yaw, raylib_pitch, raylib_roll, GRAY);

    // Virtual Arm
    float theta_arm = dozer->pos_virtual_lift_arm;
    float arm_x = dozer->position_x + (dozer->arm_pivot_x + dozer->arm_length * 0.5f * cosf(theta_arm)) * cosf(dozer->angular_z);
    float arm_y = dozer->position_y + (dozer->arm_pivot_x + dozer->arm_length * 0.5f * cosf(theta_arm)) * sinf(dozer->angular_z);
    float arm_z = dozer->position_z + dozer->arm_pivot_z + dozer->arm_length * 0.5f * sinf(theta_arm);
    
    Vector3 arm_pos = { arm_x, arm_z, arm_y };
    Vector3 arm_size = { dozer->arm_length, 0.2f, dozer->blade_width * 0.8f };
    draw_rectangular_prism(arm_pos, arm_size, raylib_yaw, raylib_pitch - theta_arm, raylib_roll, DARKGRAY);

    // Blade
    Vector3 blade_pos = { dozer->blade_x, dozer->blade_z, dozer->blade_y };
    Vector3 blade_size = { 0.1f, dozer->blade_height, dozer->blade_width };
    
    float blade_yaw = -dozer->angular_z; 
    float blade_pitch = dozer->angular_y + dozer->pos_blade_pitch;
    float blade_roll = dozer->angular_x + dozer->pos_blade_roll;

    draw_rectangular_prism(blade_pos, blade_size, blade_yaw, blade_pitch, blade_roll, YELLOW);
}

static inline void render_step(SoilEnv* env) {
    Dozer* dozer = &env->dozer;
    
    // Follow camera
    camera.target = (Vector3){ dozer->position_x, 0.0f, dozer->position_y };
    camera.position = (Vector3){ dozer->position_x - 10.0f, 8.0f, dozer->position_y };
    
    UpdateCamera(&camera, CAMERA_THIRD_PERSON);

    BeginDrawing();
    ClearBackground(RAYWHITE);
    
    BeginMode3D(camera);
    
    draw_heightmap(env);
    draw_dozer(env);
    
    DrawGrid(GRID_SIZE, CELL_SIZE);
    
    EndMode3D();
    
    DrawFPS(10, 10);
    DrawText("Use Xbox Controller to operate Dozer", 10, 40, 20, DARKGRAY);
    EndDrawing();
}

static inline void close_render() {
    CloseWindow();
}

#endif
