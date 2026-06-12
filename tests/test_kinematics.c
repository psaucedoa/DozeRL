#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "../src/dozerl.h"

static int global_test_step = 0;

void write_test_frame(SoilEnv* env) {
    if (outfile) {
        struct { int step; float p[16]; float e[6]; int gs; float cs; } h = { 
            global_test_step, 
            {env->blade.loader_x, env->blade.loader_z, env->blade.loader_y, env->blade.yaw, env->blade.pitch, env->blade.roll, env->blade.arm_height, env->blade.vel_arm_height, env->blade.blade_pitch_rel, env->blade.vel_pitch_rel, env->blade.blade_roll_rel, env->blade.vel_roll_rel, env->blade.blade_yaw_rel, env->blade.vel_yaw_rel, env->blade.v_linear, env->blade.v_rotational}, 
            {env->blade.effort_linear, env->blade.effort_rotational, env->blade.effort_lift, env->blade.effort_pitch, env->blade.effort_roll, env->blade.effort_yaw},
            GRID_SIZE, 
            CELL_SIZE 
        };
        fwrite(&h, sizeof(h), 1, outfile);
        float flat[GRID_SIZE * GRID_SIZE];
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) flat[i * GRID_SIZE + j] = env->grid_H[i][j] + env->grid_L[i][j];
        }
        fwrite(flat, 4, GRID_SIZE * GRID_SIZE, outfile);
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) flat[i * GRID_SIZE + j] = env->grid_G[i][j];
        }
        fwrite(flat, 4, GRID_SIZE * GRID_SIZE, outfile);
        global_test_step++;
    }
}

int main() {
    printf("Starting test_kinematics...\n");
    fflush(stdout);
    outfile = fopen("out/sim_test.bin", "wb");
    if (!outfile) {
        printf("Failed to open out/sim_test.bin for writing. Does the 'out' directory exist?\n");
        return 1;
    }

    struct {
        float loader_length, loader_width, loader_height;
        float blade_height, blade_thickness, blade_width;
        float track_length, track_width, track_height, track_gauge;
        float arm_r, pivot_x, pivot_z, arm_y_left, arm_y_right;
        float blade_pitch_length;
    } geom = { 
        2.54f, 0.889f, 1.8669f, 
        0.76f, 0.05f, 1.85f, 
        TRACK_LENGTH, TRACK_WIDTH, 0.4064f, TRACK_GAUGE, 
        3.8f, -2.2f, 1.8987f, -0.5f, 0.5f,
        0.83019f
    };
    fwrite(&geom, sizeof(geom), 1, outfile);

    SoilEnv* env = calloc(1, sizeof(SoilEnv));
    env->observations = calloc(5011, sizeof(float));
    env->actions = calloc(6, sizeof(float));
    env->rewards = calloc(1, sizeof(float));
    env->terminals = calloc(1, sizeof(float));

    printf("Allocated env\n"); fflush(stdout);
    c_reset(env);
    printf("Finished c_reset\n"); fflush(stdout);
    env->step_num = 0;


    // 1. Default Start Position ----------------------------------------------------
    printf("1. Default Start Position\n");
    fflush(stdout);
    for(int i=0; i<30; i++) write_test_frame(env);


    // 2. Zero Position ----------------------------------------------------
    printf("2. Zero Position\n");
    fflush(stdout);
    env->blade.arm_height = 0.0f;
    env->blade.blade_pitch_rel = 0.0f;
    env->blade.blade_roll_rel = 0.0f;
    env->blade.blade_yaw_rel = 0.0f;
    update_kinematics(env);
    for(int i=0; i<30; i++) write_test_frame(env);

    #define RESET_JOINTS() \
        env->blade.arm_height = 0.0f; \
        env->blade.blade_pitch_rel = 0.0f; \
        env->blade.blade_roll_rel = 0.0f; \
        env->blade.blade_yaw_rel = 0.0f;


    // 3. Lift (Max/Min) ----------------------------------------------------
    printf("3. Lift (Max/Min)\n");
    fflush(stdout);
    RESET_JOINTS();

    env->blade.arm_height = 0.6f;
    update_kinematics(env);
    for(int i=0; i<30; i++) write_test_frame(env);
    RESET_JOINTS();

    env->blade.arm_height = -0.4f;
    update_kinematics(env);
    for(int i=0; i<30; i++) write_test_frame(env);


    // 4. Pitch (Max/Min) ----------------------------------------------------
    printf("4. Pitch (Max/Min)\n");
    fflush(stdout);
    RESET_JOINTS();

    env->blade.blade_pitch_rel = 0.6f;
    update_kinematics(env);
    for(int i=0; i<30; i++) write_test_frame(env);
    RESET_JOINTS();

    env->blade.blade_pitch_rel = -0.6f;
    update_kinematics(env);
    for(int i=0; i<30; i++) write_test_frame(env);


    // 5. Roll (Max/Min) ----------------------------------------------------
    printf("5. Roll (Max/Min)\n"); fflush(stdout);
    RESET_JOINTS();

    env->blade.blade_roll_rel = 0.4f;
    update_kinematics(env);
    for(int i=0; i<30; i++) write_test_frame(env);
    RESET_JOINTS();

    env->blade.blade_roll_rel = -0.4f;
    update_kinematics(env);
    for(int i=0; i<30; i++) write_test_frame(env);


    // 6. Yaw (Max/Min) ----------------------------------------------------
    printf("6. Yaw (Max/Min)\n");
    fflush(stdout);
    RESET_JOINTS();

    env->blade.blade_yaw_rel = 0.5f;
    update_kinematics(env);
    for(int i=0; i<30; i++) write_test_frame(env);
    RESET_JOINTS();

    env->blade.blade_yaw_rel = -0.5f;
    update_kinematics(env);
    for(int i=0; i<30; i++) write_test_frame(env);


    // 5. Soil Interaction (blade level, slightly beneath ground)
    printf("5. Soil Interaction (blade level, beneath ground)\n");
    c_reset(env);
    env->blade.effort_linear = 1.0f; // move forward 2 meters into the map
    env->blade.arm_height = -0.1f; // lower arm slightly beneath ground
    env->blade.blade_pitch_rel = -0.785f; // pitch forward 45 deg to exactly cancel base rake
    env->blade.blade_roll_rel = 0.0f;
    env->blade.blade_yaw_rel = 0.0f;
    update_kinematics(env);
    for(int i=0; i<300; i++) write_test_frame(env);

    fclose(outfile);
    free(env);
    printf("Done. Run 'python viz.py' to view the 5 test configurations.\n");
    return 0;
}
