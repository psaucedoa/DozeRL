#include "dozerl.h"
#include "puffernet.h"
#include "raylib_render.h"
#include <stdio.h>
#include <time.h>

void demo(const char* checkpoint_path) {
    SoilEnv* env = (SoilEnv*)malloc(sizeof(SoilEnv));
    env->observations = (float*)calloc(5011, sizeof(float));
    env->actions = (float*)calloc(6, sizeof(float));
    env->rewards = (float*)calloc(1, sizeof(float));
    env->terminals = (float*)calloc(1, sizeof(float));
    env->rng = 42; 

    // Setup initial properties
    env->loose_soil_density = 1200.0f;
    env->soil_gamma = 15000.0f;
    env->soil_c = 10000.0f;
    env->soil_phi = 30.0f * M_PI / 180.0f;
    env->swell_ratio = 1.2f;
    
    env->dozer.track_length = 2.0f;
    env->dozer.track_width = 2.0f;
    env->dozer.blade_width = 2.2f;
    env->dozer.blade_height = 0.8f;
    
    for(int i = 0; i < GRID_SIZE; i++) {
        for(int j = 0; j < GRID_SIZE; j++) {
            env->grid_H[i][j] = 1.0f; 
        }
    }

    c_reset(env);
    
    env->dozer.position_x = (GRID_SIZE * CELL_SIZE) / 2.0f;
    env->dozer.position_y = (GRID_SIZE * CELL_SIZE) / 2.0f;
    env->dozer.position_z = 1.0f;

    Weights* weights = NULL;
    PufferNet* net = NULL;

    if (checkpoint_path != NULL) {
        printf("Loading trained policy weights from %s...\n", checkpoint_path);
        weights = load_weights(checkpoint_path);
        if (weights != NULL) {
            int logit_sizes[6] = {1, 1, 1, 1, 1, 1};
            net = make_puffernet(weights, 1, 5011, 128, 2, logit_sizes, 6);
            printf("PufferNet policy successfully initialized in C.\n");
        }
    } else {
        printf("No checkpoint path provided. Running interactive Raylib demo...\n");
    }

    init_render();

    while (!WindowShouldClose()) {
        if (net != NULL) {
            forward_puffernet(net, env->observations, env->actions);
        } else {
            if (IsGamepadAvailable(0)) {
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
            } else {
                env->actions[0] = 0; env->actions[1] = 0; env->actions[2] = 0; env->actions[3] = 0; env->actions[4] = 0;
                if (IsKeyDown(KEY_W)) env->actions[0] = 1.0f;
                if (IsKeyDown(KEY_S)) env->actions[0] = -1.0f;
                if (IsKeyDown(KEY_A)) env->actions[1] = -1.0f;
                if (IsKeyDown(KEY_D)) env->actions[1] = 1.0f;
                if (IsKeyDown(KEY_UP)) env->actions[2] = 1.0f;
                if (IsKeyDown(KEY_DOWN)) env->actions[2] = -1.0f;
            }
        }
        
        c_step(env);
        render_step(env);
    }
    
    close_render();

    if (net != NULL) free_puffernet(net);
    if (weights != NULL) free(weights);

    free(env->observations);
    free(env->actions);
    free(env->rewards);
    free(env->terminals);
    free(env);
}

int main(int argc, char** argv) {
    const char* checkpoint_path = NULL;
    if (argc > 1) {
        checkpoint_path = argv[1];
    }
    demo(checkpoint_path);
    return 0;
}
