#include "dsm.h"

int main() {
    // Simulation parameters
    const int width = 256;
    const int height = 256;
    const int num_agents = 1;
    const int horizon = 1024;
    const float agent_speed = 1;
    const int vision = 5;
    const bool discretize = true;

    const int render_cell_size = 4;
    const int seed = 42;

    // Initialize environment
    Env* env = alloc_room_env();
    reset_room(env);
 
    // Initialize Raylib renderer
    Renderer* renderer = init_renderer(render_cell_size, width, height);

    // Main simulation loop
    while (!WindowShouldClose()) {
        // User Input Handling: Map keys to environment actions
        env->action = PASS;
        if (IsKeyDown(KEY_W)) env->action = SPEED_UP;
        if (IsKeyDown(KEY_S)) env->action = SPEED_DOWN;
        if (IsKeyDown(KEY_A)) env->action = LEFT;
        if (IsKeyDown(KEY_D)) env->action = RIGHT;
        if (IsKeyDown(KEY_Q)) env->action = YAW_LEFT;
        if (IsKeyDown(KEY_E)) env->action = YAW_RIGHT;
        if (IsKeyDown(KEY_DOWN)) env->action = BLADE_DOWN;
        if (IsKeyDown(KEY_UP)) env->action = BLADE_UP;
        if (IsKeyDown(KEY_SPACE)) env->action = CONTINUE;
        if (IsKeyDown(KEY_R)) reset_room(env);

        // Step the simulation
        if (step(env)) {
            printf("Done\n");
            reset_room(env);
        }

        // Render the current state
        render_global(renderer, env);
    }

    // Cleanup resources
    close_renderer(renderer);
    free_allocated_grid(env);
    return 0;
}
