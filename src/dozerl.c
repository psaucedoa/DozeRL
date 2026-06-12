#include "dozerl.h"
#include "puffernet.h"

void demo(const char* checkpoint_path) {
    SoilEnv* env = (SoilEnv*)malloc(sizeof(SoilEnv));
    env->observations = (float*)calloc(5011, sizeof(float));
    env->actions = (float*)calloc(6, sizeof(float));
    env->rewards = (float*)calloc(1, sizeof(float));
    env->terminals = (float*)calloc(1, sizeof(float));
    env->rng = 42; // fixed seed for deterministic testing

#ifndef BENCHMARK
    outfile = fopen("out/sim_out.bin", "wb");
    if (!outfile) {
        printf("Error: Could not open out/sim_out.bin for writing.\n");
        return;
    }
    
    // Write geometry header once
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
#endif

    Weights* weights = NULL;
    PufferNet* net = NULL;

    if (checkpoint_path != NULL) {
        printf("Loading trained policy weights from %s...\n", checkpoint_path);
        weights = load_weights(checkpoint_path);
        if (weights == NULL) {
            printf("Error: Failed to load weights from %s.\n", checkpoint_path);
            return;
        }
        int logit_sizes[6] = {1, 1, 1, 1, 1, 1};
        net = make_puffernet(weights, 1, 5011, 128, 2, logit_sizes, 6);
        printf("PufferNet policy successfully initialized in C.\n");
    } else {
        printf("No checkpoint path provided. Running manual heuristic policy...\n");
    }

    c_reset(env);

#ifdef BENCHMARK
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    int steps = 1000;
    for (int t = 0; t < steps; t++) {
        if (net != NULL) {
            forward_puffernet(net, env->observations, env->actions);
        }
        c_step(env);
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    double elapsed = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    printf("Benchmarking DozeRL Sim via c_step: %d steps in %.4f seconds (%.2f Hz)\n", steps, elapsed, steps / elapsed);
#else
    printf("Starting 6DOF 3D Soil Simulation...\n");
    for (int t = 0; t < 600; t++) {
        if (net != NULL) {
            // Forward pass through the trained policy
            forward_puffernet(net, env->observations, env->actions);
        } else {
            // Reset/zero actions and run manual heuristic control
            for (int i = 0; i < 6; i++) {
                env->actions[i] = 0.0f;
            }

            if (t < 10) {
                env->actions[0] = 0.8f;  // effort_linear
                env->actions[3] = 0.5f;  // effort_pitch
            }
            if (t < 10) {
                env->actions[3] = 0.0f;
                env->actions[2] = -0.06f; // effort_lift
            }
            if (t >= 10) {
                env->actions[0] = 0.8f;  // effort_linear
                env->actions[2] = 0.0f;
            }
            if (t > 400) {
                env->actions[2] = 0.5f;  // effort_lift
            }
        }
        c_step(env);
    }
    if (outfile) {
        fclose(outfile);
    }
    printf("Simulation Complete. Data written to out/sim_out.bin.\n");
#endif

    if (net != NULL) {
        free_puffernet(net);
    }
    if (weights != NULL) {
        free(weights);
    }

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
