#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "raylib.h"
#include "rlgl.h"

#define PASS 0
#define SPEED_UP 1
#define SPEED_DOWN 2
#define LEFT 3
#define RIGHT 4
#define YAW_LEFT 5
#define YAW_RIGHT 6

#define BLADE_UP 7
#define BLADE_DOWN 8
#define CONTINUE 9

#define EMPTY 0
#define WALL 1
#define LAVA 2
#define GOAL 3
#define REWARD 4
#define OBJECT 5
#define AGENT_1 6

#define TIMESTEP 0.1

// ---------------------------------------------------------------

/**
 * @brief Rotates a 2D vector by a given angle theta.
 * Note: The Y-axis is inverted in the rotation to match grid-space.
 * @param vector The input Vector2.
 * @param theta The angle in radians.
 * @return The rotated Vector2.
 */
Vector2 rotate(Vector2 vector, float theta)
{
  Vector2 rotated = {
    vector.x * cosf(theta) + vector.y * sinf(theta),
    -1*vector.x * sinf(theta) + vector.y * cosf(theta)
  };
  return rotated;
}

/**
 * @brief Represents a single bulldozer agent in the environment.
 */
typedef struct Agent Agent;
struct Agent
{
  float y;               /**< Y-coordinate of the agent position */
  float x;               /**< X-coordinate of the agent position */
  float vel;             /**< Current forward/backward velocity */
  float theta;           /**< Agent heading in radians */
  float theta_dot;       /**< Angular velocity */
  float blade_pos;       /**< Relative vertical position of the blade */

  float avg_height;      /**< Cached average height of the terrain beneath the agent */

  float blade_width;     /**< Width of the bulldozer blade */
  float blade_thick;     /**< Thickness of the bulldozer blade */
  int blade_fore;        /**< Distance from agent center to the blade */
  float blade_yaw;       /**< Horizontal rotation/yaw of the blade */

  float spawn_y;         /**< Initial Y-coordinate for reset */
  float spawn_x;         /**< Initial X-coordinate for reset */
};

/**
 * @brief Represents the simulation environment state.
 */
typedef struct Env Env;
struct Env
{
  float* observations;   /**< Flattened observations for RL */
  float* actions;        /**< Flattened actions for RL */
  float* rewards;        /**< Rewards for agents */
  float* dones;          /**< Done signals for agents */

  int width;             /**< Grid width */
  int height;            /**< Grid height */
  int num_agents;        /**< Number of agents in the simulation */
  int horizon;           /**< Max steps per episode */

  int cell_size;         /**< Size of a single grid cell in pixels */

  float meters_per_pixel; /**< Physical scale of the grid */

  unsigned char* grid;   /**< Collision/object grid */
  float* height_map;     /**< Height values for each grid cell */
  float* dx_r;           /**< X-gradient (right) */
  float* dy_d;           /**< Y-gradient (down) */
  Agent* agents;         /**< Array of agents in the environment */
  int action;            /**< Current action for the primary agent */

  float max;             /**< Maximum height in the map */
  double mean;           /**< Mean height in the map */
};

/**
 * @brief Initializes the grid and environment state.
 */
Env* init_grid(
  int width, int height, int num_agents, int horizon,
  int vision, float speed, bool discretize) 
{
  Env* env = (Env*)calloc(1, sizeof(Env));

  env->width = width;
  env->height = height;
  env->num_agents = num_agents;
  env->horizon = horizon;

  size_t size = (size_t)width * height;
  env->grid = (unsigned char*)calloc(size, sizeof(unsigned char));
  env->height_map = (float*)calloc(size, sizeof(float));
  env->dx_r = (float*)calloc(size, sizeof(float));
  env->dy_d = (float*)calloc(size, sizeof(float));
  env->meters_per_pixel = 0.5f;
  env->agents = (Agent*)calloc(num_agents, sizeof(Agent));

  int obs_size = 2 * vision + 1;
  env->observations = (float*)calloc((size_t)num_agents * obs_size * obs_size, sizeof(float));
  env->actions = (float*)calloc(num_agents, sizeof(float));
  env->rewards = (float*)calloc(num_agents, sizeof(float));
  env->dones = (float*)calloc(num_agents, sizeof(float));

  return env;
}

/**
 * @brief Allocates memory and initializes a new environment.
 */
Env* allocate_grid(
  int width, int height, int num_agents, int horizon,
  int vision, float speed, bool discretize)
{
  return init_grid(width, height, num_agents, horizon, vision, speed, discretize);
}

/**
 * @brief Frees the environment and its associated memory.
 */
void free_env(Env* env)
{
  if (!env) return;
  free(env->grid);
  free(env->height_map);
  free(env->dx_r);
  free(env->dy_d);
  free(env->agents);
  free(env->observations);
  free(env->actions);
  free(env->rewards);
  free(env->dones);
  free(env);
}

/**
 * @brief Helper for freeing the grid.
 */
void free_allocated_grid(Env* env)
{
  free_env(env);
}

/**
 * @brief Maps a 2D coordinate to a 1D grid index.
 */
static inline int grid_offset(Env* env, int y, int x)
{
  return y*env->width + x;
}

/**
 * @brief Maps a 2D coordinate to a scaled heightgrid index.
 */
static inline int heightgrid_offset(Env* env, int y, int x)
{
  return (y / env->cell_size)*env->width + (x / env->cell_size);
}

/**
 * @brief Resets the environment to its initial state.
 * @param env Pointer to the environment.
 * @param seed Random seed (currently unused).
 */
void reset(Env* env, int seed)
{
  int size = env->width * env->height;
  for (int i = 0; i < size; i++) env->height_map[i] = 20.0f;

  for (int r = 250; r < 300; r++)
  {
    for (int c = 250; c < 300; c++)
    {
      int adr = grid_offset(env, r, c);
      float x = 0.1f * (c - 275);
      float y = 0.1f * (r - 275);
      env->height_map[adr] = -1.0f * (x * x + y * y) + 30.0f;
    }
  }

  for (int i = 0; i < env->num_agents; i++)
  {
    Agent* agent = &env->agents[i];
    agent->y = agent->spawn_y;
    agent->x = agent->spawn_x;
    agent->blade_width = 20;
    agent->blade_thick = 2;
    agent->blade_fore = 20;
    agent->theta = 110.0f * PI / 180.0f;
    agent->vel = 0;
    agent->theta_dot = 0;
    agent->blade_pos = 0;
    agent->blade_yaw = 0;
  }
}

/**
 * @brief Calculates the height gradients for the entire map.
 */
void gradient(Env* env)
{
  env->max = 0;
  env->mean = 0;
  int width = env->width;
  int height = env->height;

  for (int r = 1; r < height-1; r++)
  {
    for (int c = 1; c < width-1; c++)
    {
      int adr = r * width + c;
      env->dx_r[adr] = env->height_map[adr + 1] - env->height_map[adr];
      env->dy_d[adr] = env->height_map[adr + width] - env->height_map[adr];

      env->mean += env->height_map[adr];
      if (env->height_map[adr] > env->max) env->max = env->height_map[adr];
    }
  }
  env->mean /= (width * height);
}

/**
 * @brief Performs a basic erosion pass on the height map to simulate soil stability.
 */
void erode(Env *env)
{
  int width = env->width;
  for (int r = 1; r < env->height-2; r++)
  {
    for (int c = 1; c < width-2; c++)
    {
      int adr = r * width + c;
      float dx_r = env->dx_r[adr];
      float dx_l = -env->dx_r[adr - 1];
      float dy_d = env->dy_d[adr];
      float dy_u = -env->dy_d[adr - width];

      float grads[4] = {dx_l, dx_r, dy_u, dy_d};
      int offsets[4] = {-1, 1, -width, width};

      int index = 0;
      float min_grad = 0;
      for (int i = 0; i < 4; i++)
      {
        if (grads[i] < min_grad)
        {
          min_grad = grads[i];
          index = i;
        }
      }

      if (min_grad < -2.0f)
      {
        float diff = 0.5f * min_grad;
        env->height_map[adr] += diff;
        env->height_map[adr + offsets[index]] -= diff;
      }
    }
  }
}

/**
 * @brief Calculates the average height of the terrain in the agent's immediate neighborhood.
 * Used for setting the relative blade height.
 */
void calculate_neighborhood_height(Env* env)
{
  Agent* agent = &env->agents[0];
  float x = agent->x;
  float y = agent->y;
  float theta = agent->theta;

  const int n_len = 100;
  const int n_wid = 50;
  const float half_wid = n_wid / 2.0f;
  const float half_len = n_len / 2.0f;

  double sum = 0;
  float cos_t = cosf(theta);
  float sin_t = sinf(theta);

  for (int x_n = 0; x_n < n_wid; x_n++)
  {
    float px = x_n - half_wid;
    for (int y_n = 0; y_n < n_len; y_n++)
    {
      float py = y_n - half_len;
      // Optimized rotation inline
      float rot_x = px * cos_t + py * sin_t;
      float rot_y = -px * sin_t + py * cos_t;
      
      int ix = (int)(x + rot_x);
      int iy = (int)(y + rot_y);
      
      // Bounds check
      if (ix >= 0 && ix < env->width && iy >= 0 && iy < env->height)
        sum += env->height_map[iy * env->width + ix];
    }
  }

  agent->avg_height = (float)(sum / (n_wid * n_len));
}

/**
 * @brief Processes the interaction between the bulldozer blade and the height map.
 * Moves "soil" (height values) based on blade position, yaw, and agent movement.
 */
void blade_interaction(Env* env)
{
  Agent* agent = &env->agents[0];
  float true_blade_height = agent->avg_height + agent->blade_pos;
  float theta = agent->theta;
  float cos_t = cosf(theta);
  float sin_t = sinf(theta);
  int direction = (agent->vel >= 0) ? 1 : -1;
  float blade_yaw = agent->blade_yaw * direction;
  float cos_y = cosf(blade_yaw);
  float sin_y = sinf(blade_yaw);

  int n[20] = {-1, 0, -2, 1, -3, 2, -4, 3, -5, 4, -6, 5, -7, 6, -8, 7, -9, 8, -10, 9};

  for (int i = 0; i < (int)agent->blade_width && i < 20; i++)
  {
    float x_n = (float)n[i];
    
    // Blade local coordinates for cutting and depositing
    float yaw_y[5] = {0, 1, 2, 3, 4};
    Vector2 cuts[2], deposits[3];

    for (int j = 0; j < 5; j++) {
        float ly = yaw_y[j];
        // Rotate by blade yaw
        float rx = x_n * cos_y + ly * sin_y;
        float ry = -x_n * sin_y + ly * cos_y;
        
        // Transform to world coordinates
        float wx = agent->x + (agent->blade_fore + ry * direction) * sin_t + rx * cos_t;
        float wy = agent->y + (agent->blade_fore + ry * direction) * cos_t - rx * sin_t;
        
        if (j < 2) {
            cuts[j] = (Vector2){wx, wy};
        } else {
            deposits[j-2] = (Vector2){wx, wy};
        }
    }

    int adr1 = grid_offset(env, (int)cuts[0].y, (int)cuts[0].x);
    int adr2 = grid_offset(env, (int)cuts[1].y, (int)cuts[1].x);
    
    // Safety check for map boundaries
    if (adr1 < 0 || adr1 >= env->width * env->height || adr2 < 0 || adr2 >= env->width * env->height) continue;

    float h1 = env->height_map[adr1];
    float h2 = env->height_map[adr2];

    if (true_blade_height <= h1 || true_blade_height <= h2)
    {
      float delta_soil = (h1 + h2 - 2 * true_blade_height) / 6.0f;

      for (int j = 0; j < 3; j++) {
          int d_adr = grid_offset(env, (int)deposits[j].y, (int)deposits[j].x);
          if (d_adr >= 0 && d_adr < env->width * env->height)
            env->height_map[d_adr] += delta_soil * 2.0f;
      }
      env->height_map[adr1] = true_blade_height;
      env->height_map[adr2] = true_blade_height;
    }
  }
}

/**
 * @brief Advances the simulation by one time step.
 * @return true if the episode is done.
 */
bool step(Env* env)
{
  bool done = false;
  for (int agent_idx = 0; agent_idx < env->num_agents; agent_idx++)
  {
    Agent* agent = &env->agents[agent_idx];
    int action = env->action;

    switch (action) {
        case PASS: continue;
        case SPEED_UP:   agent->vel = fminf(agent->vel + 1.0f, 10.0f); break;
        case SPEED_DOWN: agent->vel = fmaxf(agent->vel - 1.0f, -10.0f); break;
        case LEFT:       agent->theta_dot = fminf(agent->theta_dot + 0.1f, 1.0f); break;
        case RIGHT:      agent->theta_dot = fmaxf(agent->theta_dot - 0.1f, -1.0f); break;
        case YAW_LEFT:   agent->blade_yaw = fminf(agent->blade_yaw + 0.01f, 0.5f); break;
        case YAW_RIGHT:  agent->blade_yaw = fmaxf(agent->blade_yaw - 0.01f, -0.5f); break;
        case BLADE_UP:   agent->blade_pos = fminf(agent->blade_pos + 1.0f, 15.0f); break;
        case BLADE_DOWN: agent->blade_pos = fmaxf(agent->blade_pos - 1.0f, -10.0f); break;
        case CONTINUE:   break;
        default: printf("Invalid action: %i\n", action); exit(1);
    }

    agent->theta += agent->theta_dot * TIMESTEP;
    if (agent->theta > 2*PI) agent->theta -= 2*PI;
    if (agent->theta < 0)    agent->theta += 2*PI;

    float dest_y = TIMESTEP * agent->vel * cosf(agent->theta) + agent->y;
    float dest_x = TIMESTEP * agent->vel * sinf(agent->theta) + agent->x;

    // Boundary constraints
    agent->y = fmaxf(50.0f, fminf(dest_y, (float)env->height - 50.0f));
    agent->x = fmaxf(50.0f, fminf(dest_x, (float)env->width - 50.0f));

    calculate_neighborhood_height(env);
    blade_interaction(env);
    gradient(env);
    erode(env);
  }

  return done;
}

// Raylib client -------------------------------------------------------------//
Color COLORS[] = {
  (Color){6, 24, 24, 255},
  (Color){0, 0, 255, 255},
  (Color){0, 128, 255, 255},
  (Color){128, 128, 128, 255},
  (Color){255, 0, 0, 255},
  (Color){255, 255, 255, 255},
  (Color){255, 85, 85, 255},
  (Color){170, 170, 170, 255},
  (Color){0, 255, 255, 255},
  (Color){255, 255, 0, 255},
};

Rectangle UV_COORDS[7] = {
  (Rectangle){0, 0, 0, 0},
  (Rectangle){512, 0, 128, 128},
  (Rectangle){0, 0, 0, 0},
  (Rectangle){0, 0, 128, 128},
  (Rectangle){128, 0, 128, 128},
  (Rectangle){256, 0, 128, 128},
  (Rectangle){384, 0, 128, 128},
};

typedef struct {
  int cell_size;
  int width;
  int height;
  Texture2D gato;
} Renderer;

Renderer* init_renderer(int cell_size, int width, int height)
{
  Renderer* renderer = (Renderer*)calloc(1, sizeof(Renderer));
  renderer->cell_size = cell_size;
  renderer->width = width;
  renderer->height = height;

  InitWindow(width*cell_size, height*cell_size, "PufferLib Ray Grid");
  SetTargetFPS(10);

  return renderer;
}

void close_renderer(Renderer* renderer)
{
  CloseWindow();
  free(renderer);
}

void render_debug(Renderer* renderer, Env* env)
{
  if (IsKeyDown(KEY_ESCAPE))
  {
    exit(0);
  }
  BeginDrawing();
  ClearBackground((Color){6, 24, 24, 255});

  int ts = renderer->cell_size;
  for (int r = 0; r < env->height; r++)
  {
    for (int c = 0; c < env->width; c++)
    {
      int adr = grid_offset(env, r, c);
      int dx = env->dx_r[adr];
      dx *= 5;
      dx += 100;
      DrawRectangle(c*ts, r*ts, ts, ts, (Color){dx, dx, dx, 255});

    }
  }
  EndDrawing();
}

void render_global(Renderer* renderer, Env* env)
{
  if (IsKeyDown(KEY_ESCAPE))
  {
    exit(0);
  }

  BeginDrawing();
  ClearBackground((Color){6, 24, 24, 255});

  int ts = renderer->cell_size;
  for (int r = 0; r < env->height; r++)
  {
    for (int c = 0; c < env->width; c++)
    {
      int adr = grid_offset(env, r, c);
      int height = env->height_map[adr] * 3;

      if (height > 255)
      {
        DrawRectangle(c*ts, r*ts, ts, ts, (Color){255, 0, 0, 255});
      }
      else
      {
        DrawRectangle(c*ts, r*ts, ts, ts, (Color){height, height, height, 255});
      }

    }
  }

  Agent* agent = &env->agents[0];
  float deg = (agent->theta) * 180.0f / PI;
  float yaw_deg = (agent->blade_yaw) * 180.0f / PI;

  Rectangle blade;
  blade.x  = agent->x * renderer->cell_size;
  blade.y  = agent->y * renderer->cell_size;
  blade.height = agent->blade_thick * renderer->cell_size;
  blade.width  = agent->blade_width * renderer->cell_size;

  DrawRectanglePro(
    blade,
    (Vector2){(agent->blade_width / 2.0f) * renderer->cell_size, -1.0f * agent->blade_fore * renderer->cell_size},
    -1.0f * (deg + yaw_deg),
    YELLOW
  );

  DrawCircle(
    agent->x * renderer->cell_size,
    agent->y * renderer->cell_size,
    5 * renderer->cell_size,
    RED
  );

  DrawText(TextFormat("A-VEL: %02.02f deg/s", agent->theta_dot), 20, 20, 10, WHITE);
  DrawText(TextFormat("THETA: %02.02f deg", deg), 20, 40, 10, WHITE);
  DrawText(TextFormat("YAW:   %02.02f deg", yaw_deg), 20, 60, 10, WHITE);
  DrawText(TextFormat("SPEED: %02.02f px/s", agent->vel), 20, 80, 10, WHITE);
  DrawText(TextFormat("X POS: %02.02f px", agent->x), 20, 100, 10, WHITE);
  DrawText(TextFormat("Y POS: %02.02f px", agent->y), 20, 120, 10, WHITE);
  DrawText(TextFormat("BLADE: %02.02f px", agent->blade_pos), 20, 140, 10, WHITE);
  EndDrawing();

}

Env* alloc_room_env()
{
  int width = 500;
  int height = 500;
  int num_agents = 1;
  int horizon = 512;
  float agent_speed = 1;
  int vision = 3;
  bool discretize = true;

  Env* env = allocate_grid(width+2*vision, height+2*vision, num_agents, horizon,
  vision, agent_speed, discretize);

  env->cell_size = 1;
  env->agents[0].spawn_y = 150;
  env->agents[0].spawn_x = 150;
  return env;
}


void reset_room(Env* env)
{
  for (int r = 0; r < env->height; r++)
  {
    for (int c = 0; c < env->width; c++)
    {
      int adr = grid_offset(env, r, c);
      env->grid[adr] = EMPTY;
    }
  }
  reset(env, 0);
}
