#include "../include/raylib.h"
#include "../include/raymath.h"
#include <cmath>
#include <cstdlib>
#include <memory>
#include <random>
#include <vector>

#define RAYGUI_IMPLEMENTATION
#include "../include/raygui.h"

const float SCREEN_WIDTH = 800;
const float SCREEN_HEIGHT = 900;
const float WIDTH = 800;
const float HEIGHT = 800;
const int TARGET_FPS = 60;
const int NUM_BOIDS = 100;
const float RADIUS = 5.0f;
const float MAX_SPEED = 5.0f;
const float MAX_FORCE = 0.5f;
const float DETECTION_RADIUS = 20.0f;

/* TODO 
 *  QUADTREE + SUBDIVISION
 *  Interace:
 *    - Perception radius
 *    - Max Speed
 *    - Max Force
 *  Design of boids
 *  View
 *  Snapshot
 * */

struct Boid {
  Vector2 pos;
  Vector2 vel;
  Vector2 acc;
};

Vector2 Steer(
    Boid& boid, 
    std::vector<Boid>& boids,
    float ALIGNMENT,
    float COHESION,
    float SEPARATION) {
  Vector2 force {};

  Vector2 avg_vel {};
  Vector2 avg_pos {};
  Vector2 avg_cntr_steer {};
  int n = 0; 

  for (Boid& other_boid : boids) {

    float dist = Vector2Distance(boid.pos, other_boid.pos);
    if (dist < DETECTION_RADIUS && dist > 0.0f) {
      avg_pos = Vector2Add(avg_pos, other_boid.pos);
      avg_vel = Vector2Add(avg_vel, other_boid.vel);

      Vector2 diff = Vector2Subtract(boid.pos, other_boid.pos);
      Vector2 dist_scale {};

      dist_scale = Vector2Scale(Vector2Normalize(diff), 1 / dist);

      avg_cntr_steer = Vector2Add(avg_cntr_steer, dist_scale);
      n++;
    }
  }

  Vector2 alignment {};
  Vector2 cohesion {};
  Vector2 separation {};

  if (n > 0) {
    alignment = Vector2Subtract(Vector2Scale(avg_vel, 1.0f / n), boid.vel);
    cohesion = Vector2Subtract(Vector2Scale(avg_pos, 1.0f / n), boid.pos);
    separation = Vector2Scale(avg_cntr_steer, 1.0f / n);
  }

  force = Vector2Add(force, Vector2Scale(alignment, ALIGNMENT));
  force = Vector2Add(force, Vector2Scale(cohesion, COHESION));
  force = Vector2Add(force, Vector2Scale(separation, SEPARATION));

  return force;
}

void UpdateBoid(Boid& boid) {

  boid.vel = Vector2Add(boid.vel, boid.acc);

  if (Vector2Length(boid.vel) > MAX_SPEED) 
    boid.vel = Vector2Scale(Vector2Normalize(boid.vel), MAX_SPEED); 

  Vector2 pos_next {
    .x = fmodf(boid.pos.x + boid.vel.x, WIDTH),
    .y = fmodf(boid.pos.y + boid.vel.y, HEIGHT) 
  };
  if (pos_next.x < 0)
    pos_next.x += WIDTH;
  if (pos_next.y < 0)
    pos_next.y += HEIGHT;
  
  boid.pos = pos_next;
  boid.acc = Vector2Scale(boid.acc, 0.0f);
}

int main(int argc, char** argv) { 

  std::vector<Boid> boids;

  std::random_device device;
  std::uniform_real_distribution<float> uniform_dist(-PI, PI);
  std::mt19937 gen(device());

  for (int i = 0 ; i < NUM_BOIDS; i++) {
    
    /* Random angle from -pi to pi */ 
    float theta = uniform_dist(gen); 
   
    Boid boid {
      .pos {
        .x=WIDTH/2.0f,
        .y=HEIGHT/2.0f,
        },
      .vel {
        MAX_SPEED * std::cos(theta),
        MAX_SPEED * std::sin(theta)
      },
      .acc {}
    };

    boids.push_back(boid);
  }

  InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Boids");
  SetTargetFPS(TARGET_FPS);

  std::unique_ptr<float> separation = std::make_unique<float>(0.0f);
  std::unique_ptr<float> cohesion = std::make_unique<float>(0.0f);
  std::unique_ptr<float> alignment = std::make_unique<float>(0.0f);

  float SLIDER_HEIGHT = 50.0f;
  float SLIDER_Y_POS = (HEIGHT - SLIDER_HEIGHT / 2) + (SCREEN_HEIGHT - HEIGHT) / 2.0f;
  float SLIDER_WIDTH = (SCREEN_WIDTH / 3.0f);

  while(!WindowShouldClose()) {

    BeginDrawing();
    ClearBackground(RAYWHITE);

    DrawLineEx({0, 0}, {WIDTH,0}, 5.0f, GRAY);
    DrawLineEx({0, 0}, {0,HEIGHT}, 5.0f, GRAY);
    DrawLineEx({WIDTH, 0}, {WIDTH,HEIGHT}, 5.0f, GRAY);
    DrawLineEx({0, HEIGHT}, {WIDTH,HEIGHT}, 5.0f, GRAY);

    float offset = 0.0f;
    DrawText("Alignment", offset, SLIDER_Y_POS - 20.0f, 20.0f, SKYBLUE);
    GuiSlider({offset, SLIDER_Y_POS, SLIDER_WIDTH, SLIDER_HEIGHT}, nullptr, nullptr, alignment.get(), 0.0f, 10.0f); 
    offset += SLIDER_WIDTH;

    DrawText("Cohesion", offset, SLIDER_Y_POS - 20.0f, 20.0f, SKYBLUE);
    GuiSlider({offset, SLIDER_Y_POS, SLIDER_WIDTH, SLIDER_HEIGHT}, nullptr, nullptr, cohesion.get(), 0.0f, 10.0f); 
    offset += SLIDER_WIDTH;

    DrawText("Separation", offset, SLIDER_Y_POS - 20.0f, 20.0f, SKYBLUE);
    GuiSlider({offset, SLIDER_Y_POS, SLIDER_WIDTH, SLIDER_HEIGHT}, nullptr, nullptr, separation.get(), 0.0f, 10.0f);
    offset += SLIDER_WIDTH;
    
    for (Boid& boid : boids) {
      DrawCircleV(boid.pos, RADIUS, RED);
      Vector2 force_align = Vector2Scale(Vector2Normalize(Steer(boid, boids, *alignment, *cohesion, *separation)), MAX_FORCE);
      boid.acc = force_align; 
      UpdateBoid(boid);
    }

    EndDrawing();
  }

  CloseWindow();

  return 0;
}
