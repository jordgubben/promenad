#include <stdlib.h>
#include <stdio.h>
#include <raylib.h>

#define IN_APP_ROOT
#include "overview.h"

static const float step_time = 1.f/60.f;
static app_t app= { 0 };


int main(int argc, char** argv) {
#define PRINT_SIZE_OF(t) printf("%s: %zub\n", #t, sizeof(t))
	PRINT_SIZE_OF(vec3_t);
	PRINT_SIZE_OF(vec4_t);
	PRINT_SIZE_OF(mat4_t);
	PRINT_SIZE_OF(limb_table_t);
	PRINT_SIZE_OF(app_t);

	// Get things up and running
	InitWindow(1024, 768, "Hello, Promenad!");
	SetTargetFPS(144);

	// Define the camera to look into our 3d world
	Camera3D camera = { 0 };
	camera.position = (Vector3){ 20.0f, 5.0f, 20.0f };
	camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
	camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
	camera.fovy = 45.0f;
	camera.type = CAMERA_PERSPECTIVE;
	SetCameraMode(camera, CAMERA_FREE);
	UpdateCamera(&camera);

	// App setup
	init_app(am_actor_pair, &app);

	// Ah-Gogogoggogogogo!
	while(!WindowShouldClose()) {
		// Update
		float dt = maxf(GetFrameTime(), 1.f/30.f);
		process_input(dt, &app);
		update_app(dt, &app);

		// Render
		UpdateCamera(&camera);
		BeginDrawing();
		ClearBackground(RAYWHITE);
		render_app(&camera, &app);
		DrawFPS(0,0);
		EndDrawing();
	}

	// K, thx, bye
	term_app(&app);
	CloseWindow();
	return 0;
}


/**
Update all the things.
**/
void update_app(float dt, app_t *app) {
	if (app->step_once) {
		dt += step_time;
		app->step_once = false;
	} else if (app->paused) {
		return;
	}

	// Simulate in a fixed time step
	app->buffered_time += dt;
	if (app->buffered_time < step_time) {
		return;
	} else {
		app->buffered_time -= step_time;
	}

	// Keep history
	unsigned old_frame = app->frame_count % max_pop_history_frames;
	app->frame_count++;
	unsigned new_frame = (app->frame_count % max_pop_history_frames);
	app->population_history[new_frame] = app->population_history[old_frame];
	population_t *pop = &app->population_history[new_frame];

	// Update world
	update_population(step_time, pop);
}


/**
Update the dynamically changing part of the simulation.
**/
void update_population(float dt, population_t *pop) {

	// Move whole actors
	move_actors(dt, &pop->actors);
	calculate_actor_transforms(&pop->actors);

	// Move limbs attached to actors
	reposition_attached_limbs(&pop->arms, &pop->actors, &pop->limbs);
	reposition_attached_limbs(&pop->legs, &pop->actors, &pop->limbs);

	// Update kinematics
	animate_walking_actor_legs(dt, &pop->actors, &pop->legs, &pop->limb_goals, &pop->limbs);
	perpetuate_limb_momentums(dt, &pop->limb_swings, &pop->limbs);
	move_limbs_toward_goals(dt, &pop->limb_goals, &pop->limbs);
	move_limb_tips_to_their_linked_partners(&pop->limb_tip_links, &pop->limbs);
	apply_gravity_to_limbs(dt, vec3(0,-4,0), &pop->limb_swings, &pop->limbs);
	move_limbs_directly_to_end_effectors(&pop->limbs);
	delete_accomplished_limb_goals(&pop->limbs, &pop->limb_goals);
}
