#include <stdio.h>
#include <raylib.h>

#define IN_INPUT
#include "overview.h"

static const float actor_speed = 1;


//// Input ////

void process_input(float dt, app_t *app) {
	population_t *pop = &app->population_history[app->frame_count % max_pop_history_frames];

	// Toggle pause
	if (IsKeyPressed(KEY_P)) { app->paused = !app->paused; }

	// Rewind
	if (IsKeyDown(KEY_R) && app->frame_count >= 2) { app->frame_count -= 2; }

	// Move global cursor with arrow keys
	if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
		if (IsKeyDown(KEY_RIGHT)) { app->world_cursor.z -= dt; }
		if (IsKeyDown(KEY_LEFT)) { app->world_cursor.z += dt; }
	} else {
		if (IsKeyDown(KEY_RIGHT)) { app->world_cursor.x += dt; }
		if (IsKeyDown(KEY_LEFT)) { app->world_cursor.x -= dt; }
	}
	if (IsKeyDown(KEY_UP)) { app->world_cursor.y += dt; }
	if (IsKeyDown(KEY_DOWN)) { app->world_cursor.y -= dt; }

	// Set goal for first limb
	if (IsKeyPressed(KEY_SPACE)) {
		limb_id_t id = { 0 };
		put_limb_goal(id, app->world_cursor, 1, 5, &pop->limb_goals);
	}

	// Tank controls
	if (IsKeyDown(KEY_W)) {
		pop->actors.location[0].position = vec3_add(
			pop->actors.location[0].position,
			vec3_mul(get_actor_forward_dir(get_actor_id(0, &pop->actors), &pop->actors),
				dt * actor_speed)
			);
	}
	if (IsKeyDown(KEY_S)) {
		pop->actors.location[0].position = vec3_sub(
			pop->actors.location[0].position,
			vec3_mul(get_actor_forward_dir(get_actor_id(0, &pop->actors), &pop->actors),
				dt * actor_speed)
			);
	}
	if (IsKeyDown(KEY_A)) { pop->actors.location[0].orientation_y += dt * 0.25 * tau; }
	if (IsKeyDown(KEY_D)) { pop->actors.location[0].orientation_y -= dt * 0.25 * tau; }
}

