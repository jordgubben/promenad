#include <stdio.h>
#include <raylib.h>

#define IN_INPUT
#include "overview.h"

const float actor_walking_speed = 6.75;

//// Input ////

void process_input(float dt, app_t *app) {
	population_t *pop = &app->population_history[app->frame_count % max_pop_history_frames];

	// Meta keys
	bool shift_down = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

	// Toggle pause
	if (IsKeyPressed(KEY_P)) { app->paused = !app->paused; }

	// Step once
	if (IsKeyPressed(KEY_N)) { app->step_once = true; }

	// Control playback
	if (app->paused) {
		int step_count =  (shift_down ? 10 : 1);

		// Step backwards
		if (IsKeyPressed(KEY_R) && app->frame_count >= step_count) {
			app->frame_count -= step_count;
		}

		// Step forwards
		if (IsKeyPressed(KEY_F)) {
			app->frame_count += step_count;
		}
	} else if (IsKeyDown(KEY_R) && app->frame_count >= 2) {
		app->frame_count -= 2;
	}

	// Move global cursor with arrow keys
	if (shift_down) {
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
		push_limb_goal(id, app->world_cursor, 1, 5, &pop->limb_goals);
	}

	// Tank controls
	int actor_index = 0;
	actor_id_t actor = get_actor_id(actor_index, &pop->actors);
	vec3_t actor_forward = get_actor_forward_dir(actor, &pop->actors);
	if (IsKeyDown(KEY_W)) {
		pop->actors.movement[actor_index].velocity = vec3_mul(actor_forward, +1.f * actor_walking_speed);
	} else if (IsKeyDown(KEY_S)) {
		pop->actors.movement[actor_index].velocity = vec3_mul(actor_forward, -1.f * actor_walking_speed);
	} else {
		pop->actors.movement[actor_index].velocity = vec3(0,0,0);
	}
	if (IsKeyDown(KEY_A)) { pop->actors.location[0].orientation_y += dt * 0.25 * tau; }
	if (IsKeyDown(KEY_D)) { pop->actors.location[0].orientation_y -= dt * 0.25 * tau; }

	// Hand holding in video games
	if (app->mode == am_actor_pair && IsKeyPressed(KEY_H)) {
		printf("%s() – Activate hand holding\n", __func__);
		limb_id_t limb_1 = {1}, limb_2 = {4};
		link_limb_to(limb_1, limb_2, &pop->limb_tip_links);
		link_limb_to(limb_2, limb_1, &pop->limb_tip_links);
	}
}

