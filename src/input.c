#include <stdio.h>
#include <raylib.h>

#define IN_INPUT
#include "overview.h"

const float actor_walking_speed = 2.25;

typedef struct tank_controls_ {
	KeyboardKey rot_left, rot_right, move_forward, move_backward;
} tank_controls_t;

void process_tank_controls(float dt, actor_id_t, const tank_controls_t *, actor_table_t *);

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

	// Controls
	{
		actor_id_t actor_1 = { 0 }, actor_2 = { 1 };
		tank_controls_t controls_1 = { KEY_A, KEY_D, KEY_W, KEY_S };
		tank_controls_t controls_2 = { KEY_J, KEY_L, KEY_I, KEY_K };
		process_tank_controls(dt, actor_1, &controls_1, &pop->actors);
		process_tank_controls(dt, actor_2, &controls_2, &pop->actors);
	}

	// Hand holding in video games
	if (app->mode == am_actor_pair && IsKeyPressed(KEY_H)) {
		printf("%s() – Toggle hand holding\n", __func__);
		limb_id_t limb_1 = {1}, limb_2 = {4};

		// Toggle first limb
		if (limb_has_link(limb_1, &pop->limb_tip_links)) {
			unlink_limb(limb_1, &pop->limb_tip_links);
		} else {
			link_limb_to(limb_1, limb_2, &pop->limb_tip_links);
		}

		// Toggle second limb
		if (limb_has_link(limb_2, &pop->limb_tip_links)) {
			unlink_limb(limb_2, &pop->limb_tip_links);
		} else {
			link_limb_to(limb_2, limb_1, &pop->limb_tip_links);
		}
	}
}


void process_tank_controls(float dt, actor_id_t actor, const tank_controls_t *controls, actor_table_t *actors) {
	if (!actor_exists(actor, actors)) { return; }
	int actor_index = get_actor_index(actor, actors);
	vec3_t actor_forward = get_actor_forward_dir(actor, actors);

	if (IsKeyDown(controls->move_forward)) {
		actors->movement[actor_index].velocity = vec3_mul(actor_forward, +1.f * actor_walking_speed);
	} else if (IsKeyDown(controls->move_backward)) {
		actors->movement[actor_index].velocity = vec3_mul(actor_forward, -1.f * actor_walking_speed);
	} else {
		actors->movement[actor_index].velocity = vec3(0,0,0);
	}

	if (IsKeyDown(controls->rot_left)) { actors->location[actor_index].orientation_y += dt * 0.25 * tau; }
	if (IsKeyDown(controls->rot_right)) { actors->location[actor_index].orientation_y -= dt * 0.25 * tau; }
}
