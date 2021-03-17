#include <stdio.h>
#include <raylib.h>

#define IN_VIEW
#include "overview.h"

#define DRAW_COORDINATE_SYSTEM_HELPERS 1

//// Input ////

void process_input(float dt, app_t *app) {
	// Toggle pause
	if (IsKeyPressed(KEY_P)) { app->paused = !app->paused; }

	// Move common end effector with arrow keys
	if (IsKeyDown(KEY_RIGHT)) { app->common_end_effector.x += dt; }
	if (IsKeyDown(KEY_LEFT)) { app->common_end_effector.x -= dt; }
	if (IsKeyDown(KEY_UP)) { app->common_end_effector.y += dt; }
	if (IsKeyDown(KEY_DOWN)) { app->common_end_effector.y -= dt; }

	// Tank controls
	if (IsKeyDown(KEY_W)) {
		app->actors.location[0].position = vec3_add(
			app->actors.location[0].position,
			vec3_mul(get_actor_forward_dir(get_actor_id(0, &app->actors), &app->actors), dt * 2)
			);
	}
	if (IsKeyDown(KEY_S)) {
		app->actors.location[0].position = vec3_sub(
			app->actors.location[0].position,
			vec3_mul(get_actor_forward_dir(get_actor_id(0, &app->actors), &app->actors), dt * 2)
			);
	}
	if (IsKeyDown(KEY_A)) { app->actors.location[0].orientation_y += dt * 0.25 * tau; }
	if (IsKeyDown(KEY_D)) { app->actors.location[0].orientation_y -= dt * 0.25 * tau; }
}

//// Rendering ////

void render_orientation_gizmo(vec3_t pos, quat_t ori);
void draw_matrix_as_text(const char* title, mat4_t m, float x, float y, float s, Color c);

/**
Render all the things.
**/
void render_app(const struct Camera3D *camera,  const app_t *app) {

	// Render something at origo
	BeginMode3D(*camera);
	{
		render_actors(app->actor_model, &app->actors);

		DrawSphere(app->common_end_effector.rl, 0.1f, GOLD);
		{
			vec3_t shadow = app->common_end_effector;
			shadow.y = 0;
			DrawSphere(shadow.rl, 0.1f, ORANGE);
		}

		render_limb_skeletons(app->common_end_effector, &app->limbs);

#if DRAW_COORDINATE_SYSTEM_HELPERS
		DrawCube(vec3(5,0,0).rl, 1.f, .1f, .1f, RED);
		DrawCube(vec3(0,5,0).rl, .1f, 1.f, .1f, GREEN);
		DrawCube(vec3(0,0,5).rl, .1f, .1f, 1.f, BLUE);
#endif // DRAW_COORDINATE_SYSTEM_HELPERS

		DrawGrid(10, 1.f);
	}
	EndMode3D();

	draw_matrix_as_text("Actor 'to world' transform", app->actors.to_world[0], 50, 10, 15, BLACK);
	draw_matrix_as_text("Actor 'to object' transform", app->actors.to_object[0], 250, 10, 15, BLACK);
}

/**
Render actors in table.
(Expects to be called inside raylibs 'Draw3D' mode)
**/
void render_actors(const Model* actor_model, const actor_table_t *table) {
	FOR_ROWS(a, *table){
		Model model = *actor_model;
		model.transform = mat4_transpose(table->to_world[a]).rl;
		DrawModel(model, vec3(0,0,0).rl, 1.0f, BLUE);
	}
}

void render_limb_skeletons(vec3_t end_effector, const limb_table_t *table) {
	void render_segment_joint_orientations(vec3_t, limb_segment_t [], size_t);

	FOR_ROWS(l, *table) {
		limb_id_t limb = get_limb_id(l, table);

		// Render root
		vec3_t root_pos = table->position[l];
		quat_t root_ori = table->orientation[l];
		render_orientation_gizmo(root_pos, root_ori);
		DrawSphere(root_pos.rl, 0.15, BLACK);

		// Render segments in their current positions
		int segment = table->root_segment[l];
		while (segment) {
			limb_segment_t seg = table->segments[segment];
			DrawLine3D(seg.joint_pos.rl, seg.tip_pos.rl, ORANGE);
			DrawSphere(seg.joint_pos.rl, 0.10, MAROON);
			DrawSphere(seg.tip_pos.rl, 0.05, MAROON);

			// Next segment (if any)
			segment = table->segment_nodes[segment].next_index;
			if (segment == table->root_segment[l]) { segment = 0; }
		}

		// Render limb segments IK target positions
		limb_segment_t segments[32];
		size_t num_segments = collect_limb_segments(limb, table, segments, 32);
		vec3_t origin = table->position[l];
		render_segment_joint_orientations(origin, segments, num_segments);
		reposition_limb_segments_with_fabrik(origin, end_effector, segments, num_segments);
		render_segment_joint_orientations(origin, segments, num_segments);
		FOR_ITR(limb_segment_t, seg_itr, segments, num_segments) {
			DrawLine3D(seg_itr->joint_pos.rl, seg_itr->tip_pos.rl, BLUE);
		}
	}
}


void render_segment_joint_orientations(vec3_t origin_pos, limb_segment_t segments[], size_t num_segments) {
	// Draw joint spaces
	FOR_IN(i, num_segments) {
		vec3_t joint_pos = segments[i].joint_pos;
		quat_t joint_ori = segments[i].orientation;
		render_orientation_gizmo(joint_pos, joint_ori);
	}
}

void render_orientation_gizmo(vec3_t pos, quat_t ori) {
	DrawLine3D(pos.rl, vec3_add(pos, quat_rotate_vec3(ori, vec3(1,0,0))).rl, RED);
	DrawLine3D(pos.rl, vec3_add(pos, quat_rotate_vec3(ori, vec3(0,1,0))).rl, GREEN);
	DrawLine3D(pos.rl, vec3_add(pos, quat_rotate_vec3(ori, vec3(0,0,1))).rl, BLUE);
}

void draw_matrix_as_text(const char* title, mat4_t m, float x, float y, float s, Color c) {
	char str[1024];
	snprintf(str, 1024, "%s:\n"
		"[ %1.2f, %1.2f, %1.2f, %1.2f|\n"
		"| %1.2f, %1.2f, %1.2f, %1.2f|\n"
		"| %1.2f, %1.2f, %1.2f, %1.2f|\n"
		"| %1.2f, %1.2f, %1.2f, %1.2f]",
		title,
		m.m11, m.m12, m.m13, m.m14,
		m.m21, m.m22, m.m23, m.m24,
		m.m31, m.m32, m.m33, m.m34,
		m.m41, m.m42, m.m43, m.m44
		);
	DrawText(str, x, y, s, c);
}
