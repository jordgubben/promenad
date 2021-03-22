#include <stdio.h>
#include <raylib.h>

#define IN_VIEW
#include "overview.h"

#define DRAW_COORDINATE_SYSTEM_HELPERS 1

//// Input ////

void process_input(float dt, app_t *app) {
	population_t *pop = &app->population_history[app->frame_count % max_pop_history_frames];

	// Toggle pause
	if (IsKeyPressed(KEY_P)) { app->paused = !app->paused; }

	// Rewind
	if (IsKeyDown(KEY_R) && app->frame_count >= 2) { app->frame_count -= 2; }

	// Move common end effector with arrow keys
	if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
		if (IsKeyDown(KEY_RIGHT)) { pop->common_end_effector.z -= dt; }
		if (IsKeyDown(KEY_LEFT)) { pop->common_end_effector.z += dt; }
	} else {
		if (IsKeyDown(KEY_RIGHT)) { pop->common_end_effector.x += dt; }
		if (IsKeyDown(KEY_LEFT)) { pop->common_end_effector.x -= dt; }
	}

	if (IsKeyDown(KEY_UP)) { pop->common_end_effector.y += dt; }
	if (IsKeyDown(KEY_DOWN)) { pop->common_end_effector.y -= dt; }

	// Tank controls
	if (IsKeyDown(KEY_W)) {
		pop->actors.location[0].position = vec3_add(
			pop->actors.location[0].position,
			vec3_mul(get_actor_forward_dir(get_actor_id(0, &pop->actors), &pop->actors), dt * 2)
			);
	}
	if (IsKeyDown(KEY_S)) {
		pop->actors.location[0].position = vec3_sub(
			pop->actors.location[0].position,
			vec3_mul(get_actor_forward_dir(get_actor_id(0, &pop->actors), &pop->actors), dt * 2)
			);
	}
	if (IsKeyDown(KEY_A)) { pop->actors.location[0].orientation_y += dt * 0.25 * tau; }
	if (IsKeyDown(KEY_D)) { pop->actors.location[0].orientation_y -= dt * 0.25 * tau; }
}

//// Rendering ////

void render_orientation_gizmo(vec3_t pos, quat_t ori);
void draw_matrix_as_text(const char* title, mat4_t m, float x, float y, float s, Color c);

/**
Render all the things.
**/
void render_app(const struct Camera3D *camera,  const app_t *app) {
	const population_t *pop = &app->population_history[app->frame_count % max_pop_history_frames];

	// Render something at origo
	BeginMode3D(*camera);
	{
		render_actors(app->actor_model, &pop->actors);

		DrawSphere(pop->common_end_effector.rl, 0.1f, GOLD);
		{
			vec3_t shadow = pop->common_end_effector;
			shadow.y = 0;
			DrawSphere(shadow.rl, 0.1f, ORANGE);
		}

		render_limb_skeletons(pop->common_end_effector, &pop->limbs);

#if DRAW_COORDINATE_SYSTEM_HELPERS
		DrawCube(vec3(5,0,0).rl, 1.f, .1f, .1f, RED);
		DrawCube(vec3(0,5,0).rl, .1f, 1.f, .1f, GREEN);
		DrawCube(vec3(0,0,5).rl, .1f, .1f, 1.f, BLUE);
#endif // DRAW_COORDINATE_SYSTEM_HELPERS

		DrawGrid(10, 1.f);
	}
	EndMode3D();

	draw_matrix_as_text("Actor 'to world' transform", pop->actors.to_world[0], 100, 10, 15, BLACK);
	draw_matrix_as_text("Actor 'to object' transform", pop->actors.to_object[0], 300, 10, 15, BLACK);

	// Frame count
	{
		char str[128];
		snprintf(str, 128, "Frame:\n #%02u", app->frame_count);
		DrawText(str, 0, 24, 20, DARKGREEN);
	}
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

		vec3_t nose_pos = mat4_mul_vec3(table->to_world[a], vec3(0.4, 0.5, 0), 1.f);
		DrawSphere(nose_pos.rl, 0.2, PINK);
	}
}

void render_limb_skeletons(vec3_t end_effector, const limb_table_t *table) {
	void render_bone_joint_orientations(vec3_t, bone_t [], size_t);

	FOR_ROWS(l, *table) {
		limb_id_t limb = get_limb_id(l, table);

		// Render root
		const vec3_t root_pos = table->position[l];
		const quat_t root_ori = table->orientation[l];
		render_orientation_gizmo(root_pos, root_ori);
		DrawSphere(root_pos.rl, 0.15, BLACK);

		// Render individual end effectors
		const vec3_t end_effector_pos = table->end_effector[l];
		DrawSphere(end_effector_pos.rl, 0.05, GOLD);

		// Render bones in their current positions
		int bone = table->root_bone[l];
		while (bone) {
			bone_t seg = table->bones[bone];
			DrawLine3D(seg.joint_pos.rl, seg.tip_pos.rl, GRAY);
			DrawSphere(seg.joint_pos.rl, 0.10, MAROON);
			DrawSphere(seg.tip_pos.rl, 0.05, MAROON);

			// Next bone (if any)
			bone = table->bone_nodes[bone].next_index;
			if (bone == table->root_bone[l]) { bone = 0; }
		}

		// Render limb bones orientation gizmoz
		bone_t bones[32];
		size_t num_bones = collect_bones(limb, table, bones, 32);
		render_bone_joint_orientations(root_pos, bones, num_bones);

		// Render pairing
		limb_id_t paired_limb = table->paired_with[l];
		if (limb.id < paired_limb.id) {
			vec3_t other_root_pos = get_limb_position(paired_limb, table);
			DrawLine3D(root_pos.rl, other_root_pos.rl, BLACK);
		}
	}
}


void render_bone_joint_orientations(vec3_t origin_pos, bone_t bones[], size_t num_bones) {
	// Draw joint spaces
	FOR_IN(i, num_bones) {
		vec3_t joint_pos = bones[i].joint_pos;
		quat_t joint_ori = bones[i].orientation;
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
