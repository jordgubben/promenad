#include <stdio.h>
#include <raylib.h>

#define IN_VIEW
#include "overview.h"

#define DRAW_COORDINATE_SYSTEM_HELPERS 1


//// Rendering ////

void render_orientation_gizmo(float l, vec3_t pos, quat_t ori);
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

		DrawSphere(app->world_cursor.rl, 0.1f, GOLD);
		{
			vec3_t shadow = app->world_cursor;
			float x = app->world_cursor.x;
			float z = app->world_cursor.z;
			shadow.y = get_terrain_height(x, z, &app->landscape.ground);
			DrawSphere(shadow.rl, 0.1f, ORANGE);
		}

		render_limb_skeletons(&pop->limbs);
		render_limb_goals(&pop->limb_goals, &pop->limbs);

#if DRAW_COORDINATE_SYSTEM_HELPERS
		DrawCube(vec3(5,0,0).rl, 1.f, .1f, .1f, RED);
		DrawCube(vec3(0,5,0).rl, .1f, 1.f, .1f, GREEN);
		DrawCube(vec3(0,0,5).rl, .1f, .1f, 1.f, BLUE);
#endif // DRAW_COORDINATE_SYSTEM_HELPERS

		render_terrain(&app->landscape.ground);

		DrawGrid(20, 1.f);
	}
	EndMode3D();

	// Actor debug
	if (pop->actors.num_rows > 0) {
		draw_matrix_as_text("Actor 'to world' transform", pop->actors.to_world[0], 100, 10, 15, BLACK);
		draw_matrix_as_text("Actor 'to object' transform", pop->actors.to_object[0], 300, 10, 15, BLACK);
	}

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

void render_limb_skeletons(const limb_table_t *table) {
	void render_bone_joint_orientations(vec3_t, bone_t [], size_t);

	FOR_ROWS(l, *table) {
		limb_id_t limb = get_limb_id(l, table);

		// Render root
		const vec3_t root_pos = table->position[l];
		const quat_t root_ori = table->orientation[l];
		render_orientation_gizmo(0.6, root_pos, root_ori);
		DrawSphere(root_pos.rl, 0.15, BLACK);

		// Render individual end effectors
		const vec3_t end_effector_pos = table->end_effector[l];
		DrawSphere(end_effector_pos.rl, 0.05, GOLD);

		// Render distance from limb tip to end effector
		const vec3_t limb_tip_pos = get_limb_tip_position(limb, table);
		DrawLine3D(end_effector_pos.rl, limb_tip_pos.rl, PURPLE);

		// Render bones in their current positions
		int bone = table->root_bone[l];
		while (bone) {
			bone_t seg = table->bones[bone];
			vec3_t tip_pos = get_bone_tip_position(bone, table);
			DrawLine3D(seg.joint_pos.rl, tip_pos.rl, GRAY);
			DrawSphere(seg.joint_pos.rl, 0.10, MAROON);
			DrawSphere(tip_pos.rl, 0.05, MAROON);

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


void render_limb_goals(const limb_goal_table_t *goals, const limb_table_t *limbs) {
	FOR_ROWS(goal_index, *goals) {
		// Get the data
		limb_id_t limb = goals->dense_id[goal_index];
		int limb_index = get_limb_index(limb, limbs);
		vec3_t prev_pos = limbs->end_effector[limb_index];
		float threshold = goals->threshold[goal_index];

		// Render the remaining curve
		FOR_RANGE(i, goals->curve_index[goal_index], goals->curve_length[goal_index]) {
			vec3_t curve_pos = goals->curve_points[goal_index][i];
			DrawLine3D(prev_pos.rl, curve_pos.rl, LIME);
			DrawSphere(curve_pos.rl, threshold, LIME);
			prev_pos = curve_pos;
		}
	}
}


void render_bone_joint_orientations(vec3_t origin_pos, bone_t bones[], size_t num_bones) {
	// Draw joint spaces
	FOR_IN(i, num_bones) {
		vec3_t joint_pos = bones[i].joint_pos;
		quat_t joint_ori = bones[i].orientation;
		render_orientation_gizmo(0.3, joint_pos, joint_ori);
	}
}

//// Landscape ////

void render_terrain(const terrain_table_t *table) {
	FOR_ROWS(i, *table) {
		vec3_t center = {
			(table->block[i].x1 + table->block[i].x2)/2.f,
			table->block[i].height/2.f,
			(table->block[i].z1 + table->block[i].z2)/2.f,
		};
		vec3_t size = {
			table->block[i].x2 - table->block[i].x1,
			table->block[i].height,
			table->block[i].z2 - table->block[i].z1,
		};
		Color color = GRAY;
		color.g = 128 + (127.f * table->block[i].height * 2.f);
		DrawCubeV(center.rl, size.rl, color);
	}
}


//// Helpers ////

void render_orientation_gizmo(float l, vec3_t pos, quat_t ori) {
	DrawLine3D(pos.rl, vec3_add(pos, quat_rotate_vec3(ori, vec3(l,0,0))).rl, RED);
	DrawLine3D(pos.rl, vec3_add(pos, quat_rotate_vec3(ori, vec3(0,l,0))).rl, GREEN);
	DrawLine3D(pos.rl, vec3_add(pos, quat_rotate_vec3(ori, vec3(0,0,l))).rl, BLUE);
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
