#include <raylib.h>

#define IN_VIEW
#include "overview.h"


//// Input ////

void process_input(float dt, app_t *app) {
	// Toggle pause
	if (IsKeyPressed(KEY_P)) { app->paused = !app->paused; }

	// Move common end effector with arrow keys
	if (IsKeyDown(KEY_RIGHT)) { app->common_end_effector.x += dt; }
	if (IsKeyDown(KEY_LEFT)) { app->common_end_effector.x -= dt; }
	if (IsKeyDown(KEY_UP)) { app->common_end_effector.y += dt; }
	if (IsKeyDown(KEY_DOWN)) { app->common_end_effector.y -= dt; }
}

//// Rendering ////

/**
Render all the things.
**/
void render_app(const app_t *app) {
	// Define the camera to look into our 3d world
	Camera3D camera = { 0 };
	camera.position = (Vector3){ 10.0f, 10.0f, 10.0f };
	camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
	camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
	camera.fovy = 45.0f;
	camera.type = CAMERA_PERSPECTIVE;
	SetCameraMode(camera, CAMERA_FREE);
	UpdateCamera(&camera);

	// Render something at origo
	BeginMode3D(camera);
	{
		DrawModel(*app->actor_model, vec3(0,1,0).rl, 1.0f, BLUE);

		DrawSphere(app->common_end_effector.rl, 0.1f, GOLD);
		{
			vec3_t shadow = app->common_end_effector;
			shadow.y = 0;
			DrawSphere(shadow.rl, 0.1f, ORANGE);
		}

		render_limb_skeletons(app->common_end_effector, &app->limbs);

		DrawGrid(10, 1.f);
	}
	EndMode3D();
}


void render_limb_skeletons(vec3_t end_effector, const limb_table_t *table) {
	FOR_ROWS(l, *table) {
		row_id_t limb = get_limb_id(l, table);

		// Render root
		vec3_t pos = table->position[l];
		DrawSphere(pos.rl, 0.1, BLACK);

		// Render segments in their current positions
		int segment = table->root_segment[l];
		while (segment) {
			vec3_t seg_pos = table->segments[segment].position;
			DrawLine3D(pos.rl, seg_pos.rl, ORANGE);
			DrawSphere(seg_pos.rl, 0.05, MAROON);

			// Next segment (if any)
			pos = seg_pos;
			segment = table->segment_nodes[segment].next_index;
			if (segment == table->root_segment[l]) { segment = 0; }
		}

		// Render limb segments IK target positions
		limb_segment_t segments[32];
		size_t num_segments = collect_limb_segments(limb, table, segments, 32);
		vec3_t origin = table->position[l];
		reposition_limb_segments_with_fabrik(origin, end_effector, segments, num_segments);
		pos = origin;
		FOR_ITR(limb_segment_t, seg_itr, segments, num_segments) {
			DrawLine3D(pos.rl, seg_itr->position.rl, BLUE);
			pos = seg_itr->position;
		}

	}
}
