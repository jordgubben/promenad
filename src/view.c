#include <raylib.h>

#define IN_VIEW
#include "overview.h"

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
		Vector3 origo = { 0,0,0};
		DrawSphere(origo, 1, RED);

		render_limb_skeletons(&app->limbs);

		DrawGrid(10, 1.f);
	}
	EndMode3D();
}


void render_limb_skeletons(const limb_table_t *table) {
	FOR_ROWS(l, *table) {
		vec3_t pos = table->position[l];
		DrawSphere(pos.rl, 0.1, BLACK);

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
	}
}
