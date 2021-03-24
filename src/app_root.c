#include <stdlib.h>
#include <stdio.h>
#include <raylib.h>

#define IN_APP_ROOT
#include "overview.h"

static app_t app= { 0 };

float minf(float a, float b) { return (a > b ? a : b); }

int main(int argc, char** argv) {
#define PRINT_SIZE_OF(t) printf("%s: %zub\n", #t, sizeof(t))
	PRINT_SIZE_OF(vec3_t);
	PRINT_SIZE_OF(vec4_t);
	PRINT_SIZE_OF(mat4_t);
	PRINT_SIZE_OF(limb_table_t);
	PRINT_SIZE_OF(app_t);

	// Get things up and running
	InitWindow(640, 480, "Hello, Promenad!");
	SetTargetFPS(144);

	// Define the camera to look into our 3d world
	Camera3D camera = { 0 };
	camera.position = (Vector3){ 10.0f, 10.0f, 10.0f };
	camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
	camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
	camera.fovy = 45.0f;
	camera.type = CAMERA_PERSPECTIVE;
	SetCameraMode(camera, CAMERA_FREE);
	UpdateCamera(&camera);

	// App setup
	init_app(am_single_actor, &app);

	// Ah-Gogogoggogogogo!
	while(!WindowShouldClose()) {
		// Update
		float dt = minf(GetFrameTime(), 1.f/30.f);
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
