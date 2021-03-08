#include <stdlib.h>
#include <stdio.h>
#include <raylib.h>

#define IN_APP_ROOT
#include "overview.h"

float minf(float a, float b) { return (a > b ? a : b); }

int main(int argc, char** argv) {
	// Get things up and running
	InitWindow(640, 480, "Hello, Promenad!");
	SetTargetFPS(144);

	// App setup
	app_t app= {};
	init_app(&app);

	// Ah-Gogogoggogogogo!
	while(!WindowShouldClose()) {
		// Update
		float dt = minf(GetFrameTime(), 1.f/30.f);
		update_app(dt, &app);

		// Render
		BeginDrawing();
		ClearBackground(RAYWHITE);
		render_app(&app);
		DrawFPS(0,0);
		EndDrawing();
	}

	CloseWindow();
	return 0;
}
