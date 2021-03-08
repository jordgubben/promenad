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

		DrawGrid(10, 1.f);
	}
	EndMode3D();
}
