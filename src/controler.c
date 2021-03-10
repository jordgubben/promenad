#define IN_CONTROLER
#include "overview.h"
#include "linalg.h"

/**
Update all the things.
**/
void update_app(float dt, app_t *a) {

}



//// Limb kinematics ////
void reposition_limb_segments_with_fabrik(vec3_t origin, vec3_t end, limb_segment_t arr[], size_t num) {
	// Forward pass
	arr[num - 1].position = end;
	for (int i = num - 2; i >= 0 ; i--) {
		float d = vec3_distance(arr[i].position, arr[i + 1].position);
		vec3_t n = vec3_normal(vec3_between(arr[i].position, arr[i + 1].position));
		float constraint = arr[i + 1].distance;

		// Move forward along n if longer than the constraint
		// (and backwards if shorter than constraint)
		float change = d - constraint;
		arr[i].position.x += n.x * change;
		arr[i].position.y += n.y * change;
		arr[i].position.z += n.z * change;
	}

	// Inverse pass
	vec3_t prev_pos = origin;
	for (int i = 0; i < num; i++) {
		float d = vec3_distance(prev_pos, arr[i].position);
		vec3_t n = vec3_normal(vec3_between(prev_pos, arr[i].position));
		float constraint = arr[i].distance;

		// Move back along n if to far from previous position
		float change = d - constraint;
		arr[i].position.x -= n.x * change;
		arr[i].position.y -= n.y * change;
		arr[i].position.z -= n.z * change;

		// Save as previous position
		prev_pos = arr[i].position;
	}
}
