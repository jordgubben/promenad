#define IN_CONTROLER
#include "overview.h"

/**
Update all the things.
**/
void update_app(float dt, app_t *app) {
	if (app->paused) { return; }

	calculate_actor_transforms(&app->actors);

	// Set common end_effector for all limbs (for now)
	FOR_ROWS(l, app->limbs) {
		limb_id_t limb = get_limb_id(l, &app->limbs);
		app->limbs.end_effector[l] = app->common_end_effector;
	}

	move_limbs_towards_end_effectors(dt, &app->limbs);
}


//// Limb kinematics ////
/**
Gradually move limb segments toward their desired positions.
**/
void move_limbs_towards_end_effectors(float dt, limb_table_t *table) {
	FOR_ROWS(l, *table) {
		limb_id_t limb = get_limb_id(l, table);

		// Move limb segments
		limb_segment_t segments[32];
		size_t num_segments = collect_limb_segments(limb, table, segments, 32);
		vec3_t origin = table->position[l];
		vec3_t end_effector = table->end_effector[l];
		reposition_limb_segments_with_fabrik(origin, end_effector, segments, num_segments);

		// Reapply changes (gradually)
		uint16_t seg_index = table->root_segment[l];
		FOR_ITR(limb_segment_t, seg_itr, segments, num_segments) {
			// Difference
			vec3_t *current_pos = &table->segments[seg_index].position;
			vec3_t diff = vec3_between(*current_pos, seg_itr->position);

			// Move there in one second from now (🐢 ⬅️  🐰)
			current_pos->x += diff.x * dt;
			current_pos->y += diff.y * dt;
			current_pos->z += diff.z * dt;

			// Continue to next segment
			seg_index = table->segment_nodes[seg_index].next_index;
		}

	}
}

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

/**
Calculate a transformation matrix from the gven location.
**/
mat4_t mat4_from_location(location_t l) {
	mat4_t t = mat4_translate(l.position);
	mat4_t r = mat4_rotation_y(l.orientation_y);
	return mat4_mul(t, r);
}
