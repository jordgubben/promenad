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

	reposition_attached_limbs(&app->limb_attachments, &app->actors, &app->limbs);
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
			limb_segment_t *current_seg = &table->segments[seg_index];

			// Move tip there in one second from now (🐢 ⬅️  🐰)
			vec3_t tip_diff = vec3_between(current_seg->tip_pos, seg_itr->tip_pos);
			vec3_t tip_change = vec3_mul(tip_diff, dt);
			current_seg->tip_pos = vec3_add(current_seg->tip_pos, tip_change);

			// Move joint there in one second from now (🐢 ⬅️  🐰)
			vec3_t joint_diff = vec3_between(current_seg->joint_pos, seg_itr->joint_pos);
			vec3_t joint_change = vec3_mul(joint_diff, dt);
			current_seg->joint_pos = vec3_add(current_seg->joint_pos, joint_change);

			// Adjust orientation
			current_seg->orientation =
				quat_from_vec3_pair(vec3(1,0,0), vec3_between(current_seg->joint_pos, current_seg->tip_pos));

			// Continue to next segment
			seg_index = table->segment_nodes[seg_index].next_index;
		}

	}
}

void reposition_limb_segments_with_fabrik(vec3_t origin, vec3_t end, limb_segment_t arr[], size_t num) {
	// Forward pass
	arr[num - 1].joint_pos = vec3_add(arr[num -1].joint_pos, vec3_between(arr[num -1].tip_pos, end));
	arr[num - 1].tip_pos = end;
	arr[num - 1].orientation =
		quat_from_vec3_pair(vec3(1,0,0), vec3_between(arr[num-1].joint_pos, arr[num-1].tip_pos));
	for (int i = num - 2; i >= 0 ; i--) {
		float d = vec3_distance(arr[i].tip_pos, arr[i + 1].tip_pos);
		vec3_t n = vec3_normal(vec3_between(arr[i].tip_pos, arr[i + 1].tip_pos));
		float length = arr[i + 1].distance;

		// Move forward along n if longer than the constraint
		// (and backwards if shorter than constraint)
		float change = d - length;
		arr[i].tip_pos = vec3_add(arr[i].tip_pos, vec3_mul(n, change));
		arr[i].joint_pos = vec3_sub(arr[i].tip_pos, vec3_mul(n, +1 * length));
		arr[i].orientation =
			quat_from_vec3_pair(vec3(1,0,0), vec3_between(arr[i].joint_pos, arr[i].tip_pos));
	}

	// Inverse pass
	vec3_t prev_pos = origin;
	for (int i = 0; i < num; i++) {
		float d = vec3_distance(prev_pos, arr[i].tip_pos);
		vec3_t n = vec3_normal(vec3_between(prev_pos, arr[i].tip_pos));
		float length = arr[i].distance;

		// Move back along if to far from previous position
		float change = d - length;
		arr[i].tip_pos = vec3_sub(arr[i].tip_pos, vec3_mul(n, change));
		arr[i].joint_pos = vec3_sub(arr[i].tip_pos, vec3_mul(n, +1 * length));
		arr[i].orientation =
			quat_from_vec3_pair(vec3(1,0,0), vec3_between(arr[i].joint_pos, arr[i].tip_pos));

		// Save as previous position
		prev_pos = arr[i].tip_pos;
	}
}

/**
Calculate 'to world' transformation matrix from the gven location.
**/
mat4_t to_world_from_location(location_t l) {
	mat4_t t = mat4_translate(l.position);
	mat4_t r = mat4_rotation_y(l.orientation_y);
	return mat4_mul(t, r);
}


/**
Calculate 'to object' transformation matrix from the gven location.
**/
mat4_t to_object_from_location(location_t l) {
	mat4_t t = mat4_translate(vec3_mul(l.position, -1));
	mat4_t r = mat4_rotation_y(-l.orientation_y);
	return mat4_mul(r, t);
}
