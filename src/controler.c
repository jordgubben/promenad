#include <assert.h>

#define IN_CONTROLER
#include "overview.h"

#ifdef LOG_CONTROLER
#include <stdio.h>
#define TRACE_VEC3(v) printf("%s():%u \t| " #v " = (%f, %f, %f)\n", __func__, __LINE__, (v).x, (v).y, (v).z)
#else
#define TRACE_VEC3(_) (_ = _)
#endif


void accelrate_toward_goal_velocity(vec3_t target, float max_speed_change, vec3_t *current);
void update_leg_end_effectors(float dt,
	const actor_table_t *, const limb_attachment_table_t *, limb_goal_table_t *, limb_table_t *);

/**
Update all the things.
**/
void update_app(float dt, app_t *app) {
	if (app->paused) { return; }

	// Keep history
	unsigned old_frame =  app->frame_count % max_pop_history_frames;
	app->frame_count++;
	unsigned new_frame = (app->frame_count % max_pop_history_frames);
	app->population_history[new_frame] = app->population_history[old_frame];
	population_t *pop = &app->population_history[new_frame];

	calculate_actor_transforms(&pop->actors);

	// Move limbs attached to actors
	reposition_attached_limbs(&pop->arms, &pop->actors, &pop->limbs);
	reposition_attached_limbs(&pop->legs, &pop->actors, &pop->limbs);

	// Update kinematics
	move_limbs_toward_goals(dt, &pop->limb_goals, &pop->limbs);
	update_leg_end_effectors(dt, &pop->actors, &pop->legs, &pop->limb_goals, &pop->limbs);
	move_limbs_directly_to_end_effectors(&pop->limbs);
	delete_accomplished_limb_goals(&pop->limbs, &pop->limb_goals);
}


//// Actor animation ////


/**
Move limb end effectors towards their goals.
**/
void move_limbs_toward_goals(float dt, limb_goal_table_t *goals, limb_table_t *limbs) {

	FOR_ROWS(goal_index, *goals) {
		// Get limb data
		limb_id_t limb = goals->dense_id[goal_index];
		int limb_index = get_limb_index(limb, limbs);
		vec3_t ee_pos = limbs->end_effector[limb_index];

		// Get goal data
		vec3_t goal_pos = goals->goal_position[goal_index];
		float max_speed = goals->max_speed[goal_index];
		float acceleration = goals->max_acceleration[goal_index];
		float max_speed_change = acceleration * dt;

		// Move end effectors
		vec3_t target_vel = vec3_mul(vec3_direction(ee_pos, goal_pos), max_speed);
		accelrate_toward_goal_velocity(target_vel, max_speed_change, &goals->velocity[goal_index]);
		limbs->end_effector[limb_index] =
			vec3_add(limbs->end_effector[limb_index], vec3_mul(goals->velocity[goal_index], dt));
	}

}


/*
Accelrate towards the given goal velocity, limited by the given max speed change.
*/
void accelrate_toward_goal_velocity(vec3_t  goal_vel, float max_speed_change, vec3_t *current_vel) {
	vec3_t diff_vel = vec3_direction(*current_vel, goal_vel);
	float diff_speed = vec3_length(diff_vel);

	// Snap to goal velocity or nudge toward it
	if (diff_speed <= max_speed_change) {
		*current_vel = goal_vel;
	} else {
		current_vel->x += max_speed_change * (diff_vel.x / diff_speed);
		current_vel->y += max_speed_change * (diff_vel.y / diff_speed);
		current_vel->z += max_speed_change * (diff_vel.z / diff_speed);
	}
}


void update_leg_end_effectors(float dt,
		const actor_table_t *actors,
		const limb_attachment_table_t *leg_attachments,
		limb_goal_table_t *goals,
		limb_table_t *limbs) {

	const float leg_acceleration = 100.f;
	const float leg_forward_speed = 4.f;
	const float leg_lift_speed = 0.5;
	const float leg_drop_speed = 1.5;

	FOR_ROWS(i, *leg_attachments) {
		actor_id_t actor = leg_attachments->owner[i];
		limb_id_t limb = leg_attachments->limb[i];
		int limb_index = get_limb_index(limb, limbs);

		// End effector in actors object space
		const mat4_t to_world = get_actor_to_world_transform(actor, actors);
		const mat4_t to_obj = get_actor_to_object_transform(actor, actors);
		const vec3_t leg_ee_wpos = limbs->end_effector[limb_index];
		vec3_t leg_ee_opos = mat4_mul_vec3(to_obj, leg_ee_wpos, 1);

		// Move foot forward if behind actor
		if (leg_ee_opos.x < -1) {
			vec3_t leg_goal_opos = vec3_add(leg_ee_opos, vec3(3,1,0));
			vec3_t leg_goal_wpos = mat4_mul_vec3(to_world, leg_goal_opos, 1);
			put_limb_goal(limb, leg_goal_wpos, leg_forward_speed, leg_acceleration, goals);
		}

		// Drop foot if in front of actor
		if (leg_ee_opos.x > +1 && leg_ee_wpos.y > 0 ) {
			leg_ee_opos.y -= dt * leg_drop_speed;
		}

		// Snap foot placement in front of actor
		leg_ee_opos.z = leg_attachments->relative_position[i].z;

		// Transform back to worls space
		limbs->end_effector[limb_index] = mat4_mul_vec3(to_world, leg_ee_opos, 1);
	}
}

//// Limb kinematics ////

/**
Use IK to move all limbs to (or as close as possible to) their end effectors.
**/
void move_limbs_directly_to_end_effectors(limb_table_t *table) {
	FOR_ROWS(limb_index, *table) {
		limb_id_t limb = get_limb_id(limb_index, table);
		move_limb_directly_to(limb, table->end_effector[limb_index], table);
	}
}

void move_limb_directly_to(limb_id_t limb, vec3_t end_pos, limb_table_t *table) {
	int limb_index = get_limb_index(limb, table);

	// Move copy of limb bones
	bone_t bones[32];
	size_t num_bones = collect_bones(limb, table, bones, 32);
	vec3_t root_pos = table->position[limb_index];
	quat_t root_ori = table->orientation[limb_index];
	reposition_bones_with_fabrik(root_pos, root_ori, end_pos, bones, num_bones);

	// Reapply changes (directly)
	uint16_t seg_index = table->root_bone[limb_index];
	FOR_ITR(bone_t, seg_itr, bones, num_bones) {
		// Overwrite
		bone_t *current_seg = &table->bones[seg_index];
		current_seg->joint_pos = seg_itr->joint_pos;
		current_seg->tip_pos = seg_itr->tip_pos;
		current_seg->orientation =
			quat_from_vec3_pair(
				vec3(1,0,0),
				vec3_between(current_seg->joint_pos, current_seg->tip_pos));

		// Continue to next bone
		seg_index = table->bone_nodes[seg_index].next_index;
	}
}

/**
Gradually move limb bones toward their desired positions.

(Deprecated or put on hold)
**/
void move_limbs_gradually_towards_end_effectors(float dt, limb_table_t *table) {
	FOR_ROWS(l, *table) {
		limb_id_t limb = get_limb_id(l, table);

		// Move limb bones
		bone_t bones[32];
		size_t num_bones = collect_bones(limb, table, bones, 32);
		vec3_t root_pos = table->position[l];
		quat_t root_ori = table->orientation[l];
		vec3_t end_effector = table->end_effector[l];
		reposition_bones_with_fabrik(root_pos, root_ori, end_effector, bones, num_bones);

		// Reapply changes (gradually)
		uint16_t seg_index = table->root_bone[l];
		FOR_ITR(bone_t, seg_itr, bones, num_bones) {
			// Difference
			bone_t *current_seg = &table->bones[seg_index];

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
			seg_index = table->bone_nodes[seg_index].next_index;
		}

	}
}

void reposition_bones_with_fabrik(
		vec3_t root_pos, quat_t root_ori, const vec3_t end_pos,
		bone_t arr[], size_t num) {
	void apply_fabrik_forward_pass(vec3_t origin, const vec3_t end_pos, bone_t arr[], size_t num);
	void apply_fabrik_inverse_pass(vec3_t, quat_t, const vec3_t end_pos, bone_t arr[], size_t num);

	apply_fabrik_forward_pass(root_pos, end_pos, arr, num);
	apply_fabrik_inverse_pass(root_pos, root_ori, end_pos, arr, num);
}

void apply_fabrik_forward_pass(vec3_t origin, const vec3_t end_pos, bone_t arr[], size_t num) {
	vec3_t calc_tip_pos(vec3_t joint_pos, quat_t ori, float length);

	// Forward pass
	vec3_t goal_pos = end_pos;
	quat_t goal_ori = quat_identity;
	bone_constraint_e goal_constraint_type = jc_no_constraint;
	for (int i = num - 1; i >= 0 ; i--) {
		vec3_t b = vec3_between(arr[i].joint_pos, goal_pos);

		// Constrain to child
		switch (goal_constraint_type) {
			case jc_no_constraint: {} break;
			case jc_hinge: {
				vec3_t axis = quat_rotate_vec3(goal_ori, vec3(0,0,1));
				vec3_t projection = vec3_mul(b, vec3_dot(axis, b));
				arr[i].joint_pos = vec3_sub(arr[i].joint_pos, projection);
				b = vec3_between(arr[i].joint_pos, goal_pos);
			} break;
			case num_bone_constraints: { assert(false); } break;
		}

		// Relative placement (after constrains)
		float d = vec3_length(b);
		vec3_t n = vec3_normal(b);
		float length = arr[i].distance;

		// Move forward along n if longer than the constraint
		// (and backwards if shorter than constraint)
		float change = d - length;
		arr[i].joint_pos = vec3_add(arr[i].joint_pos, vec3_mul(n, change));

		// Rotate as little as possible
		vec3_t dir = quat_rotate_vec3(arr[i].orientation, vec3(1,0,0));
		arr[i].orientation = quat_mul(quat_from_vec3_pair(dir, n), arr[i].orientation);

		// Calculate tip (secondary value)
		arr[i].tip_pos = calc_tip_pos(arr[i].joint_pos, arr[i].orientation, length);

		// Continue to the next one
		goal_pos = arr[i].joint_pos;
		goal_ori = arr[i].orientation;
		goal_constraint_type = arr[i].constraint.type;
	}
}

void apply_fabrik_inverse_pass(
		vec3_t root_pos, quat_t root_ori, const vec3_t end_pos,
		bone_t arr[], size_t num) {
	vec3_t calc_tip_pos(vec3_t joint_pos, quat_t ori, float length);

	// Inverse pass
	// (Pretend root is a limb segment without length)
	vec3_t prev_tip_pos = root_pos;
	quat_t prev_ori = root_ori;
	for (int i = 0; i < num; i++) {
		// Place joint at the previous tip
		arr[i].joint_pos = prev_tip_pos;

		// Point bone towards next bone joint
		// (or the end effector if we are at the last joint)
		vec3_t next_pos  = (i+1 < num ? arr[i+1].joint_pos : end_pos) ;
		vec3_t n = vec3_normal(vec3_between(arr[i].joint_pos, next_pos));

		// Constrain segment to parrent
		switch (arr[i].constraint.type) {
			case jc_no_constraint: {} break;
			case jc_pole: {
				vec3_t prev_dir = quat_rotate_vec3(prev_ori, vec3(1,0,0));
				if (vec3_dot(prev_dir, n) < 1) {
					TRACE_VEC3(n);
					TRACE_VEC3(prev_dir);
					quat_t r = quat_from_vec3_pair(n, prev_dir);
					arr[i].orientation = quat_mul(r, arr[i].orientation);
					n = prev_dir;
				}
			} break;
			case jc_hinge: {
				vec3_t axis = quat_rotate_vec3(prev_ori, vec3(0,0,1));
				vec3_t projection = vec3_mul(axis, vec3_dot(axis, n));
				n = vec3_normal(vec3_sub(n, projection));
			} break;
			case num_bone_constraints: { assert(false); } break;
		}

		// Rotate as little as posible
		vec3_t dir = quat_rotate_vec3(arr[i].orientation, vec3(1,0,0));
		arr[i].orientation = quat_mul(quat_from_vec3_pair(dir, n), arr[i].orientation);

		// Calculate tip (secondary value)
		arr[i].tip_pos = calc_tip_pos(arr[i].joint_pos, arr[i].orientation, arr[i].distance);

		// Continue to the next one
		prev_tip_pos = arr[i].tip_pos;
		prev_ori = arr[i].orientation;
	}
}

vec3_t calc_tip_pos(vec3_t joint_pos, quat_t ori, float length) {
	vec3_t n = quat_rotate_vec3(ori, vec3(1,0,0));
	return vec3_add(joint_pos, vec3_mul(n, length));
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
