#include <assert.h>

#define IN_CONTROLER
#include "overview.h"

#ifdef LOG_CONTROLER
#include <stdio.h>
#define TRACE_FLOAT(f) printf("%s():%u \t| " #f " = %f\n", __func__, __LINE__, (f));
#define TRACE_VEC3(v) printf("%s():%u \t| " #v " = (%f, %f, %f)\n", __func__, __LINE__, (v).x, (v).y, (v).z)
#else
#define TRACE_FLOAT(_) // _
#define TRACE_VEC3(_) // _
#endif


static const float step_time = 1.f/60.f;

void accelrate_toward_goal_velocity(vec3_t target, float max_speed_change, vec3_t *current);
void update_leg_end_effectors(float dt,
	const actor_table_t *, const limb_attachment_table_t *, limb_goal_table_t *, limb_table_t *);

vec3_t calc_tip_pos(vec3_t joint_pos, quat_t ori, float length);

/**
Update all the things.
**/
void update_app(float dt, app_t *app) {
	if (app->paused) { return; }

	// Simulate in a fixed time step
	app->buffered_time += dt;
	if (app->buffered_time < step_time) {
		return;
	} else {
		app->buffered_time -= step_time;
	}

	// Keep history
	unsigned old_frame =  app->frame_count % max_pop_history_frames;
	app->frame_count++;
	unsigned new_frame = (app->frame_count % max_pop_history_frames);
	app->population_history[new_frame] = app->population_history[old_frame];
	population_t *pop = &app->population_history[new_frame];

	// Update world
	update_population(step_time, pop);
}

void update_population(float dt, population_t *pop) {

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
		int8_t curve_index = goals->curve_index[goal_index];
		vec3_t goal_pos = goals->curve_points[goal_index][curve_index];
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
	vec3_t diff_vel = vec3_between(*current_vel, goal_vel);
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

	// Speeds
	const float leg_acceleration = 50.f;
	const float leg_forward_speed = 4.f;
	const float leg_drop_speed = 1.5;

	// Limits
	const float front_limit_x = +0.75f;
	const float back_limit_x = -0.25f;

	FOR_ROWS(i, *leg_attachments) {
		actor_id_t actor = leg_attachments->owner[i];
		limb_id_t limb = leg_attachments->limb[i];
		int limb_index = get_limb_index(limb, limbs);

		// If paired with other leg
		if (limbs->paired_with[limb_index].id != limb.id) {
			// ..then skip this limb if it's parner is already going somewhere
			if (has_limb_goal(limbs->paired_with[limb_index], goals)) {
				continue;
			}
		}

		// Get transforms
		// (We will use them quite a bit)
		const mat4_t to_world = get_actor_to_world_transform(actor, actors);
		const mat4_t to_obj = get_actor_to_object_transform(actor, actors);

		// Root position in world and actors bject space
		const vec3_t leg_root_wpos = limbs->position[limb_index];
		const vec3_t leg_root_opos = mat4_mul_vec3(to_obj, leg_root_wpos, 1);

		// End effector in world and actors object space
		const vec3_t foot_wpos = get_limb_tip_position(limb, limbs);
		const vec3_t foot_opos = mat4_mul_vec3(to_obj, foot_wpos, 1);

		// Move foot forward if behind actor
		// (goal relative to root, i.e. hip joint)
		if (foot_opos.x < back_limit_x) {
			printf("Move foot [#%u|%u] forward!\n", limb.id, limb_index);
			vec3_t leg_goal_opos = vec3_add(leg_root_opos, vec3(+2.5f, -1,0));
			vec3_t leg_goal_wpos = mat4_mul_vec3(to_world, leg_goal_opos, 1);
			put_limb_goal(limb, leg_goal_wpos, leg_forward_speed, leg_acceleration, goals);
		}

		// Drop foot if in front of actor (and in air)
		if (foot_opos.x > front_limit_x && foot_wpos.y > 0 ) {
			printf("Move foot [#%u|%u] down!\n", limb.id, limb_index);
			vec3_t leg_goal_opos = foot_opos;
			vec3_t leg_goal_wpos = mat4_mul_vec3(to_world, leg_goal_opos, 1);
			leg_goal_wpos.y = 0;
			put_limb_goal(limb, leg_goal_wpos, leg_drop_speed, leg_acceleration, goals);
		}
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

	bone_t next_bone = {{jc_no_constraint}, end_pos, end_pos, quat_identity, 0.f};
	bone_constraint_e goal_constraint_type = jc_no_constraint;
	for (int i = num - 1; i >= 0 ; i--) {
		constrain_to_next_bone(&next_bone, &arr[i]);

		// Relative placement (after constrains)
		vec3_t b = vec3_between(arr[i].joint_pos, next_bone.joint_pos);
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
		next_bone = arr[i];
	}
}

/**
Constrain this bone relative to the next one (or the end effector semi bone).
**/
void constrain_to_next_bone(const bone_t *next_bone, bone_t *this_bone) {
	vec3_t b = vec3_between(this_bone->joint_pos, next_bone->joint_pos);
	TRACE_VEC3(b);

	switch (next_bone->constraint.type) {
		case jc_no_constraint: {} break;
		case jc_pole: { /* TODO */ } break;
		case jc_hinge: {
			// Local axies
			vec3_t next_forward = quat_rotate_vec3(next_bone->orientation, vec3(1,0,0));
			vec3_t next_up = quat_rotate_vec3(next_bone->orientation, vec3(0,1,0));
			TRACE_VEC3(next_forward);
			TRACE_VEC3(next_up);

			// Projection on local axies
			vec3_t n = vec3_normal(b);
			float bone_forward = vec3_dot(n, next_forward);
			float bone_up = vec3_dot(n, next_up);
			TRACE_FLOAT(bone_forward);
			TRACE_FLOAT(bone_up);

			// Angle?
			// atan(0, +1) = 0
			float angle = atan2(bone_up, bone_forward);
			angle = (angle > pi ? angle - tau : angle);
			TRACE_FLOAT(angle);
			TRACE_FLOAT(180 * angle / pi);

			// Clamp angle to constraint
			TRACE_FLOAT(next_bone->constraint.max_ang);
			TRACE_FLOAT(next_bone->constraint.min_ang);
			if (angle > next_bone->constraint.max_ang) { angle = next_bone->constraint.max_ang; }
			if (angle < next_bone->constraint.min_ang) { angle = next_bone->constraint.min_ang; }
			TRACE_FLOAT(angle);
			TRACE_FLOAT(180 * angle / pi);

			// New joint position from angle
			n = vec3_add(
				vec3_mul(next_forward, cos(angle)),
				vec3_mul(next_up, sin(angle)));
			TRACE_VEC3(n);

			this_bone->joint_pos = vec3_sub(
				next_bone->joint_pos,
				vec3_mul(n, this_bone->distance));
			TRACE_VEC3(this_bone->joint_pos);
		} break;
		case num_bone_constraints: { assert(false); } break;
	}
}


void apply_fabrik_inverse_pass(
		vec3_t root_pos, quat_t root_ori, const vec3_t end_pos,
		bone_t arr[], size_t num) {
	vec3_t calc_tip_pos(vec3_t joint_pos, quat_t ori, float length);

	// Inverse pass
	// (Pretend root is a limb segment without length)
	bone_t prev_bone = {{jc_no_constraint}, root_pos, root_pos, root_ori, 0};
	for (int i = 0; i < num; i++) {
		// Place joint at the previous tip
		arr[i].joint_pos = prev_bone.tip_pos;

		// Point bone towards next bone joint
		// (or the end effector if we are at the last joint)
		vec3_t next_pos  = (i+1 < num ? arr[i+1].joint_pos : end_pos) ;
		vec3_t new_dir = vec3_normal(vec3_between(arr[i].joint_pos, next_pos));
		vec3_t bone_dir = quat_rotate_vec3(arr[i].orientation, vec3(1,0,0));
		arr[i].orientation = quat_mul(quat_from_vec3_pair(bone_dir, new_dir), arr[i].orientation);

		constrain_to_prev_bone(&prev_bone, &arr[i]);

		// Calculate tip (secondary value)
		arr[i].tip_pos = calc_tip_pos(arr[i].joint_pos, arr[i].orientation, arr[i].distance);

		// Continue to the next one
		prev_bone = arr[i];
	}
}


void constrain_to_prev_bone(const bone_t *prev_bone, bone_t *this_bone) {
	vec3_t n = quat_rotate_vec3(this_bone->orientation, vec3_positive_x);

	switch (this_bone->constraint.type) {
		case jc_no_constraint: {} break;
		case jc_pole: {
			vec3_t prev_dir = quat_rotate_vec3(prev_bone->orientation, vec3(1,0,0));
			if (vec3_dot(prev_dir, n) < 1) {
				TRACE_VEC3(n);
				TRACE_VEC3(prev_dir);
				quat_t r = quat_from_vec3_pair(n, prev_dir);
				this_bone->orientation = quat_mul(r, this_bone->orientation);
				n = prev_dir;
			}
		} break;
		case jc_hinge: {
			// Local axies
			vec3_t local_forward = quat_rotate_vec3(prev_bone->orientation, vec3(1,0,0));
			vec3_t local_up = quat_rotate_vec3(prev_bone->orientation, vec3(0,1,0));
			TRACE_VEC3(local_forward);
			TRACE_VEC3(local_up);

			// Projection on local axies
			float bone_forward = vec3_dot(n, local_forward);
			float bone_up = vec3_dot(n, local_up);
			TRACE_FLOAT(bone_forward);
			TRACE_FLOAT(bone_up);

			// Angle?
			// atan(0, +1) = 0
			float angle = atan2(bone_up, bone_forward);
			angle = (angle > pi ? angle - tau : angle);
			TRACE_FLOAT(180 * angle / pi);

			// Clamp angle to constraint
			if (angle > this_bone->constraint.max_ang) { angle = this_bone->constraint.max_ang; }
			if (angle < this_bone->constraint.min_ang) { angle = this_bone->constraint.min_ang; }

			// New normal from angle
			n = vec3_add(
				vec3_mul(local_forward, cos(angle)),
				vec3_mul(local_up, sin(angle)));
		} break;
		case num_bone_constraints: { assert(false); } break;
	}

	// Rotate as little as posible
	vec3_t dir = quat_rotate_vec3(this_bone->orientation, vec3(1,0,0));
	this_bone->orientation = quat_mul(quat_from_vec3_pair(dir, n), this_bone->orientation);

	TRACE_VEC3(this_bone->joint_pos);
	TRACE_VEC3(get_bone_tip(*this_bone));
}

/**
Get world position of the given bones tip.

TODO: Gradually remove `tip_pos` field in favour of this.
**/
vec3_t get_bone_tip(bone_t bone) {
	return calc_tip_pos(bone.joint_pos, bone.orientation, bone.distance);
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
