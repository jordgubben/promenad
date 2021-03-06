#include <assert.h>
#include <stdio.h>

#define IN_KINEMATICS
#include "overview.h"

#ifdef LOG_CONTROLER
#define TRACE_FLOAT(f) printf("%s():%u \t| " #f " = %f\n", __func__, __LINE__, (f));
#define TRACE_VEC3(v) printf("%s():%u \t| " #v " = (%f, %f, %f)\n", __func__, __LINE__, (v).x, (v).y, (v).z)
#else
#define TRACE_FLOAT(_) // _
#define TRACE_VEC3(_) // _
#endif


static const int num_fabrik_passes = 3;

void accelrate_toward_goal_velocity(vec3_t target, float max_speed_change, vec3_t *current);

vec3_t get_bone_forward(const bone_t *b) { return quat_rotate_vec3(b->orientation, vec3_positive_x); }
vec3_t get_bone_up(const bone_t *b) { return quat_rotate_vec3(b->orientation, vec3_positive_y); }
vec3_t get_bone_right(const bone_t *b) { return quat_rotate_vec3(b->orientation, vec3_positive_z); }

vec3_t calc_tip_pos(vec3_t joint_pos, quat_t ori, float length);


//// Actor movement ////

/**
Update all actor locations based on their movement and delta time.
**/
void move_actors(float dt, actor_table_t *table) {
	move_locations(dt, table->movement, table->num_rows, table->location);
}


/**
Move all locations based on current movement and delta time.
**/
void move_locations(float dt, const movement_t m[], size_t num, location_t l[]) {
	FOR_IN(i, num) {
		add_vec3(vec3_mul(m[i].velocity, dt), &l[i].position);
		l[i].orientation_y += dt * m[i].rotation_y;
	}
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


/**
Move legs forward one at the time.
**/
void animate_walking_actor_legs(float dt, const animation_env_i *env) {

	// Get what you need
	const actor_table_t *actors = env->actors;
	const limb_attachment_table_t *leg_attachments = env->leg_attachments;
	const terrain_table_t * ground = env->ground;
	limb_goal_table_t *goals = env->goals;
	limb_table_t *limbs = env->limbs;

	// Speeds
	const float leg_acceleration_factor = 30;
	const float leg_forward_speed_factor = 1.75;
	const float leg_drop_speed_factor = 3;

	// Step curve positions (relative to root joint)
	const float up_x = 0.75;
	const float step_height = 0.5;
	const float contact_x = 1.5f;

	FOR_ROWS(i, *leg_attachments) {
		actor_id_t actor = leg_attachments->owner[i];
		limb_id_t limb = leg_attachments->limb[i];
		int limb_index = get_limb_index(limb, limbs);

		// Current forward velocity
		const float vel_x = get_actor_velocity_in_object_space(actor, actors).x;
		if (vel_x <= 0) { continue; }

		// Finnish what you..
		if (has_limb_goal(limb, goals)) { continue; }

		// Let other leg finnish
		limb_id_t other_limb = limbs->paired_with[limb_index];
		if (other_limb.id != limb.id && has_limb_goal(other_limb, goals)) {
			continue;
		}

		// Get transforms
		const mat4_t to_obj = get_actor_to_object_transform(actor, actors);

		// This foots position in world and actors object space
		const vec3_t this_foot_wpos = get_limb_tip_position(limb, limbs);
		const vec3_t this_foot_opos = mat4_mul_vec3(to_obj, this_foot_wpos, 1);

		// Use other limb if it's further behind
		if (limb.id != other_limb.id) {
			const vec3_t other_foot_wpos = get_limb_tip_position(other_limb, limbs);
			const vec3_t other_foot_opos = mat4_mul_vec3(to_obj, other_foot_wpos, 1);
			if (other_foot_opos.x < this_foot_opos.x) {
				continue;
			}
		}

		// Move foot forward only if behind actor
		if (this_foot_opos.x >= 0) { continue; }

		// Start where the foot's actually at right now
		limbs->end_effector[limb_index] = get_limb_tip_position(limb, limbs);

		// Root position in world and actors bject space
		const vec3_t leg_root_wpos = limbs->position[limb_index];
		const vec3_t leg_root_opos = mat4_mul_vec3(to_obj, leg_root_wpos, 1);

		// Start moving
		printf("Move foot [#%u|%u] forward!\n", limb.id, limb_index);
		const float leg_acceleration = vel_x * leg_acceleration_factor;

		// First lift foot forward
		const mat4_t to_world = get_actor_to_world_transform(actor, actors);
		{
			vec3_t leg_goal_opos = vec3_add(leg_root_opos, vec3(up_x, 0, 0));
			vec3_t leg_goal_wpos = mat4_mul_vec3(to_world, leg_goal_opos, 1);
			leg_goal_wpos.y =
				step_height + get_terrain_height(leg_goal_wpos.x, leg_goal_wpos.z, ground);
			const float speed = vel_x * leg_forward_speed_factor;
			put_limb_goal(limb, leg_goal_wpos, speed, leg_acceleration, goals);
		}

		// Then drop foot once in front of actor
		{
			vec3_t leg_goal_opos = vec3_add(leg_root_opos, vec3(contact_x, 0,0));
			vec3_t leg_goal_wpos = mat4_mul_vec3(to_world, leg_goal_opos, 1);
			leg_goal_wpos.y =
				get_terrain_height(leg_goal_wpos.x, leg_goal_wpos.z, ground);
			const float speed = vel_x * leg_forward_speed_factor;
			push_limb_goal(limb, leg_goal_wpos, speed, leg_acceleration, goals);
		}
	}
}

/**
Keep actors at a fixed height above the ground.
**/
void keep_actors_actors_above_ground(float h, const terrain_table_t *ground, actor_table_t *actors) {
	FOR_ROWS(a, *actors) {
		vec3_t *pos = &actors->location[a].position;
		pos->y = get_terrain_height(pos->x, pos->z, ground) + h;
	}
}


//// Limb linking (hand holding) ////

/**
Move the end effecto of a linked limb to the tip the limb it is linked to.
**/
void move_limb_tips_to_their_linked_partners(const limb_link_table_t *links, limb_table_t * limbs) {
	FOR_ROWS(link_index, *links) {
		limb_id_t this_limb = links->dense_id[link_index];
		limb_id_t other_limb = links->other_limb[link_index];

		vec3_t ee_pos = get_limb_end_effector_position(this_limb, limbs);
		vec3_t tip_pos = get_limb_tip_position(other_limb, limbs);
		vec3_t new_pos = vec3_lerp(0.5, ee_pos, tip_pos);
		set_limb_end_effector(this_limb, new_pos, limbs);
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
	// (iterate multiple times for better stability)
	bone_t bones[32];
	size_t num_bones = collect_bones(limb, table, bones, 32);
	vec3_t root_pos = table->position[limb_index];
	quat_t root_ori = table->orientation[limb_index];
	FOR_IN(i, num_fabrik_passes) {
		reposition_bones_with_fabrik(root_pos, root_ori, end_pos, bones, num_bones);
	}

	// Reapply changes (directly)
	uint16_t seg_index = table->root_bone[limb_index];
	FOR_ITR(bone_t, seg_itr, bones, num_bones) {
		// Overwrite
		bone_t *current_seg = &table->bones[seg_index];
		current_seg->joint_pos = seg_itr->joint_pos;
		current_seg->orientation = seg_itr->orientation;

		// Continue to next bone
		seg_index = table->bone_nodes[seg_index].next_index;
	}
}

/**
Apply FABRIK (Forward and Backwards Reaching Inverse Kinnematics) to given array of bones.
**/
void reposition_bones_with_fabrik(
		vec3_t root_pos, quat_t root_ori, const vec3_t end_pos,
		bone_t arr[], size_t num) {
	void apply_fabrik_forward_pass(vec3_t origin, const vec3_t end_pos, bone_t arr[], size_t num);
	void apply_fabrik_inverse_pass(vec3_t, quat_t, const vec3_t end_pos, bone_t arr[], size_t num);

	apply_fabrik_forward_pass(root_pos, end_pos, arr, num);
	apply_fabrik_inverse_pass(root_pos, root_ori, end_pos, arr, num);
}

void apply_fabrik_forward_pass(vec3_t origin, const vec3_t end_pos, bone_t arr[], size_t num) {

	bone_t next_bone = {{jc_no_constraint}, end_pos, quat_identity, 0.f};
	for (int i = num - 1; i >= 0 ; i--) {

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
		vec3_t dir = get_bone_forward(&arr[i]);
		arr[i].orientation = quat_mul(quat_from_vec3_pair(dir, n), arr[i].orientation);

		constrain_to_next_bone(&next_bone, &arr[i]);

		// Continue to the next one
		next_bone = arr[i];
	}
}

/**
Constrain this bone relative to the next one (or the end effector semi-bone).
**/
void constrain_to_next_bone(const bone_t *next_bone, bone_t *this_bone) {
	vec3_t b = vec3_between(this_bone->joint_pos, next_bone->joint_pos);

	switch (next_bone->constraint.type) {
		case jc_no_constraint: {} break;
		case jc_pole: { /* TODO */ } break;
		case jc_hinge: {
			// Local axies
			vec3_t next_forward = get_bone_forward(next_bone);
			vec3_t next_up = get_bone_up(next_bone);
			vec3_t next_side = get_bone_right(next_bone);

			// Align hinge axis with next bone
			this_bone->orientation = quat_mul(
				quat_from_vec3_pair(get_bone_right(this_bone), next_side),
				this_bone->orientation
				);
			vec3_t n = get_bone_forward(this_bone);

			// Projection on local axies
			float bone_forward = vec3_dot(n, next_forward);
			float bone_up = vec3_dot(n, next_up);
			TRACE_FLOAT(bone_forward);
			TRACE_FLOAT(bone_up);

			// Angle?
			// atan(0, +1) = 0
			float angle = atan2(bone_up, bone_forward);
			angle = (angle > pi ? angle - tau : angle);
			TRACE_FLOAT(180 * angle / pi);

			// Clamp angle to constraint
			if (angle > next_bone->constraint.max_ang) { angle = next_bone->constraint.max_ang; }
			if (angle < next_bone->constraint.min_ang) { angle = next_bone->constraint.min_ang; }
			TRACE_FLOAT(180 * angle / pi);

			// Reset orientation
			quat_t new_rot = quat_from_axis_angle(next_side, angle);
			this_bone->orientation = quat_mul(new_rot, next_bone->orientation);

			// Move this bones tip to the next ones joint
			vec3_t move = vec3_between(get_bone_tip(*this_bone), next_bone->joint_pos);
			this_bone->joint_pos = vec3_add(this_bone->joint_pos, move);
			TRACE_VEC3(this_bone->joint_pos);
		} break;
		case num_bone_constraints: { assert(false); } break;
	}
}


void apply_fabrik_inverse_pass(
		vec3_t root_pos, quat_t root_ori, const vec3_t end_pos,
		bone_t arr[], size_t num) {

	// Inverse pass
	// (Pretend root is a limb segment without length)
	bone_t prev_bone = {{jc_no_constraint}, root_pos, root_ori, 0};
	for (int i = 0; i < num; i++) {
		// Place joint at the previous tip
		arr[i].joint_pos = get_bone_tip(prev_bone);

		// Point bone towards next bone joint
		// (or the end effector if we are at the last joint)
		vec3_t next_pos  = (i+1 < num ? arr[i+1].joint_pos : end_pos) ;
		vec3_t new_dir = vec3_normal(vec3_between(arr[i].joint_pos, next_pos));
		vec3_t bone_dir = get_bone_forward(&arr[i]);
		arr[i].orientation = quat_mul(quat_from_vec3_pair(bone_dir, new_dir), arr[i].orientation);

		constrain_to_prev_bone(&prev_bone, &arr[i]);

		// Continue to the next one
		prev_bone = arr[i];
	}
}


void constrain_to_prev_bone(const bone_t *prev_bone, bone_t *this_bone) {

	switch (this_bone->constraint.type) {
		case jc_no_constraint: {} break;
		case jc_pole: {
			vec3_t this_dir = get_bone_forward(this_bone);
			vec3_t prev_dir = get_bone_forward(prev_bone);
			if (vec3_dot(prev_dir, this_dir) < 1) {
				quat_t r = quat_from_vec3_pair(this_dir, prev_dir);
				this_bone->orientation = quat_mul(r, this_bone->orientation);
			}
		} break;
		case jc_hinge: {
			// Local axies
			vec3_t local_forward = get_bone_forward(prev_bone);
			vec3_t local_up = get_bone_up(prev_bone);
			vec3_t local_side = get_bone_right(prev_bone);

			// Align hinge axis with previous bone
			this_bone->orientation = quat_mul(
				quat_from_vec3_pair(get_bone_right(this_bone), local_side),
				this_bone->orientation
				);

			// Projection on local axies
			vec3_t n = get_bone_forward(this_bone);
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
			TRACE_FLOAT(180 * angle / pi);

			// Reset orientation
			quat_t new_rot = quat_from_axis_angle(local_side, angle);
			this_bone->orientation = quat_mul(new_rot, prev_bone->orientation);
		} break;
		case num_bone_constraints: { assert(false); } break;
	}

}


//// Limb momentum ////

/**
Perpetuate motion from the previous simulation step.
**/
void perpetuate_limb_momentums(float dt, limb_swing_table_t *momentums, limb_table_t *limbs) {

	FOR_ROWS(momentum_index, *momentums) {
		limb_id_t limb = momentums->dense_id[momentum_index];
		int limb_index = get_limb_index(limb, limbs);

		// Get data
		vec3_t prev_pos = momentums->prev_position[momentum_index];
		vec3_t curr_pos = get_limb_tip_position(limb, limbs);

		// Move things (assuming fixed time step)
		vec3_t last_move = vec3_between(prev_pos, curr_pos);
		vec3_t next_pos = vec3_add(curr_pos, last_move);
		limbs->end_effector[limb_index] = next_pos;

		// Save position for next pass
		momentums->prev_position[momentum_index] = curr_pos;
	}
}


/**
Apply gravity to limbs with momentum.
**/
void apply_gravity_to_limbs(float dt, vec3_t gravity, limb_swing_table_t *momentums, limb_table_t *limbs) {
	vec3_t gravity_step = vec3_mul(gravity, dt * dt / 2);

	FOR_ROWS(momentum_index, *momentums) {
		limb_id_t limb = momentums->dense_id[momentum_index];
		int limb_index = get_limb_index(limb, limbs);

		// Move end effectors
		add_vec3(gravity_step, &limbs->end_effector[limb_index]);
	}
}


//// Misc. ////

/**
Get world position of the given bones tip.
**/
vec3_t get_bone_tip(bone_t bone) {
	return calc_tip_pos(bone.joint_pos, bone.orientation, bone.distance);
}

vec3_t calc_tip_pos(vec3_t joint_pos, quat_t ori, float length) {
	vec3_t n = quat_rotate_vec3(ori, vec3_positive_x);
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
