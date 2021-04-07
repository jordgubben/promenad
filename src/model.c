#include <assert.h>
#include <raylib.h>

#define IN_MODEL
#include "overview.h"


//// Sparce table macros
#define T(t,r,c) (t).c[r]

#define T_HAS_ID(t, r) \
	((t).sparse_id[(r).id] < (t).num_rows && (t).dense_id[(t).sparse_id[(r).id]].id == (r).id)

#define T_INDEX(t,r) \
	(assert(T_HAS_ID((t), (r))), (t).sparse_id[(r).id])

#define T_ID(table, index) \
	(assert( (index) < (table).num_rows), (table).dense_id[index])

#define T_CELL(t, r, c) \
	(assert(T_HAS_ID(t,r)), (t).c[T_INDEX(t,r)])

#define T_SET_CELL(t, r, c, v) \
	(assert(T_HAS_ID(t,r)), (t).c[T_INDEX(t,r)] = (v))


void create_some_terrain(app_t *);

void init_cl_pool(cl_node_t [], size_t num_nodes);
unsigned short take_free_cl_node(cl_node_t []);
unsigned short append_cl_node_after(unsigned short anchor, cl_node_t []);

//// App

/**
Init all the things.
**/
void init_app(app_mode_e mode, app_t * app) {
	app->paused = false;
	app->step_once = false;
	app->buffered_time = 0;
	app->mode = mode;

	app->frame_count = 0;
	app->world_cursor = vec3(3, 2, 0);
	population_t *pop = &app->population_history[app->frame_count % max_pop_history_frames];

	init_limb_table(&pop->limbs);

	// Create a bunch of limbs with their roots in a grid
	if (mode == am_limb_forest) {
		FOR_RANGE(x, -5,5) {
			FOR_RANGE(z, -1, 2) {
				vec3_t pos = {1*x, 0, 3*z};
				limb_id_t id = create_limb(pos, quat_identity, &pop->limbs);

				// First segment
				pos.y += 1;
				add_bone_to_limb(id, pos, &pop->limbs);

				// Second segment
				FOR_IN(i, abs(x) * abs(x)) {
					pos.y += 0.5;
					add_bone_to_limb(id, pos, &pop->limbs);
				}
			}
		}
	}

	// Create actor model
	app->actor_model = malloc(sizeof(Model));
	*app->actor_model = LoadModelFromMesh(GenMeshCube(0.5f, 2.0f, 1.0f));

	// Setup single actor
	if (mode == am_single_actor) {
		create_person(vec3(0,3,0), 0, pop);
		create_some_terrain(app);
	}

	// Setup actor pair
	if (mode == am_actor_pair) {
		create_person(vec3(-2,+3,0), -pi/2, pop);
		create_person(vec3(+2,+3,0), -pi/2, pop);
		create_some_terrain(app);
	}

	// Setp row of actors
	if (mode == am_actor_row) {
		float movement_speed = 0.5;
		for (float z = +10; z >= -10; z -= 2.5) {
			actor_id_t actor = create_person(vec3(0, 3, z), 0, pop);
			vec3_t new_vel = vec3_mul(get_actor_forward_dir(actor, &pop->actors), movement_speed);
			movement_speed += 0.25;
			set_actor_velocity(actor, new_vel, &pop->actors);
		}
		create_some_terrain(app);
	}

	// Large arm from origo
	if (mode == am_robot_arm) {
		quat_t arm_ori = quat_from_axis_angle(vec3(0,0,1), pi/2);
		limb_id_t arm = create_limb(vec3_origo, arm_ori, &pop->limbs);

		// Segment #1
		uint16_t s1 = add_bone_to_limb(arm, vec3(0,3,0), &pop->limbs);
		apply_hinge_constraint(s1, -pi/2, +pi/2, &pop->limbs);

		// Segment #2
		uint16_t s2 = add_bone_to_limb(arm, vec3(0,6,0), &pop->limbs);
		apply_hinge_constraint(s2, -pi/3, +pi/3, &pop->limbs);

		// Segment 3
		uint16_t s3 = add_bone_to_limb(arm, vec3(0,9,0), &pop->limbs);
		apply_hinge_constraint(s3, -pi, +pi, &pop->limbs);

		set_limb_end_effector(arm, vec3(1,2,0), &pop->limbs);
		put_limb_goal(arm, vec3(0,5,0), 1, 10, &pop->limb_goals);
	}
}

void create_some_terrain(app_t *app) {
	// Some terrain (A simple staircase)
	create_terrain_block(5,6, -10, 10, 0.10, &app->landscape.ground);
	create_terrain_block(6,7, -10, 10, 0.25, &app->landscape.ground);
	create_terrain_block(7,8, -10, 10, 0.50, &app->landscape.ground);

	// More terain (Up and down)
	create_terrain_block(-3,-1, 2.25, 3.50, 0.3, &app->landscape.ground);
	create_terrain_block(-3,-1, 4.50, 5.75, 0.5, &app->landscape.ground);
	create_terrain_block(-3,-1, 7.25, 8.50, 0.3, &app->landscape.ground);
}

actor_id_t create_person(vec3_t center_point, float ori_y, population_t *pop) {
	limb_id_t create_arm(actor_id_t actor, vec3_t root_opos, population_t *);
	limb_id_t create_leg(actor_id_t actor, vec3_t root_opos, population_t *);

	actor_id_t actor = create_actor(center_point,  ori_y, &pop->actors);

	// Arms
	limb_id_t right_arm = create_arm(actor, vec3(0, +0.5, +1), pop);
	limb_id_t left_arm = create_arm(actor, vec3(0, +0.5, -1), pop);
	pair_limbs(left_arm, right_arm, &pop->limbs);

	// Legs
	limb_id_t right_leg = create_leg(actor, vec3(0, -1.5, +0.5), pop);
	limb_id_t left_leg = create_leg(actor, vec3(0, -1.5, -0.5), pop);
	pair_limbs(left_leg, right_leg, &pop->limbs);

	return actor;
}


limb_id_t create_arm(actor_id_t actor, vec3_t root_opos, population_t *pop) {
	mat4_t to_world = get_actor_to_world_transform(actor, &pop->actors);

	// Arm root
	vec3_t root_wpos = mat4_mul_vec3(to_world, root_opos, +1);
	limb_id_t arm = create_limb(root_wpos, quat_identity, &pop->limbs);
	attach_limb_to_actor(arm, actor, &pop->limbs, &pop->actors, &pop->arms);

	// Upper arm
	vec3_t elbow_opos = vec3_add(root_opos, vec3(0,-0.75, root_opos.z));
	vec3_t elbow_wpos = mat4_mul_vec3(to_world, elbow_opos, 1);
	uint16_t shoulder = add_bone_to_limb(arm, elbow_wpos, &pop->limbs);

	// Lower arm
	vec3_t wrist_opos = vec3_add(elbow_opos, vec3(0, -0.75, root_opos.z));
	vec3_t wrist_wpos = mat4_mul_vec3(to_world, wrist_opos, 1);
	uint16_t elbow = add_bone_to_limb(arm, wrist_wpos, &pop->limbs);

	//  Swing it
	create_limb_swing(arm, &pop->limbs, &pop->limb_swings);

	return arm;
}


limb_id_t create_leg(actor_id_t actor, vec3_t root_opos, population_t *pop) {
	mat4_t to_world = get_actor_to_world_transform(actor, &pop->actors);

	// Limb root
	vec3_t root_wpos = mat4_mul_vec3(to_world, root_opos, +1);
	limb_id_t leg = create_limb(root_wpos, quat_identity, &pop->limbs);
	attach_limb_to_actor(leg, actor, &pop->limbs, &pop->actors, &pop->legs);

	// Hip
	vec3_t knee_opos = vec3_add(root_opos, vec3(0.1, -0.75, 0));
	vec3_t knee_wpos = mat4_mul_vec3(to_world, knee_opos, 1);
	uint16_t hip = add_bone_to_limb(leg, knee_wpos,  &pop->limbs);
	apply_hinge_constraint(hip, -0.6 * pi, 0.4 * pi, &pop->limbs);

	// Knee
	vec3_t ancle_opos = vec3_add(root_opos, vec3(0.0, -1.5, 0));
	vec3_t ancle_wpos = mat4_mul_vec3(to_world, ancle_opos, 1);
	uint16_t knee = add_bone_to_limb(leg, ancle_wpos, &pop->limbs);
	apply_hinge_constraint(knee, -0.9 * pi, 0 * pi, &pop->limbs);

	return leg;
}

/**
Terminate all the things.
**/
void term_app(app_t *app) {
	// Free actor model
	UnloadModel(*app->actor_model);
	free(app->actor_model);
	app->actor_model = NULL;
}


//// Actor CRUD
/**
Create a single actor.
**/
actor_id_t create_actor(vec3_t pos, float rot, actor_table_t *table) {
	assert(table->num_rows < max_actor_table_rows);

	// Add row to sparse set
	actor_id_t actor_id = { table->next_id++};
	int index = table->num_rows++;
	table->sparse_id[actor_id.id] = index;
	table->dense_id[index] = actor_id;

	// Set row data
	location_t loc = {pos, rot};
	table->location[index] = loc;
	table->movement[index] = (movement_t){vec3(0,0,0), 0};
	table->to_world[index] = to_world_from_location(loc);
	table->to_object[index] = to_object_from_location(loc);

	return actor_id;
}

actor_id_t get_actor_id(uint16_t index, const actor_table_t *table) {
	return T_ID(*table, index);
}

uint16_t get_actor_index(actor_id_t actor, const actor_table_t *table) {
	return T_INDEX(*table, actor);
}

bool actor_exists(actor_id_t actor, const actor_table_t *table) {
	return T_HAS_ID(*table, actor);
}

vec3_t get_actor_forward_dir(actor_id_t actor, const actor_table_t *table) {
	return mat4_mul_vec3(table->to_world[T_INDEX(*table, actor)], vec3(1,0,0), 0);
}

mat4_t get_actor_to_object_transform(actor_id_t actor, const actor_table_t *table) {
	return table->to_object[T_INDEX(*table, actor)];
}

mat4_t get_actor_to_world_transform(actor_id_t actor, const actor_table_t *table) {
	return table->to_world[T_INDEX(*table, actor)];
}


/**
Get actors velocity in its own coordinate space (i.e. +x == forward).
**/
vec3_t get_actor_velocity_in_object_space(actor_id_t actor, const actor_table_t * table) {
	mat4_t to_obj = get_actor_to_object_transform(actor, table);
	vec3_t wvel = table->movement[T_INDEX(*table, actor)].velocity;
	vec3_t ovel = mat4_mul_vec3(to_obj, wvel, 0);
	return ovel;
}


/**
Set actor velocity (in world space).
**/
void set_actor_velocity(actor_id_t actor, vec3_t new_vel, actor_table_t *table) {
	table->movement[T_INDEX(*table, actor)].velocity = new_vel;
}

/**
Calculate the transform for every actors location.
**/
void calculate_actor_transforms(actor_table_t *table) {
	FOR_ROWS(a, *table) {
		table->to_world[a] = to_world_from_location(table->location[a]);
		table->to_object[a] = to_object_from_location(table->location[a]);
	}
}


//// Limb CRUD

/**
Init the given limb table.
**/
void init_limb_table(limb_table_t *table) {
	init_cl_pool(table->bone_nodes, max_limb_table_segnemts);
}

/**
Create a limb at the given position.
**/
limb_id_t create_limb(vec3_t pos, quat_t ori, limb_table_t *table) {
	assert(table->num_rows < max_limb_table_rows);

	// Add row to sparse set
	limb_id_t limb_id = { table->next_id++};
	int index = table->num_rows++;
	table->sparse_id[limb_id.id] = index;
	table->dense_id[index] = limb_id;

	// Set row data
	table->position[index] = pos;
	table->orientation[index] = ori;
	table->root_bone[index] = 0;
	table->paired_with[index] = limb_id;

	return limb_id;
}

/**
Get the current limb at the given index.
**/
limb_id_t get_limb_id(uint16_t index, const limb_table_t *table) {
	return T_ID(*table, index);
}


/**
Get the index of the given limb.
**/
uint16_t get_limb_index(limb_id_t limb, const limb_table_t *table) {
	return T_INDEX(*table, limb);
}

/**
Get the world space position of the given limb.
**/
vec3_t get_limb_position(limb_id_t limb, const limb_table_t *table) {
	return T_CELL(*table, limb, position);
}


/**
Get the world position of the bones joint.
**/
vec3_t get_bone_joint_position(uint16_t bone_index, const limb_table_t *table) {
	assert(bone_index < max_limb_table_segnemts);
	return table->bones[bone_index].joint_pos;
}


/**
Get the (calculated) world position of the bones tip.
**/
vec3_t get_bone_tip_position(uint16_t bone_index, const limb_table_t *table) {
	assert(bone_index < max_limb_table_segnemts);
	vec3_t joint_pos = table->bones[bone_index].joint_pos;
	quat_t joint_ori = table->bones[bone_index].orientation;
	float length = table->bones[bone_index].distance;
	return vec3_add(joint_pos, quat_rotate_vec3(joint_ori, vec3(length, 0,0)));
}


/**
Get the position of the given limbs outermost bone tip.
**/
vec3_t get_limb_tip_position(limb_id_t limb, const limb_table_t *table) {
	int limb_index = T_INDEX(*table, limb);

	// Get the root bone node
	uint16_t root_bone_index = table->root_bone[limb_index];
	assert(root_bone_index);
	const cl_node_t *root_bone_node = &table->bone_nodes[root_bone_index];

	// Get the tip position
	// (The node "before" the root is the last in a cyclic list)
	return get_bone_tip_position(root_bone_node->prev_index, table);
}

/**
Get the world space end effector position for the given limb,
**/
vec3_t get_limb_end_effector_position(limb_id_t limb, const limb_table_t *table) {
	return T_CELL(*table, limb, end_effector);
}


/**
Collect limb bones into an array (with max size).
**/
size_t collect_bones(limb_id_t limb, const limb_table_t *table, bone_t out[], size_t max) {
	int root_seg = table->root_bone[T_INDEX(*table, limb)];
	if (!root_seg) { return 0; }

	for (int i = 0, seg = root_seg;  i < max ; i++) {
		out[i] = table->bones[seg];
		seg = table->bone_nodes[seg].next_index;
		if (seg == root_seg) { return i + 1; }
	}

	return max;
}


/**
Set the end effector of the given limb.
**/
void set_limb_end_effector(limb_id_t limb, vec3_t pos, limb_table_t * table) {
	table->end_effector[T_INDEX(*table, limb)] = pos;
}


/**
Add a segment at the end of the given limb.

Bonus: Place limb end effector at the tip of the new limb segment.
**/
uint16_t add_bone_to_limb(limb_id_t limb, vec3_t pos, limb_table_t *table) {
	bone_t bone_from_root_tip(vec3_t root, vec3_t tip);

	int limb_index = T_INDEX(*table, limb);

	// Update end effector
	table->end_effector[limb_index] = pos;

	// Add limb
	int root_seg = table->root_bone[limb_index];
	if (root_seg == 0) {
		// Add first node
		int new_seg = take_free_cl_node(table->bone_nodes);
		table->root_bone[limb_index] = new_seg;

		// Set properties
		vec3_t limb_pos = table->position[limb_index];
		table->bones[new_seg] = bone_from_root_tip(limb_pos, pos);
		return new_seg;
	} else {
		// Insert at end
		uint16_t last_seg = table->bone_nodes[root_seg].prev_index;
		int new_seg = append_cl_node_after(last_seg, table->bone_nodes);

		// Set properties
		vec3_t last_seg_pos = get_bone_tip(table->bones[last_seg]);
		table->bones[new_seg] = bone_from_root_tip(last_seg_pos, pos);
		return new_seg;
	}
}


/**
Couple two limbs with each other.
**/
void pair_limbs(limb_id_t l1, limb_id_t l2, limb_table_t *table) {
	table->paired_with[T_INDEX(*table, l1)] = l2;
	table->paired_with[T_INDEX(*table, l2)] = l1;
}

void apply_pole_constraint(uint16_t bone_index, limb_table_t *table) {
	assert(bone_index < max_limb_table_segnemts);
	table->bones[bone_index].constraint.type = jc_pole;
}


void apply_hinge_constraint(uint16_t bone_index, float min_ang, float max_ang, limb_table_t *table) {
	assert(bone_index < max_limb_table_segnemts);
	table->bones[bone_index].constraint.type = jc_hinge;
	table->bones[bone_index].constraint.min_ang = min_ang;
	table->bones[bone_index].constraint.max_ang = max_ang;
}

/*
Create a bone that tstretches from one point to another.
*/
bone_t bone_from_root_tip(vec3_t joint_pos, vec3_t tip_pos) {
	quat_t orientation = quat_from_vec3_pair(vec3(1,0,0), vec3_between(joint_pos, tip_pos));
	bone_t bone = {
		{jc_no_constraint, 0.f, 0.f},
		joint_pos, orientation, vec3_distance(joint_pos, tip_pos),
		};
	return bone;
}


//// Limb attachment CRUD
void attach_limb_to_actor(
		limb_id_t limb, actor_id_t actor,
		const limb_table_t *limbs, const actor_table_t *actors,
		limb_attachment_table_t *table
		) {

	assert(table->num_rows < max_limb_attachment_table_rows);

	int n = table->num_rows++;
	table->owner[n] = actor;
	table->limb[n] = limb;

	// Relative limb placement
	vec3_t p = get_limb_position(limb, limbs);
	mat4_t to_obj = get_actor_to_object_transform(actor, actors);
	table->relative_position[n] = mat4_mul_vec4(to_obj, vec4_from_vec3(p, 1)).vec3;
}

/**
Snap limb positions in place relative to their owning actor.
**/
void reposition_attached_limbs(
		const limb_attachment_table_t * attachments, const actor_table_t *actors,
		limb_table_t *limbs
		) {

	FOR_ROWS(la, *attachments) {
		int actor_index = T_INDEX(*actors, attachments->owner[la]);
		int limb_index = T_INDEX(*limbs, attachments->limb[la]);

		// Reposition limb
		vec3_t p = attachments->relative_position[la];
		p = mat4_mul_vec4(actors->to_world[actor_index], vec4_from_vec3(p, 1)).vec3;
		limbs->position[limb_index] = p;

		// Reorient limb
		float ori_y = actors->location[actor_index].orientation_y;
		limbs->orientation[limb_index] = quat_from_axis_angle(vec3_positive_y, ori_y);
	}
}

//// Limb link CRUD

void link_limb_to(limb_id_t l1, limb_id_t l2, limb_link_table_t *table) {
	// Figgure out where to put the data
	int index;
	if (T_HAS_ID(*table, l1)) {
		index = T_INDEX(*table, l1);
	} else {
		// Check tha there is room
		assert(table->num_rows < max_limb_goal_table_rows);

		// Add new row to sparse set
		index = table->num_rows++;
		table->sparse_id[l1.id] = index;
		table->dense_id[index] = l1;
	}

	// Set row data
	table->other_limb[index] = l2;
}


/**
Does this limb have a link to another limb?
**/
bool limb_has_link(limb_id_t limb, const limb_link_table_t *table) {
	return T_HAS_ID(*table, limb);
}


/**
Remove link to other limb.
**/
void unlink_limb(limb_id_t limb, limb_link_table_t *table) {
	int index = T_INDEX(*table, limb);

	// Update meta
	int m = --table->num_rows;
	table->dense_id[index] = table->dense_id[m];
	table->sparse_id[table->dense_id[index].id] = index;

	// Update data
	table->other_limb[index] = table->other_limb[m];
}


//// Limb goal CRUD

/**
Give the limb end effector a new goal.

Maintains velocity if it replaces an older goal.
**/
void put_limb_goal(limb_id_t limb, vec3_t pos, float max_speed, float max_acc, limb_goal_table_t *table) {
	// Figgure out where to put the data
	int index;
	if (T_HAS_ID(*table, limb)) {
		index = T_INDEX(*table, limb);
	} else {
		// Check tha there is room
		assert(table->num_rows < max_limb_goal_table_rows);

		// Add new row to sparse set
		index = table->num_rows++;
		table->sparse_id[limb.id] = index;
		table->dense_id[index] = limb;

		// Reset velocity
		table->velocity[index] = vec3(0,0,0);
	}

	// Set row data
	table->curve_index[index] = 0;
	table->curve_length[index] = 1;
	table->curve_points[index][table->curve_index[index]] = pos;
	table->max_speed[index] = max_speed;
	table->max_acceleration[index] = max_acc;
	table->threshold[index] = 0.1;
}

/**
Add another point to a goal (or create the goal if does not exist already).
**/
void push_limb_goal(limb_id_t limb, vec3_t pos, float speed, float acc, limb_goal_table_t *table) {
	if (has_limb_goal(limb, table)) {
		int goal_index = T_INDEX(*table, limb);
		assert(table->curve_length[goal_index] < max_limb_goal_curve_points);
		table->curve_points[goal_index][table->curve_length[goal_index]++] = pos;
	} else {
		put_limb_goal(limb, pos, speed, acc, table);
	}
}

/**
Does this limb have a goal?
**/
bool has_limb_goal(limb_id_t limb, const limb_goal_table_t *table) {
	return T_HAS_ID(*table, limb);
}


/**
Delete all goals there the limb is close enough to it's intended destination.
**/
void delete_accomplished_limb_goals(const limb_table_t *limbs, limb_goal_table_t *goals) {
	FOR_ROWS(goal_index, * goals) {
		// Get limb data
		limb_id_t limb = goals->dense_id[goal_index];
		vec3_t ee_pos = get_limb_end_effector_position(limb, limbs);

		// Get goal data
		int8_t curve_index = goals->curve_index[goal_index];
		vec3_t goal_pos = goals->curve_points[goal_index][curve_index];
		float threshold = goals->threshold[goal_index];

		// Advance only if close enough
		float distance = vec3_distance(ee_pos, goal_pos);
		if (distance > threshold) { continue; }
		goals->curve_index[goal_index]++;

		// Remove if at the end of the curve
		if (goals->curve_index[goal_index] >= goals->curve_length[goal_index]) {
			delete_limb_goal_at_index(goal_index--, goals);
		}
	}
}


void delete_limb_goal_at_index(unsigned index, limb_goal_table_t *table) {
	assert(index < table->num_rows);

	// Remove data by copying another row
	unsigned m = --table->num_rows;
	table->dense_id[index] = table->dense_id[m];
	for (int i = 0; i < table->curve_length[m]; i++) {
		table->curve_points[index][i] = table->curve_points[m][i];
	}
	table->curve_index[index] = table->curve_index[m];
	table->curve_length[index] = table->curve_length[m];
	table->velocity[index] = table->velocity[m];
	table->max_speed[index] = table->max_speed[m];
	table->max_acceleration[index] = table->max_acceleration[m];
	table->threshold[index] = table->threshold[m];

	// Update sparse set
	table->sparse_id[table->dense_id[index].id] = index;
}


//// Limb swing CRUD ////
/**
Create a swing behaviour for the given limb.

TODO: Make this actually swing the limb like a pendelum.
**/
void create_limb_swing(limb_id_t limb, const limb_table_t *limbs, limb_swing_table_t *table) {
	int index;
	if (T_HAS_ID(*table, limb)) {
		index = T_INDEX(*table, limb);
	} else {
		// Check that there is room
		assert(table->num_rows < max_limb_swing_table_rows);

		// Add new row to sparse set
		index = table->num_rows++;
		table->sparse_id[limb.id] = index;
		table->dense_id[index] = limb;
	}

	table->prev_position[index] = get_limb_tip_position(limb, limbs);
}


//// Terrain CRUD ////


/**
Add one  block to terrain.
**/
void create_terrain_block(float x1, float x2, float z1, float z2, float height, terrain_table_t *table) {
	assert(table->num_rows < max_terrain_table_rows);

	table->block[table->num_rows].x1 = minf(x1, x2);
	table->block[table->num_rows].x2 = maxf(x1, x2);
	table->block[table->num_rows].z1 = minf(z1, z2);
	table->block[table->num_rows].z2 = maxf(z1, z2);
	table->block[table->num_rows].height = height;
	table->num_rows++;
}


/**
Get the height above the y-plane at (x,z).
**/
float get_terrain_height(float x, float z, const terrain_table_t *table) {
	float h = 0;
	FOR_ROWS(i, *table) {
		if ( x < table->block[i].x1) { continue; }
		if ( x > table->block[i].x2) { continue; }
		if ( z < table->block[i].z1) { continue; }
		if ( z > table->block[i].z2) { continue; }
		h = maxf(h, table->block[i].height);
	}
	return h;
}
//// Cyclic list ////

/*
Init a node pool shared by several cyclic lists.
*/
void init_cl_pool(cl_node_t nodes[], size_t num_nodes) {
	FOR_IN(i, num_nodes) {
		nodes[i].prev_index = (uint16_t) ((i - 1) % num_nodes);
		nodes[i].next_index = (uint16_t) ((i + 1) % num_nodes);
	}
}


/*
Take a free cyclic list node from the pool.
*/
unsigned short take_free_cl_node(cl_node_t pool[]) {
	assert(pool[0].prev_index != 0 && pool[0].next_index != 0);

	// Find a free node in the pool
	unsigned short n = pool[0].prev_index;

	// Reconnect neithbours
	pool[pool[n].prev_index].next_index = pool[n].next_index;
	pool[pool[n].next_index].prev_index = pool[n].prev_index;

	// Disconnect from former neighbours
	pool[n].prev_index = n;
	pool[n].next_index = n;

	return n;
}

/*
Append cyclic list node after.
*/
unsigned short append_cl_node_after(unsigned short anchor, cl_node_t pool[]) {
	unsigned int new_node = take_free_cl_node(pool);

	// Connect new node to neighbours
	pool[new_node].prev_index = anchor;
	pool[new_node].next_index = pool[anchor].next_index;

	// Insert new node into list
	pool[pool[new_node].prev_index].next_index = new_node;
	pool[pool[new_node].next_index].prev_index = new_node;

	return new_node;
}
