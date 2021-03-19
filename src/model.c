#include <assert.h>
#include <raylib.h>

#define IN_MODEL
#include "overview.h"

#define EXAMPLE_ACTORS 1
#define EXAMPLE_ARM 0
#define LIMB_FOREST 0

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


void init_cl_pool(cl_node_t [], size_t num_nodes);
unsigned short take_free_cl_node(cl_node_t []);
unsigned short append_cl_node_after(unsigned short anchor, cl_node_t []);

//// App

/**
Init all the things.
**/
void init_app(app_t * app) {
	app->paused = false;
	app->common_end_effector = vec3(3, 2, 0);

	init_limb_table(&app->limbs);

#if LIMB_FOREST
	// Create a bunch of limbs with their roots in a grid
	FOR_RANGE(x, -5,5) {
		FOR_RANGE(z, -1, 2) {
			vec3_t pos = {1*x, 0, 3*z};
			limb_id_t id = create_limb(pos, &app->limbs);

			// First segment
			pos.y += 1;
			add_segment_to_limb(id, pos, &app->limbs);

			// Second segment
			FOR_IN(i, abs(x) * abs(x)) {
				pos.y += 0.5;
				add_segment_to_limb(id, pos, &app->limbs);
			}
		}
	}
#endif

	// Create actor model
	app->actor_model = malloc(sizeof(Model));
	*app->actor_model = LoadModelFromMesh(GenMeshCube(0.5f, 2.0f, 1.0f));

#if EXAMPLE_ACTORS
	// Setup actors
	{
		actor_id_t actor = create_actor(vec3(0,3,0), 0, &app->actors);

		float shoulder_height = 3.5f;

		// Right arm
		limb_id_t right_arm = create_limb(vec3(0, shoulder_height, +1), quat_identity, &app->limbs);
		add_segment_to_limb(right_arm, vec3(0, shoulder_height, +3), &app->limbs);
		add_segment_to_limb(right_arm, vec3(0, shoulder_height, +4), &app->limbs);
		attach_limb_to_actor(right_arm, actor, &app->limbs, &app->actors, &app->limb_attachments);
		set_limb_end_effector(right_arm, vec3(2, shoulder_height, +1), &app->limbs);

		// Left arm
		limb_id_t left_arm = create_limb(vec3(0, shoulder_height, -1), quat_identity, &app->limbs);
		add_segment_to_limb(left_arm, vec3(0, shoulder_height, -3), &app->limbs);
		add_segment_to_limb(left_arm, vec3(0, shoulder_height, -4), &app->limbs);
		attach_limb_to_actor(left_arm, actor, &app->limbs, &app->actors, &app->limb_attachments);
		set_limb_end_effector(left_arm, vec3(2, shoulder_height, -1), &app->limbs);
	}

	create_actor(vec3(0, 1, -3), -0.5 * pi, &app->actors);
	create_actor(vec3(0, 1, +3), +0.5 * pi, &app->actors);
#endif

#if EXAMPLE_ARM
	// Large arm from origo
	{
		quat_t arm_ori = quat_from_axis_angle(vec3(0,0,1), pi/2);
		limb_id_t arm = create_limb(vec3_origo, arm_ori, &app->limbs);

		// Segment #1
		uint16_t s1 = add_segment_to_limb(arm, vec3(0,3,0), &app->limbs);
		apply_hinge_constraint(s1, 0, pi/2, &app->limbs);

		// Segment #2
		uint16_t s2 = add_segment_to_limb(arm, vec3(0,6,0), &app->limbs);
		apply_hinge_constraint(s2, 0, pi/2, &app->limbs);

		// Segment 3
		uint16_t s3 = add_segment_to_limb(arm, vec3(0,9,0), &app->limbs);
		apply_hinge_constraint(s3, 0, pi/2, &app->limbs);
	}
#endif // EXAMPLE_ARM
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

	// Set row datao
	location_t loc = {pos, rot};
	table->location[index] = loc;
	table->to_world[index] = to_world_from_location(loc);
	table->to_object[index] = to_object_from_location(loc);

	return actor_id;
}

actor_id_t get_actor_id(uint16_t index, const actor_table_t *table) {
	return T_ID(*table, index);
}

vec3_t get_actor_forward_dir(actor_id_t actor, const actor_table_t *table) {
	return mat4_mul_vec3(table->to_world[T_INDEX(*table, actor)], vec3(1,0,0), 0);
}

mat4_t get_actor_to_object_transform(actor_id_t actor, const actor_table_t *table) {
	return table->to_object[T_INDEX(*table, actor)];
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
	init_cl_pool(table->segment_nodes, max_limb_table_segnemts);
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
	table->root_segment[index] = 0;

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

vec3_t get_segment_joint_position(uint16_t segment_index, const limb_table_t *table) {
	assert(segment_index < max_limb_table_segnemts);
	return table->segments[segment_index].joint_pos;
}

/**
Collect limb segments into an array (with max size).
**/
size_t collect_limb_segments(limb_id_t limb, const limb_table_t *table, limb_segment_t out[], size_t max) {
	int root_seg = table->root_segment[T_INDEX(*table, limb)];
	if (!root_seg) { return 0; }

	for (int i = 0, seg = root_seg;  i < max ; i++) {
		out[i] = table->segments[seg];
		seg = table->segment_nodes[seg].next_index;
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
**/
uint16_t add_segment_to_limb(limb_id_t limb, vec3_t pos, limb_table_t *table) {
	limb_segment_t limb_segment_from_root_tip(vec3_t root, vec3_t tip);

	int limb_index = T_INDEX(*table, limb);
	int root_seg = table->root_segment[limb_index];
	if (root_seg == 0) {
		// Add first node
		int new_seg = take_free_cl_node(table->segment_nodes);
		table->root_segment[limb_index] = new_seg;

		// Set properties
		vec3_t limb_pos = table->position[limb_index];
		table->segments[new_seg] = limb_segment_from_root_tip(limb_pos, pos);
		return new_seg;
	} else {
		// Insert at end
		uint16_t last_seg = table->segment_nodes[root_seg].prev_index;
		int new_seg = append_cl_node_after(last_seg, table->segment_nodes);

		// Set properties
		vec3_t last_seg_pos = table->segments[last_seg].tip_pos;
		table->segments[new_seg] = limb_segment_from_root_tip(last_seg_pos, pos);
		return new_seg;
	}
}

void apply_pole_constraint(uint16_t segment_index, limb_table_t *table) {
	assert(segment_index < max_limb_table_segnemts);
	table->segments[segment_index].constraint.type = jc_pole;
}


void apply_hinge_constraint(uint16_t segment_index, float min_ang, float max_ang, limb_table_t *table) {
	assert(segment_index < max_limb_table_segnemts);
	table->segments[segment_index].constraint.type = jc_hinge;
	table->segments[segment_index].constraint.min_ang = min_ang;
	table->segments[segment_index].constraint.max_ang = max_ang;
}

/*
Create a limb segment that tstretches from one point to another.
*/
limb_segment_t limb_segment_from_root_tip(vec3_t joint_pos, vec3_t tip_pos) {
	quat_t orientation = quat_from_vec3_pair(vec3(1,0,0), vec3_between(joint_pos, tip_pos));
	limb_segment_t segment = {
		{jc_no_constraint, 0.f, 0.f},
		joint_pos, tip_pos,
		orientation, vec3_distance(joint_pos, tip_pos),
		};
	return segment;
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
		vec3_t p = attachments->relative_position[la];
		p = mat4_mul_vec4(actors->to_world[actor_index], vec4_from_vec3(p, 1)).vec3;
		limbs->position[limb_index] = p;
	}
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
