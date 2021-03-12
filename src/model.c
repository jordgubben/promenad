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


void init_cl_pool(cl_node_t [], size_t num_nodes);
unsigned short take_free_cl_node(cl_node_t []);
unsigned short append_cl_node_after(unsigned short anchor, cl_node_t []);

//// App

/**
Init all the things.
**/
void init_app(app_t * app) {
	app->paused = false;

	init_limb_table(&app->limbs);

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

	// Create actor model
	app->actor_model = malloc(sizeof(Model));
	*app->actor_model = LoadModelFromMesh(GenMeshCube(0.5f, 2.0f, 1.0f));

	// Setup actors
	create_actor(vec3(0,1,0), 0, &app->actors);
	create_actor(vec3(0, 1, -3), -0.5 * pi, &app->actors);
	create_actor(vec3(0, 1, +3), +0.5 * pi, &app->actors);
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
	table->transform[index] = mat4_from_location(loc);

	return actor_id;
}


/**
Calculate the transform for every actors location.
**/
void calculate_actor_transforms(actor_table_t *table) {
	FOR_ROWS(a, *table) {
		table->transform[a] = mat4_from_location(table->location[a]);
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
limb_id_t create_limb(vec3_t pos, limb_table_t *table) {
	assert(table->num_rows < max_limb_table_rows);

	// Add row to sparse set
	limb_id_t limb_id = { table->next_id++};
	int index = table->num_rows++;
	table->sparse_id[limb_id.id] = index;
	table->dense_id[index] = limb_id;

	// Set row data
	table->position[index] = pos;
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
Add a segment at the end of the given limb.
**/
void add_segment_to_limb(limb_id_t limb, vec3_t pos, limb_table_t *table) {

	int limb_index = T_INDEX(*table, limb);
	int root_seg = table->root_segment[limb_index];
	if (root_seg == 0) {
		// Add first node
		int new_seg = take_free_cl_node(table->segment_nodes);
		table->root_segment[limb_index] = new_seg;

		// Set properties
		vec3_t limb_pos = table->position[limb_index];
		limb_segment_t segment = { pos, vec3_distance(pos, limb_pos) };
		table->segments[new_seg] = segment;
	} else {
		// Insert at end
		uint16_t last_seg = table->segment_nodes[root_seg].prev_index;
		int new_seg = append_cl_node_after(last_seg, table->segment_nodes);

		// Set properties
		vec3_t last_seg_pos = table->segments[last_seg].position;
		limb_segment_t segment = { pos, vec3_distance(last_seg_pos, pos)};
		table->segments[new_seg] = segment;
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
