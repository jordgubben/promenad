#include <assert.h>

#define IN_MODEL
#include "overview.h"
#include "linalg.h"

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


void init_cl_pool(cl_node_t nodes[], size_t num_nodes);
uint16_t take_free_cl_node(cl_node_t []);

//// App

/**
Init all the things.
**/
void init_app(app_t * app) {
	init_limb_table(&app->limbs);

	// Create a bunch of limbs with their roots in a grid
	FOR_RANGE(x, -5,5) {
		FOR_RANGE(z, -5, 5) {
			vec3_t pos = {x, 0, z};
			row_id_t id = create_limb(pos, &app->limbs);

			vec3_t segment_pos;
			segment_pos = pos;
			segment_pos.y += 1;
			add_segment_to_limb(id, segment_pos, &app->limbs);
		}
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
row_id_t create_limb(vec3_t pos, limb_table_t *table) {
	assert(table->num_rows < max_limb_table_rows);

	// Add row to sparse set
	row_id_t limb_id = { table->next_id++};
	int index = table->num_rows++;
	table->sparse_id[limb_id.id] = index;
	table->dense_id[index] = limb_id;

	// Set row data
	table->position[index] = pos;
	table->root_segment[index] = 0;

	return limb_id;
}


/**
Add a aegment at the end of the given limb.
**/
void add_segment_to_limb(row_id_t limb, vec3_t pos, limb_table_t *table) {
	int new_seg = take_free_cl_node(table->segment_nodes);

	int limb_index = T_INDEX(*table, limb);
	if (table->root_segment[limb_index] == 0) {
		table->segment_positions[new_seg] = pos;
		table->segment_distances[new_seg] =
			vec3_distance(table->position[limb_index], pos);
		table->root_segment[limb_index] = new_seg;
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
uint16_t take_free_cl_node(cl_node_t pool[]) {
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
