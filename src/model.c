#include <assert.h>

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


//// App

/**
Init all the things.
**/
void init_app(app_t * app) {

	// Create a bunch of limbs with their roots in a grid
	FOR_RANGE(x, -5,5) {
		FOR_RANGE(z, -5, 5) {
			vec3_t pos = {x, 0, z};
			create_limb(pos, &app->limbs);
		}
	}
}



//// Limb CRUD

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

	return limb_id;
}
