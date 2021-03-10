#ifndef OVERVIEW_H
#define OVERVIEW_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// Basic types
typedef struct row_id_ { uint16_t id; } row_id_t;

typedef struct vec3_ {
	union {
		struct { float x, y, z, w_; };
#ifdef RAYLIB_H
		Vector3 rl;
#endif
	};
} vec3_t;

/** A single node in a Cyclic List **/
typedef struct cl_node {
	unsigned short next_index, prev_index;
} cl_node_t;

// Limbs
typedef struct limb_segment_ {
	vec3_t position;
	float distance;
} limb_segment_t;
enum {
	max_limb_table_rows = 128,
	limb_table_id_range = 1024,
	max_limb_table_segnemts = max_limb_table_rows * 8,
};
typedef struct limb_table_ {
	// Meta
	uint16_t sparse_id[limb_table_id_range];
	row_id_t dense_id[max_limb_table_rows];
	uint16_t num_rows, next_id;

	// Columns
	vec3_t end_effector[max_limb_table_rows];
	vec3_t position[max_limb_table_rows];
	uint16_t root_segment[max_limb_table_rows];

	// Segment pool
	cl_node_t segment_nodes[max_limb_table_segnemts];
	limb_segment_t segments[max_limb_table_segnemts];
} limb_table_t;

// Limb CRUD
void init_limb_table(limb_table_t *);
row_id_t create_limb(vec3_t pos, limb_table_t *);
row_id_t get_limb_id(uint16_t index, const limb_table_t *);
size_t collect_limb_segments(row_id_t, const limb_table_t *, limb_segment_t out[], size_t max);
void add_segment_to_limb(row_id_t, vec3_t pos, limb_table_t *);

// Limb kinematics
void move_limbs_towards_end_effectors(float dt, limb_table_t *);
void reposition_limb_segments_with_fabrik(vec3_t origin, vec3_t end, limb_segment_t [], size_t num);

// Render limbs
void render_limb_skeletons(vec3_t end_effector, const limb_table_t *);

// App
typedef struct app_ {
	bool paused;
	limb_table_t limbs;
	vec3_t common_end_effector;
} app_t;


void init_app(app_t *);
void process_input(float dt, app_t*);
void update_app(float dt, app_t *);
void render_app(const app_t *);

//// Utils
// Loops
#define FOR_IN(i,n) for (int i = 0; i < (n); i++)
#define FOR_RANGE(i, s, n) for (int i = (s); i < (n); i++)
#define FOR_ITR(type, itr, arr, num) for (type *itr = arr; itr != arr + (num); itr++)
#define FOR_ROWS(r,t) for (size_t r = 0; r < (t).num_rows; r++)

#else
#warning "Included overvieew.h twice!"
#endif
