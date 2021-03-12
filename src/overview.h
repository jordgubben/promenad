#ifndef OVERVIEW_H
#define OVERVIEW_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "linalg.h"

// Raylib without Raylib
#ifndef RAYLIB_H
struct Model;
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Basic types

/** A single node in a Cyclic List **/
typedef struct cl_node {
	unsigned short next_index, prev_index;
} cl_node_t;

typedef struct location_ {
	vec3_t position;
	float orientation_y;
} location_t;
mat4_t to_world_from_location(location_t l);
mat4_t to_object_from_location(location_t l);

//// Actor ////
typedef struct actor_id_ { uint16_t id; } actor_id_t;
enum {
	max_actor_table_rows = 128,
	actor_table_id_range = 1024,
};
typedef struct actor_table_ {
	// Meta
	uint16_t sparse_id[actor_table_id_range];
	actor_id_t dense_id[max_actor_table_rows];
	uint16_t num_rows, next_id;

	// Column(s)
	location_t location[max_actor_table_rows];
	mat4_t to_world[max_actor_table_rows];
	mat4_t to_object[max_actor_table_rows];
} actor_table_t;

// Actor CRUD
actor_id_t create_actor(vec3_t, float, actor_table_t *);
void calculate_actor_transforms(actor_table_t *);

// Actor render
void render_actors(const struct Model *, const actor_table_t *);

//// Limbs ////
typedef struct limb_id_ { uint16_t id; } limb_id_t;
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
	limb_id_t dense_id[max_limb_table_rows];
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
limb_id_t create_limb(vec3_t pos, limb_table_t *);
limb_id_t get_limb_id(uint16_t index, const limb_table_t *);
vec3_t get_limb_position(limb_id_t, const limb_table_t *);
size_t collect_limb_segments(limb_id_t, const limb_table_t *, limb_segment_t out[], size_t max);
void add_segment_to_limb(limb_id_t, vec3_t pos, limb_table_t *);

// Limb kinematics
void move_limbs_towards_end_effectors(float dt, limb_table_t *);
void reposition_limb_segments_with_fabrik(vec3_t origin, vec3_t end, limb_segment_t [], size_t num);

// Render limbs
void render_limb_skeletons(vec3_t end_effector, const limb_table_t *);

//// Limb attachments
enum {max_limb_attachment_table_rows = max_limb_table_rows };
typedef struct limb_attachment_table_ {
	actor_id_t owner[max_limb_attachment_table_rows];
	limb_id_t limb[max_limb_attachment_table_rows];
	vec3_t relative_position[max_limb_attachment_table_rows];
	uint16_t num_rows;
} limb_attachment_table_t;

// Limb attachment CRUD
void attach_limb_to_actor(limb_id_t, actor_id_t, const limb_table_t*, limb_attachment_table_t *);
void reposition_attached_limbs(const limb_attachment_table_t *, const actor_table_t *, limb_table_t *);


// App
typedef struct app_ {
	bool paused;
	struct Model *actor_model;
	actor_table_t actors;
	limb_table_t limbs;
	limb_attachment_table_t limb_attachments;
	vec3_t common_end_effector;
} app_t;


void init_app(app_t *);
void term_app(app_t *);
void process_input(float dt, app_t*);
void update_app(float dt, app_t *);
void render_app(const app_t *);

//// Utils
// Loops
#define FOR_IN(i,n) for (int i = 0; i < (n); i++)
#define FOR_RANGE(i, s, n) for (int i = (s); i < (n); i++)
#define FOR_ITR(type, itr, arr, num) for (type *itr = arr; itr != arr + (num); itr++)
#define FOR_ROWS(r,t) for (size_t r = 0; r < (t).num_rows; r++)

#ifdef __cplusplus
} // extern "C"
#endif

#else
#warning "Included overvieew.h twice!"
#endif
