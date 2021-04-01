#ifndef OVERVIEW_H
#define OVERVIEW_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "linalg.h"

// Raylib without Raylib
#ifndef RAYLIB_H
struct Camera3D;
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


typedef struct movement_ {
	vec3_t velocity;
	float rotation_y;
} movement_t;

void move_locations(float dt, const movement_t [], size_t, location_t []);


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
	movement_t movement[max_actor_table_rows];
	mat4_t to_world[max_actor_table_rows];
	mat4_t to_object[max_actor_table_rows];
} actor_table_t;
extern const float actor_walking_speed;

// Actor CRUD
actor_id_t create_actor(vec3_t, float, actor_table_t *);
actor_id_t get_actor_id(uint16_t index, const actor_table_t *);
vec3_t get_actor_forward_dir(actor_id_t, const actor_table_t *);
mat4_t get_actor_to_object_transform(actor_id_t, const actor_table_t *);
mat4_t get_actor_to_world_transform(actor_id_t, const actor_table_t *);
void calculate_actor_transforms(actor_table_t *);

// Actor movement
void move_actors(float dt, actor_table_t *);

// Actor render
void render_actors(const struct Model *, const actor_table_t *);

//// Limbs ////
typedef struct limb_id_ { uint16_t id; } limb_id_t;
typedef enum bone_constraint_ {
	jc_no_constraint = 0, //(length only)
	jc_pole,
	jc_hinge,

	num_bone_constraints // Not a constraint :P
} bone_constraint_e;
typedef struct bone_ {
	struct {
		bone_constraint_e type;
		float min_ang, max_ang;
	} constraint;
	vec3_t joint_pos;
	quat_t orientation;
	float distance;
} bone_t;
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
	quat_t orientation[max_limb_table_rows];
	uint16_t root_bone[max_limb_table_rows];
	limb_id_t paired_with[max_limb_table_rows];

	// Segment pool
	cl_node_t bone_nodes[max_limb_table_segnemts];
	bone_t bones[max_limb_table_segnemts];
} limb_table_t;

// Limb CRUD
void init_limb_table(limb_table_t *);
limb_id_t create_limb(vec3_t pos, quat_t ori, limb_table_t *);
limb_id_t get_limb_id(uint16_t index, const limb_table_t *);
uint16_t get_limb_index(limb_id_t, const limb_table_t *);
vec3_t get_limb_position(limb_id_t, const limb_table_t *);
vec3_t get_bone_joint_position(uint16_t seg, const limb_table_t *);
vec3_t get_bone_tip_position(uint16_t seg, const limb_table_t *);
vec3_t get_limb_tip_position(limb_id_t, const limb_table_t *);
vec3_t get_limb_end_effector_position(limb_id_t, const limb_table_t *);
size_t collect_bones(limb_id_t, const limb_table_t *, bone_t out[], size_t max);
void set_limb_end_effector(limb_id_t, vec3_t, limb_table_t *);
uint16_t add_bone_to_limb(limb_id_t, vec3_t pos, limb_table_t *);
void pair_limbs(limb_id_t, limb_id_t, limb_table_t *);
void apply_pole_constraint(uint16_t seg, limb_table_t *);
void apply_hinge_constraint(uint16_t seg, float min_ang, float max_ang, limb_table_t *);

// Limb kinematics
void move_limbs_directly_to_end_effectors(limb_table_t *table);
void move_limb_directly_to(limb_id_t, vec3_t end, limb_table_t *);
void reposition_bones_with_fabrik(
	vec3_t root_pos, quat_t root_ori, vec3_t end,
	bone_t [], size_t num);
vec3_t get_bone_tip(bone_t);

#if defined(IN_CONTROLER) || defined(IN_TESTS)
void constrain_to_next_bone(const bone_t *next_bone, bone_t *this_bone);
void constrain_to_prev_bone(const bone_t *prev_bone, bone_t *this_bone);
bone_t bone_from_root_tip(vec3_t root, vec3_t tip);
#endif

// Render limbs
void render_limb_skeletons(const limb_table_t *);

//// Limb attachments
enum {max_limb_attachment_table_rows = max_limb_table_rows };
typedef struct limb_attachment_table_ {
	actor_id_t owner[max_limb_attachment_table_rows];
	limb_id_t limb[max_limb_attachment_table_rows];
	vec3_t relative_position[max_limb_attachment_table_rows];
	uint16_t num_rows;
} limb_attachment_table_t;

// Limb attachment CRUD
void attach_limb_to_actor(
	limb_id_t, actor_id_t, const limb_table_t*, const actor_table_t *,
	limb_attachment_table_t *);
void reposition_attached_limbs(const limb_attachment_table_t *, const actor_table_t *, limb_table_t *);


//// Limb path
enum {max_limb_goal_table_rows = max_limb_table_rows };
enum {max_limb_goal_curve_points = 4 };
typedef struct limb_goal_table_ {
	// Table meta
	uint16_t sparse_id[max_limb_goal_table_rows];
	limb_id_t dense_id[max_limb_table_rows];
	uint16_t num_rows;

	// Column data
	vec3_t curve_points[max_limb_goal_table_rows][max_limb_goal_curve_points];
	int8_t curve_index[max_limb_goal_table_rows];
	int8_t curve_length[max_limb_goal_table_rows];
	vec3_t velocity[max_limb_goal_table_rows];
	float max_speed[max_limb_goal_table_rows];
	float max_acceleration[max_limb_goal_table_rows];
	float threshold[max_limb_goal_table_rows];
} limb_goal_table_t;

// Limb goal CRUD
void put_limb_goal(limb_id_t, vec3_t, float speed, float acc, limb_goal_table_t *);
void push_limb_goal(limb_id_t, vec3_t, float speed, float acc, limb_goal_table_t *);
bool has_limb_goal(limb_id_t, const limb_goal_table_t *);
void move_limbs_toward_goals(float dt, limb_goal_table_t *, limb_table_t *);
void delete_accomplished_limb_goals(const limb_table_t *, limb_goal_table_t *);
void delete_limb_goal(limb_id_t, limb_goal_table_t *);
void delete_limb_goal_at_index(unsigned, limb_goal_table_t *);

// Limb goal rendering
void render_limb_goals(const limb_goal_table_t *, const limb_table_t *);

//// Limb swing
enum { max_limb_swing_table_rows = max_limb_table_rows };
typedef struct limb_swing_table_ {
	// Table meta
	uint16_t sparse_id[limb_table_id_range];
	limb_id_t dense_id[max_limb_swing_table_rows];
	uint16_t num_rows;

	// Column data
	vec3_t prev_position[max_limb_swing_table_rows];
} limb_swing_table_t;

// Limb swing CRUD
void create_limb_swing(limb_id_t, const limb_table_t *, limb_swing_table_t *);

// Limb swing kinematics
void perpetuate_limb_momentums(float dt, limb_swing_table_t *, limb_table_t *);
void apply_gravity_to_limbs(float dt, vec3_t gravity, limb_swing_table_t *, limb_table_t *);

//// Animate actors
void animate_walking_actor_legs(float dt,
	const actor_table_t *, const limb_attachment_table_t *, limb_goal_table_t *, limb_table_t *);

//// Population (everything that changes)
typedef struct population_ {
	actor_table_t actors;
	limb_table_t limbs;
	limb_attachment_table_t arms, legs;
	limb_goal_table_t limb_goals;
	limb_swing_table_t limb_swings;
} population_t;

void update_population(float dt, population_t *pop);

// App
typedef enum app_mode_ {
	am_limb_forest,
	am_single_actor,
	am_robot_arm,

	num_app_modes // Not a mode :P
} app_mode_e;
enum {
	max_pop_history_frames = 1024,
};
typedef struct app_ {
	app_mode_e mode;
	bool paused;
	bool step_once;
	float buffered_time;
	struct Model *actor_model;
	population_t population_history[max_pop_history_frames];
	vec3_t world_cursor;
	unsigned frame_count;
} app_t;


void init_app(app_mode_e, app_t *);
void term_app(app_t *);
void process_input(float dt, app_t*);
void update_app(float dt, app_t *);
void render_app(const struct Camera3D *, const app_t *);

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
