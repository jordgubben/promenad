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


// Limbs
enum { max_limb_table_rows = 128, limb_table_id_range = 1024 };
typedef struct limb_table_ {
	// Meta
	uint16_t sparse_id[limb_table_id_range];
	row_id_t dense_id[max_limb_table_rows];
	uint16_t num_rows, next_id;

	// Data
	vec3_t position[max_limb_table_rows];
} limb_table_t;

// Limb CRUD
row_id_t create_limb(vec3_t pos, limb_table_t *);

// App
typedef struct app_ {
	limb_table_t limbs;
} app_t;


void init_app(app_t *);
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
