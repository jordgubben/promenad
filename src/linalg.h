#ifndef LINALG_H
#define LINALG_H

#include <math.h>

/***
Vector with 3 elements (+ paddding)
***/
typedef struct vec3_ {
	union {
		struct { float x, y, z, w_; };
#ifdef RAYLIB_H
		Vector3 rl;
#endif
	};
} vec3_t;

/**
A vector from one point to another.
**/
static inline vec3_t vec3_between(vec3_t p1, vec3_t p2) {
	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;
	float dz = p2.z - p1.z;

	vec3_t d = {dx, dy, dz};
	return d;
}

/**
The lenght (or 'norm') of the given vector.
**/
static inline float vec3_length(vec3_t v) {
	return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}


/**
The normal (unit length vector) of the given vector.
**/
static inline vec3_t vec3_normal(vec3_t v) {
	float l = vec3_length(v);
	vec3_t n = {v.x/l, v.y/l, v.z/l};
	return n;
}
/**
Distance between two points.
**/
static inline float vec3_distance(vec3_t p1, vec3_t p2) {
	return vec3_length(vec3_between(p1, p2));
}

#else
#warning "Header linalg.h included more than once"
#endif

