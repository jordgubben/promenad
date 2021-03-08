#ifndef LINALG_H
#define LINALG_H

#include <math.h>

static inline float vec3_distance(vec3_t p1, vec3_t p2) {
	float dx = p1.x - p2.x;
	float dy = p1.y - p2.y;
	float dz = p1.z - p2.z;
	return sqrtf(dx*dx + dy*dy + dz*dz);
}

#else
#warning "Header linalg.h included more than once"
#endif

