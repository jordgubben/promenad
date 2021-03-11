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
Create a vector from 3 scalars.
**/
static inline vec3_t vec3(float x, float y, float z) {
	vec3_t r = {x, y, z, NAN};
	return r;
}

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
The *squared* lenght of the given vector.
**/
static inline float vec3_squared_length(vec3_t v) {
	return (v.x*v.x + v.y*v.y + v.z*v.z);
}


/**
The lenght (or 'norm') of the given vector.
**/
static inline float vec3_length(vec3_t v) {
	return sqrtf(vec3_squared_length(v));
}


/**
The normal (unit length vector) of the given vector.
**/
static inline vec3_t vec3_normal(vec3_t v) {
	float l = vec3_length(v);
	if (l == 0) { return vec3(0,0,0); }
	vec3_t n = {v.x/l, v.y/l, v.z/l};
	return n;
}
/**
Distance between two points.
**/
static inline float vec3_distance(vec3_t p1, vec3_t p2) {
	return vec3_length(vec3_between(p1, p2));
}


/**
Add two vectors.
**/
static inline vec3_t vec3_add(vec3_t v1, vec3_t v2) {
	vec3_t r = {
		v1.x + v2.x,
		v1.y + v2.y,
		v1.z + v2.z,
	};
	return r;
}


/**
Subrtact the second vector from the first.
**/
static inline vec3_t vec3_sub(vec3_t v1, vec3_t v2) {
	vec3_t r = {
		v1.x - v2.x,
		v1.y - v2.y,
		v1.z - v2.z,
	};
	return r;
}


/**
Multiply vector by scalar.
**/
static inline vec3_t vec3_mul(vec3_t v, float s) {
	vec3_t r = {
		v.x * s,
		v.y * s,
		v.z * s,
	};
	return r;
}

/**
Divide vector by scalar.
**/
static inline vec3_t vec3_div(vec3_t v, float s) {
	vec3_t r = {
		v.x / s,
		v.y / s,
		v.z / s,
	};
	return r;
}


/**
The dot product of the two given vectors.
**/
static inline float vec3_dot(vec3_t v1, vec3_t v2) {
	return  (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
}


/**
The cross product of the two given vectors.
**/
static inline vec3_t vec3_cross(vec3_t v1, vec3_t v2) {
	vec3_t r = {
		v1.y * v2.z - v1.z * v2.y,
		v1.z * v2.x - v1.x * v2.z,
		v1.x * v2.y - v1.y * v2.x
	};
	return r;
}

#ifdef IN_TESTS

bool operator== (const vec3_t& v1, const vec3_t& v2) {
	return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
}

TEST_CASE("Operations on 3 element vectors") {
	SECTION("A  vector with only zeros has the length 0") {
		REQUIRE(vec3_length(vec3(0,0,0)) == 0);
	}

	SECTION("A  a length > 0 has aproximately the right length") {
		using namespace Catch::literals;
		vec3_t v = vec3(1,2,3);
		REQUIRE(vec3_length(v) == (3.742_a).margin(0.01));
	}

	SECTION("A  vector with only zeros the right squared length") {
		vec3_t v = vec3(1, 2, 3);
		REQUIRE(vec3_squared_length(v) == (1*1 + 2*2 + 3*3));
	}

	SECTION("Normal of a zero-length vector") {
		vec3_t n = vec3_normal(vec3(0,0,0));
		CHECK(n.x == 0);
		CHECK(n.y == 0);
		CHECK(n.z == 0);
	}

	SECTION("Normals of vectors along all axes") {
		CHECK((vec3_normal(vec3(10, 0, 0)) == vec3(1, 0, 0)));
		CHECK((vec3_normal(vec3(0, 10, 0)) == vec3(0, 1, 0)));
		CHECK((vec3_normal(vec3(0, 0, 10)) == vec3(0, 0, 1)));
		CHECK((vec3_normal(vec3(10, 0, 0)) == vec3(1, 0, 0)));
		CHECK((vec3_normal(vec3(0, -10, 0)) == vec3(0, -1, 0)));
		CHECK((vec3_normal(vec3(0, 0, -10)) == vec3(0, 0, -1)));
	}

	SECTION("The dot product of stricly orthagonal vectors is always zero") {
		CHECK(vec3_dot(vec3(1, 0, 0), vec3(0, 1, 0)) == 0);
		CHECK(vec3_dot(vec3(1, 0, 0), vec3(0, 0, 1)) == 0);
		CHECK(vec3_dot(vec3(1, 0, 0), vec3(0,-1, 0)) == 0);
		CHECK(vec3_dot(vec3(1, 0, 0), vec3(0, 0,-1)) == 0);

		CHECK(vec3_dot(vec3(0, 1, 0), vec3(1, 0, 0)) == 0);
		CHECK(vec3_dot(vec3(0, 1, 0), vec3(0, 0, 1)) == 0);
		CHECK(vec3_dot(vec3(0, 1, 0), vec3(-1, 0, 0)) == 0);
		CHECK(vec3_dot(vec3(0, 1, 0), vec3(0, 0, -1)) == 0);

		CHECK(vec3_dot(vec3(0, 0, 1), vec3(1, 0, 0)) == 0);
		CHECK(vec3_dot(vec3(0, 0, 1), vec3(0, 1, 0)) == 0);
		CHECK(vec3_dot(vec3(0, 0, 1), vec3(-1, 0, 0)) == 0);
		CHECK(vec3_dot(vec3(0, 0, 1), vec3(0, -1, 0)) == 0);
	}

	SECTION("The dot product of two axises vectors is always parallell to the third axis") {
		CHECK(vec3_cross(vec3(1, 0, 0), vec3( 0, 1, 0)) == vec3( 0, 0, 1));
		CHECK(vec3_cross(vec3(1, 0, 0), vec3( 0, 0, 1)) == vec3( 0,-1, 0));
		CHECK(vec3_cross(vec3(1, 0, 0), vec3( 0,-1, 0)) == vec3( 0, 0,-1));
		CHECK(vec3_cross(vec3(1, 0, 0), vec3( 0, 0,-1)) == vec3( 0, 1, 0));

		CHECK(vec3_cross(vec3(0, 1, 0), vec3( 1, 0, 0)) == vec3( 0, 0,-1));
		CHECK(vec3_cross(vec3(0, 1, 0), vec3( 0, 0, 1)) == vec3( 1, 0, 0));
		CHECK(vec3_cross(vec3(0, 1, 0), vec3(-1, 0, 0)) == vec3( 0, 0, 1));
		CHECK(vec3_cross(vec3(0, 1, 0), vec3( 0, 0,-1)) == vec3(-1, 0, 0));

		CHECK(vec3_cross(vec3(0, 0, 1), vec3( 1, 0, 0)) == vec3( 0, 1, 0));
		CHECK(vec3_cross(vec3(0, 0, 1), vec3( 0, 1, 0)) == vec3(-1, 0, 0));
		CHECK(vec3_cross(vec3(0, 0, 1), vec3(-1, 0, 0)) == vec3( 0,-1, 0));
		CHECK(vec3_cross(vec3(0, 0, 1), vec3( 0,-1, 0)) == vec3( 1, 0, 0));
	}

	SECTION("Support + operator") {
		CHECK(vec3_add(vec3(100, 200, 300), vec3( 1, 2, 3)) == vec3(101, 202, 303));
		CHECK(vec3_add(vec3(100, 200, 300), vec3(-1,-2,-3)) == vec3(99, 198, 297));
	}

	SECTION("Support - operator") {
		CHECK(vec3_sub(vec3(100, 200, 300), vec3( 1, 2, 3)) == vec3(99, 198, 297));
		CHECK(vec3_sub(vec3(100, 200, 300), vec3(-1,-2,-3)) == vec3(101, 202, 303));
	}

	SECTION("Support * operator with a vector and a scalar") {
		CHECK(vec3_mul(vec3(100, 200, 300), 10.0) == vec3(1000, 2000, 3000));
	}

	SECTION("Support / operator with a vector and a scalar") {
		CHECK(vec3_div(vec3(100, 200, 300), 10.0) == vec3(10, 20, 30));
	}
}
#endif // IN_TESTS

#else
#warning "Header linalg.h included more than once"
#endif

