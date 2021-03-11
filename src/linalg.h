#ifndef LINALG_H
#define LINALG_H

#include <math.h>

// Important constants
static const float tau = 6.28318530717958647692f;
static const float pi = 3.14159265358979323846f;


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

std::ostream& operator<<(std::ostream& out, const vec3_t& v) {
	out << "( " << v.x << ", " << v.y << ", " << v.z << ")";
	return out;
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


/***
A vector wit four elements (no padding).
***/
typedef union vec4_ {
	struct { float x,y,z,w; };
	vec3_t vec3;
#ifdef RAYLIB_H
	Vector4 rl;
#endif
} vec4_t;

// All the axis directions
static const vec4_t vec4_positive_x = {+1, 0, 0, 0};
static const vec4_t vec4_positive_y = { 0,+1, 0, 0};
static const vec4_t vec4_positive_z = { 0, 0,+1, 0};
static const vec4_t vec4_negative_x = {-1, 0, 0, 0};
static const vec4_t vec4_negative_y = { 0,-1, 0, 0};
static const vec4_t vec4_negative_z = { 0, 0,-1, 0};

static inline vec4_t vec4(float x, float y, float z, float w) {
	vec4_t v = { x, y, z, w };
	return v;
}

static inline vec4_t vec4_from_vec3(vec3_t v, float w) {
	vec4_t r = { v.x, v.y, v.z, w };
	return r;
}


/**
Dot product of two vectors.
**/
static inline float vec4_dot(vec4_t v1, vec4_t v2) {
	return  (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z) + (v1.w * v2.w);
}



/**
Round every scalar in the vector.
**/
static inline vec4_t vec4_round(vec4_t v){
	vec4_t r = { roundf(v.x), roundf(v.y), roundf(v.z), roundf(v.w) };
	return r;

}

/***
Matrix 4x4 (column major)
***/
typedef union mat4_ {
	// Matrix elements m(Row, Column)
	struct {
		float m11, m21, m31, m41;
		float m12, m22, m32, m42;
		float m13, m23, m33, m43;
		float m14, m24, m34, m44;
	};
	struct { vec4_t c1, c2, c3, c4; };
	struct { float e[16]; };
#ifdef RAYLIB_H
	Matrix rl;
#endif
} mat4_t;

static const mat4_t mat4_identity = {
	1,0,0,0,
	0,1,0,0,
	0,0,1,0,
	0,0,0,1,
};


/**
Matrix that translate a vector by the given amount.
**/
static inline mat4_t mat4_translate(vec3_t t) {
	mat4_t m = mat4_identity;
	m.c4 = vec4_from_vec3(t, 1);
	return m;
}

/**
Transpose of the given matrix.
**/
static inline mat4_t mat4_transpose(mat4_t m) {
	mat4_t r;
	r.m11 = m.m11; r.m21 = m.m12; r.m31 = m.m13; r.m41 = m.m14;
	r.m12 = m.m21; r.m22 = m.m22; r.m32 = m.m23; r.m42 = m.m24;
	r.m13 = m.m31; r.m23 = m.m32; r.m33 = m.m33; r.m43 = m.m34;
	r.m14 = m.m41; r.m24 = m.m42; r.m34 = m.m43; r.m44 = m.m44;
	return r;
}


/**
Rotate clockwise (in radians) around the y-axis.
**/
static inline mat4_t mat4_rotation_y(float r) {
	mat4_t m = mat4_identity;
	m.m11 = m.m33 = cosf(r);
	m.m13 = sinf(r);
	m.m31 = -m.m13;
	return m;
}

/**
Multiply a *column* vector with a matrix.
**/
static inline vec4_t mat4_mul_vec4(mat4_t m, vec4_t v) {
	vec4_t r1 = { m.m11, m.m12, m.m13, m.m14};
	vec4_t r2 = { m.m21, m.m22, m.m23, m.m24};
	vec4_t r3 = { m.m31, m.m32, m.m33, m.m34};
	vec4_t r4 = { m.m41, m.m42, m.m43, m.m44};

	vec4_t r = {
		vec4_dot(r1, v),
		vec4_dot(r2, v),
		vec4_dot(r3, v),
		vec4_dot(r4, v)
	};
	return r;
}

#ifdef IN_TESTS
bool operator== (const vec4_t& v1, const vec4_t& v2) {
	return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z && v1.w == v2.w;
}

std::ostream& operator<<(std::ostream& out, const vec4_t& v) {
	out << "( " << v.x << ", " << v.y << ", " << v.z << ", " <<  v.w << ")";
	return out;
}

TEST_CASE("Operations using  4x4 matricies") {
	SECTION("Matrix translation") {
		mat4_t t = mat4_translate(vec3(1,2,3));
		CHECK(t.e[12] == 1);
		CHECK(t.e[13] == 2);
		CHECK(t.e[14] == 3);
		vec4_t p = mat4_mul_vec4(t, vec4(10,20,30,1));
		CHECK(p.vec3 == vec3(11,22,33));
	}

	SECTION("Rotate clockwise 90 degrees around the y-axis") {
		mat4_t r = mat4_rotation_y(tau / 4);

		SECTION("Transfrom +x -> -z") {
			CHECK(vec4_round(mat4_mul_vec4(r, vec4_positive_x)) == vec4_negative_z);
			CHECK(vec4_round(mat4_mul_vec4(r, vec4_negative_x)) == vec4_positive_z);
		}

		SECTION("Has no effect on an vector on the y axis") {
			CHECK(vec4_round(mat4_mul_vec4(r, vec4_positive_y)) == vec4_positive_y);
			CHECK(vec4_round(mat4_mul_vec4(r, vec4_negative_y)) == vec4_negative_y);
		}

		SECTION("Transfrom +z -> +x") {
			CHECK(vec4_round(mat4_mul_vec4(r, vec4_positive_z)) == vec4_positive_x);
			CHECK(vec4_round(mat4_mul_vec4(r, vec4_negative_z)) == vec4_negative_x);
		}
	}
}
#endif // IN_TESTS

#else
#warning "Header linalg.h included more than once"
#endif

