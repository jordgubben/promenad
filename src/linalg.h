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

static const vec3_t vec3_origo = {0,0,0};

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

/**
Round every scalar in the vector.
**/
static inline vec3_t vec3_round(vec3_t v){
	vec3_t r = { roundf(v.x), roundf(v.y), roundf(v.z), NAN };
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
Multiply two 4x4 matrices with each other.
**/
static inline mat4_t mat4_mul(mat4_t m1, mat4_t m2) {
	// Rows from the first matrix
	vec4_t r1 = { m1.m11, m1.m12, m1.m13, m1.m14};
	vec4_t r2 = { m1.m21, m1.m22, m1.m23, m1.m24};
	vec4_t r3 = { m1.m31, m1.m32, m1.m33, m1.m34};
	vec4_t r4 = { m1.m41, m1.m42, m1.m43, m1.m44};

	// Columns from the second matrix
	vec4_t c1 = m2.c1, c2 = m2.c2, c3 = m2.c3, c4 = m2.c4;

	// Combine them
	mat4_t r;
	r.m11 = vec4_dot(r1, c1); r.m12 = vec4_dot(r1, c2); r.m13 = vec4_dot(r1, c3); r.m14 = vec4_dot(r1, c4);
	r.m21 = vec4_dot(r2, c1); r.m22 = vec4_dot(r2, c2); r.m23 = vec4_dot(r2, c3); r.m24 = vec4_dot(r2, c4);
	r.m31 = vec4_dot(r3, c1); r.m32 = vec4_dot(r3, c2); r.m33 = vec4_dot(r3, c3); r.m34 = vec4_dot(r3, c4);
	r.m41 = vec4_dot(r4, c1); r.m42 = vec4_dot(r4, c2); r.m43 = vec4_dot(r4, c3); r.m44 = vec4_dot(r4, c4);
	return r;
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

/**
Multiply a 3-element *column* vector (assisted by a fourth element) with a matrix.
**/
static inline vec3_t mat4_mul_vec3(mat4_t m, vec3_t v, float s) {
	return mat4_mul_vec4(m, vec4_from_vec3(v, s)).vec3;
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

	SECTION("Multiplying a y-axis 90 deg rotation with it self multiple times creates the desired result") {
		SECTION("using o * r") {
			const mat4_t ry = mat4_rotation_y(pi / 2);

			// Two times
			const mat4_t r2 = mat4_mul(ry, ry);
			CHECK(vec4_round(mat4_mul_vec4(r2, vec4_positive_x)) == vec4_negative_x);
			CHECK(vec4_round(mat4_mul_vec4(r2, vec4_positive_z)) == vec4_negative_z);

			// Three times
			const mat4_t r3 = mat4_mul(r2, ry);
			CHECK(vec4_round(mat4_mul_vec4(r3, vec4_positive_x)) == vec4_positive_z);
			CHECK(vec4_round(mat4_mul_vec4(r3, vec4_positive_z)) == vec4_negative_x);

			// Four times (all the way round)
			const mat4_t r4 = mat4_mul(r3, ry);
			CHECK(vec4_round(mat4_mul_vec4(r4, vec4_positive_x)) == vec4_positive_x);
			CHECK(vec4_round(mat4_mul_vec4(r4, vec4_positive_z)) == vec4_positive_z);
		}
	}
}
#endif // IN_TESTS


/***
Unit quaternion.
***/
typedef union quat_ {
	struct {float x,y, z, w;};
	vec4_t vec4;
	vec3_t vec3;
#ifdef RAYLIB_H
	Quaternion rl;
#endif
} quat_t;

static const quat_t quat_identity = {0,0,0,1};

/**
The conjugate of the given quaternion.
**/
static inline quat_t quat_conjugate(quat_t q) {
	quat_t qi = {-q.x, -q.y, -q.z, q.w};
	return qi;
}

/**
The inverse of the given (unit) quaternion.
**/
static inline quat_t quat_inverse(quat_t q) {
	return quat_conjugate(q);
}

static inline quat_t quat_from_axis_angle(vec3_t axis, float angle) {
	vec3_t n = vec3_normal(axis);
	quat_t q = {
		axis.x * sinf(angle/2),
		axis.y * sinf(angle/2),
		axis.z * sinf(angle/2),
		cosf(angle/2)
	};
	return q;
}

static inline quat_t quat_mul(quat_t q1, quat_t q2) {
	quat_t r;
	r.vec3 = vec3_cross(q1.vec3, q2.vec3);
	r.vec3 = vec3_add(r.vec3, vec3_mul(q1.vec3, q2.w));
	r.vec3 = vec3_add(r.vec3, vec3_mul(q2.vec3, q1.w));
	r.w = q1.w * q2.w - vec3_dot(q1.vec3, q2.vec3);
	return r;
}


/**
Apply the rotation of a quaternion to a vector.
**/
static vec3_t quat_rotate_vec3(quat_t q, vec3_t v) {
	quat_t p = {v.x, v.y,  v.z, 0 };
	quat_t qi = quat_inverse(q);
	return quat_mul(q, quat_mul(p, qi)).vec3;
}

static inline bool quat_eq(quat_t q1, quat_t q2) {
	return q1.x == q2.x && q1.y == q2.y && q2.z == q2.z && q1.w == q2.w;
}

#ifdef IN_TESTS

bool operator==(const quat_t& q1, const quat_t& q2) {
	return quat_eq(q1, q2);
}

std::ostream& operator<<(std::ostream& out, const quat_t& q) {
	out << "(" << q.vec3 << "| " << q.w << ")";
	return out;
}

TEST_CASE("Quaternion operations") {
	SECTION("Multiplying a quatenion with it's opposite yields an identity quaternion") {
		quat_t q = quat_from_axis_angle(vec3(1,0,0), pi);
		quat_t qo = quat_from_axis_angle(vec3(1,0,0), -pi);

		// True both one way..
		quat_t r1 = quat_mul(q, qo);
		CHECK(r1 == quat_identity);

		// ..and the other
		quat_t r2 = quat_mul(qo, q);
		CHECK(r2 == quat_identity);
	}

	SECTION("Multiplying a quatenion with it's inverse yields an identity quaternion") {
		quat_t q = quat_from_axis_angle(vec3(1,0,0), pi);
		quat_t qi = quat_inverse(q);

		// True both one way..
		quat_t r1 = quat_mul(q, qi);
		CHECK(r1 == quat_identity);

		// ..and the other
		quat_t r2 = quat_mul(qi, q);
		CHECK(r2 == quat_identity);
	}

	SECTION("Rotating a point 90 deg with a quaternion yyields the expected result") {
		SECTION("x-axis") {
			quat_t q = quat_from_axis_angle(vec3(+1,0,0), pi/2);
			CHECK(vec3_round(quat_rotate_vec3(q, vec3(+1,0,0))) == vec3(+1,0,0));
			CHECK(vec3_round(quat_rotate_vec3(q, vec3(0,+1,0))) == vec3(0,0,+1));
			CHECK(vec3_round(quat_rotate_vec3(q, vec3(0,0,+1))) == vec3(0,-1,0));
		}

		SECTION("y-axis") {
			quat_t q = quat_from_axis_angle(vec3(0,+1,0), pi/2);
			CHECK(vec3_round(quat_rotate_vec3(q, vec3(+1,0,0))) == vec3(0,0,-1));
			CHECK(vec3_round(quat_rotate_vec3(q, vec3(0,+1,0))) == vec3(0,+1,0));
			CHECK(vec3_round(quat_rotate_vec3(q, vec3(0,0,+1))) == vec3(+1,0,0));
		}

		SECTION("z-axis") {
			quat_t q = quat_from_axis_angle(vec3(0,0,+1), pi/2);
			CHECK(vec3_round(quat_rotate_vec3(q, vec3(+1,0,0))) == vec3(0,+1,0));
			CHECK(vec3_round(quat_rotate_vec3(q, vec3(0,+1,0))) == vec3(-1,0,0));
			CHECK(vec3_round(quat_rotate_vec3(q, vec3(0,0,+1))) == vec3(0,0,+1));
		}
	}
}

#endif // IN_TESTS
#else
#warning "Header linalg.h included more than once"
#endif

