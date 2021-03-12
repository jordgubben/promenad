#include <catch2/catch.hpp>
#include <raylib.h>

#define IN_TESTS
#include "overview.h"

SCENARIO("Example windowd test", "[.][windowed]"){
	InitWindow(200, 100, "Play subsystem windowed test");

	// TODO: Preform windowed tests

	CloseWindow();
}

SCENARIO("Location transforms") {

	GIVEN("Two transforms from some location") {
		location_t l = { {1,2,3}, pi/4};
		mat4_t w = to_world_from_location(l);
		mat4_t o = to_object_from_location(l);
		WHEN("those transformes are combined") {
			mat4_t r = mat4_mul(w, o);
			THEN("they cancel each other out") {
				CHECK(r.m11 == Approx(mat4_identity.m11));
				CHECK(r.m12 == Approx(mat4_identity.m12));
				CHECK(r.m13 == Approx(mat4_identity.m13));
				CHECK(r.m14 == Approx(mat4_identity.m14));

				CHECK(r.m21 == Approx(mat4_identity.m21));
				CHECK(r.m22 == Approx(mat4_identity.m22));
				CHECK(r.m23 == Approx(mat4_identity.m23));
				CHECK(r.m24 == Approx(mat4_identity.m24));

				CHECK(r.m31 == Approx(mat4_identity.m31));
				CHECK(r.m32 == Approx(mat4_identity.m32));
				CHECK(r.m33 == Approx(mat4_identity.m33));
				CHECK(r.m34 == Approx(mat4_identity.m34).margin(0.01));

				CHECK(r.m41 == Approx(mat4_identity.m41));
				CHECK(r.m42 == Approx(mat4_identity.m42));
				CHECK(r.m43 == Approx(mat4_identity.m43));
				CHECK(r.m44 == Approx(mat4_identity.m44));
			}
		}
	}
}
