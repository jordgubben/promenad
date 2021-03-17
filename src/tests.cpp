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

SCENARIO("Joint constraints") {
	limb_table_t limbs;
	init_limb_table(&limbs);

	GIVEN("A two segment arm with pointing toward +x") {
		limb_id_t arm = create_limb(vec3_origo, quat_identity, &limbs);
		uint16_t s1 = add_segment_to_limb(arm, vec3(2,0,0), &limbs);
		uint16_t s2 = add_segment_to_limb(arm, vec3(4,0,0), &limbs);

		THEN("Joint posisitons are where we expect") {
			CHECK( vec3(0,0,0) == vec3_round(get_segment_joint_position(s1, &limbs)));
			CHECK( vec3(2,0,0) == vec3_round(get_segment_joint_position(s2, &limbs)));
		}

		WHEN("attempting to reach point directly above") {
			move_limb_directly_to(arm, vec3(0,10,0), &limbs);
			THEN("Joints reposition as expected") {
				CHECK( vec3(0,0,0) == vec3_round(get_segment_joint_position(s1, &limbs)));
				CHECK( vec3(0,2,0) == vec3_round(get_segment_joint_position(s2, &limbs)));
			}
		}

		AND_GIVEN("the first segment may only rotate around it's extended axis") {
			set_segment_constraint(s1, jc_rotate_along_extention, &limbs);
			WHEN("attempting to reach point above") {
				move_limb_directly_to(arm, vec3(4,4,0), &limbs);
				THEN("Second joint stays in place (but may be rotated)") {
					vec3_t p2 = get_segment_joint_position(s2, &limbs);
					CHECK(vec3_round(p2) == vec3(2,0,0));
				}
			}
		}
	}
}
