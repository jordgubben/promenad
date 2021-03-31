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

	GIVEN("Two bones at a 90 deg angle") {
		vec3_t p1 = vec3(10,10,0), p2 = vec3(10,20,0), p3 = vec3(20,20,0);
		bone_t bone_a = {{jc_no_constraint}, p1, quat_from_axis_angle(vec3_positive_z, pi/2), 10};
		bone_t bone_b = {{jc_no_constraint}, p2, quat_identity, 10};

		THEN("Tips are where we expect") {
			CHECK(vec3_round(get_bone_tip(bone_a)) == p2);
			CHECK(vec3_round(get_bone_tip(bone_b)) == p3);
		}

		AND_GIVEN("A hinge joint limits it between -45 and 45 degrees") {
			bone_b.constraint.type = jc_hinge;
			bone_b.constraint.min_ang = -pi/4;
			bone_b.constraint.max_ang = +pi/4;

			WHEN("The first bone is constrained by the second") {
				constrain_to_next_bone(&bone_b, &bone_a);

				THEN("The first bone is tilted to 45 degree angle") {
					CHECK(vec3_round(get_bone_tip(bone_a)) == bone_b.joint_pos);
					CHECK(vec3_round(bone_a.joint_pos) == vec3(3, 13, 0));
				}
			}

			WHEN("The second bone is constrained by the first") {
				constrain_to_prev_bone(&bone_a, &bone_b);

				THEN("The second bone is tilted to 45 degree angle") {
					CHECK(get_bone_tip(bone_a) == bone_b.joint_pos);
					CHECK(vec3_round(get_bone_tip(bone_b)) == vec3(17, 27, 0));
				}
			}
		}

		AND_GIVEN("A hinge joint limits it between 0 and 90 degrees") {
			bone_b.constraint.type = jc_hinge;
			bone_b.constraint.min_ang = 0;
			bone_b.constraint.max_ang = pi/2;

			WHEN("The first bone is constrained by the second") {
				constrain_to_next_bone(&bone_b, &bone_a);

				THEN("The first bone is more or less unchanged") {
					CHECK(get_bone_tip(bone_a) == bone_b.joint_pos);
					CHECK(vec3_round(bone_a.joint_pos) == p1);
				}
			}

			WHEN("The second bone is constrained by the first") {
				constrain_to_prev_bone(&bone_a, &bone_b);

				THEN("The second bone is strighted out parallell wit the first") {
					CHECK(get_bone_tip(bone_a) == bone_b.joint_pos);
					CHECK(vec3_round(get_bone_tip(bone_b)) == vec3(10, 30, 0));
				}
			}
		}
	}

	GIVEN("Given two parallell bones with a twist") {
		vec3_t p1 = vec3(0,10,0), p2 = vec3(0,20,0), p3 = vec3(0,30,0);
		bone_t bone_a = {{jc_no_constraint}, p1, quat_from_axis_angle(vec3_positive_z, pi/2), 10};
		bone_t bone_b = {{jc_no_constraint}, p2, quat_from_axis_angle(vec3_positive_z, pi/2), 10};
		bone_b.orientation = quat_mul(quat_from_axis_angle(vec3_positive_y, pi/2), bone_b.orientation);

		THEN("Tips are where we expect them") {
			CHECK(get_bone_tip(bone_a) == vec3(0,20,0));
			CHECK(get_bone_tip(bone_b) == vec3(0,30,0));
		}

		THEN("The second bone still transforms to something sane") {
			vec3_t local_x = quat_rotate_vec3(bone_b.orientation, vec3_positive_x);
			vec3_t local_y = quat_rotate_vec3(bone_b.orientation, vec3_positive_y);
			vec3_t local_z = quat_rotate_vec3(bone_b.orientation, vec3_positive_z);

			CHECK(vec3_round(vec3_cross(local_x, local_y)) == vec3_round(local_z));
		}

		AND_GIVEN("they are constrained with a hinge joint") {
			bone_b.constraint.type = jc_hinge;
			bone_b.constraint.min_ang = -pi/2;
			bone_b.constraint.max_ang = +pi/2;

			WHEN("first bone constrains the second") {
				constrain_to_prev_bone(&bone_a, &bone_b);
				THEN("Second bone is oriented like the first") {
					CHECK(vec3_round(quat_rotate_vec3(bone_b.orientation, vec3_positive_x)) == vec3_positive_y);
					CHECK(vec3_round(quat_rotate_vec3(bone_b.orientation, vec3_positive_y)) == vec3_negative_x);
					CHECK(vec3_round(quat_rotate_vec3(bone_b.orientation, vec3_positive_z)) == vec3_positive_z);
				}

				THEN("Tip positions are maintained") {
					CHECK(vec3_round(get_bone_tip(bone_a)) == vec3(0,20,0));
					CHECK(vec3_round(get_bone_tip(bone_b)) == vec3(0,30,0));
				}
			}

			WHEN("second bone constrains the first") {
				constrain_to_next_bone(&bone_b, &bone_a);
				THEN("the first bone is oriented like the second") {
					// First bone
					CHECK(vec3_round(quat_rotate_vec3(bone_a.orientation, vec3_positive_x)) == vec3_positive_y);
					CHECK(vec3_round(quat_rotate_vec3(bone_a.orientation, vec3_positive_y)) == vec3_positive_z);
					CHECK(vec3_round(quat_rotate_vec3(bone_a.orientation, vec3_positive_z)) == vec3_positive_x);

					// Second bone
					CHECK(vec3_round(quat_rotate_vec3(bone_b.orientation, vec3_positive_x)) == vec3_positive_y);
					CHECK(vec3_round(quat_rotate_vec3(bone_b.orientation, vec3_positive_y)) == vec3_positive_z);
					CHECK(vec3_round(quat_rotate_vec3(bone_b.orientation, vec3_positive_z)) == vec3_positive_x);
				}

				THEN("Tip positions are maintained") {
					CHECK(vec3_round(get_bone_tip(bone_a)) == vec3(0,20,0));
					CHECK(vec3_round(get_bone_tip(bone_b)) == vec3(0,30,0));
				}
			}
		}
	}


	GIVEN("An arm with a single bone pointing toward +x") {
		limb_id_t arm = create_limb(vec3_origo, quat_identity, &limbs);
		uint16_t s1 = add_bone_to_limb(arm, vec3(10,0,0), &limbs);
		CHECK(get_limb_end_effector_position(arm, &limbs) == get_limb_tip_position(arm, &limbs));

		AND_GIVEN("It's constrained by a hinge joint with a 45 degree limit") {
			apply_hinge_constraint(s1, -pi/4, pi/4, &limbs);

			WHEN("Attempting to move to the cooresponding distance on the positive y-axis") {
				move_limb_directly_to(arm, vec3(0,+10,0), &limbs);

				THEN("the bone is constrained to an 45 deg angle") {
					CHECK( vec3(+7,+7,0) == vec3_round(get_bone_tip_position(s1, &limbs)));
				}
			}

			WHEN("Attempting to move to the cooresponding distance on the positive y-axis") {
				move_limb_directly_to(arm, vec3(0,-10,0), &limbs);

				THEN("the bone is constrained to an 45 deg angle") {
					CHECK( vec3(+7,-7,0) == vec3_round(get_bone_tip_position(s1, &limbs)));
				}
			}
		}
	}

	GIVEN("A two segment arm with pointing toward +x") {
		limb_id_t arm = create_limb(vec3_origo, quat_identity, &limbs);
		uint16_t s1 = add_bone_to_limb(arm, vec3(2,0,0), &limbs);
		uint16_t s2 = add_bone_to_limb(arm, vec3(4,0,0), &limbs);
		CHECK(get_limb_end_effector_position(arm, &limbs) == get_limb_tip_position(arm, &limbs));

		THEN("Joint posisitons are where we expect") {
			CHECK( vec3(0,0,0) == vec3_round(get_bone_joint_position(s1, &limbs)));
			CHECK( vec3(2,0,0) == vec3_round(get_bone_joint_position(s2, &limbs)));
		}

		WHEN("attempting to reach point directly above") {
			move_limb_directly_to(arm, vec3(0,10,0), &limbs);
			THEN("Joints reposition as expected") {
				CHECK( vec3(0,0,0) == vec3_round(get_bone_joint_position(s1, &limbs)));
				CHECK( vec3(0,2,0) == vec3_round(get_bone_joint_position(s2, &limbs)));
			}
		}

		AND_GIVEN("the first segment may only rotate around it's extended axis") {
			apply_pole_constraint(s1, &limbs);
			WHEN("attempting to reach point above") {
				move_limb_directly_to(arm, vec3(4,4,0), &limbs);
				THEN("Second joint stays in place (but may be rotated)") {
					vec3_t p2 = get_bone_joint_position(s2, &limbs);
					CHECK(vec3_round(p2) == vec3(2,0,0));
				}
			}
		}
	}
}
