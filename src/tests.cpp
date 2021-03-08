#include <catch2/catch.hpp>
#include <raylib.h>

#define IN_TESTS
#include "overview.h"

SCENARIO("Example windowd test", "[.][windowed]"){
	InitWindow(200, 100, "Play subsystem windowed test");

	// TODO: Preform windowed tests

	CloseWindow();
}

SCENARIO("Example test") {
	CHECK(true);
	REQUIRE(true);
}
