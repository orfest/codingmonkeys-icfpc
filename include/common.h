#ifndef COMMON_H
#define COMMON_H

#include <map>

#define _USE_MATH_DEFINES
#include <math.h>

typedef double data_t;
typedef unsigned int uint32;
typedef uint32 code_t;
typedef code_t addr_t;

typedef std::map<addr_t, data_t> PortMapping;

typedef std::pair<data_t, data_t> pointF;

static const addr_t SCENARIO_PORT = 0x3E80;
static const addr_t VX_PORT = 0x2;
static const addr_t VY_PORT = 0x3;

static const addr_t SCORE_PORT = 0x0;
static const addr_t FUEL_PORT = 0x1;
static const addr_t EARTH_X = 0x2;
static const addr_t EARTH_Y = 0x3;

//B1
static const addr_t TARGET_RADIUS = 0x4;

//B2
static const addr_t TARGET_X = 0x4;
static const addr_t TARGET_Y = 0x5;

//B3
static const addr_t SATELLITE_X = 0x4;
static const addr_t SATELLITE_Y = 0x5;

//B4
static const addr_t STATION_X = 0x4;
static const addr_t STATION_Y = 0x5;
static const addr_t FUEL_ON_STATION = 0x6;

#define DECL_TARGETN(n)	\
	static const addr_t TARGET##n##_X = (0x7 + 3*n);		\
	static const addr_t TARGET##n##_Y = (0x8 + 3*n);		\
	static const addr_t TARGET##n##_COLLECTED = (0x9 + 3*n);

DECL_TARGETN(0)
DECL_TARGETN(1)
DECL_TARGETN(2)
DECL_TARGETN(3)
DECL_TARGETN(4)
DECL_TARGETN(5)
DECL_TARGETN(6)
DECL_TARGETN(7)
DECL_TARGETN(8)
DECL_TARGETN(9)
DECL_TARGETN(10)
DECL_TARGETN(11)

#undef DECL_TARGETN(n)

static const addr_t MOON_X = 0x64;
static const addr_t MOON_Y = 0x65;

// various constants
static const double GRAVITATIONAL_CONST = 6.67428e-11;
static const double EARTH_MASS = 6.0e24;
static const double EARTH_RADIUS = 6.357e6;
static const double MU_CONST = GRAVITATIONAL_CONST * EARTH_MASS;
static const double ALPHA_CONST = 2.0 * M_PI / sqrt(MU_CONST);

#endif //COMMON_H
