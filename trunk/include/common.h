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
static const addr_t SATELLITE_X = 0x4;
static const addr_t SATELLITE_Y = 0x5;

//B1
static const addr_t TARGET_RADIUS = 0x4;

//B2
static const addr_t TARGET_X = 0x4;
static const addr_t TARGET_Y = 0x5;

static const double GRAVITATIONAL_CONST = 6.67428e-11;
static const double EARTH_MASS = 6.0e24;
static const double EARTH_RADIUS = 6.357e6;
static const double MU_CONST = GRAVITATIONAL_CONST * EARTH_MASS;
static const double ALPHA_CONST = 2.0 * M_PI / sqrt(MU_CONST);

#endif //COMMON_H
