#ifndef COMMON_H
#define COMMON_H

#include <map>

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

static const addr_t TARGET_RADIUS = 0x4;

#endif //COMMON_H
