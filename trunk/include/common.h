#ifndef COMMON_H
#define COMMON_H

#include <map>

typedef double data_t;
typedef unsigned int uint32;
typedef uint32 code_t;
typedef code_t addr_t;

typedef std::map<addr_t, data_t> PortMapping;

#endif //COMMON_H
