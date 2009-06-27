#include "tracer.h"

#include <string>
#include <fstream>
#include <iostream>
#include <exception>

using namespace std;

static const int TEAM_ID = 48;

Tracer::Tracer(const string& file, int scenario_number) :
                of(file.c_str(), ios_base::binary | ios_base::out | ios_base::trunc){
    if (of.fail()){
        cerr << "Cannot create dump file\n";
        throw exception("Cannot create dump file");
    }
    of << 0xCAFEBABE << TEAM_ID << scenario_number;
}

void Tracer::add(const PortMapping& data, int timestep){
    PortMapping change;
    for (PortMapping::const_iterator it = data.begin(); it != data.end(); it++){
        if (prev.find(it->first) == prev.end() || prev[it->first] != it->second){
            change[it->first] = it->second;
        }
    }
    if (change.size() == 0) return;
    of << timestep << change.size();
    for (PortMapping::const_iterator it = change.begin(); it != change.end(); it++){
        of << it->first << it->second;
        prev[it->first] = it->second;
    }
}

void Tracer::dump(int timestep) {
    of << timestep << 0;
    of.close();
}
