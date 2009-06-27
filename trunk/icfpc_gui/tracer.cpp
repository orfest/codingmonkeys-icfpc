#include "tracer.h"

#include <string>
#include <fstream>
#include <iostream>
#include <exception>

using namespace std;

static const int TEAM_ID = 46;

Tracer::Tracer(const string& file, int scenario_number) :
                of(file.c_str(), ios_base::binary | ios_base::out | ios_base::trunc){
    if (of.fail()){
        cerr << "Cannot create dump file\n";
        throw exception("Cannot create dump file");
    }
    trace_data.push_back(0xCAFEBABE);
    trace_data.push_back(TEAM_ID);
    trace_data.push_back(scenario_number);
}

void Tracer::add(const PortMapping& data, int timestep){
    PortMapping change;
    for (PortMapping::const_iterator it = data.begin(); it != data.end(); it++){
        if (it->first == SCENARIO_PORT) continue;
        if (prev.find(it->first) == prev.end() || prev[it->first] != it->second){
            change[it->first] = it->second;
        }
    }
    if (change.size() == 0) return;
    trace_data.push_back(timestep);
    trace_data.push_back(change.size());
    for (PortMapping::const_iterator it = change.begin(); it != change.end(); it++){
        trace_data.push_back(it->first);
        data_t to_store = it->second;
        data_t* ptr = &to_store;
        void* vptr = static_cast<void*>(ptr);
        uint32* uiptr = static_cast<uint32*>(vptr);
        trace_data.push_back(uiptr[0]);
        trace_data.push_back(uiptr[1]);
        prev[it->first] = it->second;
    }
}

void Tracer::dump(int timestep) {
    trace_data.push_back(timestep);
    trace_data.push_back(0);
    of.write(reinterpret_cast<const char*>(&(trace_data[0])), trace_data.size() * sizeof(uint32));
    of.close();
}
