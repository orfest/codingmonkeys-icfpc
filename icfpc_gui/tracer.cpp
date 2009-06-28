#include "tracer.h"

#include <string>
#include <fstream>
#include <iostream>
#include <cassert>

using namespace std;

static const int TEAM_ID = 46;

std::ofstream& operator<<(std::ofstream& os, const Frame& frame){
    return frame.operator <<(os);
}

Frame::Frame(const PortMapping& that, int step):timestep(step),count(that.size()),changes(that){
}

int Frame::zeropadding(int v) const{
    int res = v;
    //char buf[4];          //
    //buf[0] = 0;
    //buf[1] = 0;
    //buf[2] = (v >> 8);
    //buf[3] = v & ((1 << 8)-1);
    //int res = 0;
    //for (int i = 0; i < 4; i++){
    //    res <<= 8;
    //    res |= buf[3-i];
    //}
    return res;
}

ofstream& Frame::operator<<(ofstream& os) const{
    assert(count == changes.size());
    char* buf = new char[2*sizeof(int) + (sizeof(int)+sizeof(double))*changes.size()];
    char* ptr = buf;
    memcpy(ptr, &timestep, sizeof(int)); ptr += sizeof(int);
    memcpy(ptr, &count, sizeof(int)); ptr += sizeof(int);
    for (PortMapping::const_iterator it = changes.begin(); it != changes.end(); it++){
        int zeropadded = zeropadding(it->first);
        memcpy(ptr, &zeropadded, sizeof(int)); ptr += sizeof(int);
        double tostore = it->second;
        memcpy(ptr, &tostore, sizeof(double)); ptr += sizeof(double);
    }
    os.write(buf, ptr-buf);
    return os;
}

Tracer::Tracer(const string& file, int scenario_number) :
                of(file.c_str(), ios_base::binary | ios_base::out | ios_base::trunc){
    if (of.fail()){
        cerr << "Cannot create dump file\n";
        assert("Cannot create dump file" == 0);
    }
    trace_data.push_back(0xCAFEBABE);
    trace_data.push_back(TEAM_ID);
    trace_data.push_back(scenario_number);
}

void Tracer::add(const PortMapping& data, int timestep){
    PortMapping change;
    for (PortMapping::const_iterator it = data.begin(); it != data.end(); it++){
        //if (it->first == SCENARIO_PORT) continue;
        if (prev.find(it->first) == prev.end() || prev[it->first] != it->second){
            change[it->first] = it->second;
        }
    }
    if (change.size() == 0) return;
    frames.push_back(Frame(change, timestep));
    for (PortMapping::const_iterator it = change.begin(); it != change.end(); it++){
        prev[it->first] = it->second;
    }
}

void Tracer::dump(int timestep) {
    frames.push_back(Frame(PortMapping(), timestep));
    of.write(reinterpret_cast<const char*>(&(trace_data[0])), trace_data.size() * sizeof(uint32));
    for (std::vector<Frame>::const_iterator it = frames.begin(); it != frames.end(); it++){
        of << (*it);
    }
    of.close();
}
