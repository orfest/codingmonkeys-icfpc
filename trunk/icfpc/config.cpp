#include "config.h"

#include <string>
#include <iostream>

using namespace std;

bool Config::parse(int argc, char* argv[]){
    for (int i = 1; i < argc;){
        string a(argv[i]);
        if (a.compare("--gui") == 0){
            gui = true;
            i++;
        } else if (a.substr(0,2).compare("-i") == 0){
            if (i >= argc - 1){
                cerr << "Program path not specified\n";
                return false;
            }
            program_file = argv[i+1];
            i += 2;
        } else if (a.substr(0,2).compare("-o") == 0){
            if (i >= argc - 1){
                cerr << "Output path not specified\n";
                return false;
            }
            trace_output = argv[i+1];
            i += 2;
        } else if (a.substr(0,2).compare("-p") == 0){
            if (i >= argc - 1){
                cerr << "Problem type not specified\n";
                return false;
            }
            problem = atoi(argv[i+1]);
            if (problem < 0 || problem > MAX_PROBLEM_TYPE){
                cerr << "Incorrect problem type\n";
                return false;
            }
            i += 2;
        } else {
            cerr << "Warning: unknown option " << a << "\n";
            i++;
        }
    }
    return true;
}
