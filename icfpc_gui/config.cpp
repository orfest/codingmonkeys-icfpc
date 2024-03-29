#include "config.h"

#include <string>
#include <iostream>
#include <cassert>

using namespace std;

bool Config::parse(int argc, char* argv[]){
    for (int i = 1; i < argc;){
        string a(argv[i]);
        if (a.compare("--gui") == 0){
            gui = true;
            i++;
        } else if (a.compare("-i") == 0){
            if (i >= argc - 1){
                cerr << "Program path not specified\n";
                return false;
            }
            program_file = argv[i+1];
            i += 2;
        } else if (a.compare("-o") == 0){
            if (i >= argc - 1){
                cerr << "Output path not specified\n";
                return false;
            }
            trace_output = argv[i+1];
            i += 2;
        } else if (a.compare("-p") == 0){
            if (i >= argc - 1){
                cerr << "Problem type not specified\n";
                return false;
            }
            problem = atoi(argv[i+1]);
            //if (problem < 0 || problem > MAX_PROBLEM_TYPE){
            //    assert("Incorrect problem type" != 0);
            //    return false;
            //}
            i += 2;
        } else if (a.compare("-s") == 0){
            if (i >= argc - 1){
                cerr << "Scenario number not specified\n";
                return false;
            }
            scenario_number = atoi(argv[i+1]);
            i += 2;
        } else if (a.compare("-l") == 0){
            if (i >= argc - 1){
                cerr << "Log file name not specified\n";
                return false;
            }
            log_file_name = argv[i+1];
            i += 2;
        } else {
            cerr << "Warning: unknown option " << a << "\n";
            i++;
        }
    }
    return true;
}
