#ifndef CONFIG_H
#define CONFIG_H

#include <string>

static const int MAX_PROBLEM_TYPE = 3;

struct Config{
    std::string program_file;
    std::string trace_output;
    std::string log_file_name;
    bool gui;
    int problem;
    int scenario_number;
    Config():program_file("../download/bin1.obf"), 
            trace_output("result.osf"), 
            log_file_name("log.txt"),
            gui(false), 
            problem(0), 
            scenario_number(1001){}
    bool parse(int argc, char* argv[]);
};

#endif //CONFIG_H