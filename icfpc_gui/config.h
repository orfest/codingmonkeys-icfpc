#ifndef CONFIG_H
#define CONFIG_H

#include <string>

static const int MAX_PROBLEM_TYPE = 3;

struct Config{
    std::string program_file;
    std::string trace_output;
    bool gui;
    int problem;
    int scenario_number;
    Config():program_file("../download/bin1.obf"), trace_output("result.osf"), gui(false), problem(0), scenario_number(1002){}
    bool parse(int argc, char* argv[]);
};

#endif //CONFIG_H