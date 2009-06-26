#ifndef CONFIG_H
#define CONFIG_H

#include <string>

static const int MAX_PROBLEM_TYPE = 3;

struct Config{
    std::string program_file;
    std::string trace_output;
    bool gui;
    int problem;
    Config():program_file("../downloads/bin1.obf"), trace_output("result.osf"), gui(false), problem(0){}
    bool parse(int argc, char* argv[]);
};

#endif //CONFIG_H