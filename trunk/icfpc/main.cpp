#include <iostream>
#include <string>

#include "config.h"
#include "executer.h"

using namespace std;

int main(int argc, char* argv[]){
    Config config;
    if (!config.parse(argc, argv)){
        return -1;
    }
    Executer* ex = new Executer(config);
    ex->run();
    return 0;
}