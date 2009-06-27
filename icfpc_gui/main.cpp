#include "icfpc_gui.h"
#include <QtGui/QApplication>

#include "config.h"
#include "executer.h"

#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char *argv[]) {
    freopen("err","wt",stderr);
    Config config;
    if (!config.parse(argc, argv)){
        return -1;
    }

    freopen(config.log_file_name.c_str(), "a", stdout);
    Executer* ex = new Executer(config);
    if (!config.gui){
        ex->run();
        return 0;
    } 
    QApplication a(argc, argv);
    icfpc_gui w(ex);
    w.show();
    return a.exec();
}
