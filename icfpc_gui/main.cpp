#include "icfpc_gui.h"
#include <QtGui/QApplication>

#include "config.h"
#include "executer.h"

int main(int argc, char *argv[])
{
    Config config;
    if (!config.parse(argc, argv)){
        return -1;
    }
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
