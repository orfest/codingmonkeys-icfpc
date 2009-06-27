#include "icfpc_gui.h"

#include <QTimer>
#include <vector>
#include <QTableWidget>
#include <QTableWidgetItem>

#include <map>

#include "common.h"

icfpc_gui::icfpc_gui(Executer* ex_, QWidget *parent, Qt::WFlags flags)
    : QMainWindow(parent, flags), ex(ex_), timer(0), table(0) {
    ui.setupUi(this);
    ui.space->setShipsNumber(ex->getShipsNumber());
    timer = new QTimer(this);
    updateTimeout(0);
    connect(ui.action_Start, SIGNAL(triggered()), this, SLOT(start()));
    connect(ui.actionS_top, SIGNAL(triggered()), this, SLOT(stop()));
    connect(ui.timeoutSpinBox, SIGNAL(valueChanged(int)), this, SLOT(updateTimeout(int)));

    QStringList headers;
    headers << "Ports" << "Values";
    table = new QTableWidget(0,2,0); //rows, columns, parent
    table->setHorizontalHeaderLabels(headers);
    table->show();
}

icfpc_gui::~icfpc_gui() {
    delete timer;
}

void icfpc_gui::next(){
    if (!ex->nextStep()){
        timer->stop();
    }
    
    std::vector<std::pair<double, double> > shipsPositions = ex->getShipsPositions();
    ui.space->addShipsPositions(shipsPositions);

    updateTable();

    update();
}

void icfpc_gui::start(){
    connect(timer, SIGNAL(timeout()), this, SLOT(next()));
    timer->start(timeout);
}

void icfpc_gui::stop(){
    timer->stop();
    disconnect(timer, SIGNAL(timeout()), this, SLOT(next()));
}

void icfpc_gui::updateTable(){
    table->clear();
    table->setRowCount(ex->getOutput().size());
    int cnt = 0;
    for (std::map<addr_t, data_t>::const_iterator it = ex->getOutput().begin(); it != ex->getOutput().end(); it++, cnt++){
        QTableWidgetItem* port = new QTableWidgetItem(QString("%1").arg(it->first));
        QTableWidgetItem* value = new QTableWidgetItem(QString("%1").arg(it->second));
        table->setItem(cnt, 0, port);
        table->setItem(cnt, 1, value);
    }
}

void icfpc_gui::updateTimeout(int t){
    timeout = ui.timeoutSpinBox->value();
    if (timer->isActive()){
        timer->stop();
        timer->start(timeout);
    }
}
