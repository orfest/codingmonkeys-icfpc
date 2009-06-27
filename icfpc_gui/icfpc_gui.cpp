#include "icfpc_gui.h"

#include <QTimer>
#include <vector>
#include <QTableWidget>
#include <QTableWidgetItem>

#include <map>
#include <exception>

#include "common.h"

icfpc_gui::icfpc_gui(Executer* ex_, QWidget *parent, Qt::WFlags flags)
    : QMainWindow(parent, flags), ex(ex_), timer(0), table(0) {
    ui.setupUi(this);
    ui.space->setShipsNumber(ex->getShipsNumber());
    timer = new QTimer(this);
    updateTimeout(0);
    connect(ui.action_Start, SIGNAL(triggered()), this, SLOT(start()));
    connect(ui.actionPause, SIGNAL(triggered()), this, SLOT(pause()));
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
    for (int i = 0; i < 100; i++){
        if (!ex->nextStep()){
            timer->stop();
            break;
        }
    }
    
    std::vector<std::pair<double, double> > shipsPositions = ex->getShipsPositions();
    ui.space->addShipsPositions(shipsPositions);

    updateTable();

	ui.space->update();
    update();
}

void icfpc_gui::start(){
    reset();
    connect(timer, SIGNAL(timeout()), this, SLOT(next()));
    timer->start(timeout);
}

void icfpc_gui::pause(){
    if (timer->isActive()){
        timer->stop();
        disconnect(timer, SIGNAL(timeout()), this, SLOT(next()));
    } else {
        connect(timer, SIGNAL(timeout()), this, SLOT(next()));
        timer->start(timeout);
    }
}

void icfpc_gui::stop(){
    timer->stop();
    disconnect(timer, SIGNAL(timeout()), this, SLOT(next()));
}

void icfpc_gui::updateTable(){
    int rows = ex->getOutput().size() + 1; // + second number
    if (table->rowCount() > rows){
        throw std::exception("Number of rows decreases O__o");
    }
    int cnt = 0;
    for (std::map<addr_t, data_t>::const_iterator it = ex->getOutput().begin(); it != ex->getOutput().end(); it++, cnt++){
        QTableWidgetItem* port = new QTableWidgetItem(QString("%1").arg(it->first));
        QTableWidgetItem* value = new QTableWidgetItem(QString("%1").arg(it->second));
        if (table->rowCount() <= cnt){
            table->insertRow(cnt);
        }
        table->setItem(cnt, 0, port);
        table->setItem(cnt, 1, value);
    }
    {
        QTableWidgetItem* port = new QTableWidgetItem("Seconds");
        QTableWidgetItem* value = new QTableWidgetItem(QString("%1").arg(ex->getTimestep()));
        if (table->rowCount() <= cnt){
            table->insertRow(cnt);
        }
        table->setItem(cnt, 0, port);
        table->setItem(cnt, 1, value);
    }
}

void icfpc_gui::updateTimeout(int){
    timeout = ui.timeoutSpinBox->value();
    if (timer->isActive()){
        timer->stop();
        timer->start(timeout);
    }
}

void icfpc_gui::reset(){
    Executer* ex2 = new Executer(ex->getConfig());
    delete ex;
    ex = ex2;
	ui.space->reset();
}
