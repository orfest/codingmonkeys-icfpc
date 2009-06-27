#ifndef ICFPC_GUI_H
#define ICFPC_GUI_H

#include <QtGui/QMainWindow>
#include "ui_icfpc_gui.h"

#include "executer.h"

class QTableWidget;

class icfpc_gui : public QMainWindow {
    Q_OBJECT

public slots:
    void start();
    void pause();
    void stop();

public:
    icfpc_gui(Executer* ex_, QWidget *parent = 0, Qt::WFlags flags = 0);
    ~icfpc_gui();

private slots:
    void next();
    void updateTimeout(int t);

private:
    Ui::icfpc_guiClass ui;
    int timeout;
    QTimer* timer;
    Executer* ex;
    QTableWidget* table;
    void updateTable();
    void reset();
};

#endif // ICFPC_GUI_H
