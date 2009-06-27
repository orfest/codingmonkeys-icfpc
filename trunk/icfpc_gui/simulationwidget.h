#ifndef SIMULATION_WIDGET_H
#define SIMULATION_WIDGET_H

#include <QWidget>
#include "ship.h"

#include <vector>

class SimulationWidget : public QWidget{
    Q_OBJECT

    static const qreal EARTH_RADIUS;
    static const qreal MIN_X_OR_Y;
    static const int SHIP_SIZE = 4;

public:
    SimulationWidget(QWidget* parent = 0);
    void addShipsPositions(const std::vector<std::pair<double,double> >& points);
    void setShipsNumber(int ships_number);

protected:
    void paintEvent(QPaintEvent* event);

private:
    QVector<Ship> ships;
};

#endif //SIMULATION_WIDGET_H
