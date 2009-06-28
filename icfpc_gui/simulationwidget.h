#ifndef SIMULATION_WIDGET_H
#define SIMULATION_WIDGET_H

#include <QWidget>
#include "ship.h"

#include <vector>

class SimulationWidget : public QWidget{
    Q_OBJECT

    static const qreal EARTH_RADIUS;
    static const int SHIP_SIZE = 8;
	static const int TOTAL_MAX_POINTS = 2000;

public:
    SimulationWidget(QWidget* parent = 0);
    void addShipsPositions(const std::vector<std::pair<double,double> >& points);
    void setShipsNumber(int ships_number);
	void reset();

public slots:
    void zoomChanged(int);

protected:
    void paintEvent(QPaintEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);

private:
    QVector<Ship> ships;
    bool dragging;
    QPointF center;
    QPoint prevMouse;
    qreal MIN_X_OR_Y;
};

#endif //SIMULATION_WIDGET_H
