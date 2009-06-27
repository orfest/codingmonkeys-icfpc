#include "simulationwidget.h"

#include <QPainter>
#include <QColor>
#include <vector>

const qreal SimulationWidget::EARTH_RADIUS = 6357000;
const qreal SimulationWidget::MIN_X_OR_Y = 3*EARTH_RADIUS;

SimulationWidget::SimulationWidget(QWidget* parent) : QWidget(parent){
}

void SimulationWidget::setShipsNumber(int ships_number){
    ships.resize(ships_number);
    for (int i = 0; i < ships.size(); i++){
        QColor color;
        color.setHsv(255 * i / ships.size(), 230, 230);
        ships[i].setColor(color);
    }
}

void SimulationWidget::paintEvent(QPaintEvent* event){
    int wid = width();
    int hei = height();
    int smallest_dimension = qMin(wid,hei);
    qreal meter_per_pixel = MIN_X_OR_Y / ((qreal)smallest_dimension);
    int earth = qRound(EARTH_RADIUS / meter_per_pixel);
    int mid_x = qRound(wid*0.5);
    int mid_y = qRound(hei*0.5);
    QPainter painter(this);
    painter.setBrush(QBrush(Qt::blue));
    painter.drawEllipse(qRound(mid_x-earth*0.5), qRound(mid_y-earth*0.5), earth, earth);

    int h,s,v;
    for (QVector<Ship>::const_iterator it = ships.begin(); it != ships.end(); it++){
        it->getColor().getHsv(&h,&s,&v);
        QColor color(it->getColor());
        for (int i = it->getTrack().size()-1; i >= 0; i--){
            v -= 5;
            v = qMax(v,0);
            color.setHsv(h,s,v);
            painter.setBrush(QBrush(color));
            QPointF p = it->getTrack()[i];
            int px = mid_x + qRound(p.x() / meter_per_pixel);
            int py = mid_y + qRound(p.y() / meter_per_pixel);
            painter.drawEllipse(qRound(px-0.5*SHIP_SIZE), qRound(py-0.5*SHIP_SIZE), SHIP_SIZE, SHIP_SIZE);
        }
    }
}

void SimulationWidget::addShipsPositions(const std::vector<std::pair<double,double> >& points){
    std::vector<std::pair<double,double> >::const_iterator pt = points.begin();
    for (QVector<Ship>::iterator it = ships.begin(); it != ships.end(); it++, pt++){
        it->pushPosition(QPointF(pt->first, pt->second));
    }
}
