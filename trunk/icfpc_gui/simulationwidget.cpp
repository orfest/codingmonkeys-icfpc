#include "simulationwidget.h"

#include <QPainter>
#include <QColor>
#include <vector>
#include <math.h>

const qreal SimulationWidget::EARTH_RADIUS = 6357000;
qreal SimulationWidget::MIN_X_OR_Y = 4*EARTH_RADIUS;

SimulationWidget::SimulationWidget(QWidget* parent) : QWidget(parent){
}

void SimulationWidget::setShipsNumber(int ships_number){
    ships.resize(ships_number);
	int pointsPerTrack = TOTAL_MAX_POINTS / ships_number;
    for (int i = 0; i < ships.size(); i++){
        QColor color;
        color.setHsv(255 * i / ships.size(), 230, 230);
        ships[i].setColor(color);
		ships[i].setMaxPoints(pointsPerTrack);
    }
}

void SimulationWidget::paintEvent(QPaintEvent*){
    int wid = width();
    int hei = height();

	for (QVector<Ship>::const_iterator it = ships.begin(); it != ships.end(); it++){
		for (int i = it->getTrack().size()-1; (i >= 0); i--){
			QPointF p = it->getTrack()[i];
			MIN_X_OR_Y = qMax(2.2*abs(p.x()),MIN_X_OR_Y);
			MIN_X_OR_Y = qMax(2.2*abs(p.y()),MIN_X_OR_Y);
		}
	}
    int smallest_dimension = qMin(wid,hei);
    qreal meter_per_pixel = MIN_X_OR_Y / ((qreal)smallest_dimension);
    qreal earth = EARTH_RADIUS / meter_per_pixel;
    qreal mid_x = wid*0.5;
    qreal mid_y = hei*0.5;
    QPainter painter(this);
    painter.setBrush(QBrush(Qt::blue));
    painter.drawEllipse(qRound(mid_x-earth), qRound(mid_y-earth), qRound(2.0*earth), qRound(2.0*earth));

    int h,s,v;
    for (QVector<Ship>::const_iterator it = ships.begin(); it != ships.end(); it++){
        it->getColor().getHsv(&h,&s,&v);
        QColor color(it->getColor());
		qreal alpha;
		qreal ship_paint_size;
        for (int i = it->getTrack().size()-1; (i >= 0); i--){
            painter.setBrush(QBrush(color));
            QPointF p = it->getTrack()[i];
            qreal px = mid_x + p.x() / meter_per_pixel;
            qreal py = mid_y + p.y() / meter_per_pixel;
			alpha = 1.0/pow(it->getTrack().size()-i,0.25);
			ship_paint_size = alpha*SHIP_SIZE;
			if (ship_paint_size >= 2)
				painter.drawEllipse(qRound(px-0.5*ship_paint_size), qRound(py-0.5*ship_paint_size), ship_paint_size, ship_paint_size);
			else
				painter.drawPoint(qRound(px), qRound(py));
        }
    }
}

void SimulationWidget::addShipsPositions(const std::vector<std::pair<double,double> >& points){
    std::vector<std::pair<double,double> >::const_iterator pt = points.begin();
    for (QVector<Ship>::iterator it = ships.begin(); it != ships.end(); it++, pt++){
        it->pushPosition(QPointF(pt->first, pt->second));
    }
}

void SimulationWidget::reset(){
	 for (QVector<Ship>::iterator it = ships.begin(); it != ships.end(); it++){
		it->reset();
	 }
}