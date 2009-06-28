#include "simulationwidget.h"

#include <QPainter>
#include <QColor>
#include <QMouseEvent>

#include <vector>
#include <cmath>
#include <cassert>

const qreal SimulationWidget::EARTH_RADIUS = 6357000;

SimulationWidget::SimulationWidget(QWidget* parent) : QWidget(parent), dragging(false), center(0,0){
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

	//for (QVector<Ship>::const_iterator it = ships.begin(); it != ships.end(); it++){
	//	for (int i = it->getTrack().size()-1; (i >= 0); i--){
	//		QPointF p = it->getTrack()[i];
	//		MIN_X_OR_Y = qMax(2.2*abs(p.x()),MIN_X_OR_Y);
	//		MIN_X_OR_Y = qMax(2.2*abs(p.y()),MIN_X_OR_Y);
	//	}
	//}
    int smallest_dimension = qMin(wid,hei);
    qreal meter_per_pixel = MIN_X_OR_Y / ((qreal)smallest_dimension);
    qreal earth = EARTH_RADIUS / meter_per_pixel;
    qreal mid_x = wid*0.5;
    qreal mid_y = hei*0.5;
    QPointF earthPos(-center);
    QPointF earthPosScreen(earthPos / meter_per_pixel);
    QPainter painter(this);
    painter.setBrush(QBrush(Qt::blue));
    QPoint ep(qRound(mid_x-earthPosScreen.x()),qRound(mid_y-earthPosScreen.y()));
    painter.drawEllipse(ep, qRound(earth), qRound(earth) );

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
            if (ship_paint_size >= 2){
                painter.drawEllipse(
                    qRound(px-0.5*ship_paint_size-earthPosScreen.x()), 
                    qRound(py-0.5*ship_paint_size-earthPosScreen.y()), 
                    ship_paint_size, 
                    ship_paint_size);
            } else { 
				painter.drawPoint(
                    qRound(px-earthPosScreen.x()), 
                    qRound(py-earthPosScreen.y()));
            }
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

void SimulationWidget::mousePressEvent(QMouseEvent* event){
    if (event->button() == Qt::LeftButton) {
        assert(!dragging);
        dragging = true;
        prevMouse = event->pos();
    }
    QWidget::mousePressEvent(event);
}

void SimulationWidget::mouseReleaseEvent(QMouseEvent* event){
    if (event->button() == Qt::LeftButton) {
        assert(dragging);
        dragging = false;
    }
    QWidget::mouseReleaseEvent(event);
}

void SimulationWidget::mouseMoveEvent(QMouseEvent* event){
    if (dragging){
        QPoint diff = event->pos() - prevMouse;
        prevMouse = event->pos();
        int wid = width();
        int hei = height();
        int smallest_dimension = qMin(wid,hei);
        qreal meter_per_pixel = MIN_X_OR_Y / ((qreal)smallest_dimension);
        QPointF centerToMove(
            diff.x() * meter_per_pixel,
            diff.y() * meter_per_pixel
        );
        center += centerToMove;
        update();
    }
    QWidget::mouseMoveEvent(event);
}

void SimulationWidget::zoomChanged(int value){
    qreal lv = value * 0.01;
    qreal expv10 = pow(10, lv);
    MIN_X_OR_Y = expv10 * EARTH_RADIUS;
    assert(MIN_X_OR_Y > 0);
    update();
}
