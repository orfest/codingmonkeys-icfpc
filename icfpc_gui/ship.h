#ifndef SHIP_H
#define SHIP_H

#include <QtGlobal>
#include <QVector>
#include <QPointF>
#include <QColor>

class Ship{
    QVector<QPointF> track;
    QColor color;
public:
    QColor getColor() const { return color; }
    const QVector<QPointF>& getTrack() const { return track; }
    void pushPosition(const QPointF& p);
    void setColor(QColor col) { color = col; }
	const void reset() { track.clear(); }
};

#endif //SHIP_H
