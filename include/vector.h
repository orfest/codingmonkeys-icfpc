#ifndef VECTOR_H
#define VECTOR_H

#include "common.h"

class Vector {
public:
	double x;
	double y;

	Vector(double x = 0.0, double y = 0.0);
	Vector(const Vector & vec);
	Vector(const pointF & pnt) : x(pnt.first), y(pnt.second) {};

	operator pointF() const { return pointF(x, y); };
	
	double operator[](int ind) const;
	double & operator[](int ind);

	Vector & operator+=(const Vector & vec);
	Vector & operator-=(const Vector & vec);
	Vector & operator*=(double mul);
	Vector & operator/=(double div);

	Vector operator+(const Vector & vec) const;
	Vector operator-(const Vector & vec) const;
	Vector operator*(double mul) const;
	Vector operator/(double div) const;
	Vector operator-() const;

	bool operator==(const Vector & vec) const;
	bool operator!=(const Vector & vec) const;

	double length() const;
	double sqLength() const;

	Vector & normalize();

	static double dotProduct(const Vector & a, const Vector & b);
	static double crossProduct(const Vector & a, const Vector & b);
};

#endif