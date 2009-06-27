#include "vector.h"
#include "assert.h"
#include <math.h>

Vector::Vector(double x, double y) : x(x), y(y) {}

Vector::Vector(const Vector &vec) : x(vec.x), y(vec.y) {}

double Vector::operator[](int ind) const {
	assert(ind >= 0 && ind < 2);
	return ind == 0 ? x : y;
}

double & Vector::operator[](int ind) {
	assert(ind >= 0 && ind < 2);
	return ind == 0 ? x : y;
}

Vector & Vector::operator+=(const Vector & vec) {
	x += vec.x;
	y += vec.y;
	return *this;
}

Vector & Vector::operator-=(const Vector & vec) {
	x -= vec.x;
	y -= vec.y;
	return *this;
}

Vector & Vector::operator*=(double mul) {
	x *= mul;
	y *= mul;
	return *this;
}

Vector & Vector::operator/=(double div) {
	assert(div != 0.0);
	x /= div;
	y /= div;
	return *this;
}

Vector Vector::operator+(const Vector & vec) const {
	Vector r(*this);
	r += vec;
	return r;
}
	
Vector Vector::operator-(const Vector & vec) const {
	Vector r(*this);
	r -= vec;
	return r;
}

Vector Vector::operator*(double mul) const {
	Vector r(*this);
	r *= mul;
	return r;
}

Vector Vector::operator/(double div) const {
	Vector r(*this);
	r /= div;
	return r;
}

Vector Vector::operator-() const {
	return Vector(-x, -y);
}

bool Vector::operator==(const Vector & vec) const {
	return x == vec.x && y == vec.y;
}

bool Vector::operator!=(const Vector & vec) const {
	return x != vec.x || y != vec.y;
}

double Vector::length() const {
	return sqrt(sqLength());
}

double Vector::sqLength() const {
	return x*x + y*y;
}

Vector & Vector::normalize() {
	double len = length();
	if (len > 0.0)
		*this /= len;
	return *this;
}

double Vector::dotProduct(const Vector & a, const Vector & b) {
	return a.x * b.x + a.y * b.y;
}

double Vector::crossProduct(const Vector & a, const Vector & b) {
	return a.x * b.y - a.y * b.x;
}
