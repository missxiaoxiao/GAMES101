#include<cmath>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<iostream>
using namespace Eigen;
int main() {

	Vector3f p(2, 1, 1);
	Eigen::Matrix3f rotMat;
	Eigen::Matrix3f tranMat;
	float f = 45.0f;

	rotMat << std::cos(f / 180.0 * acos(-1)), -std::sin(f / 180.0 * acos(-1)), 0,
		std::sin(f / 180.0 * acos(-1)), std::cos(f / 180.0 * acos(-1)), 0,
		0, 0, 1;


	tranMat << 1, 0, 1,
		0, 1, 2,
		0, 0, 1;
	Vector3f p1 = tranMat*rotMat * p;

	std::cout << "before  : \n";
	std::cout << p << std::endl;
	std::cout << "after : \n";
	std::cout << p1 << std::endl;

	return 0;
}