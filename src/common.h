#ifndef COMMON_H
#define COMMON_H
#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include<iostream>

inline geometry_msgs::Vector3 operator*(geometry_msgs::Vector3 o1, double o2) {
	geometry_msgs::Vector3 rez;
	rez.x = o1.x * o2;
	rez.y = o1.y * o2;
	rez.z = o1.z * o2;
	return rez;
}

inline geometry_msgs::Vector3 operator*(double o2, geometry_msgs::Vector3 o1) {
	geometry_msgs::Vector3 rez;
	rez.x = o1.x * o2;
	rez.y = o1.y * o2;
	rez.z = o1.z * o2;
	return rez;
}

inline geometry_msgs::Vector3 operator/(geometry_msgs::Vector3 o1, double o2) {
	geometry_msgs::Vector3 rez;
	rez.x = o1.x / o2;
	rez.y = o1.y / o2;
	rez.z = o1.z / o2;
	return rez;
}

 geometry_msgs::Vector3  Convert(const tf::Vector3& o1) {
	geometry_msgs::Vector3 rez;
	rez.x = o1.getX();
	rez.y = o1.getY();
	rez.z = o1.getZ();
	return rez;
}

inline geometry_msgs::Vector3 operator+(geometry_msgs::Vector3 o1,
		tf::Vector3 o2) {
	geometry_msgs::Vector3 rez;
	rez.x = o1.x + o2.getX();
	rez.y = o1.y + o2.getY();
	rez.z = o1.z + o2.getZ();
	return rez;
}

inline geometry_msgs::Vector3 operator-(geometry_msgs::Vector3 o1,
		geometry_msgs::Vector3 o2) {
	geometry_msgs::Vector3 rez;
	rez.x = o1.x - o2.x;
	rez.y = o1.y - o2.y;
	rez.z = o1.z - o2.z;
	return rez;
}

inline geometry_msgs::Vector3 CrossProduct(geometry_msgs::Vector3 v_A,
		geometry_msgs::Vector3 v_B) {
	geometry_msgs::Vector3 c_P;
	c_P.x = v_A.y * v_B.z - v_A.z * v_B.y;
	c_P.y = -(v_A.x * v_B.z - v_A.z * v_B.x);
	c_P.z = v_A.x * v_B.y - v_A.y * v_B.x;
	return c_P;
}

inline geometry_msgs::Vector3 sum_vector(geometry_msgs::Vector3 v1,
		geometry_msgs::Vector3 v2) {
	geometry_msgs::Vector3 v3;
	v3.x = v1.x + v2.x;
	v3.y = v1.y + v2.y;
	v3.z = v1.z + v2.z;
	return v3;
}

inline geometry_msgs::Vector3 normalize_vector(geometry_msgs::Vector3 v) {
	double dist = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	v.x = v.x / dist;
	v.y = v.y / dist;
	v.z = v.z / dist;
	return v;
}

inline double VectorSize(geometry_msgs::Vector3 vector) {
	return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

inline double DotProduct(geometry_msgs::Vector3 v_A,
		geometry_msgs::Vector3 v_B) {
	return v_A.x * v_B.x + v_A.y * v_B.y + v_A.z * v_B.z;
}

double GetAngleBetweenVectors(geometry_msgs::Vector3 v1,
		geometry_msgs::Vector3 v2) {
	return acos(DotProduct(v1, v2) / VectorSize(v1) / VectorSize(v2));
}
inline geometry_msgs::Vector3 operator+(geometry_msgs::Vector3 o1,
		geometry_msgs::Vector3 o2) {
	geometry_msgs::Vector3 rez;
	rez.x = o1.x + o2.x;
	rez.y = o1.y + o2.y;
	rez.z = o1.z + o2.z;

	return rez;
}
void ispisi_vektor(geometry_msgs::Vector3 vektor, std::string str) {
	std::cout << str << " " << vektor.x << " " << vektor.y << " " << vektor.z
			<< std::endl;
}

inline geometry_msgs::Vector3 transform_vector(geometry_msgs::Vector3 vector,
		tf::Matrix3x3 rotation) {
	geometry_msgs::Vector3 result;

	result.x = rotation.getRow(0).getX() * vector.x
			+ rotation.getRow(0).getY() * vector.y
			+ rotation.getRow(0).getZ() * vector.z;
	result.y = rotation.getRow(1).getX() * vector.x
			+ rotation.getRow(1).getY() * vector.y
			+ rotation.getRow(1).getZ() * vector.z;
	result.z = rotation.getRow(2).getX() * vector.x
			+ rotation.getRow(2).getY() * vector.y
			+ rotation.getRow(2).getZ() * vector.z;
	return result;
}

void ispisi_vektor(tf::Vector3 vektor, std::string str) {
	std::cout << str << " " << vektor.getX() << " " << vektor.getY() << " "
			<< vektor.getZ() << std::endl;
}

inline geometry_msgs::Vector3 operator-(geometry_msgs::Vector3 o1,
		tf::Vector3 o2) {
	geometry_msgs::Vector3 rez;
	rez.x = o1.x - o2.getX();
	rez.y = o1.y - o2.getY();
	rez.z = o1.z - o2.getZ();
	return rez;
}


#endif
