#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry> 

struct BoxQ
{
	// translation vector to represent the position of the box
	Eigen::Vector3f bboxTransform;
	// quaternion to represent the rotation of the box
	Eigen::Quaternionf bboxQuaternion;
	// length, width, and height of the box
	float cube_length;
    float cube_width;
	// height of the box
    float cube_height;
};
struct Box
{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
};
#endif