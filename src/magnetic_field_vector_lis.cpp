#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>
#include "asa047.hpp"

#define MAX_DATA 1000
#define PI 3.14159265
int data_count = 0;
bool first = true;
sensor_msgs::MagneticField data[MAX_DATA];
int received = 0;
std::string magnetic_vector_topic;

void magnetometer_callback(const sensor_msgs::MagneticField::ConstPtr &msg) {
	if (data_count >= MAX_DATA) {
		first = false;
		data_count = 0;
	}
	received++;
	data[data_count] = *msg;
	data_count++;
}

geometry_msgs::Vector3 CrossProduct(geometry_msgs::Vector3 v_A,
		geometry_msgs::Vector3 v_B) {
	geometry_msgs::Vector3 c_P;
	c_P.x = v_A.y * v_B.z - v_A.z * v_B.y;
	c_P.y = -(v_A.x * v_B.z - v_A.z * v_B.x);
	c_P.z = v_A.x * v_B.y - v_A.y * v_B.x;
	return c_P;
}

double DotProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B) {
	return v_A.x * v_B.x + v_A.y * v_B.y + v_A.z * v_B.z;
}

geometry_msgs::Vector3 normalize_vector(geometry_msgs::Vector3 v) {
	double dist = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	v.x = v.x / dist;
	v.y = v.y / dist;
	v.z = v.z / dist;
	return v;
}
void ispisi_vektor(geometry_msgs::Vector3 vektor, std::string str) {
	std::cout << str << " " << vektor.x << " " << vektor.y << " " << vektor.z
			<< std::endl;
}

double measurements[MAX_DATA];
int measurements_count = 0;
double frequency = 50;
double magnetometer_sampling_time = 0.002;
double maxa, mina;
double Objective(double x[3]) {
	double a = x[0] * 1000000;
	double fi = x[1];
	double dc = x[2] * 1000000;

	double y = 0;
	for (int i = 0; i < measurements_count; i++) {
		double t = i * magnetometer_sampling_time; //
		double value = dc + a * sin(2 * 3.14159265 * frequency * t + fi);

		y = y
				+ (value - measurements[i] * 1000000)
						* (value - measurements[i] * 1000000);
	}
	return y;
}

double Optimize(int n, int f, double *data, double *phase) {
	int icount, numres, ifault;
	double initial_xd[3] = { 0.00001, 1, 0.000001 };
	double optim_krit;
	double optim_x[3];
	double optim_x_final[3];

	double step[3] = { 0.000001, 0.1, 0.0000001 };

	measurements_count = n;
	double minvalue = 0;
	double maxvalue = 0;
	double sum = 0;
	for (int i = 0; i < n; i++) {
		measurements[i] = data[i] * 400e-6 / 27368;
//		std::cout << measurements[i] << " ";
		if (data[i] < minvalue || i == 0) {
			minvalue = data[i];
		}
		if (data[i] > maxvalue || i == 0) {
			maxvalue = data[i];
		}
		sum = sum + data[i];
	}
	maxvalue = maxvalue * 400e-6 / 27368;
	minvalue = minvalue * 400e-6 / 27368;
	sum = sum * 400e-6 / 27368;
//	std::cout << std::endl;
	
	
	double mean = sum / n;
	double min = 1000000000000;
//	std::cout << std::endl;
	maxa = (maxvalue - minvalue) / 2 * 1.2 * 1000000;
	mina = (maxvalue - minvalue) / 2 * 0.8 * 1000000;
	initial_xd[0] = (maxvalue - minvalue) / 2;
	initial_xd[1] = 0.5;
	;
	initial_xd[2] = mean;
	nelmin(Objective, 3, initial_xd, optim_x, &optim_krit, 1.0e-8, step, 10,
			500, &icount, &numres, &ifault);

	min = optim_krit;
	optim_x_final[0] = optim_x[0];
	optim_x_final[1] = optim_x[1];
	optim_x_final[2] = optim_x[2];

	if (fabs(optim_x[0]) < (maxvalue - minvalue) / 2 / 5) {
		initial_xd[0] = (maxvalue - minvalue) / 2;
		initial_xd[1] = 0.5 + 3;
		initial_xd[2] = mean;
		nelmin(Objective, 3, initial_xd, optim_x, &optim_krit, 1.0e-8, step, 10,
				500, &icount, &numres, &ifault);

	}
//	std::cout << "max min vs optim " << (maxvalue - minvalue) / 2 << " "
//			<< optim_x[0] << "     optim 1  optim 2 " << min << " "
//			<< optim_krit << std::endl;
	if (optim_krit < min) {
		min = optim_krit;
		optim_x_final[0] = optim_x[0];
		optim_x_final[1] = optim_x[1];
		optim_x_final[2] = optim_x[2];
	}
	//std::cout << "final" << optim_x_final[0] << std::endl;
	*phase = optim_x_final[1];
	return optim_x_final[0];
}


geometry_msgs::Vector3 GetVectorUsingOptimization(int n, int f,
		ros::Time *stamp) {
	geometry_msgs::Vector3 result;
	double x[MAX_DATA];
	double y[MAX_DATA];
	double z[MAX_DATA];
	double time[MAX_DATA];
	int countx = 0, county = 0, countz = 0;
	int data_count_current = data_count;
	double poc = 0, kr = 0;
//	int current_element_old;
	ros::Time begin_stamp_x;
	ros::Time begin_stamp_y;
	ros::Time begin_stamp_z;
	ros::Time begin_stamp;

	bool first_x = true;
	bool first_y = true;
	bool first_z = true;

	double start_element_x = 0;
	double start_element_y = 0;
	double start_element_z = 0;

	for (int i = 0; i < n; i++) {
		int current_element = data_count_current + i - n;
		if (current_element < 0)
			current_element = current_element + MAX_DATA;
		x[countx] = data[current_element].magnetic_field.x;
		y[county] = data[current_element].magnetic_field.y;
		z[countz] = data[current_element].magnetic_field.z;

		if (i > 0) {
			if (x[countx] != x[countx - 1]) {
				countx++;
				if (first_x) {
					start_element_x = current_element;
					begin_stamp_x = data[current_element].header.stamp;
					first_x = false;
				}

			}
			if (y[county] != y[county - 1]) {
				county++;
				if (first_y) {
					start_element_y = current_element;
					begin_stamp_y = data[current_element].header.stamp;
					first_y = false;
				}
			}
			if (z[countz] != z[countz - 1]) {
				if (first_z) {
					start_element_z = current_element;
					begin_stamp_z = data[current_element].header.stamp;
					first_z = false;
				}
				countz++;
			}

		} else {
			countx++;
			county++;
			countz++;
		}

	}
	double anglex, angley, anglez;
	result.x = Optimize(countx, f, x, &anglex);
	result.y = Optimize(county, f, y, &angley);
	result.z = Optimize(countz, f, z, &anglez);

//	std::cout << result.x << " " << result.y << " " << result.z << std::endl;

	//Add pi if result is negative
	if (result.x < 0) {
		anglex = anglex + 3.14159;
		result.x = fabs(result.x);
	}
	if (result.y < 0) {
		angley = angley + 3.14159;
		result.y = fabs(result.y);
	}
	if (result.z < 0) {
		anglez = anglez + 3.14159;
		result.z = fabs(result.z);
	}

	//convert to range [0,2*pi]
	anglex = anglex - ((int) (anglex / (2 * 3.14159))) * 2 * 3.14159;
	angley = angley - ((int) (angley / (2 * 3.14159))) * 2 * 3.14159;
	anglez = anglez - ((int) (anglez / (2 * 3.14159))) * 2 * 3.14159;

	//find angle of component with largest amplitude between x,y,z
	double maxx = result.x, angle_max = anglex;
	begin_stamp = begin_stamp_x;
	int selected_angle = 0;
	if (result.y > maxx) {
		maxx = result.y;
		angle_max = angley;
		begin_stamp = begin_stamp_y;
		selected_angle = 1;
	}
	if (result.z > maxx) {
		maxx = result.z;
		angle_max = anglez;
		begin_stamp = begin_stamp_z;
		selected_angle = 2;
	}

	//change sign of components with angle offset greater than 90deg
	if (fabs(anglex - angle_max) > 3.14159 / 2
			&& fabs(fabs(anglex - angle_max) - 3.14159 * 2) > 3.14159 / 2) {
		result.x = -result.x;
	}
	if (fabs(angley - angle_max) > 3.14159 / 2
			&& fabs(fabs(angley - angle_max) - 3.14159 * 2) > 3.14159 / 2) {
		result.y = -result.y;
	}
	if (fabs(anglez - angle_max) > 3.14159 / 2
			&& fabs(fabs(anglez - angle_max) - 3.14159 * 2) > 3.14159 / 2) {
		result.z = -result.z;
	}
	if (angle_max < 0)
		angle_max += 2 * 3.14159265;

	//calculate time difference with regards to angle_max and create new timestamp
	ros::Duration difference(0,
			0.02 * (2 * 3.14159 - angle_max) / 2 / 3.14159265 * 1000000000);

	ros::Time new_time(begin_stamp + difference);

	int present = new_time.nsec / 1000000;


	*stamp = new_time;
	return result;
}

double Median(double v1, double v2, double v3) {
	if (v1 >= v2 && v1 <= v3)
		return v1;
	if (v1 <= v2 && v1 >= v3)
		return v1;
	if (v2 >= v1 && v2 <= v3)
		return v2;
	if (v2 <= v1 && v2 >= v3)
		return v2;
	return v3;
}

geometry_msgs::Vector3 Median(geometry_msgs::Vector3 v1,
		geometry_msgs::Vector3 v2, geometry_msgs::Vector3 v3) {
	geometry_msgs::Vector3 rez;
	rez.x = Median(v1.x, v2.x, v3.x);
	rez.y = Median(v1.y, v2.y, v3.y);
	rez.z = Median(v1.z, v2.z, v3.z);
	return rez;

}

double MedianN(int n, double* data) {
	for (int i=0;i<n-1;i++)
	{
		for(int j=i+1;j<n;j++)
		{
			if (data[i]>data[j])
			{
				double temp=data[i];
				data[i]=data[j];
				data[j]=temp;
			}
		}

	}
	return data[n/2];
}

geometry_msgs::Vector3 MedianN(int n, geometry_msgs::Vector3 *v) {
	geometry_msgs::Vector3 rez;
	double x[n],y[n],z[n];
	for (int i=0;i<n;i++)
	{
		x[i]=v[i].x;
		y[i]=v[i].y;
		z[i]=v[i].z;
	}

	rez.x = MedianN(n,x);
	rez.y = MedianN(n,y);
	rez.z = MedianN(n,z);
	return rez;

}




int main(int argc, char **argv) {
	ros::init(argc, argv, "magnetic_field_vector_fast");
	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");

	int magnetic_field_frequency, cycles_for_analysis;
	std::string magnetometer_topic;
	std::string magnetometer_frame = "/magnetometer1";

	int number_of_points_for_analysis;
	nh_ns.param("magnetometer_topic", magnetometer_topic,
			(std::string) "/imu_magnetic0");
	nh_ns.param("vector_topic", magnetic_vector_topic,
			(std::string) "/magnetic_vector0");
	nh_ns.param("magnetometer_frame", magnetometer_frame,
			(std::string) "/magnetometer0");
	nh_ns.param("magnetic_field_frequency", magnetic_field_frequency, 50);
	nh_ns.param("number_of_points_for_analysis", number_of_points_for_analysis,
			200);
	nh_ns.param("number_of_cycles_for_analysis", cycles_for_analysis, 5);
	nh_ns.param("magnetometer_sampling_time", magnetometer_sampling_time,
			0.01282);

	ros::Subscriber magnetometer_subscriber = nh.subscribe(magnetometer_topic,
			1000, magnetometer_callback);

	ros::Publisher magnetic_vector_publisher = nh.advertise<
			sensor_msgs::MagneticField>(magnetic_vector_topic, 1000);
	ros::Rate loop_rate(1000);


	//size of median filter
	geometry_msgs::Vector3 vector;
	geometry_msgs::Vector3 vector_final;
	geometry_msgs::Vector3 vector_final_old [200];

	int old_vectors=21;



	for (int i=0;i<old_vectors;i++)
	{
		vector_final_old[i].x=0;
		vector_final_old[i].y=0;
		vector_final_old[i].z=0;
	}
	int counter = 0;
	int cycle_count = 0;
	double x_size = 0;
	double y_size = 0;
	double z_size = 0;
	double vector_size = 0;
	vector_final.x = 0;
	vector_final.y = 0;
	vector_final.z = 0;

	tf::Transform transform1;
	tf::TransformBroadcaster br;

	while (ros::ok()) {

		ros::spinOnce();
		if (first == false && received > number_of_points_for_analysis) {

		/*
		 //////testing receiving rate
		 int data_count_current=data_count;
			int n=number_of_points_for_analysis;
			printf("%.4f   %d     ",1000*(data[data_count_current-1].header.stamp.toSec()-(int)(data[data_count_current-1].header.stamp.toSec())),data_count_current-1);
			for (int i=0;i<number_of_points_for_analysis;i++)
			{
				int current_element = data_count_current + i - n;
							if (current_element < 0)
								current_element = current_element + MAX_DATA;
				printf("  d %.4f   ",1000*(data[current_element].header.stamp.toSec()-(int)(data[current_element].header.stamp.toSec())));
			}
			printf("\n"); received=0; 		loop_rate.sleep();continue;*/
//			std::cout << "received " << received << std::endl;

			double pom1 = 0, pom2 = 0, pom3 = 0;

			//Calculate 50Hz vector by optimization od 50Hz sinusoidal
			ros::Time stamp;
			vector = GetVectorUsingOptimization(number_of_points_for_analysis,
					magnetic_field_frequency, &stamp);
			received = 0;


//			std::cout << vector.x << " " << vector.y << " " << vector.z
//					<< std::endl;

			//check if one value is NaN
			if (vector_final.x != vector_final.x)
				vector_final.x = 0;
			if (vector_final.y != vector_final.y)
				vector_final.y = 0;
			if (vector_final.z != vector_final.z)
				vector_final.z = 0;
			geometry_msgs::Vector3 pomocni_vektor = vector;


			//median last old_vector values

			for (int i=old_vectors-1;i>=1;i--)
			{
				vector_final_old[i]=vector_final_old[i-1];
			}
			vector_final_old[0]=pomocni_vektor;
			vector = MedianN(old_vectors, vector_final_old);



			vector_final.x = vector.x;
			vector_final.y = vector.y;
			vector_final.z = vector.z;
			vector_final.x *= 1000000;
			vector_final.y *= 1000000;
			vector_final.z *= 1000000;




			geometry_msgs::Vector3 pomocni;
			pomocni.x = 1;
			pomocni.y = 0;
			pomocni.z = 0;
			geometry_msgs::Vector3 kros = normalize_vector(
					CrossProduct(pomocni, normalize_vector(vector_final)));

			double w = 1 + DotProduct(pomocni, normalize_vector(vector_final));

			transform1.setOrigin(tf::Vector3(0, 0, 0));
			transform1.setRotation(tf::Quaternion(kros.x, kros.y, kros.z, w));
			br.sendTransform(
					tf::StampedTransform(transform1, ros::Time::now(),
							magnetometer_frame, magnetic_vector_topic));

			sensor_msgs::MagneticField publish_field;
			publish_field.magnetic_field = vector_final;
			publish_field.header.stamp = stamp;

			magnetic_vector_publisher.publish(publish_field);
			x_size = 0;
			y_size = 0;
			z_size = 0;
			vector_size = 0;
			cycle_count = 0;
//			std::cout << magnetometer_frame << std::endl;

		}
		loop_rate.sleep();

	}
}

