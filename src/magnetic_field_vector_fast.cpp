
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
int data_count=0;
bool first = true;
sensor_msgs::MagneticField data[MAX_DATA];
int received=0;
void magnetometer_callback(const sensor_msgs::MagneticField::ConstPtr& msg){
	if (data_count>=MAX_DATA)
	{
		first =false;
		data_count=0;
	}
	received ++;
	data[data_count]=*msg;
	data_count++;
}
void GetSumOfDifferences(double* data1, double* data2, int n,double *result1,double *result2)
{
	double suma1 = 0;
	double suma2 = 0;
	double pod1;
	double pod2;
	for (int i = 1; i < n; i++)
	{
		pod1=0;
		pod2=0;
		if (data1[i] - data1[i - 1] > 0)  pod1 = 1;
		if (data1[i] - data1[i - 1] < 0)  pod1 = -1;
		if (data2[i] - data2[i - 1] > 0)  pod2 = 1;
		if (data2[i] - data2[i - 1] < 0)  pod2 = -1;

		suma1 = suma1 + fabs(pod1);
		suma2 = suma2 + fabs(pod1 - pod2);

	}
	*result1 = suma1;
	*result2 = suma2;

}


double GetRMS(double* time, double* data, int n, double f)
{
	double sum=0;
	for (int i = 0; i < n; i++)
	{
		double timediff=0;
		sum=sum+data[i]*data[i];
	}
	return sqrt(sum/n);
}

geometry_msgs::Vector3 GetVectorUsingRMS(int n, int f, double *suma1, double *suma2, double *suma3)
{
	geometry_msgs::Vector3 result;
	double x[MAX_DATA];
	double y[MAX_DATA];
	double z[MAX_DATA];
	double time[MAX_DATA];

	for (int i=0;i<n;i++)
	{
		int current_element = data_count + i - n;
		if (current_element<0) current_element = current_element + MAX_DATA;
		x[i] = data[current_element].magnetic_field.x;
		y[i] = data[current_element].magnetic_field.y;
		z[i] = data[current_element].magnetic_field.z;
	}
	double anglex,angley,anglez;
	result.x = GetRMS(time, x, n, f);
	result.y = GetRMS(time, y, n, f);
	result.z = GetRMS(time, z, n, f);

	GetSumOfDifferences(x, y, n,suma1,suma2);
	GetSumOfDifferences(x, z, n,suma1,suma3);


	return result;
}


geometry_msgs::Vector3 CrossProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B) {
	geometry_msgs::Vector3 c_P;
   c_P.x = v_A.y * v_B.z - v_A.z * v_B.y;
   c_P.y = -(v_A.x * v_B.z - v_A.z * v_B.x);
   c_P.z = v_A.x * v_B.y - v_A.y * v_B.x;
   return c_P;
}


double DotProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B)
{
	return v_A.x * v_B.x + v_A.y * v_B.y + v_A.z * v_B.z;
}


geometry_msgs::Vector3 normalize_vector(geometry_msgs::Vector3 v)
{
	double dist=sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	v.x = v.x / dist;
	v.y = v.y / dist;
	v.z = v.z / dist;
	return v;
}

double measurements[MAX_DATA];
int measurements_count = 0;
double frequency = 50;
double magnetometer_sampling_time=0.01282;
double maxa, mina;
double Objective(double x[3])
{
	double a = x[0] * 1000000;
	double fi = x[1];
	double dc = x[2] * 1000000;
//	if (a > maxa) return 1000000000;
//	if (a < mina) return 1000000000;

	//	std::cout<<a <<" "<< fi<<" " <<dc<<std::endl;
	double y = 0;
	for (int i = 0; i < measurements_count; i++)
	{
		double t = i * magnetometer_sampling_time; //
		double value = dc + a * sin(2 * 3.14159265 * frequency * t + fi);
		//std::cout<< value << " " << measurements[i]*1000000<< std::endl;

		y = y + (value - measurements[i] * 1000000) * (value - measurements[i]*1000000);
	}
//	std::cout<< y << std::endl << std::endl;
	return y;
}

double Optimize(int n, int f, double *data, double *phase)
{
	int icount, numres, ifault;
	double initial_xd[3] = {0.00001, 1, 0.000001};
	double optim_krit;
	double optim_x[3];
	double optim_x_final[3];

	double step[3] = {0.000001, 0.1, 0.0000001};

	measurements_count = n;
	double minvalue=0;
	double maxvalue=0;
	double sum=0;
	for (int i = 0; i < n; i++)
	{
//		std::cout<<"data "<<data[i]<<std::endl;
		measurements[i] = data[i];
		if (data[i] < minvalue || i==0)
		{
			minvalue = data[i];
		}
		if (data[i] > maxvalue || i==0)
		{
			maxvalue = data[i];
		}
		sum = sum + data[i];
	}
	double mean = sum / n;
	double min=1000000000000;
	std::cout<<std::endl;
//	std::cout<<"mean "<<mean<<std::endl;
	maxa = (maxvalue - minvalue) / 2 * 1.2 * 1000000;
	mina = (maxvalue - minvalue) / 2 * 0.8 * 1000000;
// 	for (int k=0 ;k<10; k++)
//	{

		nelmin(Objective, 3, initial_xd, optim_x, &optim_krit, 1.0e-8, step, 10, 500, &icount, &numres, &ifault );
		initial_xd[0]=(maxvalue-minvalue)/2;
		initial_xd[1]=0.5;//*k;
		initial_xd[2]=mean;
//		std::cout<< "k "<<optim_krit<<" ";
		if (optim_krit<min)
		{
			min=optim_krit;
			optim_x_final[0]=optim_x[0];
			optim_x_final[1]=optim_x[1];
			optim_x_final[2]=optim_x[2];
		}
	//}
/*
	std::cout<<"start"<<std::endl;
 	for (double fi=0;fi<6.28;fi=fi+0.01)
 	{
 		for (double amp=(maxvalue - minvalue) / 2 * 0.8 ; amp < (maxvalue - minvalue) / 2 * 1.2 ; amp= amp + (maxvalue - minvalue) / 2 * 0.01)
 		{
// 			std::cout<<fi <<" "<<amp<<std::endl;
 			initial_xd[0]=amp;
 			initial_xd[1]=fi;
 			initial_xd[2]=mean;
 			optim_krit=Objective(initial_xd);
 			if (optim_krit<min)
 			{
 				min=optim_krit;
 				optim_x_final[0]=amp;
 				optim_x_final[1]=fi;
 				optim_x_final[2]=mean;
 			}
 		}
 	}
		*/
// 	std::cout<<"magnetometer_sampling_time "<<magnetometer_sampling_time<<std::endl;


		//std::cout<< "min"<<min<<std::endl;

 	//std::cout<<std::endl;
//	std::cout<< min<< "    count"<<measurements_count<<" optim "<<optim_x_final[0]*1000000<<" minmax "<<(maxvalue-minvalue)/2*1000000<<std::endl;
	*phase = optim_x_final[1];
	return optim_x_final[0];
}


void ispisi_vektor(geometry_msgs::Vector3 vektor, std::string str)
{
	std::cout<<str<<" "<<vektor.x<<" "<<vektor.y<< " "<<vektor.z<<std::endl;
}


geometry_msgs::Vector3 GetVectorUsingOptimization(int n, int f, double *suma1, double *suma2, double *suma3)
{
	geometry_msgs::Vector3 result;
	double x[MAX_DATA];
	double y[MAX_DATA];
	double z[MAX_DATA];
	double time[MAX_DATA];
	int countx=0, county = 0, countz=0 ;
	int data_count_current = data_count;
	double poc=0,kr=0;
//	int current_element_old;

	for (int i=0;i<n;i++)
	{
		int current_element = data_count_current + i - n;
		if (current_element<0) current_element = current_element + MAX_DATA;

		x[countx] = data[current_element].magnetic_field.x;
		y[county] = data[current_element].magnetic_field.y;
		z[countz] = data[current_element].magnetic_field.z;


//		std::cout<<data[current_element].magnetic_field.x<<" "<<data[current_element].magnetic_field.y<<" "<<data[current_element].magnetic_field.z<<" "<<std::endl;;
//		current_element_old=current_element;


		if (i>0)
		{
			if (x[countx] != x[countx-1]) countx++;
			if (y[county] != y[county-1]) county++;
			if (z[countz] != z[countz-1]) countz++;
		}
		else
		{
			countx++;
			county++;
			countz++;
		}

	}
	double anglex,angley,anglez;
	result.x = Optimize(countx, f, x, &anglex);
	result.y = Optimize(county, f, y, &angley);
	result.z = Optimize(countz, f, z, &anglez);

	if (result.x < 0){ anglex = anglex + 3.14159; result.x = fabs(result.x);}
	if (result.y < 0){ angley = angley + 3.14159; result.y = fabs(result.y);}
	if (result.z < 0){ anglez = anglez + 3.14159; result.z = fabs(result.z);}
	std::cout<<" angle init "<<anglex<< " "<< angley<<" "<<anglez<<std::endl;

	anglex = anglex - ((int)(anglex / (2 * 3.14159)))*2*3.14159;
	angley = angley - ((int)(angley / (2 * 3.14159)))*2*3.14159;
	anglez = anglez - ((int)(anglez / (2 * 3.14159)))*2*3.14159;

	double maxx = result.x, angle_max = anglex;
	if (result.y > maxx) { maxx = result.y; angle_max=angley;}
	if (result.z > maxx) { maxx = result.z; angle_max=anglez;}

	if (fabs(anglex - angle_max) > 3.14159/2 && fabs(fabs(anglex - angle_max) - 3.14159 * 2) > 3.14159/2)
	{
		result.x = -result.x;
	}
	if (fabs(angley - angle_max) > 3.14159/2 && fabs(fabs(angley - angle_max) - 3.14159 * 2) > 3.14159/2)
	{
		result.y = -result.y;
	}
	if (fabs(anglez - angle_max) > 3.14159/2 && fabs(fabs(anglez - angle_max) - 3.14159 * 2) > 3.14159/2)
	{
		result.z = -result.z;
	}
	ispisi_vektor(result," vektor ");
	std::cout<<" angle "<<anglex<< " "<< angley<<" "<<anglez<<std::endl;
	GetSumOfDifferences(x, y, n,suma1,suma2);
	GetSumOfDifferences(x, z, n,suma1,suma3);


	return result;
}



double Median (double v1, double v2, double v3)
{
	if (v1 >= v2 && v1 <= v3) return v1;
	if (v1 <= v2 && v1 >= v3) return v1;
	if (v2 >= v1 && v2 <= v3) return v2;
	if (v2 <= v1 && v2 >= v3) return v2;
	return v3;
}
geometry_msgs::Vector3 Median(geometry_msgs::Vector3 v1,
		geometry_msgs::Vector3 v2,
		geometry_msgs::Vector3 v3
		)
{
	geometry_msgs::Vector3 rez;
	rez.x= Median(v1.x, v2.x, v3.x);
	rez.y= Median(v1.y, v2.y, v3.y);
	rez.z= Median(v1.z, v2.z, v3.z);
	return rez;

}

int main (int argc, char** argv){
    ros::init(argc, argv, "magnetic_field_vector_fast");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    int magnetic_field_frequency, cycles_for_analysis;
    std::string magnetometer_topic;
    std::string magnetic_vector_topic;
    std::string magnetometer_frame="/magnetometer1";

    int number_of_points_for_analysis;
    nh_ns.param("magnetometer_topic", magnetometer_topic, (std::string) "/imu_magnetic0");
    nh_ns.param("vector_topic", magnetic_vector_topic, (std::string) "/magnetic_vector0");
    nh_ns.param("magnetometer_frame", magnetometer_frame, (std::string) "/magnetometer0");
    nh_ns.param("magnetic_field_frequency", magnetic_field_frequency, 50);
    nh_ns.param("number_of_points_for_analysis", number_of_points_for_analysis, 200);
    nh_ns.param("number_of_cycles_for_analysis", cycles_for_analysis, 5);
    nh_ns.param("magnetometer_sampling_time", magnetometer_sampling_time,0.01282);


    ros::Subscriber magnetometer_subscriber = nh.subscribe(magnetometer_topic, 1000, magnetometer_callback);

    ros::Publisher magnetic_vector_publisher = nh.advertise<geometry_msgs::Vector3>(magnetic_vector_topic, 1000);

    ros::Rate loop_rate(1000);

    geometry_msgs::Vector3 vector;
    geometry_msgs::Vector3 vector_final;
    geometry_msgs::Vector3 vector_final_old;
    geometry_msgs::Vector3 vector_final_old_old;

    
    int counter=0;
    int cycle_count = 0;
    double x_size = 0;
    double y_size = 0;
    double z_size = 0;
    double vector_size = 0;
    double suma1 = 0;
    double suma2 = 0;
    double suma3 = 0;
    vector_final.x = 0;
    vector_final.y = 0;
    vector_final.z = 0;
    vector_final_old.x = 0;
    vector_final_old.y = 0;
    vector_final_old.z = 0;
    vector_final_old_old.x = 0;
    vector_final_old_old.y = 0;
    vector_final_old_old.z = 0;


    tf::Transform transform1;
    tf::TransformBroadcaster br;



    while(ros::ok()){

        ros::spinOnce();
        suma1=0;
        suma2=0;
        suma3=0;
        if (first == false && received > number_of_points_for_analysis)
        {
        	std::cout<<"received "<<received<<std::endl;
        	double pom1=0,pom2=0,pom3=0;
//    		vector=GetVectorUsingRMS(number_of_points_for_analysis, magnetic_field_frequency,&pom1,&pom2,&pom3);
//    		std::cout<<magnetometer_topic<<" ";

        	vector=GetVectorUsingOptimization(number_of_points_for_analysis/*received*/, magnetic_field_frequency,&pom1,&pom2,&pom3);
        	received=0;

        	//    		std::cout<<std::endl;
    		suma1=suma1+pom1;
    		suma2=suma2+pom2;
    		suma3=suma3+pom3;
    		if (vector_final.x != vector_final.x) vector_final.x = 0;
    		if (vector_final.y != vector_final.y) vector_final.y = 0;
    		if (vector_final.z != vector_final.z) vector_final.z = 0;
    		geometry_msgs::Vector3 pomocni_vektor = vector;

//        	ispisi_vektor(vector_final_old_old,"i-2 ");
//        	ispisi_vektor(vector_final_old,"i-1 ");
//        	ispisi_vektor(vector,"i ");


    		vector= Median(vector, vector_final_old, vector_final_old_old);



//        	ispisi_vektor(vector_final,"final ");

    		vector_final.x = /*vector_final.x / 1000000* 0.9 + */vector.x;// * 0.1;
          	vector_final.y = /*vector_final.y / 1000000 * 0.9 +*/ vector.y;// * 0.1;
        	vector_final.z = /*vector_final.z / 1000000 * 0.9 + */vector.z;// * 0.1;
        	vector_final.x *= 1000000;
        	vector_final.y *= 1000000;
        	vector_final.z *= 1000000;

    		vector_final_old_old = vector_final_old;
    		vector_final_old = pomocni_vektor;


        	/*if (suma2>suma1)
        	{
        		vector_final.y=-vector_final.y;
        	}
        	if (suma3>suma1)
        	{
        		vector_final.z=-vector_final.z;
        	}*/
//        	ROS_INFO_STREAM("publishing " << magnetometer_frame << " "<< magnetometer_topic << " vector"<<vector_final);
//        		ROS_INFO_STREAM("direction sum1 sum2 sum3 "<<suma1<<" "<<suma2<<" "<< suma3);


            geometry_msgs::Vector3 pomocni;
            pomocni.x=1;
            pomocni.y=0;
            pomocni.z=0;
            geometry_msgs::Vector3 kros=normalize_vector(CrossProduct(pomocni,normalize_vector(vector_final)));

            double w=1+DotProduct(pomocni,normalize_vector(vector_final));
//            std::cout<<kros.x<<" "<< kros.y<<" "<< kros.z<<" "<< w<<std::endl;
            transform1.setOrigin(tf::Vector3(0,0,0));
            transform1.setRotation(tf::Quaternion(kros.x, kros.y, kros.z, w));
            br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), magnetometer_frame, magnetic_vector_topic));



        	magnetic_vector_publisher.publish(vector_final);
            x_size=0;
            y_size=0;
            z_size=0;
            suma1=0;
            suma2=0;
            suma3=0;
            vector_size=0;
        	cycle_count = 0;

        }
        loop_rate.sleep();


    }
}


