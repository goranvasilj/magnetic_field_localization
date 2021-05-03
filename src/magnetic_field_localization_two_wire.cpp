
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <mrpt/math/CLevenbergMarquardt.h>

#include "asa047.hpp"


using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

geometry_msgs::Vector3 received_vector0;
geometry_msgs::Vector3 received_vector1;
geometry_msgs::Vector3 received_vector2;

void magnetic_vector0_callback(const geometry_msgs::Vector3::ConstPtr& msg){

	received_vector0 = *msg;
	received_vector0.x=-received_vector0.x;
	received_vector0.y=-received_vector0.y;
	received_vector0.z=-received_vector0.z;

}

void magnetic_vector1_callback(const geometry_msgs::Vector3::ConstPtr& msg){
	received_vector1 = *msg;
	received_vector1.x=-received_vector1.x;
	received_vector1.y=-received_vector1.y;
	received_vector1.z=-received_vector1.z;

}

void magnetic_vector2_callback(const geometry_msgs::Vector3::ConstPtr& msg){
	received_vector2 = *msg;
	received_vector2.x=-received_vector2.x;
	received_vector2.y=-received_vector2.y;
	received_vector2.z=-received_vector2.z;

}

inline geometry_msgs::Vector3 CrossProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B) {
	geometry_msgs::Vector3 c_P;
   c_P.x = v_A.y * v_B.z - v_A.z * v_B.y;
   c_P.y = -(v_A.x * v_B.z - v_A.z * v_B.x);
   c_P.z = v_A.x * v_B.y - v_A.y * v_B.x;
   return c_P;
}

inline geometry_msgs::Vector3 sum_vector(geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2) {
	geometry_msgs::Vector3 v3;
	v3.x = v1.x + v2.x;
	v3.y = v1.y + v2.y;
	v3.z = v1.z + v2.z;
   return v3;
}

inline geometry_msgs::Vector3 normalize_vector(geometry_msgs::Vector3 v)
{
	double dist=sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	v.x = v.x / dist;
	v.y = v.y / dist;
	v.z = v.z / dist;
	return v;
}

inline double VectorSize(geometry_msgs::Vector3 vector)
{
	return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

inline double DotProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B)
{
	return v_A.x * v_B.x + v_A.y * v_B.y + v_A.z * v_B.z;
}

inline geometry_msgs::Vector3 transform_vector(geometry_msgs::Vector3 vector, tf::Matrix3x3 rotation)
{
	geometry_msgs::Vector3 result;

	result.x = rotation.getRow(0).getX() * vector.x +  rotation.getRow(0).getY() * vector.y + rotation.getRow(0).getZ() * vector.z;
	result.y = rotation.getRow(1).getX() * vector.x +  rotation.getRow(1).getY() * vector.y + rotation.getRow(1).getZ() * vector.z;
	result.z = rotation.getRow(2).getX() * vector.x +  rotation.getRow(2).getY() * vector.y + rotation.getRow(2).getZ() * vector.z;
	return result;
}

void ispisi_vektor(geometry_msgs::Vector3 vektor, std::string str)
{
	cout<<str<<" "<<vektor.x<<" "<<vektor.y<< " "<<vektor.z<<endl;
}

bool ispis=false;

inline geometry_msgs::Vector3 getClosestPointOnLine(geometry_msgs::Vector3 line_point, geometry_msgs::Vector3 line_vector)
{
	double x1 = line_point.x;
	double y1 = line_point.y;
	double z1 = line_point.z;
	double vx = line_vector.x;
	double vy = line_vector.y;
	double vz = line_vector.z;
	geometry_msgs::Vector3 p1, vector, result;
	p1.x = x1;
	p1.y = y1;
	p1.z = z1;
	vector.x = vx;
	vector.y = vy;
	vector.z = vz;
	if (ispis) ispisi_vektor(line_point, " closest point line point ");
	if (ispis) ispisi_vektor(line_vector, " closest point line vektor ");

	double t=-DotProduct(p1, vector) / VectorSize(vector)/*/VectorSize(p1)*/;
	result.x = x1 + t * vx;
	result.y = y1 + t * vy;
	result.z = z1 + t * vz;
	if (ispis) ispisi_vektor(result, " closest point result ");

	return result;
}

void ispisi_vektor(tf::Vector3 vektor, std::string str)
{
	cout<<str<<" "<<vektor.getX()<<" "<<vektor.getY()<< " "<<vektor.getZ()<<endl;
}

inline geometry_msgs::Vector3 GetFieldVectorOneWire(geometry_msgs::Vector3 line_point, geometry_msgs::Vector3 line_vector, double current, tf::StampedTransform magnetometer_location)
{
	line_point.x = line_point.x - magnetometer_location.getOrigin().getX();
	line_point.y = line_point.y - magnetometer_location.getOrigin().getY();
	line_point.z = line_point.z - magnetometer_location.getOrigin().getZ();

	geometry_msgs::Vector3 closest_point = getClosestPointOnLine(line_point, line_vector);

    if (ispis) ispisi_vektor(line_point,"line point");
    if (ispis) ispisi_vektor(closest_point,"closest point");
    if (ispis)    cout<<" vector size "<<VectorSize(closest_point)<<endl;

    double distance = VectorSize(closest_point) ;

	geometry_msgs::Vector3 magnetic_vector = CrossProduct(closest_point, line_vector);
	magnetic_vector=normalize_vector(magnetic_vector);

	magnetic_vector.x = magnetic_vector.x / distance * current * 2;
	magnetic_vector.y = magnetic_vector.y / distance * current * 2;
	magnetic_vector.z = magnetic_vector.z / distance * current * 2;

	return magnetic_vector;
}
inline bool GetPowerLineLocation(geometry_msgs::Vector3 m_vector1,
					geometry_msgs::Vector3 m_vector2,
					tf::StampedTransform transform,
					geometry_msgs::Vector3 *power_line_vector,
					geometry_msgs::Vector3 *power_line_point,
					tf::StampedTransform calib1,
					tf::StampedTransform calib2)
{
	//adjust vectors according to calibration
	tf::Matrix3x3 basis1 = calib1.getBasis();
	tf::Matrix3x3 basis2 = calib2.getBasis();

	m_vector1 = transform_vector(m_vector1, basis1);
	m_vector2 = transform_vector(m_vector2, basis2);

	//to do rotacija m_vector2 to m_vector1 frame  - za paralelne senzore nije potrebna
	*power_line_vector=CrossProduct(m_vector1, m_vector2);

	//line between sensors does not point towards power line
	geometry_msgs::Vector3 towards_center1=CrossProduct(m_vector1, *power_line_vector);
	geometry_msgs::Vector3 towards_center2=CrossProduct(m_vector2, *power_line_vector);

	geometry_msgs::Vector3 a0;
	a0.x = 0; a0.y = 0; a0.z = 0;
	geometry_msgs::Vector3 b0;

	b0.x = (double) transform.getOrigin().x();
	b0.y = (double) transform.getOrigin().y();
	b0.z = (double) transform.getOrigin().z();

	geometry_msgs::Vector3 a = normalize_vector(towards_center1);
	geometry_msgs::Vector3 b = normalize_vector(towards_center2);
	geometry_msgs::Vector3 cn = normalize_vector(CrossProduct(b, a));

	cn=CrossProduct(b, a);
	geometry_msgs::Vector3 n2 = CrossProduct(b, cn);
	double dot1 = DotProduct(b0, n2)/DotProduct(a,n2);
	geometry_msgs::Vector3 a2 = a;
	a2.x *= dot1;
	a2.y *= dot1;
	a2.z *= dot1;
	geometry_msgs::Vector3 closest_approach = a2;

	*power_line_point = closest_approach;
	*power_line_vector = normalize_vector(*power_line_vector);

	return true;
}


inline geometry_msgs::Vector3 operator-(geometry_msgs::Vector3 o1, tf::Vector3 o2)
{
	geometry_msgs::Vector3 rez;
	rez.x=o1.x-o2.getX();
	rez.y=o1.y-o2.getY();
	rez.z=o1.z-o2.getZ();
	return rez;
}


inline geometry_msgs::Vector3 operator/(geometry_msgs::Vector3 o1,  double o2)
{
	geometry_msgs::Vector3 rez;
	rez.x=o1.x / o2;
	rez.y=o1.y / o2;
	rez.z=o1.z / o2;
	return rez;
}


inline geometry_msgs::Vector3 operator+(geometry_msgs::Vector3 o1, tf::Vector3 o2)
{
	geometry_msgs::Vector3 rez;
	rez.x=o1.x+o2.getX();
	rez.y=o1.y+o2.getY();
	rez.z=o1.z+o2.getZ();
	return rez;
}


inline geometry_msgs::Vector3 operator-(geometry_msgs::Vector3 o1, geometry_msgs::Vector3 o2)
{
	geometry_msgs::Vector3 rez;
	rez.x=o1.x-o2.x;
	rez.y=o1.y-o2.y;
	rez.z=o1.z-o2.z;
	return rez;
}


inline geometry_msgs::Vector3 operator+(geometry_msgs::Vector3 o1, geometry_msgs::Vector3 o2)
{
	geometry_msgs::Vector3 rez;
	rez.x=o1.x+o2.x;
	rez.y=o1.y+o2.y;
	rez.z=o1.z+o2.z;
	return rez;
}


tf::StampedTransform m1_loc;
tf::StampedTransform m2_loc;
geometry_msgs::Vector3 m_0;
geometry_msgs::Vector3 m_1;
geometry_msgs::Vector3 m_2;
geometry_msgs::Vector3 line_vector;

geometry_msgs::Vector3 output_pose;




int objective_count=0;

//objective function for optimization
//receives one point on the conductor and current going through the conductor
//assumes that two conductors have the same current and are on known distance from each other
void Objective(
	const CVectorDouble& x, const CVectorDouble& y, CVectorDouble& out_f)
{
	out_f.resize(1);
	geometry_msgs::Vector3 point;
	point.x = x[0];
	point.y = x[1];
	point.z = x[2];


	double i1 = x[3];
	objective_count++;
	tf::StampedTransform m0_loc;
	tf::StampedTransform m12_loc;
	tf::StampedTransform calib0;

	//get transformation from magnetometer 1 to magnetometer 2
	tf::Transform t1= m1_loc.inverse() * m2_loc;
	m12_loc.setBasis(m2_loc.getBasis());
	m12_loc.setOrigin(m2_loc.getOrigin()-m1_loc.getOrigin());

	//calib0 is used for pose calibration, currently identity matrix
	calib0.setOrigin(tf::Vector3(0,0,0));
	calib0.setRotation(tf::Quaternion(0, 0, 0, 1));
	m0_loc.setOrigin(tf::Vector3(0,0,0));

	if (ispis)	ispisi_vektor(point, "point ");
	if (ispis)	ispisi_vektor(line_vector, "line_vector ");
	if (ispis)	ispisi_vektor(m0_loc.getOrigin(), "m0_loc ");
	if (ispis)	ispisi_vektor(m1_loc.getOrigin(), "m1_loc ");
	if (ispis)	ispisi_vektor(m2_loc.getOrigin(), "m2_loc ");

	//calculate magnetic field  of magnetometers based only on one conductor
	geometry_msgs::Vector3 m00 = GetFieldVectorOneWire(point, line_vector, i1, m0_loc);
	geometry_msgs::Vector3 m01 = GetFieldVectorOneWire(point, line_vector, i1, m1_loc);
	geometry_msgs::Vector3 m02 = GetFieldVectorOneWire(point, line_vector, i1, m2_loc);
	geometry_msgs::Vector3 m10, m11, m12;

	//get magnetic field  of magnetometers of the second conductor
	m10 = m_0 - m00;
	m11 = m_1 - m01;
	m12 = m_2 - m02;
	if (ispis)	ispisi_vektor(m_0, "m_0 ");
	if (ispis)	ispisi_vektor(m_1, "m_1 ");
	if (ispis)	ispisi_vektor(m_2, "m_2 ");

	if (ispis)	ispisi_vektor(m00, "m00 ");
	if (ispis)	ispisi_vektor(m01, "m01 ");
	if (ispis)	ispisi_vektor(m02, "m02 ");
	if (ispis)	ispisi_vektor(m10, "m10 ");
	if (ispis)	ispisi_vektor(m11, "m11 ");
	if (ispis)	ispisi_vektor(m12, "m12 ");

	geometry_msgs::Vector3 plv0, plv1, plv2,plp0, plp1, plp2;

	//calculate locations of second conductor based on previously calculated magnetometers readings
	//3 locations: plp - power line point and plv - power line vector
	GetPowerLineLocation(m10, m11, m1_loc, &plv0, &plp0, calib0, calib0); //magnetomer1 and magnetometer2
	GetPowerLineLocation(m10, m12, m2_loc, &plv1, &plp1, calib0, calib0); //magnetometer1 and magnetometer3
	GetPowerLineLocation(m11, m12, m12_loc, &plv2, &plp2, calib0, calib0); //magnetometer2 and magnetometer3

	plp2=plp2+m1_loc.getOrigin();

	//distance from magnetometer to line multiplied with magnetic field strength
	// mij = Bij - magnetic field caused by conductor i on location of magnetometer j
	// |m10| * |d10| =
	double pp1 = VectorSize(plp0 - m0_loc.getOrigin()) * VectorSize(m10); //
	double pp2 = VectorSize(plp0 - m1_loc.getOrigin()) * VectorSize(m11);
	double pp3 = VectorSize(plp1 - m0_loc.getOrigin()) * VectorSize(m10);
	double pp4 = VectorSize(plp1 - m2_loc.getOrigin()) * VectorSize(m12);
	double pp5 = VectorSize(plp2 - m1_loc.getOrigin()) * VectorSize(m11);
	double pp6 = VectorSize(plp2 - m2_loc.getOrigin()) * VectorSize(m12);

	if (ispis) ispisi_vektor(plp0,"plp0 ");
	if (ispis) ispisi_vektor(plp1,"plp1 ");
	if (ispis) ispisi_vektor(plp2,"plp2 ");

	//distance from magnetometer to line
	double r1 = VectorSize(plp0 - m0_loc.getOrigin());
	double r2 = VectorSize(plp1 - m1_loc.getOrigin());
	double r3 = VectorSize(plp2 - m2_loc.getOrigin());


	if (ispis) cout<<"i1/r1 size(m10)"<<i1/r1<<" "<<VectorSize(m10)<<endl;
	if (ispis) cout<<"i1/r2 size(m11)"<<i1/r2<<" "<<VectorSize(m11)<<endl;
	if (ispis) cout<<"i1/r3 size(m12)"<<i1/r3<<" "<<VectorSize(m12)<<endl;


	//B1 = i1 / d1 * C
	//krit_1 = B1_dist - B1_measured
	double krit1 = i1 / r1 - VectorSize(m10);
	double krit2 = i1 / r2 - VectorSize(m11);
	double krit3 = i1 / r3 - VectorSize(m12);

	double yrez=0;
	if (ispis) cout<<"krit1 ktir2 krit3 "<<krit1<<" "<<krit2<<" "<< krit3<<endl;
	if (ispis) cout<<"pp1 pp2 pp3 pp4 pp5 pp6 "<<pp1<<" "<<pp2<<" "<<pp3<<" "<<pp4<<" "<<pp5<<" "<<pp6<<endl;

	//criteria 1 = krit_1 + krit_2 + krit_3
	yrez=(fabs(krit1)+fabs(krit2)+fabs(krit3))*10;
	if (ispis)	cout<<"krit1 "<<yrez<<endl;

	//criteria 2 = (B1 * d1 - B2 * d2) + (
	yrez=yrez+(fabs(pp1-pp2)+fabs(pp3-pp4)+fabs(pp5-pp6))  * 10;
	if (ispis)	cout<<"krit2 "<<yrez<<endl;

	yrez=yrez+VectorSize(plp0-plp1);
	yrez=yrez+VectorSize(plp0-plp2)+VectorSize(plp1-plp2);
	if (ispis)	cout<<"krit3 "<<yrez<<endl;

	double space_between_wires=0.4;
	yrez=yrez+fabs(VectorSize(plp0-point)-space_between_wires)  *10000;
	if (ispis) ispisi_vektor(plp0-point, "poz diff ");
	if (ispis)	cout<<"krit4 "<<yrez<<endl;
	yrez=yrez + (VectorSize(plp0-plp1) + VectorSize(plp0-plp2) + VectorSize(plp1-plp2))*1000;
	if (ispis)	cout<<"krit5 "<<yrez<<endl;
	output_pose=plp0;
	out_f[0] = yrez;
	if (ispis)	cout<<"r1 "<< r1 <<" r2 "<< r2<<" r3 "<< r3 << "       m10 "<<VectorSize(m10)<< "  m11 "<<VectorSize(m11)<<" m12 "<<VectorSize(m12)<<endl;
	if (ispis)	cout<<"plp0 "<<plp0.x<<" "<<plp0.y<<" "<<plp0.z<<" "<<endl;

	if (ispis)	cout<<"x "<<point.x<<" "<<point.y<<" "<<point.z<<" "<<i1<< "      "<<yrez<<endl;

}

double Objective1(double x[4])
{

	CVectorDouble initial_x,y,f;
	initial_x.resize(4);
	initial_x[0] = x[0];  // x
	initial_x[1] = x[1];  // y
	initial_x[2] = x[2];  // y
	initial_x[3] = x[3];  // y
	Objective(initial_x,y,f);
	return f[0];


}

double levmarq_final_error = 1e10;

// The error function F(x):
void myFunction(
	const CVectorDouble& x, const CVectorDouble& y, CVectorDouble& out_f)
{
	out_f.resize(1);
	// 1-cos(x+1) *cos(x*y+1)
	out_f[0] = 1 - cos(x[0] + 1) * cos(x[0] * x[1] + 1);
}

void TestLevMarq()
{

}

void FindBestLocation(double *final,double *crit)
{
	double optim_x[4];
	double step[4]={0.01,0.01,0.01,1};


	double initial_xd[4];
	double optimal_x[4];

	double xi, yi;
	double min=10000000000000;
	double y, optim_krit;
	int count_less_than_1000=0;
	for (double xi = -1; xi < 1 ; xi = xi + 0.05)
	{
//		std::cout<<xi<<std::endl;
		for (double yi = -1; yi < 1 ; yi = yi + 0.05)
		{
			for (double ii=2;ii<50;ii+=1)
			{

				yi=-0.6;
				xi=-0.25;
				initial_xd[0] = 0;  // x
				initial_xd[1] = yi;  // y
				initial_xd[2] = xi;//zi;  // y
				initial_xd[3] = ii;  // y



				int icount, numres, ifault;
				optim_krit = Objective1(initial_xd);

				optimal_x[0]=initial_xd[0];
				optimal_x[1]=initial_xd[1];
				optimal_x[2]=initial_xd[2];
				optimal_x[3]=initial_xd[3];
				levmarq_final_error=optim_krit;
/*				if (optim_krit<1000)
				{*/
					nelmin(Objective1, 4, initial_xd, optim_x, &optim_krit, 1.0e-8, step, 10, 300, &icount, &numres, &ifault );
					optimal_x[0]=optim_x[0];
					optimal_x[1]=optim_x[1];
					optimal_x[2]=optim_x[2];
					optimal_x[3]=optim_x[3];
					levmarq_final_error=optim_krit;
/*					double diffx=0.05;
					double diffy=0.05;
					double diffi=1;
					for (double xi2 = -diffx; xi2 < diffx ; xi2 = xi2 + 2 * diffx / 10)
					{
						for (double yi2 = -diffy; yi2 < diffy ; yi2 = yi2 +  2 * diffy / 10)
						{
							for (double ii2=-diffi; ii2< diffi; ii2 += 2 * diffi / 5)
							{
								initial_xd[0] = 0;  // x
								initial_xd[1] = yi + yi2;  // y
								initial_xd[2] = xi + xi2;//zi;  // y
								initial_xd[3] = ii + ii2;  // y
								optim_krit = Objective1(initial_xd);
								if (optim_krit < levmarq_final_error)
								{
									levmarq_final_error = optim_krit;
									optimal_x[0]=initial_xd[0];
									optimal_x[1]=initial_xd[1];
									optimal_x[2]=initial_xd[2];
									optimal_x[3]=initial_xd[3];
								}
							}
						}
					}*/
					count_less_than_1000++;
				//}
				if (levmarq_final_error<min)

				{

					y = Objective1(optimal_x);
					ispis=false;
//									cout<<"error "<<levmarq_final_error<<" xi yi"<<xi<<" "<<yi<<endl;
					//				cout<< " error 2 " << y[0]<<endl;
					min=levmarq_final_error;
					final[0] = optimal_x[0];
					final[1] = optimal_x[1];
					final[2] = optimal_x[2];
					final[3] = optimal_x[3];

				}
			}
			break;

		}
		break;

	}
	std::cout<<"less than 1000 count"<<count_less_than_1000<<std::endl<<std::endl;
	*crit=min;
}


bool GetPowerLinesLocation(geometry_msgs::Vector3 m_vector0,
					geometry_msgs::Vector3 m_vector1,
					geometry_msgs::Vector3 m_vector2,
					tf::StampedTransform transform0,
					tf::StampedTransform transform1,
					geometry_msgs::Vector3 *power_line_vector,
					geometry_msgs::Vector3 *power_line_point0,
					geometry_msgs::Vector3 *power_line_point1,
					tf::StampedTransform calib0,
					tf::StampedTransform calib1,
					tf::StampedTransform calib2)
{
	tf::Matrix3x3 basis0 = calib0.getBasis();
	tf::Matrix3x3 basis1 = calib1.getBasis();
	tf::Matrix3x3 basis2 = calib2.getBasis();
	m1_loc=transform0;
	m2_loc=transform1;
	m_0 = transform_vector(m_vector0, basis0);
	m_1 = transform_vector(m_vector1, basis1);
	m_2 = transform_vector(m_vector2, basis2);

//	ROS_INFO_STREAM("vector 1 tran " << m_vector0.x << " " << m_vector0.y << " " << m_vector0.z<< " k");
//	ROS_INFO_STREAM("vector 2 tran " << m_vector1.x << " " << m_vector1.y << " " << m_vector1.z<< " k");
//	ROS_INFO_STREAM("vector 3 tran " << m_vector2.x << " " << m_vector2.y << " " << m_vector2.z<< " k");

	geometry_msgs::Vector3 pom_vector1 = CrossProduct(m_0, m_1);
	geometry_msgs::Vector3 pom_vector2 = CrossProduct(m_0, m_2);
	geometry_msgs::Vector3 pom_vector3 = CrossProduct(m_1, m_2);

	power_line_vector->x = (pom_vector1.x + pom_vector2.x + pom_vector3.x) / 3;
	power_line_vector->y = (pom_vector1.y + pom_vector2.y + pom_vector3.y) / 3;
	power_line_vector->z = (pom_vector1.z + pom_vector2.z + pom_vector3.z) / 3;

	line_vector = *power_line_vector;

	//ROS_INFO_STREAM("power_line " << (*power_line_vector).x << " " << (*power_line_vector).y << " " << (*power_line_vector).z);

/*	CLevenbergMarquardt lm;
	CLevenbergMarquardt::TResultInfo info;
	CVectorDouble optimal_x;
	CVectorDouble initial_x;
	CVectorDouble y;
	initial_x.resize(4);
	initial_x[0] = 1;
	initial_x[1] = 1;
	initial_x[2] = 1;
	initial_x[3] = 10;

	CVectorDouble increments_x(4);
	increments_x.fill(0.0001);*/
//	m_0 = m_0 / 1000000;
//	m_1 = m_1 / 1000000;
//	m_2 = m_2 / 1000000;


	line_vector=normalize_vector(line_vector);
	ispisi_vektor(m1_loc.getOrigin(),"m1_loc ");
	ispisi_vektor(m2_loc.getOrigin(),"m2_loc ");

	ispisi_vektor(m_0,"m_0 ");
	ispisi_vektor(m_1,"m_1 ");
	ispisi_vektor(m_2,"m_2 ");
	ispisi_vektor(line_vector,"line_vector ");
//return true;

	CVectorDouble optimal_x;
	CVectorDouble initial_x;
	CVectorDouble y;

	CLevenbergMarquardt::TResultInfo info;

	initial_x.resize(4);
	optimal_x.resize(4);
	initial_x[0] = 1.4;  // x
	initial_x[1] = 2.5;  // y
	initial_x[2] = 2.5;  // y
	initial_x[3] = 2.5;  // y


	CVectorDouble increments_x(4);
	increments_x.fill(0.001);
	increments_x[3]=0.1;

	CLevenbergMarquardt lm;
	double min=100000000;
	double initial_xd[4];
	double optim_x[4];
	double step[4]={0.01,0.01,0.01,1};
	double optim_krit=0;
	double final[4]={0,0,0,0};
	double final1[4]={0,0,0,0};

	initial_xd[0]=0.472;
	initial_xd[1]=0.067;
	initial_xd[2]=-0.6212;
	initial_xd[3]=17.5;
//	ispis=true;
//	cout<<"objective real initial "<<Objective1(initial_xd)<<endl<<endl;
	ispis=false;
	initial_xd[0]=0;
	initial_xd[1]=-0.712;
	initial_xd[2]=-0.25;
	initial_xd[3]=17.5;
//	ispis=true;
	cout<<"objective real initial "<<Objective1(initial_xd)<<endl<<endl;
	ispis=false;

/*
//	return false;
	for (double xi = -1; xi < 1 ; xi = xi + 0.1)
	{
		for (double yi = -1; yi < 1 ; yi = yi + 0.1)
		{
			for (double zi = -1; zi < 1 ; zi = zi + 0.1)
			{

			initial_x[0] = xi;  // x
			initial_x[1] = yi;  // y
			initial_x[2] = 0;  // y
			initial_x[3] = 5;  // y

			initial_xd[0] = xi;  // x
			initial_xd[1] = yi;  // y
			initial_xd[2] = 0;//zi;  // y
			initial_xd[3] = 5;  // y



			for (int k = 0; k < 1; k++)
			{
//				cout<<endl<<endl<<endl<<endl<<endl;

//				lm.execute(optimal_x, initial_x, Objective, increments_x, y, info);
//				initial_x=optimal_x;
			}
			int icount, numres, ifault;
			nelmin(Objective1, 4, initial_xd, optim_x, &optim_krit, 1.0e-8, step, 10, 300, &icount, &numres, &ifault );
//			levmarq_final_error = std::sqrt(info.final_sqr_err);

			levmarq_final_error = optim_krit;
			optimal_x[0]=optim_x[0];
			optimal_x[1]=optim_x[1];
			optimal_x[2]=optim_x[2];
			optimal_x[3]=optim_x[3];
			initial_x=optimal_x;
//			cout<<"error "<<levmarq_final_error<<" xi yi"<<xi<<" "<<yi<<"   optimal "<<optimal_x[0]<<" "<<optimal_x[1]<<"  "<<optimal_x[2]<<endl;

			if (levmarq_final_error<min)

			{

//				std::cout<<optimal_x<<endl;
	//			cout<<initial_x[0]<<" "<<initial_x[1]<<" "<<initial_x[2]<<" "<<initial_x[3]<<endl;
//				ispis=true;
//				cout<<endl<<endl;
				Objective(initial_x,optimal_x,y);
				ispis=false;
//				cout<<"error "<<levmarq_final_error<<" xi yi"<<xi<<" "<<yi<<endl;
//				cout<< " error 2 " << y[0]<<endl;
				min=levmarq_final_error;
				final[0] = optim_x[0];
				final[1] = optim_x[1];
				final[2] = optim_x[2];
				final[3] = optim_x[3];
				final1[0] = output_pose.x;plp0  0.0417116 -0.235902 0.0273557
plp1  0.0418833 -0.249311 0.0289451

				final1[1] = output_pose.y;
				final1[2] = output_pose.z;
				final1[3] = optim_x[3];

			}
			}
//			break;

		}
	//	break;

	}
	*/
	FindBestLocation(final,&min);
	ispis=true;
	Objective1(final);
	ispis=false;
	cout<<"final "<<min<<"   x y z i "<<final[0]<<" "<<final[1]<<" "<<final[2]<<" "<<final[3]<<endl;
	power_line_point0->x = final[0];
	power_line_point0->y = final[1];
	power_line_point0->z = final[2];

	final1[0] = output_pose.x;
	final1[1] = output_pose.y;
	final1[2] = output_pose.z;
	power_line_point1->x = final1[0];
	power_line_point1->y = final1[1];
	power_line_point1->z = final1[2];



}

geometry_msgs::Vector3 getClosestPointOnLine(geometry_msgs::Vector3 line_point,
		geometry_msgs::Vector3 line_vector,
		geometry_msgs::Vector3 point)
{
	double x1 = line_point.x-point.x;
	double y1 = line_point.y-point.y;
	double z1 = line_point.z-point.z;
	double vx = line_vector.x;
	double vy = line_vector.x;
	double vz = line_vector.x;
	geometry_msgs::Vector3 p1, vector, result;
	p1.x = x1;
	p1.y = y1;
	p1.z = z1;
//	std::cout<<"transform base power_line "<<p1;
	vector.x = vx;
	vector.y = vy;
	vector.z = vz;
//	std::cout<<"line vector  "<<vector;
	double t=-DotProduct(p1,vector)/VectorSize(vector)/VectorSize(p1);
	std::cout<<"t "<<t<<std::endl;
	result.x = x1 + t * vx;
	result.y = y1 + t * vy;
	result.z = z1 + t * vz;
	return result;

}
void test_objective()

{
	tf::StampedTransform tr1;
	tr1.setOrigin(tf::Vector3(-0.2, -0.2, 0));
	tf::StampedTransform tr2;
	tr2.setOrigin(tf::Vector3(-0.2, 0, 0));
	tf::StampedTransform tr3;
	tr3.setOrigin(tf::Vector3(-0.2, 0.2, 0));
	geometry_msgs::Vector3 line1_point;
	geometry_msgs::Vector3 line2_point;
	geometry_msgs::Vector3 p_line_vector;
	line1_point.x = 0; line1_point.y = 0; line1_point.z = 0;
	line2_point.x = 0.4; line2_point.y = 0; line2_point.z = 0;
	p_line_vector.x = 0; p_line_vector.y = 0; p_line_vector.z = -1;

	geometry_msgs::Vector3 m11=GetFieldVectorOneWire(line1_point, p_line_vector, 10, tr1);
	geometry_msgs::Vector3 m21=GetFieldVectorOneWire(line1_point, p_line_vector, 10, tr2);
	geometry_msgs::Vector3 m31=GetFieldVectorOneWire(line1_point, p_line_vector, 10, tr3);

	geometry_msgs::Vector3 m12=GetFieldVectorOneWire(line2_point, p_line_vector, 10, tr1);
	geometry_msgs::Vector3 m22=GetFieldVectorOneWire(line2_point, p_line_vector, 10, tr2);
	geometry_msgs::Vector3 m32=GetFieldVectorOneWire(line2_point, p_line_vector, 10, tr3);
	std::cout<<"m11 "<< m11.x<< " "<<m11.y << " "<< m11.z<<endl;
	std::cout<<"m21 "<< m21.x<< " "<<m21.y << " "<< m21.z<<endl;
	std::cout<<"m31 "<< m31.x<< " "<<m31.y << " "<< m31.z<<endl;
	std::cout<<"m12 "<< m12.x<< " "<<m12.y << " "<< m12.z<<endl;
	std::cout<<"m22 "<< m22.x<< " "<<m22.y << " "<< m22.z<<endl;
	std::cout<<"m32 "<< m32.x<< " "<<m32.y << " "<< m32.z<<endl;


	geometry_msgs::Vector3 m1 = m11 + m12;
	geometry_msgs::Vector3 m2 = m21 + m22;
	geometry_msgs::Vector3 m3 = m31 + m32;

    ispisi_vektor(m1,"m1 ");
    ispisi_vektor(m2,"m2 ");
    ispisi_vektor(m3,"m3 ");

    tf::StampedTransform  transform0;
    tf::StampedTransform  transform1;
    transform0.setOrigin(tf::Vector3(0, 0.2, 0));
    transform1.setOrigin(tf::Vector3(0, 0.4, 0));

    geometry_msgs::Vector3 power_line_point0;
    geometry_msgs::Vector3 power_line_vector;
    geometry_msgs::Vector3 power_line_point1;

    tf::StampedTransform cal1;
    cal1.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf::StampedTransform cal2;
    cal2.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf::StampedTransform cal3;
    cal3.setRotation(tf::Quaternion(0, 0, 0, 1));
    ros::Time t = ros::Time(0);

	CVectorDouble initial_x;
	CVectorDouble f_out;
	CVectorDouble y;

	initial_x.resize(4);
	initial_x[0] = -0.0107184;  // x
	initial_x[1] = 0.413035;  // y
	initial_x[2] = 0;  // y
	initial_x[3] = 4.99254;  // y
cout<<endl<<endl<<endl;


m1_loc = transform0;
m2_loc = transform1;
m_0 = m1;
m_1 = m2;
m_2 = m3;;
line_vector=p_line_vector;



ispisi_vektor(m1_loc.getOrigin(),"m1_loc ");
ispisi_vektor(m2_loc.getOrigin(),"m2_loc ");

ispisi_vektor(m_0,"m_0 ");
ispisi_vektor(m_1,"m_1 ");
ispisi_vektor(m_2,"m_2 ");
ispisi_vektor(line_vector,"line_vector ");

//ispis=true;
//    Objective(initial_x, y, f_out);
//    ispis=false;
//    cout<<f_out[0]<<endl;
    bool rez=GetPowerLinesLocation(m1,
    		m2,
    		m3,
			transform0,
			transform1,
			&power_line_vector,
			&power_line_point0,
			&power_line_point1,
			cal1,
			cal2,
			cal3);
cout<<"Objective count "<< objective_count<<endl;

}
int main (int argc, char** argv){
    ros::init(argc, argv, "magnetic_field_localization");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    int refresh_rate;
    std::string magnetic_vector0, magnetic_vector1, magnetic_vector2, frame0, frame1, frame2, power_line_frame0, power_line_frame1;
    std::string vector0_calibration, vector1_calibration, vector2_calibration;
    nh_ns.param("magnetic_vector0", magnetic_vector0, (std::string) "/magnetic_vector0");
    nh_ns.param("magnetic_vector1", magnetic_vector1, (std::string)"/magnetic_vector1");
    nh_ns.param("magnetic_vector2", magnetic_vector2, (std::string)"/magnetic_vector2");
    nh_ns.param("vector0_calibration", vector0_calibration, (std::string) "/magn0_cal");
    nh_ns.param("vector1_calibration", vector1_calibration, (std::string)"/magn1_cal");
    nh_ns.param("vector2_calibration", vector2_calibration, (std::string)"/magn2_cal");

    nh_ns.param("vector0_frame", frame0, (std::string) "/magnetometer0");
    nh_ns.param("vector1_frame", frame1, (std::string) "/magnetometer1");
    nh_ns.param("vector2_frame", frame2, (std::string) "/magnetometer2");
    nh_ns.param("power_line_frame0", power_line_frame0, (std::string) "/power_line0");
    nh_ns.param("power_line_frame1", power_line_frame1, (std::string) "/power_line1");


    ros::Subscriber magnetic_vector_subscriber1 = nh.subscribe(magnetic_vector0, 10, magnetic_vector0_callback);
    ros::Subscriber magnetic_vector_subscriber2 = nh.subscribe(magnetic_vector1, 10, magnetic_vector1_callback);
    ros::Subscriber magnetic_vector_subscriber3 = nh.subscribe(magnetic_vector2, 10, magnetic_vector2_callback);

    geometry_msgs::Vector3 power_line_vector;
    geometry_msgs::Vector3 power_line_point;

    geometry_msgs::Vector3 power_line_vector0;
    geometry_msgs::Vector3 power_line_point0;
    geometry_msgs::Vector3 power_line_vector1;
    geometry_msgs::Vector3 power_line_point1;

    geometry_msgs::Vector3 point0;
    geometry_msgs::Vector3 point1;
    point0.x=0;
    point0.y=0;
    point0.z=0;
    point1.x=0;
    point1.y=0;
    point1.z=0;


    ros::Rate loop_rate(1);
    tf::TransformListener listener;
    tf::TransformBroadcaster br;
    tf::StampedTransform  transform0;
    tf::StampedTransform  transform1;
    int counter=0;
    tf::StampedTransform cal1;
    cal1.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf::StampedTransform cal2;
    cal2.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf::StampedTransform cal3;
    cal3.setRotation(tf::Quaternion(0, 0, 0, 1));
    ros::Time t = ros::Time(0);


    try{
    	listener.lookupTransform("/magnetometer0", "/power_line_gound_truth0", t, cal1);
    	listener.lookupTransform("/magnetometer0", "/power_line_gound_truth1", t, cal2);
    	cout<<"gorund truth"<<endl;
    	ispisi_vektor(cal1.getOrigin(),"power line 0");
    	ispisi_vektor(cal2.getOrigin(),"power line 1");
    	cout<<endl<<endl;
    }
    catch (tf::TransformException ex){
    	ROS_ERROR("%s",ex.what());
    }

//    test_objective();
//    return 0;



    while(ros::ok()){

        ros::spinOnce();
        tf::StampedTransform transform;

        t = ros::Time(0);

/*        try{
        	listener.lookupTransform("/magnetometer0", "/power_line_ground_truth0", t, cal1);
        	listener.lookupTransform("/magnetometer0", "/power_line_ground_truth1", t, cal2);
        	cout<<"gorund truth"<<endl;
        	ispisi_vektor(cal1.getOrigin(),"power line 0");
        	ispisi_vektor(cal2.getOrigin(),"power line 1");
        	cout<<endl<<endl;
        }
        catch (tf::TransformException ex){
        	ROS_ERROR("%s",ex.what());
        	continue;
        }*/

        try {
        	listener.lookupTransform(frame0, vector0_calibration, t, cal1);
        	listener.lookupTransform(frame1, vector1_calibration, t, cal2);
        	listener.lookupTransform(frame2, vector2_calibration, t, cal3);
        }
        catch (tf::TransformException ex){
        	ROS_ERROR("%s",ex.what());
        }
//    	ROS_INFO_STREAM("cal1 r0 " << cal1.getBasis().getRow(0).getX() << " " <<cal1.getBasis().getRow(0).getY() << " "<<cal1.getBasis().getRow(0).getZ()<<std::endl);
 //   	ROS_INFO_STREAM("cal1 r1 " << cal1.getBasis().getRow(1).getX() << " " <<cal1.getBasis().getRow(1).getY() << " "<<cal1.getBasis().getRow(1).getZ()<<std::endl);
 //   	ROS_INFO_STREAM("cal1 r2 " << cal1.getBasis().getRow(2).getX() << " " <<cal1.getBasis().getRow(2).getY() <<" " <<cal1.getBasis().getRow(2).getZ()<<std::endl);

 //   	ROS_INFO_STREAM("cal2 r0 " << cal2.getBasis().getRow(0).getX() << " " <<cal2.getBasis().getRow(0).getY()<<" " << cal2.getBasis().getRow(0).getZ()<<std::endl);
 //   	ROS_INFO_STREAM("cal2 r1 " << cal2.getBasis().getRow(1).getX() << " " <<cal2.getBasis().getRow(1).getY()<<" " << cal2.getBasis().getRow(1).getZ()<<std::endl);
 //   	ROS_INFO_STREAM("cal2 r2 " << cal2.getBasis().getRow(2).getX() << " " <<cal2.getBasis().getRow(2).getY()<<" " << cal2.getBasis().getRow(2).getZ()<<std::endl);

        try{


        	listener.lookupTransform(frame0, frame1, t, transform0);
        	listener.lookupTransform(frame0, frame2, t, transform1);

        	point0.x=transform0.getOrigin().getX();
            point0.y=transform0.getOrigin().getY();
            point0.z=transform0.getOrigin().getZ();

        	point1.x=transform1.getOrigin().getX();
            point1.y=transform1.getOrigin().getY();
            point1.z=transform1.getOrigin().getZ();


//        	ROS_INFO_STREAM("transform" <<transform.getOrigin().getX()<<" "<<transform.getOrigin().getY()<< " "<<transform.getOrigin().getZ());

//            bool rez=GetPowerLineLocation(received_vector0, received_vector1, transform, &power_line_vector, &power_line_point, cal1, cal2);
            bool rez=GetPowerLinesLocation(received_vector0,
            		received_vector1,
            		received_vector2,
					transform0,
					transform1,
					&power_line_vector,
					&power_line_point0,
					&power_line_point1,
					cal1,
					cal2,
					cal3);
//            TestLevMarq();
//            break;
            geometry_msgs::Vector3 p1 = getClosestPointOnLine(power_line_point0, power_line_vector, point0);
            geometry_msgs::Vector3 p2 = getClosestPointOnLine(power_line_point1, power_line_vector, point0);
            std::cout<<"0 dist1 dist2 magn1 magn2  d1/d2 m2/m1 "<<VectorSize(p1)<<" "<<VectorSize(p2)<<" "<<VectorSize(received_vector0)<<" "<<VectorSize(received_vector1)<<" "
            	<<" "<<VectorSize(p1)/VectorSize(p2)<<" "<<VectorSize(received_vector1)/VectorSize(received_vector0)<<std::endl;
            /*if (fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0))>0.25)
            {
            	double d1=fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0));
                received_vector0.y = -received_vector0.y;
                bool rez1=GetPowerLineLocation(received_vector0, received_vector1, transform, &power_line_vector1, &power_line_point1, cal1, cal2);

                p1 = getClosestPointOnLine(power_line_point1,power_line_vector,point0);
                p2 = getClosestPointOnLine(power_line_point1,power_line_vector,point1);
                std::cout<<"1 dist1 dist2 magn1 magn2  d1/d2 m2/m1 "<<VectorSize(p1)<<" "<<VectorSize(p2)<<" "<<VectorSize(received_vector0)<<" "<<VectorSize(received_vector1)<<" "
                	<<" "<<VectorSize(p1)/VectorSize(p2)<<" "<<VectorSize(received_vector1)/VectorSize(received_vector0)<<std::endl;
                double d2=fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0));
                if (d2 < d1 && d2<0.5)
                {
                	d1=fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0));
                	std::cout<<"d1 = "<<d1<<std::endl;
                	power_line_vector=power_line_vector1;
                	power_line_point=power_line_point1;
                }

                received_vector1.y = -received_vector1.y;
                bool rez2=GetPowerLineLocation(received_vector0, received_vector1, transform, &power_line_vector2, &power_line_point2, cal1, cal2);
                p1 = getClosestPointOnLine(power_line_point2,power_line_vector,point0);
                p2 = getClosestPointOnLine(power_line_point2,power_line_vector,point1);
                std::cout<<"2 dist1 dist2 magn1 magn2  d1/d2 m2/m1 "<<VectorSize(p1)<<" "<<VectorSize(p2)<<" "<<VectorSize(received_vector0)<<" "<<VectorSize(received_vector1)<<" "
                	<<" "<<VectorSize(p1)/VectorSize(p2)<<" "<<VectorSize(received_vector1)/VectorSize(received_vector0)<<std::endl;
                d2=fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0));
                if (d2 < d1 && d2<0.5)
                {
                	d1=fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0));
                	std::cout<<"d1 = "<<d1<<std::endl;
                	power_line_vector=power_line_vector2;
                	power_line_point=power_line_point2;
                }


                received_vector0.y = -received_vector0.y;
                bool rez3=GetPowerLineLocation(received_vector0, received_vector1, transform, &power_line_vector3, &power_line_point3, cal1, cal2);
                p1 = getClosestPointOnLine(power_line_point3,power_line_vector,point0);
                p2 = getClosestPointOnLine(power_line_point3,power_line_vector,point1);
                received_vector1.y = -received_vector1.y;
                std::cout<<"3 dist1 dist2 magn1 magn2  d1/d2 m2/m1 "<<VectorSize(p1)<<" "<<VectorSize(p2)<<" "<<VectorSize(received_vector0)<<" "<<VectorSize(received_vector1)<<" "
                	<<" "<<VectorSize(p1)/VectorSize(p2)<<" "<<VectorSize(received_vector1)/VectorSize(received_vector0)<<std::endl;
                d2=fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0));
                if (d2 < d1 && d2<0.5)
                {
                	d1=fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0));
                	std::cout<<"d1 = "<<d1<<std::endl;
                	power_line_vector=power_line_vector3;
                	power_line_point=power_line_point3;
                }

            }*/




            //            ROS_INFO_STREAM(rez<< " vector "<< power_line_vector<<"   point "<<power_line_point);
            transform0.setOrigin(tf::Vector3(power_line_point0.x, power_line_point0.y, power_line_point0.z));
            transform1.setOrigin(tf::Vector3(power_line_point1.x, power_line_point1.y, power_line_point1.z));

            tf::Quaternion q;
            if (power_line_vector.x == power_line_vector.x &&
            		power_line_point0.x == power_line_point0.x &&
					power_line_point1.x == power_line_point1.x)
            {
//            	q.setsetRotation(tf::Vector3(power_line_vector.x, power_line_vector.y, power_line_vector.z),0.1);
            	geometry_msgs::Vector3 pomocni;
            	pomocni.x=1;
            	pomocni.y=0;
            	pomocni.z=0;
            	geometry_msgs::Vector3 kros=CrossProduct(pomocni,power_line_vector);
            	double w=1+DotProduct(pomocni,power_line_vector);

//            	geometry_msgs::Vector3 kros=CrossProduct(power_line_vector,pomocni);
//            	double w=1+DotProduct(power_line_vector,pomocni);
            	//            	transform1.setRotation(tf::Quaternion(power_line_vector.x, power_line_vector.y, power_line_vector.z,0));
            	transform0.setRotation(tf::Quaternion(kros.x, kros.y, kros.z, w));

            	transform1.setRotation(tf::Quaternion(kros.x, kros.y, kros.z, w));



            	br.sendTransform(tf::StampedTransform(transform0, ros::Time::now(), frame0, power_line_frame0));
            	br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), frame0, power_line_frame1));
            }
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }
        loop_rate.sleep();


    }
}


