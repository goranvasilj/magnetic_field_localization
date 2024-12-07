
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

geometry_msgs::Vector3 received_vector0;
geometry_msgs::Vector3 received_vector1;

void magnetic_vector0_callback(const sensor_msgs::MagneticField::ConstPtr& msg){

	received_vector0.x = (*msg).magnetic_field.x;
	received_vector0.y = (*msg).magnetic_field.y;
	received_vector0.z = (*msg).magnetic_field.z;

//	std::cout<<"received0 "<<*msg<<std::endl;
}

void magnetic_vector1_callback(const sensor_msgs::MagneticField::ConstPtr& msg){
	received_vector1.x = (*msg).magnetic_field.x;
	received_vector1.y = (*msg).magnetic_field.y;
	received_vector1.z = (*msg).magnetic_field.z;

//	std::cout<<"received1 "<<*msg<<std::endl;

}

geometry_msgs::Vector3 CrossProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B) {
	geometry_msgs::Vector3 c_P;
   c_P.x = v_A.y * v_B.z - v_A.z * v_B.y;
   c_P.y = -(v_A.x * v_B.z - v_A.z * v_B.x);
   c_P.z = v_A.x * v_B.y - v_A.y * v_B.x;
   return c_P;
}

geometry_msgs::Vector3 sum_vector(geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2) {
	geometry_msgs::Vector3 v3;
	v3.x = v1.x + v2.x;
	v3.y = v1.y + v2.y;
	v3.z = v1.z + v2.z;
   return v3;
}

geometry_msgs::Vector3 normalize_vector(geometry_msgs::Vector3 v)
{
	double dist=sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	v.x = v.x / dist;
	v.y = v.y / dist;
	v.z = v.z / dist;
	return v;
}

double VectorSize(geometry_msgs::Vector3 vector)
{
	return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

double DotProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B)
{
	return v_A.x * v_B.x + v_A.y * v_B.y + v_A.z * v_B.z;
}

geometry_msgs::Vector3 transform_vector(geometry_msgs::Vector3 vector, tf::Matrix3x3 rotation)
{
	geometry_msgs::Vector3 result;

	result.x = rotation.getRow(0).getX() * vector.x +  rotation.getRow(0).getY() * vector.y + rotation.getRow(0).getZ() * vector.z;
	result.y = rotation.getRow(1).getX() * vector.x +  rotation.getRow(1).getY() * vector.y + rotation.getRow(1).getZ() * vector.z;
	result.z = rotation.getRow(2).getX() * vector.x +  rotation.getRow(2).getY() * vector.y + rotation.getRow(2).getZ() * vector.z;
	return result;
}
bool GetPowerLineLocation(geometry_msgs::Vector3 m_vector1,
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
	/*ROS_INFO_STREAM("vector 1 init"<<m_vector1.x<<" "<<m_vector1.y<<" "<<m_vector1.z);
	ROS_INFO_STREAM("vector 2 init"<<m_vector2.x<<" "<<m_vector2.y<<" "<<m_vector2.z);*/

	m_vector1 = transform_vector(m_vector1, basis1);
	m_vector2 = transform_vector(m_vector2, basis2);
	ROS_INFO_STREAM("vector 1 tran"<<m_vector1.x<<" "<<m_vector1.y<<" "<<m_vector1.z);
	ROS_INFO_STREAM("vector 2 tran"<<m_vector2.x<<" "<<m_vector2.y<<" "<<m_vector2.z);


	//to do rotacija m_vector2 to m_vector1 frame  - za paralelne senzore nije potrebna
	*power_line_vector=CrossProduct(m_vector1, m_vector2);

	ROS_INFO_STREAM("power_line vector "<<(*power_line_vector).x<<" "<<(*power_line_vector).y<<" "<<(*power_line_vector).z);

	if (VectorSize(m_vector1) * 0.1 > VectorSize(*power_line_vector)
			&& VectorSize(m_vector2) * 0.1 > VectorSize(*power_line_vector) && 0==1)
	{
		// line between sensors points towards powerline
		if (VectorSize(m_vector1) > 0.8 * VectorSize(m_vector2)
		&& VectorSize(m_vector1) < 1.2 * VectorSize(m_vector2))
		{
			//paralel to the powerline
			return false;
		}
		else
		{

			double max_axis=m_vector1.x, max_axis2=m_vector2.x;
			if (fabs(m_vector1.y) > fabs(max_axis)) {max_axis=m_vector1.y; max_axis2 = m_vector2.y;}
			if (fabs(m_vector1.z) > fabs(max_axis)) {max_axis=m_vector1.z; max_axis2 = m_vector2.z;}
			if (max_axis * max_axis2>0) //same direction
			{
				double dist=sqrt((double)(transform.getOrigin().x()*transform.getOrigin().x()+
						transform.getOrigin().y()*transform.getOrigin().y()+
						transform.getOrigin().z()*transform.getOrigin().z()));
				double a=VectorSize(m_vector1);
				double b=VectorSize(m_vector2);
				if (a < b)
				{
					double t = a;
					a = b;
					b = t;
				}
				double r = (sqrt(dist * dist + 4 * a / b) - dist) / 2;
				if (VectorSize(m_vector1) > VectorSize(m_vector2))
				{
					double directionx = m_vector1.x - m_vector2.x;
					double directiony = m_vector1.y - m_vector2.y;
					double directionz = m_vector1.z - m_vector2.z;
					power_line_point->x = m_vector1.x + directionx * r;
					power_line_point->y = m_vector1.y + directiony * r;
					power_line_point->z = m_vector1.z + directionz * r;
				}
				else
				{
					double directionx = m_vector2.x - m_vector1.x;
					double directiony = m_vector2.y - m_vector1.y;
					double directionz = m_vector2.z - m_vector1.z;
					power_line_point->x = m_vector2.x + directionx * r;
					power_line_point->y = m_vector2.y + directiony * r;
					power_line_point->z = m_vector2.z + directionz * r;
				}
			}
			else
			{
				//opposite direction todo

			}
		}
	}
	else
	{
		//line between sensors does not point towards power line

		geometry_msgs::Vector3 towards_center1=CrossProduct(m_vector1, *power_line_vector);
		geometry_msgs::Vector3 towards_center2=CrossProduct(m_vector2, *power_line_vector);


		//closest point between two lines in 3d space;
		geometry_msgs::Vector3 a0;
		a0.x = 0; a0.y=0; a0.z=0;
		geometry_msgs::Vector3 b0;
		b0.x=(double) transform.getOrigin().x();
		b0.y=(double) transform.getOrigin().y();
		b0.z=(double) transform.getOrigin().z();

		geometry_msgs::Vector3 a=normalize_vector(towards_center1);
		geometry_msgs::Vector3 b=normalize_vector(towards_center2);
		geometry_msgs::Vector3 cn=normalize_vector(CrossProduct(b, a));



		cn=CrossProduct(b, a);
		geometry_msgs::Vector3 n2=CrossProduct(b, cn);
		double dot1=DotProduct(b0, n2)/DotProduct(a,n2);
		geometry_msgs::Vector3 a2=a;
		a2.x *= dot1;
		a2.y *= dot1;
		a2.z *= dot1;
		geometry_msgs::Vector3 closest_approach=a2;


		*power_line_point=closest_approach;

		ROS_INFO_STREAM("power_line "<<(*power_line_point).x<<" "<<(*power_line_point).y<<" "<<(*power_line_point).z);
		*power_line_vector = normalize_vector(*power_line_vector);

	}
	return true;
}

geometry_msgs::Vector3 getClosestPointOnLine(geometry_msgs::Vector3 line_point,geometry_msgs::Vector3 line_vector, geometry_msgs::Vector3 point)
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

	vector.x = vx;
	vector.y = vy;
	vector.z = vz;

	double t=-DotProduct(p1,vector)/VectorSize(vector)/VectorSize(p1);
	result.x = x1 + t * vx;
	result.y = y1 + t * vy;
	result.z = z1 + t * vz;

	return result;

}

int main (int argc, char** argv){
    ros::init(argc, argv, "magnetic_field_localization");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    int refresh_rate;
    std::string magnetic_vector0, magnetic_vector1, frame0, frame1,power_line_frame;
    std::string vector0_calibration,vector1_calibration;
    nh_ns.param("magnetic_vector0", magnetic_vector0, (std::string) "/magnetic_vector0");
    nh_ns.param("magnetic_vector1", magnetic_vector1, (std::string)"/magnetic_vector1");
    nh_ns.param("vector0_calibration", vector0_calibration, (std::string) "/magn1_cal");
    nh_ns.param("vector1_calibration", vector1_calibration, (std::string)"/magn2_cal");

    nh_ns.param("vector0_frame", frame0, (std::string) "/magnetometer0");
    nh_ns.param("vector1_frame", frame1, (std::string) "/magnetometer1");
    nh_ns.param("power_line_frame", power_line_frame, (std::string) "/power_line");

    ros::Subscriber magnetic_vector_subscriber1 = nh.subscribe(magnetic_vector0, 10, magnetic_vector0_callback);
    ros::Subscriber magnetic_vector_subscriber2 = nh.subscribe(magnetic_vector1, 10, magnetic_vector1_callback);
    std::cout<<magnetic_vector0<<" "<<magnetic_vector1<<std::endl;

    geometry_msgs::Vector3 power_line_vector;
    geometry_msgs::Vector3 power_line_point;

    geometry_msgs::Vector3 power_line_vector1;
    geometry_msgs::Vector3 power_line_point1;
    geometry_msgs::Vector3 power_line_vector2;
    geometry_msgs::Vector3 power_line_point2;
    geometry_msgs::Vector3 power_line_vector3;
    geometry_msgs::Vector3 power_line_point3;

    geometry_msgs::Vector3 point0;
    geometry_msgs::Vector3 point1;
    point0.x=0;
    point0.y=0;
    point0.z=0;
    point1.x=0;
    point1.y=0;
    point1.z=0;


    ros::Rate loop_rate(10);
    tf::TransformListener listener;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Transform transform1;
    int counter=0;
    tf::StampedTransform cal1;
    cal1.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf::StampedTransform cal2;
    cal2.setRotation(tf::Quaternion(0, 0, 0, 1));
    ros::Time t = ros::Time(0);
    while(ros::ok()){

        ros::spinOnce();
        tf::StampedTransform transform;

        t = ros::Time(0);
        try {
        	listener.lookupTransform(frame0, vector0_calibration, t, cal1);
        	listener.lookupTransform(frame1, vector1_calibration, t, cal2);


        }
        catch (tf::TransformException ex){
        	ROS_ERROR("%s",ex.what());
        }

        try{
        	listener.lookupTransform(frame0, frame1, t, transform);
            point1.x=transform.getOrigin().getX();
            point1.y=transform.getOrigin().getY();
            point1.z=transform.getOrigin().getZ();


            bool rez=GetPowerLineLocation(received_vector0, received_vector1, transform, &power_line_vector, &power_line_point, cal1, cal2);


            transform1.setOrigin(tf::Vector3(power_line_point.x, power_line_point.y, power_line_point.z));
            tf::Quaternion q;
            if (power_line_vector.x==power_line_vector.x && power_line_point.x==power_line_point.x)
            {
            	geometry_msgs::Vector3 pomocni;
            	pomocni.x=1;
            	pomocni.y=0;
            	pomocni.z=0;
            	geometry_msgs::Vector3 kros=CrossProduct(pomocni,power_line_vector);
            	double w=1+DotProduct(pomocni,power_line_vector);

            	transform1.setRotation(tf::Quaternion(kros.x, kros.y, kros.z, w));

            	br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), frame0, power_line_frame));
            }
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }
        loop_rate.sleep();


    }
}


