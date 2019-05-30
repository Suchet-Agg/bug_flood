#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/OccupancyGrid.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/pcl_base.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/crop_box.h"
#include "math.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/NavSatFix.h>
#include "nlgl.h"
#include "nlgl.cpp"
#include <memory>
#include "std_msgs/String.h"
#include <pixhwk_ctrl_node/traffic_light.h>
//#include <pixhwk_ctrl_node/OmplStatus.h>
// #include <pixhwk_ctrl_node/Lanes.h>

#include <vector>

static const float m_per_cell_ = 0.1;					// m_per_cell_ is short for meters per cell. m_per_cell_ = 0.1 means that the dimension of one cell is 10c.m x 10c.m. Increasing the scale factor = increasing resolution.
static const int m_size = 501;
static const int m_size_for_center = m_size;			//defualt value=101;				//matriz size...value is odd as it is easy to find median.

//static const int m_size_minus1 = 201;
static int Matrix[m_size][m_size] ={0};			// Purpose of this matriz is store obstacle data and is used only for GOAL SETTING
// enter padding terms w.r.t. to the center of the vehicle
int padding = 3;														//was 2 initially
// padding = 1 means , 1/(scale_factor) meter padding per block. 5 means 5*(1/(scale_factor)) meter padding.




double max_dev = 50;										//+-5 degrees error. This decides the angle of the Heading cone

/*
		 (-5 deg)		(0)	  	  (+5 deg)
			\			 |			  /
			 \			 |			 /
			  \			 |			/
			   \		 |		   /
				\        |		  /
				 \       |		 /
				  \      |		/
				   \     |	   /
				    \    |	  /
				     \   |   /
				      \  |	/
				       \ | /
				        \|/

*/


#define lidar_shift_right (1)					 // positive if right,x amount that lidar is dispalced from the center of the vehicle in left right direction
#define lidar_shift_front (3)					 // positive if froward, amount that lidar is dispalced from the center of the vehicle in front back direction
//currently not using lidar_shift


using namespace std;


//static double rotation[3] = {0.0, 0.0, 0.0,};

int loop_counter = 0;							// used to keep track of the loop and skip iteration for a specific count...currently, not skipping any iteration
//static std::vector<char> Matrix((m_size)*(m_size),0);						// Purpose of this matriz is store obstacle data and is used only for GOAL SETTING

//*******************************************************************************************************************
nav_msgs::OccupancyGrid map123;
/*
		## This represents a 2-D grid map, in which each cell represents the probability of occupancy.

		1)Header header 
		2)MapMetaData info  		//{MetaData for the map}

		# The map data, in row-major order, starting with (0,0). Occupancy probabilities are in the range [0,100].
		# Unknown is -1.
		3)int8[] datag
*/
//********************************************************************************************************************


//these are in lidar

double heading = 0;								//global variable for Heading angle that we get from /car_heading topic
//static double my_position[2] = {0,0};
// static double my_goal[3];
bool need_to_read_waypoint_file = true;
int reached_goall = 0; 													//if 1 then goal has been reached else 0
int min_distance_for_goal = 2; 											//minimum threshold of goal length
int max_distance_goal = 30; 											//max goal length
int dist_to_traffic_light = 18;											//distance to traffic light
// int loop_iterator = 0;
static int goal_x = 0;
static int goal_y = 0;
int lidar_x = (m_size_for_center+1)/2;									//here lidar and home values are same
int lidar_y = (m_size_for_center + 1)/2;								//lidar_x and lidar_y are the postions of lidar in our local coordinate system
int home_x = (m_size_for_center+1)/2;// + lidar_shift_right;			//home_x and home_y are the postion of centre of vehicle in our local coordinate system
int home_y = (m_size_for_center+1)/2;// - lidar_shift_front;
vector<int> trajectory_points[2];										//trajectory_points[0] has x coordinates and trajectory_points[0] has y coordinates.
vector<double> waypoints[2]; // waypoints vector [0] contains x and [1] contains y coordinates
double prev_position[2] = {0,0};
//int omplstatus;
//double timestamp;
int traj_count = 0, waypoints_count = 0, waypt_plot_count = 0;														//A flag to keep track of number of trajectory points that are read from the file.
vector<int> traj_turn_point[2];									//a vector to keep track of trajectory points where a turn is detected for the purpose of visualizaition or some other processing
vector<double> waypt_turn_point[2];
vector<int> waypt_turn_local_coordinate[2];
vector<std::string> waypt_turn_direction;
//path on my system = "/home/pratique/Swarath_Repo/"
const std::string path_to_swarath_folder = "/home/parasmehan123";				//path on nuc = "/home/nuc1/iiitd/""
const std::string goal_file = path_to_swarath_folder + "swarath/goal.txt";
const std::string matrix_file = path_to_swarath_folder + "swarath/matrix.txt";
//const std::string data_collection_file = path_to_swarath_folder + "swarath/matrix_data.txt";
const std::string trajectory_file = path_to_swarath_folder + "swarath/pathal.txt";				//change_back
const std::string ompl_scipt = path_to_swarath_folder + "swarath/bug_flood_script.sh";
std::string waypoints_file;
// /home/pratique/Swarath_Repo/swarath/waypoints/college_entrance_to_okhla_gate.asc
// setting up publisher
ros::Publisher map_pub;
ros::Publisher trajectory_turn_pub;
ros::Publisher waypoint_turn_pub;
/*
obstacle is 1
clear is 0
*/
///////////////////////////////////////////////////////////////
//struct Point
//{
//  int x;
//  int y;
//};

//vector<double> left_lane_points[2];
//vector<double> right_lane_points[2];

/*
bool onSegment(Point p, Point q, Point r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
            q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;
    return false;
}
*/

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise

/*
int orientation(Point p, Point q, Point r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // colinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}
*/

// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
/*
bool doIntersect(Point p1, Point q1, Point p2, Point q2)
{	
	// cout<<"Inside doIntersect \n";
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}
*/

// Returns true if the point p lies inside the polygon[] with n vertices
/*
bool isInside(Point p, Point polygon[])
{
	// cout<<"Inside isInside \n";
    // Create a point for line segment from p to infinite
    Point extreme;
    extreme.x=0;
    extreme.y=0;

    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i+1)%4;

        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(polygon[i], polygon[next], p, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(polygon[i], p, polygon[next]) == 0)
               return onSegment(polygon[i], p, polygon[next]);

            count++;
        }
        i = next;
    } while (i != 0);
    // cout<<"Outside isInside \n";
    // Return true if count is odd, false otherwise
    return count&1;  // Same as (count%2 == 1)
}

bool is_inside_lane(Point p) // Checks whether the given point is whithin the lane
{
	// cout<<"Inside is_inside_lane \n";

	bool status = false;
	Point polygon[4];
	int l=0,r=0;
	// cout<<"Lane instant size :"<<left_lane_points[0].size()<<' '<<right_lane_points[0].size()<<'\n';
	while((l+1)<(left_lane_points[0].size()-1) || (r+1)<(right_lane_points[0].size()-1))
	{
		// cout<<"inside while \n";
		polygon[0].x=left_lane_points[0][l];
		polygon[0].y=left_lane_points[1][l];
		polygon[1].x=right_lane_points[0][r];
		polygon[1].y=right_lane_points[1][r];
		if((r+1)<right_lane_points[0].size())
			r=r+1;
		if((l+1) < left_lane_points[0].size())
			l=l+1;
		polygon[2].x = right_lane_points[0][r];
		polygon[2].y = right_lane_points[1][r];
		polygon[3].x = left_lane_points[0][l];
		polygon[3].y = left_lane_points[1][l];

		status = isInside(p,polygon);
		if(status == true)
			break;
	}
	// polygon[0].x=70;
	// polygon[0].y = 0;
	// polygon[1].x = 30;
	// polygon[1].y = 0;
	// polygon[2].x = 30;
	// polygon[2].y = 100;
	// polygon[3].x = 70;
	// polygon[3].y = 100;

	// status = isInside(p,polygon);
	return status;
	// cout<<"Leaving is_inside_lane \n";
}
*/
//////////////////////////////////////////////////////////////////////

double angle_wrap(double angle)		//wrapping of angle.. or in other words mapping angle from (0 to 360) to (-180 to 180)
{
	if(angle > 180)
	{
		angle = angle - 360; 
	}
	else if(angle < -180)
	{
		angle = 360 + angle;
	}
	return angle;
}


double angle_unwrap(double theta)
{
    /*
    going from (-180,180) to (0,360)
    */
    if(theta < 0)
    {
        theta = 360 + theta;
    }
    return theta;
}


int get_goal_quadrant(double heading)										// calculates the quadrant in which the goal is based on heading it is gets
{
	int goal_quadrant = 0;

	if(heading>0 && heading<=90)
		goal_quadrant=0;		
	else if(heading>90 && heading<=180)
		goal_quadrant=1;
	else if(heading<0 && heading>=-90)						
		goal_quadrant=2;
	else if(heading<-90 && heading>=-180)
		goal_quadrant=3;

	return goal_quadrant;
}

int get_local_quadrant(double heading)										//same as get_goal_guadrant now, was different function initially
{
	int local_quadrant = 0;

	if(heading>0 && heading<=90)
		local_quadrant=0;		
	else if(heading>90 && heading<=180)
		local_quadrant=1;
	else if(heading<0 && heading>=-90)
		local_quadrant=2;
	else if(heading<-90 && heading>=-180)
		local_quadrant=3;

	return local_quadrant;
}


// void get_waypoints(string waypoints_file)
// {
//     string line;
//     double prev[2], current[2];
//     int i, flag = 0, count = 0, distance_interval = 9;	//distance_interval is the interval at which we are reading waypoints from file(default = 7)
//     ifstream infile_waypoints(waypoints_file.c_str());

//     if(!infile_waypoints.is_open())
//     {
//     	cout<<"[Error] Unable to open waypoint file"<<endl;
//     	exit(1);
//     }
    
//     else
// 	{
// 	    // cout<<"------------ GPS waypoints ------------------\n";
// 	    while (std::getline(infile_waypoints, line))
// 	    {
// 	        std::istringstream iss(line);
// 		    double lat,lon, x1, y1, d;

// 		    if (!(iss >> lat >> lon)) 
// 		    {  
// 		      cout<<"[ERROR] not reading points"<<endl;	break;														        // error
// 		    } 
		    
// 		    //adding gps coordinates to a waypoints vector which are at an interval of "distance_interval"
// 	    	if (flag == 0)
// 	    	{
// 	    		prev[0] = lat;
// 	    		prev[1] = lon;
// 	    		waypoints[0].push_back(prev[0]);
// 			    waypoints[1].push_back(prev[1]);
// 			    count++;
// 	    		flag = 1;
// 			}

// 	    	current[0] = lat;
// 	    	current[1] = lon;
// 	    	d = distance(prev[0], prev[1], current[0], current[1]);
	    	
// 	    	if(d >= distance_interval)
// 		    {
// 		    	waypoints[0].push_back(current[0]);
// 			    waypoints[1].push_back(current[1]);
// 			    count++;
// 				prev[0] = current[0];
// 		    	prev[1] = current[1];
// 		    }
// 		    waypoints_count = count;
// 		}
// 	 	infile_waypoints.close();
// 	}

// }


// /*
// This function detects a turn by reading points from waypoint file
// */
// void detect_turn_waypoint()				
// {
//   double waypoint_turn_angle_threshold = 20.0;                      //define waypoint_turn_angle_threshold in degrees// if bearing is greater than this threshold a turn is detected.
//   double delta, max_angle = 0, dist_from_last_wp = 0;
//   int i, j;
//   float angle_1, angle_2,dist;
//   double x1, y1, x2, y2, x3, y3, px1, py1;
//   float tollerance = 0.4;								            //error tollerance in distance ; range = [0,1]
//   turn_count = 0;
  
//   // if(need_to_read_waypoint_file)									//check to ensure that the waypoint file is read only once to save computational overhead
//   // {
//   // 	get_waypoints(waypoints_file);
//   // 	need_to_read_waypoint_file = false;
//   // }
  
//   //storing data in file to find out turn angle threshold
// 								// FILE *mf = fopen("/home/pratique/max_angle.txt","a");
// 								// if(mf == NULL)
// 								// 	{
// 								// 		cout<<"couldn't open max_angle file"<<endl;
// 								// 		exit(1);
// 								// 	}
// 								// 	fprintf(mf, "\n\n");

//   for(i = waypoints[0].size()-waypoints_count; i < waypoints[0].size()-2; ++i)
//   {
//     x1 = waypoints[0].at(i);
//     y1 = waypoints[1].at(i);
//     x2 = waypoints[0].at(i+1);
//     y2 = waypoints[1].at(i+1);
//     x3 = waypoints[0].at(i+2);
//     y3 = waypoints[1].at(i+2);

//     angle_1 = angle_unwrap(rad2deg(bearing(x1, y1, x2, y2)));
//     angle_2 = angle_unwrap(rad2deg(bearing(x2, y2, x3, y3)));
    
//  	delta = angle_2 - angle_1;
   
//     cout<<"angle difference = "<<delta<<endl;

//     /**************************************************************
//     ***************************************************************
// 	SIGN CONVENTION (VERIFIED VISUALLY)

// 	if delta is positive, RIGHT turn is detected
// 	ELSE if delta is negative, LEFT turn is detected

// 	***************************************************************
//     **************************************************************/

// 	    if ((fabs(delta) >= waypoint_turn_angle_threshold) && (fabs(delta) <180))
// 	    {
//     	   	turn_count++;
	    	
// 	    	waypt_turn_point[0].push_back(x2);
// 			waypt_turn_point[1].push_back(y2);
// 			dist_from_last_wp = distance(px1, py1, x2, y2);
// 				cout<<"\n\n-------TURN DETECTED------\n\n"<<endl;
// 			if( delta < 0 )
// 		    {
// 		    	waypt_turn_direction.push_back("left");
// 		 //    	setprecision(14);
// 			// fprintf(mf, "prev_lat= %lf ,prev_lon= %lf,lat= %lf ,lon= %lf, dist_from_last_wp= %lf,  dir= left %d,   delta(%lf) = angle_1(%lf) - angle_2(%lf) \n",x1, y1, x2, y2, dist_from_last_wp, turn_count-1, delta, angle_1, angle_2);
// 		    }
		
// 		    else if( delta > 0 )
// 		    {
// 		    	waypt_turn_direction.push_back("right");
// 		 //    	setprecision(14);
// 			// fprintf(mf, "prev_lat= %lf ,prev_lon= %lf,lat= %lf ,lon= %lf, dist_from_last_wp= %lf,  dir= right %d,  delta(%lf) = angle_1(%lf) - angle_2(%lf) \n",x1, y1, x2, y2, dist_from_last_wp, turn_count-1, delta, angle_1, angle_2);
// 		    }
// 		}

//   }
//   // fclose(mf);
// }


// // void waypoint_plot(double distance_to_upcomming_turn, double turn_waypoint_heading)
// // {
// // 	float tollerance = 0.0005 ;
// // 	double min_heading_diff = 10000.0;
// // 	double local_heading, local_distance;
// // 	int local_count = 0;
// // 	int a,b,q,r;							//loop variables (ignore)

// // 	if(distance_to_upcomming_turn <= 35)
// // 	{
// // 		for( a = 0; a < m_size; a++)
// // 		{
// // 			for(b = 0; b< m_size; b++)
// // 			{
// // 				local_heading = rad2deg(atan2(a - home_x, b- home_y));
// // 				local_distance = sqrt((a - home_x)*(a - home_x) + (b - home_y)*(b - home_y));
// // 				cout<<"local distance b4 conversion = "<<local_distance<<endl;
// // 				local_distance = local_distance/10;
// // 				cout<<"local distance = "<<local_distance<<endl;
// // 				cout<<"distance to waypont distance = "<<distance_to_upcomming_turn<<endl;
				
// // 				if( (local_heading >= turn_waypoint_heading-max_dev) && (local_heading < turn_waypoint_heading + max_dev))
// // 				{
// // 					if(fabs(local_heading -turn_waypoint_heading) <= min_heading_diff)
// // 					{
// // 						if(local_distance == distance_to_upcomming_turn /*>= (distance_to_upcomming_turn*(1-tollerance)) && local_distance < (distance_to_upcomming_turn*(1+tollerance))*/)	//checking if local distance is in the range [(1-tollerance),(1+tollerance)]				{
// // 						{
// // 							if(get_goal_quadrant(turn_waypoint_heading) == get_local_quadrant(local_heading))
// // 							{
// // 								min_heading_diff = fabs(local_heading -turn_waypoint_heading);
// // 								waypt_turn_local_coordinate[0].push_back(a);
// // 								waypt_turn_local_coordinate[1].push_back(b);
// // 								local_count++;
// // 								cout<<"########  x = "<<a<<endl
// // 									<<"########  y = "<<b<<endl
// // 									<<endl;
// // 							}
// // 						}
// // 					}
// // 				}
// // 			}
// // 		}
// // 	}

// // 	waypt_plot_count = local_count;
// // 	cout<<"waypt_plot_count = "<<waypt_plot_count<<endl;
// // }


// // void traffic_light_callback(const pixhwk_ctrl_node::traffic_light msg)
// // {
// // 	dist_to_traffic_light = msg.dist_light;
// // }

// int relevant_turn_index = 0;
// double prev_distance = 0;
// void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
// {
// 	int k;//, relevant_turn_index = 0;
// 	float index_weight = 10;
// 	float bearing_weight =0.6;
// 	float distance_weight = (1 - bearing_weight);
// 	float switch_on_indicator_distance = 25.0;							//this variable holds distance cap for which a indicator should  blinking
// 	double bearing_diff, current_bearing, turn_bearing ,distance_to_upcomming_turn, weighted_sum = 0, min_weighted_sum = 10000, current_position[2] = {0,0};
// 	std_msgs::String temp;
// 	std:stringstream ss;

// 	if(gps_msg->status.status == 0)
// 	{
// 		if(need_to_read_waypoint_file)									//check to ensure that the waypoint file is read only once to save computational overhead
// 		{
// 			get_waypoints(waypoints_file);
// 			detect_turn_waypoint();
// 			prev_position[0] = gps_msg->latitude;
//         	prev_position[1] = gps_msg->longitude;
//         	// prev_distance = distance(prev_position[0], prev_position[1], waypt_turn_point[0].at(0), waypt_turn_point[1].at(0));
//         	relevant_turn_index = 0;
//         	need_to_read_waypoint_file = false;
// 		}

// 		current_position[0] = gps_msg->latitude;
//         current_position[1] = gps_msg->longitude;


//         // cout<<"******************* relevant_turn_index  = "<<relevant_turn_index<<" ************"<<endl;
//        	distance_to_upcomming_turn = distance(current_position[0], current_position[1], waypt_turn_point[0].at(relevant_turn_index), waypt_turn_point[1].at(relevant_turn_index));
// 		// cout<<"distance_to_upcomming_turn = "<<distance_to_upcomming_turn<<endl;
// 		// cout<<"prev_distance = "<<prev_distance<<endl;

// 		if(distance_to_upcomming_turn >= 0 && distance_to_upcomming_turn <= switch_on_indicator_distance)
// 		{	
// 			if(waypt_turn_direction.at(relevant_turn_index) == "left")
// 			{		
// 				ss.str("");
// 				ss << "left"<<" "<<relevant_turn_index;//<<" "<<distance_to_upcomming_turn;
// 			}

// 			else if(waypt_turn_direction.at(relevant_turn_index) == "right")
// 			{
// 				ss.str("");
// 				ss << "right"<<" "<<relevant_turn_index;//<<" "<<distance_to_upcomming_turn;
// 			}
// 		}

// 			else
// 			{
// 				ss.str("");
// 				ss << "straight"<<" "<<relevant_turn_index;//<<" "<<distance_to_upcomming_turn;
// 			}
		
// 		temp.data = ss.str();
// 		waypoint_turn_pub.publish(temp);
		
// 		if(/*prev_distance < */distance_to_upcomming_turn < 5.0 && relevant_turn_index < turn_count-1 )
// 		{
// 			relevant_turn_index++;									//switch to next turn
// 			// prev_distance = distance(prev_position[0], prev_position[1], waypt_turn_point[0].at(relevant_turn_index), waypt_turn_point[1].at(relevant_turn_index)); 										//
// 			// cout<<"***** next prev_distance = "<<prev_distance<<endl;
// 		}
// 			// else
// 			// {
// 	    		// prev_distance = distance_to_upcomming_turn;			//update prev_distance
// 	    		// cout<<"updated prev_distance = "<<prev_distance<<endl;
// 	    	// }

// 		if(distance(prev_position[0], prev_position[1], current_position[0], current_position[1]) > 2)				//updating previous position at every 5m distance
// 		{			
// 			prev_position[0] = current_position[0];
// 	        prev_position[1] = current_position[1];
//     	}
// 	}

// 	// else
// 	// {
// 	// 	loop_iterator = waypt_turn_point[0].size()-turn_count;	
// 	// }

// 	else
// 	{
// 	cout<<"------------NO GPS FIX\n";
// 	}
// }


void goal_heading_callback(const std_msgs::Float64::ConstPtr& heading_msg)	// reads heading_msg and assigns it to a global variable named heading 
{																			
	heading = heading_msg->data;											// HEADINING IS SIMPLY AN ANGLE IN 0 TO 360 DEGREES.
	/*
	shift angle to accomodate for change in axis.							
	FROM GPS to occupancy grid
	*/
	heading = angle_wrap(heading);					// changing the angle according to our frame of reference

	// heading = 45;
}
//int count_lanes_callback=0;
/*void lanes_callback(const pixhwk_ctrl_node::Lanes::ConstPtr& lanes_msg)
{
	// cout<<"lanes_callback\n";
	count_lanes_callback++;
	if(count_lanes_callback<4)
	{
		// count_lanes_callback =0;
		return;
	}
	else
		count_lanes_callback=0;
	left_lane_points[0].clear();
	right_lane_points[0].clear();
	left_lane_points[1].clear();
	right_lane_points[1].clear();
	double x,y;
	for(int i=0; i<lanes_msg->lane_left_points.size(); i++)
	{
		x = lanes_msg->lane_left_points[i].x*scale_factor;
		y = lanes_msg->lane_left_points[i].y*scale_factor;
		x = x + (m_size+1)/2;
		y = y + (m_size+1)/2;
		left_lane_points[0].push_back(x);
		left_lane_points[1].push_back(y);
	}
	for(int i=0; i<lanes_msg->lane_right_points.size(); i++)
	{	
		x = lanes_msg->lane_right_points[i].x*scale_factor;
		y = lanes_msg->lane_right_points[i].y*scale_factor;
		x = x + (m_size+1)/2;
		y = y + (m_size+1)/2;
		right_lane_points[0].push_back(x);
		right_lane_points[1].push_back(y);
	}


	cout<<"lanes Left size:"<<left_lane_points[0].size()<<'\n';
	cout<<"Lanes Right size:"<<right_lane_points[0].size()<<'\n';

}*/


void get_goal()
{
	// cout<<"---------------heading : "<<heading<<"\n";

	/*****************************************************************************************************************************
	******************************************************************************************************************************						  
								  				 			[FRONT]
																(0)
													[home_x, m_size]
																|
																|
																|
																|
																|
																|																 _
 _																|																|R|
|L|																|																|I|
|E|		(90)	[m_size, home_y]-------------------------[home_x, home_y]----------------------------[0, home_y]	(-90) 		|G|
|F|																|																|H|
|T|																|																|T|
																|
																|
																|
																|
																|
																|
																|
														[home_x, 0]
																(180,-180)
	*******************************************************************************************************************************
	******************************************************************************************************************************/
	double min_heading_diff = 10000;					//some arbitary large values 				
	double final_local_heading = 0.0;
	double local_heading;
	double local_distance;
	int a,b,q,r;							//loop variables (ignore)
	double max_dist = 0;
	
	for(a = 0; a < m_size; ++a)									   
	{	
		for (b = 0; b < m_size; ++b)				//select goal only in first two quadrants
		{
			local_heading = rad2deg(atan2(a - home_x, b - home_y));			// CALCULATING HEADING AND DIST FOR EACH POINT IN OUR LOCAL COORDINATE SYSTEM
			local_distance = sqrt((a - home_x)*(a - home_x) + (b - home_y)*(b - home_y));
			// if(local_distance > max_dist)
			// 	max_dist = local_distance;
			
			if(Matrix[a][b]==0)							//if cell is free
			{
				int goal_padding = 4;							//this padding accounts for the padding around the free cell
				int flag_goal = 0;						//This flag is appropriated to check if a point in the matrix is obstacle or not
				for(q= -goal_padding; q <= goal_padding; q++)
           			for(r= -goal_padding; r <= goal_padding; r++)
               			{
               				if(q+a >=0 && q+a < m_size-1 && r+a>=0 && r+a< m_size-1)					//limiting the "x plus pad_around_goal" value our coordinate space i.e. withing the range of 0 to 100
                    		{
                    			if(q+b >=0 && q+b < m_size-1 && r+b>=0 && r+b< m_size-1)				//limiting the "y plus pad_around_goal" value our coordinate space i.e. withing the range of 0 to 100
                    			{
                    				if( Matrix[q+a][r+b] )					
                    				{
                    					flag_goal=1;							// this indicates that there is an obstacle at this point i.e. aroun Matrix[a][b]
                    				}
                    			}
                    		}
                    	}

				if(!flag_goal)												//if no obstacle is found 
					if(fabs(local_heading - heading)<= min_heading_diff)	// fabs calculates the absolute value 	
					{	
						if(local_distance >= min_distance_for_goal && local_distance <= max_distance_goal)	// checking if the local_distance is within our specified constraints 
						{	
							if(get_goal_quadrant(heading) == get_local_quadrant(local_heading))	//checking if the both the calculated heading and the desired heading is in same quadrant	
							{
								min_heading_diff = fabs(local_heading - heading);		// taking each point in the matrix and finding..
								goal_x = a;												// ..the one that alligns with our desired heading..
								goal_y = b;												// ..by reducing the error b/w desired heading and..
								final_local_heading = local_heading;					// ..local heading of the point
							}
						}
					}
			}
		}
	}
	// cout<<"max local distance"<<max_dist<< endl;
	cout<<"---------------local heading : "<<final_local_heading<<"\n";
	// if(min_heading_diff > max_dev)
	// {
	// 	cout<<"++++++MIN heading : "<<min_heading_diff<<"\n";
	// 	//if cannot find heading just move forward
	// 	goal_x = home_x;
	// 	goal_y = home_y + 5;
	// }
}

void obstacle_callback(const pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr& msg)
{
	loop_counter++;
	if(loop_counter%1 == 0)				//mod some number to skip processing of some frame 
	{	
		loop_counter = 0;
		
		//build occupancy GRID 					
		std::vector<signed char> v((m_size)*(m_size),0);		// make v((m_size)*(m_size), 0)  	// A vector which will be used to store and processing points for occupancy grid visualizationm,
		pcl::PointXYZRGBL p;//, oldp;
		int i, j, m, l, a, b;
		float maxObstDist = 0;
													//
		for(i = 0; i < msg->width; i++)
		{	
			p = msg->points[i];
		//	oldp = p;
			//
		


			// scaling the coordinates
			// p.x = scale_factor*p.x;					//kind of like zooming in 
			// p.y = scale_factor*p.y;					// need a little clartiy here about the resolution
			// p.z = scale_factor*p.z;

			//int x = ((grid_dim_/2)+scan->points[i].x/m_per_cell_);
    		//int y = ((grid_dim_/2)+scan->points[i].y/m_per_cell_);
            

			//translation							//bringing all the points in frame of reference of car located at (50,50)
			m = p.x/m_per_cell_ + home_x;
			l = p.y/m_per_cell_ + home_y;
			
			//filling up the obstacle matrix
			if((l >= padding && m >= padding) && (l < (m_size - padding) && m < (m_size - padding)))
			{
				// float obstDist = sqrt( pow(p.x,2) + pow(p.y,2) + pow(p.z,2) ); 
			// if(obstDist > maxObstDist)
			// maxObstDist = obstDist;
			// cout << "the old coordinates are " << oldp.x << " " << oldp.y << " " << oldp.z << endl;
			// cout << "the coordinates are " << p.x << " " << p.y << " " << p.z << endl;
				for (a = (-padding); a <= padding; ++a)
				{
					for (b = (-padding); b <= padding; ++b)
					{
						if (l != home_y && m != home_x)
						{
							v[(l+a)*m_size + (m+b)] = 100;			//DECLARING THE VALUE OF POINTS THAT IT GETS FROM POINTCLOUD AND PADDING AROUND IT AS 100
						}								 			//saves it in row major format in Vector v
						v[(home_x+a)*m_size + (home_y+b)] = 50;		//home location and padding around it is color coded 50
					}
				}
			}
		}

		// cout << "The max obstacle distance is " << maxObstDist << endl;

		// Checks whether the points lie whitin the lane or not±
		// int flag_obs = 0;
		// Point test;
		// if(!(left_lane_points[0].size()>2) || !(right_lane_points[0].size()>2))
		// 	flag_obs=1;
		// for (int a = 0; a < m_size; ++a)
		// {	
		// 	if(flag_obs == 1)
		// 		break;
		// 	for (int b = 0; b < m_size; ++b)
		// 	{
		//         t±est.x = a;
		//         test.y = b;
		// 		if (!is_inside_lane(test))
		// 		{
  // 		// @d - writing in terms of the 2d matrix with row (l+a)
		// 			v[a*m_size + b] = 100;
		// 		}
		// 	}
		// }
		// for(int q=0;q<left_lane_points[0].size();q++)
		// {
		// 	if(left_lane_points[0][q] >=0 && left_lane_points[0][q] < m_size-1)
		// 		if(left_lane_points[1][q] >=0 && left_lane_points[1][q] < m_size-1)
		// 			v[left_lane_points[0][q]*m_size + left_lane_points[1][q]] = 50;
		// }
		// for(int q=0;q<right_lane_points[0].size();q++)
		// {
		// 	if(right_lane_points[0][q] >=0 && right_lane_points[0][q] < m_size-1)
		// 		if(right_lane_points[1][q] >=0 && right_lane_points[1][q] < m_size-1)
		// 			v[right_lane_points[0][q]*m_size + right_lane_points[1][q]] = 70;
		// }


		//CREATING MATRIX IN BINARY {OBSTACLE =1, FREE SPACE = 0}
		for (i = 0; i < m_size; ++i)
		{
			for (j = 0; j < m_size; ++j)
			{
				if (v[i*m_size + j] == 100)			//from the vector file constructed we check which values are 100 i.e. occupied
				{
					Matrix[i][j] = 1;//occupied
				}
				else
				{
					Matrix[i][j] = 0;//free
				}
			}
		}

		int car_allowance = 25;
		/*
		Block MATRIX area(Occupancy grid) outside the cone of interest based on heading
		*/
		for (i = 0; i < m_size; ++i)
		{
			for (j = 0; j < m_size; ++j)
			{
				double local_heading = rad2deg(atan2(i - home_x, j - home_y));						
				if( (local_heading >= heading-max_dev) && (local_heading < heading + max_dev))
				{
					//inside cone
				}
				else
				{
					if(i>=(home_x - car_allowance) && i<=(home_x + car_allowance))				
					{															//done to ensure that the cone takes the car and surroundings into account
						if(j>(home_y - car_allowance) && j<=(home_y + car_allowance))			//REGION AROUND THE CAR(OR THE CENTER OF OUR GRID I,E. AROUND 50,50)
						{
							continue;
						}
					}
					//outside cone
					Matrix[i][j] = 1;//occupied				outside the cone is blocked 
					if(v[i*m_size + j]!=100)
						v[i*m_size + j] = 50;				//outside cone(visual only) things other than obstacles will be grey

				}
			
			}
		}



		//Deciding the local goal on the occupancy grid with respect to the velodyne map
		get_goal();	
		//ROS_INFO("Home(x,y): %d %d",home_x,home_y);
		//ROS_INFO("Goal(x,y): %d %d",goal_x,goal_y);
		
		// for visualising home postion, lidar postion and goal position on rviz
		v[home_x*m_size + home_y] = 10;
		v[(lidar_x)*m_size + (lidar_y)] = 30;
		v[goal_x*m_size + goal_y] = 90;
		
		
		//creating the text file with goal and home coordinates
		FILE *g = fopen(goal_file.c_str(), "w");								//enter path to goal file here

		if (g == NULL)	
		{
		    printf("Error opening goal file!\n");
		    exit(1);
		}
		fprintf(g, "%d %d\n", home_x,home_y);
		fprintf(g, "%d %d", goal_x,goal_y);
		fclose(g);
		cout<<"Goal :"<<goal_x<<' '<<goal_y<<'\n';

		//Creating the text file from occupancy matrix

		//path for my pc /home/pratique/Swarath_Repo/
		// FILE *mf = fopen(data_collection_file.c_str(), "a");
		// if (mf == NULL)
		// {
		//     printf("Error opening data_collection_file file!\n");
		//     exit(1);
		// }

		FILE *f = fopen(matrix_file.c_str(), "w");														//enter path to matrix file here
		if (f == NULL)
		{
		    printf("Error opening matrix file!\n");
		    exit(1);
		}
		fprintf(f, "%d %d\n", m_size,m_size);

		// fprintf(mf, "%d %d\n", m_size,m_size);
		// fprintf(mf, "start = %d %d\n", home_x,home_y);
		// fprintf(mf, "goal = %d %d\n", goal_x,goal_y);
		// fprintf(mf, "omplstatus=%d \t%f\n", omplstatus,timestamp);

		
		for(a = 0; a < (m_size); a++)
		{
			for (b = 0; b < (m_size); ++b)
			{
				fprintf(f, "%d ", Matrix[a][b]);
				//fprintf(mf, "%d ", Matrix[a][b]);
			}
			
			if(a != m_size-1)
			{
				fprintf(f, "\n");
				//fprintf(mf, "\n");
			}
		}
		fclose(f);
		//fclose(mf);

		// Launching the Ompl script
		if(reached_goall == 0)
		{	
			system(ompl_scipt.c_str());																	//enter path to ompl script here
		}
		else	
		{	
			FILE *f = fopen(trajectory_file.c_str(), "w");												//enter path to trajectory file here
			if (f == NULL)
			{
			    printf("Error opening trajectory_ompl file!\n");
			    exit(1);
			}
			fprintf(f, "0 0\n");								//
			fclose(f);
		}

		// detect_turn_waypoint(waypoints_file);
		// detect_turn(trajectory_file);

		int  q, r, turn_flag, trajectory_points_padding = 3;
		double InputX,InputY;
		ifstream iFile;
		iFile.open(trajectory_file.c_str());															//enter path to trajectory file here
		double ang;
		// cout<<"turn count = "<<turn_count<<endl;
		while((iFile >> InputX) && (iFile >> InputY))					//reading points
		  {	
		  	// turn_flag = 0;
		  	a = int(InputX);
		  	b = int(InputY);
	  // 		cout<<"INPUT X = "<<a<<endl;
			// cout<<"INPUT Y = "<<b<<endl;
		  	
			    for(q= -trajectory_points_padding; q <= trajectory_points_padding; q++)
	       		for(r= -trajectory_points_padding; r <= trajectory_points_padding; r++)
	   			{
	   				if( (q+a)>=0 && (r+a)< m_size-1)					
	        		{
	        			if(( q+b)>=0 && (r+b)< m_size-1)				
						{  
							v[(q+a)*(m_size) + (r+b)] = 70;					// color coding point to 70 for rviz
						}
					}
				}
			}
		  // }
		iFile.close();
		
		map123.data = v;
		map_pub.publish(map123);
	}
}

// void OmplStatus_callback(const pixhwk_ctrl_node::OmplStatus::ConstPtr& omplmsg)
// {
// 	ros::Time t;
// 	omplstatus = omplmsg->OmplSolStatus;
// 	t = omplmsg->stamp;
// 	timestamp = t.sec;
	
// }

int main(int argc, char** argv)
{
	ros::init(argc, argv, "matrix_node");
	ros::NodeHandle n;
	ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
 
	if (argc < 5)
	{
		cout << "Enter the following arguments ..\n"
            "1) padding (3)\n"
            "2) max deviation from heading (50)\n"
            "3) min distance for goal (150)\n"
            "4) max distance for goal (200)\n"
            //"5) name of waypoints file \n"
            "\n";
        return -1;
	}

  padding = atoi(argv[1]);
  max_dev = atof(argv[2]);
  min_distance_for_goal = atoi(argv[3]);
  max_distance_goal =  atoi(argv[4]);
  //waypoints_file =  string(argv[5]);
  

  map123.info.resolution = 1;			//default =1								//resolution of map....m/cell | google "nav_msgs/MapMetaData" for more information
  map123.info.width = m_size;								//default m_size									
  map123.info.height = m_size;											

  map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_d", 1);	
  // waypoint_turn_pub = n.advertise<std_msgs::String>("/waypoint_turn_direction",1);									//
  //ros::Subscriber gps_sub = n.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1000, gps_callback, noDelay);
  ros::Subscriber heading_sub = n.subscribe("/car_heading",1,goal_heading_callback);					//PLANNER.CPP NODE IS PUBLISHING TO /car_heading
  ros::Subscriber obstacle_sub = n.subscribe("/velodyne_obstacles_d",1,obstacle_callback);			//change back to("/velodyne_obstacles",1,obstacle_callback);
  // ros::Subscriber traffic_light_sub= n.subscribe<pixhwk_ctrl_node::traffic_light>("/traffic_light_data", 1, traffic_light_callback); 


  // ros::Subscriber lanes_sub = n.subscribe<pixhwk_ctrl_node::Lanes>("/lanes",1,lanes_callback);
  //ros::Subscriber OmplStatus_sub = n.subscribe("/OmplStatus",5,OmplStatus_callback);

  while (ros::ok())
  {
  	ros::spin();
  }

  return 0;
}
