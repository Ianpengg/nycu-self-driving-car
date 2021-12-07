// visualize track result from output 
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <json/json.h>
#include <ros/ros.h>
#include <fstream>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <ros/package.h>
//read all .ply in files
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <algorithm>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>

using namespace std;
string log_path, label_path, det_path;
int play_rate = 10;
typedef pcl::PointXYZI PointT;

//Ros related things
ros::Publisher pub_id, pub_lidar, pub_label, pub_det, pub_log, pub_ego, pub_traj;
visualization_msgs::MarkerArray M_id, M_label, M_det, M_ego, M_traj;
int det_max_size = 0, label_max_size = 0;
int frame_count = 0;

/*declare the id_type and a vector to record the id*/
string id_type;
vector <string> id_record;
vector <int> label_id;
class Object
{
public:
  string object_id;  //store the object_id
  float center_x, center_y, new_x, new_y;
  deque <float> location_x; //store the x position of object
  deque <float> location_y; // store the yposition of object
  
  //void readdata();
  void update(float displacement, float yaw_change, double x, double y);
  //void load_data(float x, float y);

};

void Object::update(float displacement, float yaw_change, double x, double y) {
     
  for (int i =0; i<location_x.size(); i++) {
		//printf("i=%d\n", i);
    center_x = location_x[i];
    center_y = location_y[i];
    //cout << center_x << " " << center_y << "i=" << i << endl;
    new_x = center_x * cos(yaw_change) + center_y * sin(yaw_change) -displacement;
    new_y = -center_x * sin(yaw_change) + center_y * cos(yaw_change);
    location_x[i] = new_x;
    location_y[i] = new_y;
    //cout << "update" << location_x[i] << " " << location_y[i] << "i=" << i << endl;    
  }
	if (x != 9999 && y != 9999) {
  	location_x.push_back(x);
  	location_y.push_back(y);
	}
  while (location_x.size() >=10) {
		location_x.pop_front();
		location_y.pop_front();
	}
};
class Object_ego
{
public:
  string object_id;  //store the object_id
  float center_x, center_y, new_x, new_y;
  deque <float> location_x; //store the x position of object
  deque <float> location_y; // store the yposition of object
  
  //void readdata();
  void update(float displacement, float yaw_change);
  //void load_data(float x, float y);

};

void Object_ego::update(float displacement, float yaw_change) {
     
  for (int i =0; i<location_x.size(); i++) {
		//printf("i=%d\n", i);
    center_x = location_x[i];
    center_y = location_y[i];
    //cout << center_x << " " << center_y << "i=" << i << endl;
    new_x = center_x * cos(yaw_change) + center_y * sin(yaw_change) -displacement;
    new_y = -center_x * sin(yaw_change) + center_y * cos(yaw_change);
    location_x[i] = new_x;
    location_y[i] = new_y;
    //cout << "update" << location_x[i] << " " << location_y[i] << "i=" << i << endl;    
  }
  location_x.push_back(0);
  location_y.push_back(0);
  while (location_x.size() >=30) {
		location_x.pop_front();
		location_y.pop_front();
	}
};
// void Object::load_data(float x, float y) {
//   location_x.push_back(x);
//   location_y.push_back(y);
// }

double distance(double x, double y, double z, double prev_x, double prev_y, double prev_z) {
	return sqrt(pow((x - prev_x), 2) + pow((y - prev_y), 2) + pow((z - prev_z), 2));
}


double euler_from_quaternion(double w, double x, double y, double z) { // convert quaternion to rpy angle
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y * y + z * z);
    double yaw_z = atan2(t3, t4);
    return yaw_z; // in radians
}


deque<Object> Tracker;






void showAllFiles( const char * dir_name, int shit, vector<string>& filenames)
{
	// check the parameter !
	if( NULL == dir_name )
	{
		ROS_WARN("Argo Data directory path is null !");
		return;
	}
 
	// check if dir_name is a valid dir
	struct stat s;
	lstat( dir_name , &s );
	if( ! S_ISDIR( s.st_mode ) )
	{
		ROS_WARN("Argo Data directory path is not a valid directory !");
		return;
	}
	
	struct dirent * filename;    // return value for readdir()
 	DIR * dir;                   // return value for opendir()
	dir = opendir( dir_name );
	if( NULL == dir )
	{
		ROS_WARN("Can not open dir %s",dir_name);
		return;
	}
	ROS_INFO("Successfully opened the dir !");
	
	/* read all the files in the dir ~ */
	while( ( filename = readdir(dir) ) != NULL )
	{
		// get rid of "." and ".."
		if( strcmp( filename->d_name , "." ) == 0 || 
			strcmp( filename->d_name , "..") == 0 ||
            strcmp( filename->d_name , ".ipynb_checkpoints") == 0 )
			continue;
		// cout<<filename ->d_name <<endl;
        
        if( shit==0 )
            filenames.push_back(filename->d_name);
        // else if ( shit==1 )
        //     stereos_l.push_back(filename->d_name);
        // else
        //     stereos_r.push_back(filename->d_name);
        
    }   
}

void sort_timestamp(vector<string> &filenames, vector<unsigned long int> &sorted_names){
	int i=0;

    //sorting lidar
	for(i; i<filenames.size(); i++){
		string str = filenames.at(i).substr(3, 18);//star at 3th index and read 18 char(timestamp in file name)
		istringstream is(str);
		unsigned long int ts;
		is >> ts;
		sorted_names.push_back(ts);   //insert from last one element
		//cout << sorted_plys.at(i) <<endl;
	}
	//sort(begin of array, end of array, sort_type(default is ascending))
	sort(sorted_names.begin(), sorted_names.end());
    // cout<<"\033[1;33mAfter sorting:\n\033[0m";
	// for(i=0;i<sorted_names.size();i++)
    //     cout<< i << " " << sorted_names[i]<<"\n";
    // cout << "get out of function" << endl;
    return; 
}


/* function template ex : template<class T>
T abs(T x) {
	return(x>0)?x:-x;
}
int a;float b; complex c(-5,-2) all this can use abs(a), abs(b), abs(c)
complier would auto fill in the type of data
*/
template<class T>
string convertToString(T value){
	stringstream ss;
	ss << value;
	return ss.str();
}


void initialize_marker(Json::Value label_array, Json::Value det_array, double displacement, double yaw_change){
	visualization_msgs::Marker marker, marker_id, marker_det;
	double timestamp = det_array[0]["timestamp"].asDouble(); //read detection.json timestamp only read first one
	//checking id
	deque<string> track_id;
	if ( ! id_type.compare("uuid") ){
		if (id_record.size() == 0){
			Object track_car[label_array.size()];
			
			for (int k =0; k < label_array.size(); k++) {
				string track_label_uuid = label_array[k]["track_label_uuid"].asString(); //access the value inside the "track_label_uuid"
				id_record.push_back(track_label_uuid);  //its function is like a python list append to last element
				label_id.push_back(k);
				double center_x = label_array[k]["center"]["x"].asDouble();
				double center_y = label_array[k]["center"]["y"].asDouble();
				
				
				track_car[k].object_id = track_label_uuid;
				track_car[k].location_x.push_back(center_x);
				track_car[k].location_y.push_back(center_y);

				Tracker.push_back(track_car[k]);
				
			}
		}
		else {
			for (int k = 0; k < label_array.size(); k++) {
				string track_label_uuid = label_array[k]["track_label_uuid"].asString();
				track_id.push_back(track_label_uuid);
				//cout << "Track_id" << track_id.size() << endl;
				vector<string>::iterator find_id = find(id_record.begin(), id_record.end(),track_label_uuid); //use find to find inside the STL container if find the element then its address stored in find_id
				if (find_id == id_record.end()) {// if find_id is the last element , vector needs to add new elements
					id_record.push_back(track_label_uuid);
					Object new_track;
					new_track.object_id = track_label_uuid;
					new_track.location_x.push_back(label_array[k]["center"]["x"].asDouble());
					new_track.location_y.push_back(label_array[k]["center"]["y"].asDouble());
					Tracker.push_back(new_track);
				}
				else if (find_id != id_record.end()) {
					Tracker[find_id-id_record.begin()].update(displacement, yaw_change, label_array[k]["center"]["x"].asDouble(), label_array[k]["center"]["y"].asDouble());
				
				
					
				}
				// Debugg display



			}
			// for (int j = 0; j< Tracker.size();j++) {
			// 	cout << "Tracker_id" << Tracker[j].object_id << "   "<< "ID_record=" << id_record[j] << endl;
			// //cout << "the tracker has tracked" << Tracker.size() << "Object" << endl;
			// }
			// cout << "stop" << endl;
			for (int i = 0; i < Tracker.size(); i++) {
				string stored_id = Tracker[i].object_id;
				deque<string>::iterator find_stored = find(track_id.begin(),track_id.end(),stored_id);
				if (find_stored == track_id.end() ) {
					//cout << "lost track" << endl;
					Tracker[i].update(displacement, yaw_change, 9999,9999);
				}
			}

		}
		cout << "IDrecord =" << id_record.size() << endl;	
		cout << "the tracker has tracked" << Tracker.size() << "Object" << endl;
	  //Publish the past trajectory
		
		int count = 0;
		for (int m=0; m<label_array.size(); m++){
			visualization_msgs::Marker marker_obj_traj;
			count++;
			marker_obj_traj.header.frame_id="scan";
			marker_obj_traj.header.stamp = ros::Time::now();//to show every tag  (ros::Time::now()for latest tag)
			marker_obj_traj.action = visualization_msgs::Marker::ADD;
			marker_obj_traj.lifetime = ros::Duration(0.2);
			//marker_obj_traj.ns = label_array[m]["track_label_uuid"].asString();
			marker_obj_traj.type = visualization_msgs::Marker::LINE_STRIP;
			//marker_obj_traj.pose.orientation.w = 1.0;
			marker_obj_traj.id = count;
			marker_obj_traj.color.b = 0.5f;
			marker_obj_traj.color.g = m / label_array.size(); //1.0f
			marker_obj_traj.color.r = 1.0f;
			marker_obj_traj.color.a = 0.7f;
			marker_obj_traj.scale.x = 0.2;
			marker_obj_traj.points.clear();
			geometry_msgs::Point p;
			for (int i = 0; i < Tracker.size(); i++) {
				string stored_id_pub = Tracker[i].object_id;
				if (stored_id_pub == label_array[m]["track_label_uuid"].asString()) {
					//cout <<"Size" << Tracker[i].location_x.size() << endl;
					for (int j = 0; j<Tracker[i].location_x.size(); j++) {
						//cout << "X=" << Tracker[i].location_x[j] << "Y=" << Tracker[i].location_y[j] << endl;
						p.x = Tracker[i].location_x[j];
						p.y = Tracker[i].location_y[j];
						p.z = 0;
						marker_obj_traj.points.push_back(p);
						
				  }
					M_traj.markers.push_back(marker_obj_traj);

				}
				// deque<string>::iterator find_stored_pub = find(track_id.begin(),track_id.end(),stored_id_pub);
				// if (find_stored_pub != track_id.end() ) {
				// }
			}
			
		}	
		pub_traj.publish(M_traj);
	
		// }
	
	
	
	
	
	
	
	}
	// detection
	int counter = 0;
	for (int m=0; m<det_array.size(); m++){
		//cout << "m is" << m << endl;
		float score = det_array[m]["score"].asDouble();
		if (score < 0.3)
			continue;
		counter ++;
		marker_det.header.frame_id = "scan";
		marker_det.header.stamp = ros::Time::now();
		marker_det.action = visualization_msgs::Marker::ADD;
    marker_det.lifetime = ros::Duration(1/(play_rate+10));
		marker_det.ns = "detection";
		marker_det.type = visualization_msgs::Marker::CUBE;
		
		// marker_det.id = m + 200;
		marker_det.id = counter - 1;
		
		marker_det.pose.position.x = det_array[m]["center"]["x"].asDouble();
		marker_det.pose.position.y = det_array[m]["center"]["y"].asDouble();
		marker_det.pose.position.z = det_array[m]["center"]["z"].asDouble();

		marker_det.pose.orientation.x = det_array[m]["rotation"]["x"].asDouble();
		marker_det.pose.orientation.y = det_array[m]["rotation"]["y"].asDouble();
		marker_det.pose.orientation.z = det_array[m]["rotation"]["z"].asDouble();
		marker_det.pose.orientation.w = det_array[m]["rotation"]["w"].asDouble();
		
		marker_det.scale.x = det_array[m]["length"].asDouble();
		marker_det.scale.y = det_array[m]["width"].asDouble();
		marker_det.scale.z = det_array[m]["height"].asDouble();
		
		marker_det.color.b = 0.0f;
		marker_det.color.g = 0.0f; //1.0f
		marker_det.color.r = 0.0f;
		marker_det.color.a = 0.8f;

		string det_class = det_array[m]["label_class"].asString();
		if(  det_class == "VEHICLE"){
            marker_det.color.b = 1.0f;
		}
		else if ( det_class == "PEDESTRIAN"){	
			marker_det.color.g = 1.0f;
			marker_det.color.b = 1.0f;
		}
		else
			ROS_INFO("We have undefined class %s", det_class.c_str());
		
		M_det.markers.push_back(marker_det);		
	}
	cout << "We have " << counter << " detections \tat " << timestamp << endl;
	

	for (int i=0; i<label_array.size(); i++){
		std::pair<int, string> track_label_uuid;
		if (! id_type.compare("uuid")){
			track_label_uuid.second = label_array[i]["track_label_uuid"].asString();
		}
		else{
			track_label_uuid.first = label_array[i]["track_label_uuid"].asInt();
			cout << "test" << endl;
		}

		marker.header.frame_id = "scan";
		marker.header.stamp = ros::Time::now();
		marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(1/(play_rate+10));
		marker.ns = "result";
		marker.type = visualization_msgs::Marker::CUBE;
		
		marker.id = i;
		
		marker.pose.position.x = label_array[i]["center"]["x"].asDouble();
		marker.pose.position.y = label_array[i]["center"]["y"].asDouble();
		marker.pose.position.z = label_array[i]["center"]["z"].asDouble();

		marker.pose.orientation.x = label_array[i]["rotation"]["x"].asDouble();
		marker.pose.orientation.y = label_array[i]["rotation"]["y"].asDouble();
		marker.pose.orientation.z = label_array[i]["rotation"]["z"].asDouble();
		marker.pose.orientation.w = label_array[i]["rotation"]["w"].asDouble();
		
		marker.scale.x = label_array[i]["length"].asDouble();
		marker.scale.y = label_array[i]["width"].asDouble();
		marker.scale.z = label_array[i]["height"].asDouble();
		
		marker.color.b = 0.0f;
		marker.color.g = 0.0f; //1.0f
		marker.color.r = 0.0f;
		marker.color.a = 0.8f;

		string track_class = label_array[i]["label_class"].asString();
		if(  track_class == "VEHICLE")
            marker.color.r = 1.0f;
		else if ( track_class == "PEDESTRIAN")
			marker.color.g = 1.0f;
		else
			ROS_INFO("We have undefined class %s", track_class.c_str());
	
		
		M_label.markers.push_back(marker);

		
		// id
		marker_id.header.frame_id="scan";
		marker_id.header.stamp = ros::Time::now();//to show every tag  (ros::Time::now()for latest tag)
		marker_id.action = visualization_msgs::Marker::ADD;
		marker_id.lifetime = ros::Duration(0.2);
		marker_id.ns = "tracking_id";
		marker_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker_id.pose.orientation.w = 1.0;
		// marker_id.id = i + 100;
		marker_id.id = i+1000;
		//cout << i << endl;
		marker_id.scale.z = 1.5f;
		marker_id.color.b = 0.0f;
		marker_id.color.g = 0.9f;
		marker_id.color.r = 0.9f;
		marker_id.color.a = 1.0f;

		marker_id.pose.position.x = label_array[i]["center"]["x"].asDouble();
		marker_id.pose.position.y = label_array[i]["center"]["y"].asDouble();
		marker_id.pose.position.z = label_array[i]["center"]["z"].asDouble() + 1.0f;
		//marker_id.text = "aaa";
		if(! id_type.compare("uuid")){
			for(int m=0; m<id_record.size(); m++){
				if ( !( (track_label_uuid.second).compare(id_record.at(m))) ) {
					marker_id.text = convertToString(m);
					cout << marker_id.text << endl;
				}
			}
		}
		else{
			cout << "wrong" << endl;
			marker_id.text = convertToString(track_label_uuid.first);
		}

		M_id.markers.push_back(marker_id);
	}
	cout << "We have " << M_id.markers.size() << " trackers \tat " << timestamp << endl;


	if (M_label.markers.size() > label_max_size)
		label_max_size = M_label.markers.size();
	for (int a = label_array.size(); a < label_max_size; a++)
	{
		marker.id = a;
		marker.color.a = 0;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.scale.z = 0;
		M_label.markers.push_back(marker);

		// marker_id.id = a+100;
		marker_id.id = a;
		marker_id.color.a = 0;
		marker_id.pose.position.x = 0;
		marker_id.pose.position.y = 0;
		marker_id.pose.position.z = 0;
		marker_id.scale.z = 0;
		M_id.markers.push_back(marker_id);
	}

	if (M_det.markers.size() > det_max_size)
		det_max_size = M_det.markers.size();
	for (int a = M_det.markers.size(); a < det_max_size; a++)
	{
		// marker_det.id = a + 200;
		marker_det.id = a;
		
		marker_det.color.a = 0;
		marker_det.pose.position.x = 0;
		marker_det.pose.position.y = 0;
		marker_det.pose.position.z = 0;
		marker_det.scale.z = 0;
		M_det.markers.push_back(marker_det);
	}

	pub_id.publish(M_id);
	pub_label.publish(M_label);
	pub_det.publish(M_det);
	//pub_ego.publish(M_ego);
	return;
}


void ego_traj(Json::Value pose_array, int i, double* displacement, 
double* yaw_change, double previous_pose[3], double* previous_yaw, Object_ego &ego_car) {

	visualization_msgs::Marker marker_traj;
	double w, x, y, z, p_x, p_y, p_z;
	w = pose_array["rotation"][0].asDouble();
	x = pose_array["rotation"][1].asDouble();
	y = pose_array["rotation"][2].asDouble();
	z = pose_array["rotation"][3].asDouble();

	double yaw = euler_from_quaternion(w, x, y, z);
	//printf("%f", yaw);
	p_x = pose_array["translation"][0].asDouble();
	p_y = pose_array["translation"][1].asDouble();
	p_z = pose_array["translation"][2].asDouble();
	//pose_array["rotation"][]
	if (i == 0) {
		previous_pose[0] = p_x;
		previous_pose[1] = p_y;
		previous_pose[2] = p_z;
		*previous_yaw = yaw;
		
	}
	else {
		*displacement = distance(p_x, p_y, p_z, previous_pose[0], previous_pose[1], previous_pose[2]);
		*yaw_change = yaw - *previous_yaw;
		//printf("yaw=%f", yaw);
	  //printf("%f",*yaw_change);
		ego_car.update(*displacement, *yaw_change);

	}
	previous_pose[0] = p_x;
	previous_pose[1] = p_y;
	previous_pose[2] = p_z;
	*previous_yaw = yaw;
	for (int m = 0; m < ego_car.location_x.size(); m++){
		marker_traj.header.frame_id="scan";
		marker_traj.header.stamp = ros::Time::now();//to show every tag  (ros::Time::now()for latest tag)
		marker_traj.action = visualization_msgs::Marker::ADD;
		marker_traj.lifetime = ros::Duration(1/(play_rate+10));
		//marker_traj.ns = "tracking_id";
		marker_traj.type = visualization_msgs::Marker::LINE_STRIP;
		//marker_traj.pose.orientation.w = 1.0;
		marker_traj.color.b = 0.0f;
		marker_traj.color.g = 0.0f; //1.0f
		marker_traj.color.r = 1.0f;
		marker_traj.color.a = 0.7f;
		marker_traj.scale.x = 0.2;
		geometry_msgs::Point p;
		p.x = ego_car.location_x[m];
		p.y = ego_car.location_y[m];
		p.z = 0;
		marker_traj.points.push_back(p);
		M_ego.markers.push_back(marker_traj);



	}
	pub_ego.publish(M_ego);
}

void MySigintHandler(int sig)
{
	//这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
	ROS_INFO("shutting down!");
	ros::shutdown();
}


int main(int argc, char** argv){
	ros::init(argc, argv, "viz_track_result");
	ros::NodeHandle nh("~");
  
	pub_lidar = nh.advertise<sensor_msgs::PointCloud2>("/scan", 1000);
	pub_label = nh.advertise<visualization_msgs::MarkerArray>("/tracking_result", 1000);
	pub_id	  = nh.advertise<visualization_msgs::MarkerArray>("/marker_id", 1000);
	pub_det	  = nh.advertise<visualization_msgs::MarkerArray>("/detection_result", 1000);
  pub_ego = nh.advertise<visualization_msgs::MarkerArray>("/ego_trajectory", 1000);
	pub_traj = nh.advertise<visualization_msgs::MarkerArray>("/object_trajectory", 1000);
	signal(SIGINT, MySigintHandler);

	if ( !nh.getParam("log_path", log_path) ){
		ROS_INFO("Please provide the log path.");
		return -1;
	}
	if ( !nh.getParam("label_path", label_path) ){
		ROS_INFO("Please provide the label path.");
		return -1;
	}
	if ( !nh.getParam("det_path", det_path) ){
		ROS_INFO("Please provide the detection path.");
		return -1;
	}
	
	nh.getParam("play_rate", play_rate);
	nh.getParam("id_type", id_type);

	// wait secs to open the Rviz
  double wait_rviz_sec;
	nh.getParam("rviz_wating_time", wait_rviz_sec);
	ROS_INFO("Wait %f sec for opening Rviz",wait_rviz_sec);
	ros::Duration(wait_rviz_sec).sleep();

	string lidar_string = log_path + "/lidar";
  char dir_lidar[200];
	strcpy(dir_lidar, lidar_string.c_str());

  vector <string> filenames; 
	vector <unsigned long int> sorted_filenames;
  showAllFiles( dir_lidar, 0, filenames);
  sort_timestamp(filenames, sorted_filenames);
	
	std::cout << std::fixed; std::cout << std::setprecision(0);

	ros::Rate r(play_rate);

	
	Object_ego ego_car; // ego_car
	double displacement, yaw_change=0, previous_pose[3], previous_yaw;
	

	for (int i=0; i<sorted_filenames.size(); i++){
    string lidar_time = convertToString(sorted_filenames.at(i));

		string ply_path = log_path + "/lidar/PC_" + lidar_time + ".ply";
		string label_log_path = label_path + "/per_sweep_annotations_amodal/tracked_object_labels_" + lidar_time + ".json";
		string detection_path = det_path + "/per_sweep_annotations_amodal/tracked_object_labels_" + lidar_time + ".json";
		string pose_path = log_path + "/poses/city_SE3_egovehicle_" + lidar_time + ".json";
		cout << pose_path << endl;
		pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
		if (pcl::io::loadPLYFile<PointT>(ply_path, *cloud) == -1){
            PCL_ERROR("Couldn't read ",lidar_time ," file.\n");
            cout << i << endl;
            continue;
        }
		
		Json::Reader reader_label, reader_det, read_pose;
		Json::Value label_array, det_array, pose_array;
		ifstream ifs1(label_log_path, ios::binary); //label_log store to >> label_array
		ifstream ifs2(detection_path, ios::binary); //detection store >> det_array
		ifstream ifs3(pose_path, ios::binary);
		if( !reader_label.parse(ifs1, label_array) ){
			cout<<"\033[1;34mFailed to parse "<< lidar_time <<" label.\033[0m"<<endl;
            continue;
		}
		if( !reader_det.parse(ifs2, det_array) ){
			cout<<"\033[1;34mFailed to parse "<< lidar_time <<" detection.\033[0m"<<endl;
            continue;
		}
		if( !reader_det.parse(ifs3, pose_array) ){
			cout<<"\033[1;34mFailed to parse "<< lidar_time <<" detection.\033[0m"<<endl;
            continue;
		}
		M_ego.markers.clear();
		M_traj.markers.clear();

		M_id.markers.clear();
		M_label.markers.shrink_to_fit();
		M_label.markers.clear();
		M_label.markers.shrink_to_fit();
		M_det.markers.clear();
		M_det.markers.shrink_to_fit();
		cout << "At frame "<<frame_count++<<endl;		
		ego_traj(pose_array,i,&displacement, &yaw_change, previous_pose, &previous_yaw, ego_car);
		initialize_marker(label_array, det_array, displacement, yaw_change);
		//cout.precision(6);
		//cout << previous_yaw << endl;

		sensor_msgs::PointCloud2 sensor_scan;
		pcl::toROSMsg(*cloud,sensor_scan);
		sensor_scan.header.frame_id = "scan";
		// cout << cloud->points.size() << endl;
		cout<<"We now at "<< lidar_time << "\nHave " << setw(5) << label_array.size() << " labels" << endl;
		cout << "---------------------------------------------------" << endl;
		pub_lidar.publish(sensor_scan);
		
		r.sleep();
	
	}
	
    return 0;
}