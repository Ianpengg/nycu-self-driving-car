#include <deque>
#include <iostream>
#include <stdlib.h>

using namespace std;

class Object
{
public:
  string object_id;  //store the object_id
  float center_x, center_y, new_x, new_y;
  deque <float> location_x; //store the x position of object
  deque <float> location_y; // store the yposition of object
  
	//update the past position of tracked cars.
  void update(float displacement, float yaw_change, double x, double y);
  

};

void Object::update(float displacement, float yaw_change, double x, double y) {
    /*
		NOTE:
		- displacement -> use present frame's center - previous frame's center calculate Euclidean distance.
		*/
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
  while (location_x.size() >=30) {
		location_x.pop_front();
		location_y.pop_front();
	}
};