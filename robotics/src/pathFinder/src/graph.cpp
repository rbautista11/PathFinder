#include <ros/ros.h>
#include <vector>
#include <stack>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <fstream>
#include <cmath>


int width, height;
float resolution;

struct Node vertices[1677];
int node_counter = 0;

std::vector<Node*> O;
std::vector<Node*> C;

std::stack<Node*> path;

struct Node *start,*goal,*last;
int neg = 0;
float prev_angle = 0.0;

void setGoal(float x, float y){
	float x_diff, y_diff;
	for(int i = 0; i < node_counter; i++){
		x_diff = std::abs(vertices[i].x_origin - x);
		y_diff = std::abs(vertices[i].y_origin - y);
		if(x_diff < 0.8 && y_diff < 0.8)
			goal = &vertices[i];
		if(x_diff < 0.6 && y_diff < 0.6)
			goal = &vertices[i];
	}

}

bool atNodeX(float pose_x){
        struct Node *n  = path.top();
        float x_diff = std::abs(n->x_origin - pose_x);
 	ROS_INFO_STREAM("Pose x: " << pose_x << " Goal x: " << n->x_origin << " X difference: " << x_diff); 
        if(x_diff < 0.5){
                return true;
        }
        return false;
}

bool atNodeY(float pose_y){
        struct Node *n = path.top();
	float y_diff = std::abs(n->y_origin - pose_y);
	ROS_INFO_STREAM("Pose Y: " << pose_y << " Goal y: " << n->y_origin << " y difference: " << y_diff);
	 if(y_diff < 0.5)
                return true;
        return false;
}

void adjustOrientation(int option, float current_theta){
	float diff = 0; 
	int commands = 3;
	if(prev_angle <= current_theta){
		if(option == 2){
			diff = 3.14 - current_theta;
		}
		else if(option == 4){
			diff = (1.57 - current_theta) + 3.14;
		}
	}
	else {
		if(option == 2){
			diff = (3.14 - current_theta) + 3.14;
		}	
		if(option == 4){
		
		}
	}

	ros::NodeHandle nh11;
	ros::Publisher pub11 = nh11.advertise<geometry_msgs::Twist>("/cmd_vel",100);
	geometry_msgs::Twist m3;
	while(pub11.getNumSubscribers() == 0)
                ros::Duration(0.5).sleep();
        
        while(commands > 0){
                m3.angular.z = diff;
                pub11.publish(m3);
                ros::Duration(0.5).sleep();
                m3.angular.z = 0;
                pub11.publish(m3);
                ros::Duration(0.5).sleep();
                commands--;
        }
}

void goToNodeY(float pose_y, float theta){
        struct Node *n = path.top();

        ros::NodeHandle nh4;
        ros::Publisher pub1 = nh4.advertise<geometry_msgs::Twist>("/cmd_vel",100);
        geometry_msgs::Twist m1;
        while(pub1.getNumSubscribers() == 0)
                ros::Duration(0.5).sleep();

        if(n->y_origin < pose_y){
               adjustOrientation(3, theta);
			return;
        }       
        else if(n->y_origin > pose_y){
               adjustOrientation(1,theta);
			return;
        }
	//float y_diff = std::abs(pose_y - n->y_origin) / 3.0;
//	int count = 3;
//	while(count > 0){
	
        	m1.linear.x = .3;
        	pub1.publish(m1);
        	ros::Duration(0.5).sleep();
        	m1.linear.x = 0;
        	pub1.publish(m1);
        	ros::Duration(0.5).sleep();
//		count--;	
//	}
}

void goToNodeX(float pose_x, float theta){
        struct Node *n = path.top();

        ros::NodeHandle nh5;
        ros::Publisher pub2 = nh5.advertise<geometry_msgs::Twist>("/cmd_vel",100);
        geometry_msgs::Twist m2;
        while(pub2.getNumSubscribers() == 0)
                ros::Duration(0.5).sleep();

	float x_diff = (n->x_origin - pose_x) / 3.0;

	int count = 3;
	while(count > 0){
	
        	m2.linear.x = x_diff;
        	pub2.publish(m2);
        	ros::Duration(0.5).sleep();
        	m2.linear.x = 0;
        	pub2.publish(m2);
        	ros::Duration(0.5).sleep();
		count--;
	}
}

bool inOpen(struct Node *n){
        for(int i = 0; i < O.size(); i++){
                if(O[i]->id == n->id)
                        return true;
        }
        return false;
}

bool inClosed(struct Node *n){
        for(int i = 0; i < C.size(); i++){
                if(C[i]->id == n->id)
                        return true;
        }
        return false;
}


void insert_sorted(struct Node *n, int g_val){


        for(int i = 0; i < O.size(); i++){
                if(g_val <= O[i]->g){
                        O.insert(O.begin() + i, n);
                        return;
                }
        }

        O.push_back(n); //case where g_val is the starting or g_val is the biggest

}

void addToODijkstra(struct Node *next, struct Node *n, int g_new){
        next->g = g_new;
        insert_sorted(next, g_new);
        next->parent = n;
}

void remove(struct Node *n){
        for(int i = 0; i < O.size(); i++){
                if(n->id == O[i]->id){
                        O.erase(O.begin() + i);
                        return;
                }
        }
}

bool check_goal(struct Node *n){
	float x_diff = std::abs(n->x_origin - goal->x_origin);
        float y_diff = std::abs(n->y_origin - goal->y_origin);
	if(x_diff < 0.001 && y_diff < 0.001){
               	 last = n;
		 return true;
        }
	return false;
}

int calcG(struct Node *n){
        return n->g + 1;
}

void expandNodeDijkstra(struct Node *n){
        for(int i = 0; i < n->edge_counter; i++){
                struct Node *next = n->edges[i];
                int g_new = calcG(n);
                if(!inOpen(next) && !inClosed(next)){
                        addToODijkstra(next, n, g_new);
                }
                else if(g_new < next->g){
                        remove(next); //remove from open list
                        addToODijkstra(next, n, g_new);
                }
        }

}

bool dijkstra(){
        start->g = 0; 
        insert_sorted(start,start->g);
        start->parent = NULL;
        while(!O.empty()){
		struct Node *n = O.front();
                O.erase(O.begin());
                C.push_back(n);
                if(check_goal(n))
                        return true;
                expandNodeDijkstra(n);
        }
        return false;

}


void printPath(){
        std::ofstream file;
        file.open("path.txt");
	while(last != NULL){
		path.push(last);
		file << "Node: " << last->id << "\n";
		file << "(" << last->x_origin << ", " << last->y_origin << ")" << "\n";
		last = last->parent;
		file << "\n";
	}


	file << "\n";
        file.close();

}
void setPath(){
        while(last != NULL){
                path.push(last);
                last = last->parent;
        }
}



void setEdges(){
	//sets horizontal edges
	for(int i = 0; i < node_counter - 1; i++){
		float x_diff = vertices[i+1].x_origin - vertices[i].x_origin;
		float y_diff = vertices[i+1].y_origin - vertices[i].y_origin;
		if((x_diff < 0.51) && y_diff < .0001){
                        vertices[i + 1].edges[vertices[i + 1].edge_counter] =
                                &vertices[i];
                        vertices[i].edges[vertices[i].edge_counter] =
                                &vertices[i + 1];
                        vertices[i + 1].edge_counter += 1;
                        vertices[i].edge_counter += 1;
		}
	}
	//sets vertical edges
	for(int i = 0; i < node_counter - 1; i++){
		for(int j = i + 1; j < node_counter; j++){
			float x_diff = std::abs(vertices[j].x_origin - vertices[i].x_origin);
                        float y_diff = vertices[j].y_origin - vertices[i].y_origin;
			if(y_diff > .51)
				break;
			if(x_diff < .0001){
			        vertices[j].edges[vertices[j].edge_counter] =
                                        &vertices[i];
                                vertices[i].edges[vertices[i].edge_counter] =
                                        &vertices[j];
                                vertices[j].edge_counter += 1;
                                vertices[i].edge_counter += 1;
			}
		}
	}
}

void printEdges(){
	std::ofstream file;
	file.open("graph.txt");
	for(int i = 0; i < node_counter; i++){
		file << "Node: " << i;
		file << " Origin: (" << vertices[i].x_origin << "," << vertices[i].y_origin << ")" << "\n";
		file << "Edges: ";
		for(int j = 0; j < vertices[i].edge_counter; j++){
			file << "(" << vertices[i].edges[j]->x_origin << "," <<  vertices[i].edges[j]->y_origin
				<< ") ";
		}
		file << "\n";
	}	
	file.close();

}

void move_back(){
	ros::NodeHandle nh9;
        ros::Publisher pub5 = nh9.advertise<geometry_msgs::Twist>("/cmd_vel",100);
        geometry_msgs::Twist m4;
        while(pub5.getNumSubscribers() == 0)
                ros::Duration(0.5).sleep();
	m4.linear.x = -0.2;
	pub5.publish(m4);
	ros::Duration(0.5).sleep();
	m4.linear.x = 0;
	pub5.publish(m4);
	ros::Duration(0.5).sleep();	
}	

void poseReceived(const geometry_msgs::PoseWithCovarianceStamped msg){
	ROS_INFO_STREAM("PATH SIZE: " << path.size()); 
       /* float trace = msg.pose.covariance[0] + msg.pose.covariance[7] +
		 msg.pose.covariance[14] + msg.pose.covariance[21] +
		 msg.pose.covariance[28] + msg.pose.covariance[35];	
	ROS_INFO_STREAM("Trace: " << trace);
	if(trace > 2.0){
		move_back();
		return;
	}*/
	tf::Quaternion q;
        tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
       	q.normalize();
	float theta = q.getAngle();
	float x = msg.pose.pose.position.x, y = msg.pose.pose.position.y;
	ROS_INFO_STREAM("Angle: " << theta);

	if(path.empty()){
		ROS_INFO_STREAM("empty");
	}
        else if(!atNodeX(x)){
               	ROS_INFO_STREAM("go to x");
		goToNodeX(x,theta);
        }
     /*  else if(!atNodeY(y)){
               ROS_INFO_STREAM("go to y");
               goToNodeY(y,theta);
        }*/

	else{
		ROS_INFO_STREAM("popping");
		path.pop();
		ROS_INFO_STREAM("NEW SIZE: " << path.size());
	}
	prev_angle = theta;
}


void occupancyGridReceived(const nav_msgs::OccupancyGrid msg){
	for(int r_square = 0; r_square < 39; r_square++){
		for(int c_square = 0; c_square < 43; c_square++){
			for(int r_cell = 0; r_cell < 10; r_cell++){
				for(int c_cell = 0; c_cell < 10; c_cell++){
					int cell_loc = c_cell + (c_square * 10) + (r_cell * 434) + (r_square * 4340);
					if((int)msg.data[cell_loc] == -1 || (int)msg.data[cell_loc] == 100){
						r_cell = 10;
						c_cell = 10;
					}
					else if(r_cell == 9 && c_cell == 9){
						vertices[node_counter].id = node_counter;
						vertices[node_counter].edge_counter = 0;
						vertices[node_counter].x_origin = ((c_cell/2.0) + (c_square*10.0)) * resolution;
						vertices[node_counter].y_origin = ((r_cell/2.0) + (r_square*10.0)) * resolution;
						node_counter++;
					}
				}	
			}
		}
	}
	setEdges();
	printEdges();
	start = &vertices[644];
	//following line sets goal
	setGoal(0.0,0.0);
	if(dijkstra()){
		ROS_INFO_STREAM("PATH FOUND");
	//	printPath();
		setPath();
	}
	else{
	ROS_INFO_STREAM("NO PATH");
	}
}



void mapMetaDataReceived(const nav_msgs::MapMetaData msg){
	width = msg.width;
	height = msg.height;
	resolution = msg.resolution;
	ROS_INFO_STREAM(width << " " << height << " " << resolution);

}

int main(int argc, char** argv){
	ros::init(argc, argv, "graph");
	ros::NodeHandle n1, n2, n3;
	ros::Subscriber sub1 = n1.subscribe("/map_metadata", 1, &mapMetaDataReceived);
        ros::Subscriber sub2 = n2.subscribe("/map", 1, &occupancyGridReceived);
        ros::Subscriber sub3 = n3.subscribe("/amcl_pose", 1, &poseReceived);

	while(ros::ok()){
	ROS_INFO_STREAM("SPIN");
	ros::spinOnce();
	ros::Duration(0.5).sleep();
 sub3 = n3.subscribe("/amcl_pose", 1, &poseReceived);
}
}
