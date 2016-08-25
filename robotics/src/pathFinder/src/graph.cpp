#include <ros/ros.h>
#include <vector>
#include <stack>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <fstream>
#include <cmath>
#include <node.h>

private int width, height;
private float resolution;

private struct Node vertices[1677];
private int node_counter = 0;

private std::vector<Node*> O;
private std::vector<Node*> C;

private std::stack<Node*> path;

struct Node *start,*goal,*last;
int neg = 0;


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

}
