#ifndef NODE_H
#define NODE_H
/**
  *
  *
  *
**/

struct Node
{
	float x_origin, y_origin;
	Node *edges [4];	//at most 4 edges
	int edge_counter;   //keep track of edges
	int id;
	Node *parent;
	int g;
};

#endif
