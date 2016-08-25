#ifndef NODE_H
#define NODE_H

/** A Node is a representation of a 10 X 10 pixel tile.
  * We keep track of its origin, relative to the map, pointers
  * to its edges, an edge counter, a unique id. In addition in order
  * for Dijkstra's Algorithm to work we need to keep track of its parent
  * node and its g value, which is set to 1 by default.
**/

struct Node
{
	float x_origin, y_origin;
	Node *edges [4];	
	int edge_counter; 
	int id;
	Node *parent;
	int g;
};

#endif
