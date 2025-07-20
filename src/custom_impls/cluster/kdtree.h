// Simple implementation of kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, point, id, 0);
	}

	void insertHelper(Node **node, std::vector<float> point, int id, int depth) {
		// Base case
		if (*node == NULL) {
			*node = new Node(point, id);
			return;
		}
		// if current depth is even, split on x-axis, otherwise split on y-axis
		int current_depth = depth % 2;
		// if point is less than current node, go left
		if (point[current_depth] < (*node)->point[current_depth]) {
			insertHelper(&(*node)->left, point, id, depth + 1);
		} else {
			// if point is greater than current node, go right
			insertHelper(&(*node)->right, point, id, depth + 1);
		}
		
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		int box_length = 2 * distanceTol;
		searchHelper(&root, target, distanceTol, 0, ids);
		return ids;
	}

	void searchHelper(Node **node, std::vector<float> target, float distanceTol, int depth, std::vector<int> &ids) {
		// base case 
		// check if the node is within the distance tolerance only then add it to the ids
		if (*node != NULL) {
			if (((*node)->point[0] >= (target[0] - distanceTol)) && ((*node)->point[0] <= (target[0] + distanceTol)) && ((*node)->point[1] >= (target[1] - distanceTol)) && ((*node)->point[1] <= (target[1] + distanceTol))) {
				float distance = sqrt(pow((*node)->point[0] - target[0], 2) + pow((*node)->point[1] - target[1], 2));
				if (distance <= distanceTol) {
					ids.push_back((*node)->id);
				}
			}
			
			// prune the search space
			int current_depth = depth % 2;
			if ((target[current_depth] - distanceTol) < (*node)->point[current_depth]) {
				searchHelper(&(*node)->left, target, distanceTol, depth + 1, ids);
			}
			if ((target[current_depth] + distanceTol) > (*node)->point[current_depth]) {
				searchHelper(&(*node)->right, target, distanceTol, depth + 1, ids);
			}
		}
	}
};



