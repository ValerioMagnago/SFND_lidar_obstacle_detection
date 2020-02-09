/* \author Aaron Brown */
// Quiz on implementing kd tree

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
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, 0,  point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<Node*> search(std::vector<float> target, float distanceTol)
	{
		std::vector<Node*> ids;
		searchHelper(root, 0, target, distanceTol, ids);
		return ids;
	}
	
	private:

	void searchHelper(Node *&node, const uint depth, const std::vector<float>& target, float distanceTol, std::vector<Node*>& ids){
		if(node==NULL) return;
		const auto ax = depth%2;		
		const bool major = (target[ax] + distanceTol) > node->point[ax];
		const bool minor = (target[ax] - distanceTol) < node->point[ax];
		if(minor && major){
			const auto ax2 = (depth+1)%2;
			const bool major2 = (target[ax2] + distanceTol) > node->point[ax2];
			const bool minor2 = (target[ax2] - distanceTol) < node->point[ax2];			
			if(minor2 && major2){
				const double d2Tol = distanceTol*distanceTol;
				const double lx = target[0] - node->point[0];
				const double ly = target[1] - node->point[1];
				const double d = lx*lx + ly*ly;
				if(d <= d2Tol){					
					ids.push_back(node);
				}
			}			
		}

		// Recursion to scroll the tree
		if(minor)
			searchHelper(node->left, depth+1, target, distanceTol, ids);		
		if(major)
			searchHelper(node->right, depth+1, target, distanceTol, ids);
	}

	void insertHelper(Node *&node, const uint depth, const std::vector<float>& point, const int id){
		if(node == NULL){
			node = new Node(point, id);
		}else{
			const auto ax = depth%2;
			if(point[ax] < node->point[ax]){
				insertHelper(node->left, depth + 1, point, id);
			}else{
				insertHelper(node->right, depth + 1, point, id);
			}
		}
	}
};




