/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cmath>

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
		insert_helper(&root, point, id, 0);
	}
	void insert_helper(Node **node, std::vector<float> point, int id, int depth)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if(*node == nullptr)
		{
			// creating point if nullptr then create it with the new data point
			*node  = new Node(point,id);
		}
		else // if ptr is not null then check
		{
			uint cd = depth % 3;
			if(point[cd] < ((*node)->point)[cd])
			{
				insert_helper(&((*node)->left), point, id, depth+1);
			}
			else
			{
				insert_helper(&((*node)->right), point, id, depth+1);
			}
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(&root ,target, 0, ids, distanceTol);
		return ids;
	}
	
	void search_helper(Node **node, std::vector<float> target,uint depth ,std::vector<int> &ids, float distanceTol)
	{
		if(*node == nullptr)
		{
			return;
		}
		else
		{
			float XPostolerance= target[0] + distanceTol;
			float XNegtolerance= target[0] - distanceTol;
			float YPostolerance= target[1] + distanceTol; 
			float YNegtolerance= target[1] - distanceTol;
			float ZPostolerance= target[2] + distanceTol;
			float ZNegtolerance= target[2] - distanceTol;
			// check if in box
			if((*node)->point[0] <= XPostolerance  && (*node)->point[0] >= XNegtolerance && 
				(*node)->point[1] <= YPostolerance  && (*node)->point[1] >= YNegtolerance &&
				(*node)->point[2] <= ZPostolerance  && (*node)->point[2] >= ZNegtolerance) 
			{
				if(distance(target, (*node)->point) <= distanceTol)
				{
					ids.push_back((*node)->id);
				}
			}
			// if left or down most boundary of box is greater then we go to the left
			if(target[depth%3]- distanceTol <(*node)->point[depth%3])
			{
				search_helper(&((*node)->left),target,depth+1,ids,distanceTol);
			}
			if(target[depth%3]+ distanceTol >(*node)->point[depth%3])
			{
				search_helper(&((*node)->right),target,depth+1,ids,distanceTol);
			}
		}
	}
	float distance(std::vector<float> point1, std::vector<float> point2)
	{
		return sqrt((point2[0]-point1[0])*(point2[0]-point1[0]) + (point2[1]-point1[1])*(point2[1]-point1[1] + (point2[2]-point1[2])*(point2[2]-point1[2])));
	}

};


