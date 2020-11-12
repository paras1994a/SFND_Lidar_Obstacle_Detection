


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

//  structure to define a 3 dimensional KD tree type.
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
    
	// methods for inserting data points in tree 

    void insertHelper(Node* &node, std::vector<float> point, int id, int depth)
	{ 
		
		if(node == NULL)
		{ node = new Node (point,id);
		}

        else
		{
			int cd = depth % 3;

			if(point[cd]< node->point[cd] )
			{
				insertHelper(node->left,point,id, depth+1);

			}
			else if(point[cd]> node->point[cd])
			{
				insertHelper( node->right,point,id, depth+1);
			}	
		}
			    

	}
	void insert(std::vector<float> point, int id)
	{
	
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, point,id, 0);
	}



	// Methods for Searching nearest neighbours to a target point  in tree

	void searchHelper(std::vector<float> target,Node* node, int depth,float distanceTol, std::vector<int> &ids)

    { 	if(node!= NULL)
		{	float dx = fabs(node->point[0] - target[0]);
			float dy = fabs(node->point[1] - target[1]);
			float dz = fabs(node->point[2] - target[2]);

			if (dx <= distanceTol && dy <= distanceTol && dz <= distanceTol)
            {
			  float temp = std::hypotf(dx, dy);
			  float distance = std::hypotf(temp, dz);

			  
			  if (distance <= distanceTol) 
              {
			    ids.push_back((node->id));
              }	

			 
            }
            int cd = depth%3;
			if ((target[cd]-distanceTol)< node->point[cd])
            {
              searchHelper(target,node->left,depth+1,distanceTol,ids);
            }  
            if ((target[cd]+distanceTol)> node->point[cd])
            {
              searchHelper(target,node->right,depth+1,distanceTol,ids);
            }                           

	    }


	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int>search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target,root,0,distanceTol,ids);
		return ids;
	}
	

};





