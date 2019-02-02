#ifndef __interface_H
#define __interface_H
class Path_Visualizer:public olcConsoleGameEngine {
	public:
		Path_Visualizer(){
			m_sAppName = L"Path Finding";
		}
	private:
		int shortestpath_results = 0;;
		std::chrono::duration<double> elapsedTotal;
		int mode=0;
		int pathmode = 0;
		int visitednode = 0;
		struct sNode{
			bool bObstacle = false;			// Availability of nodes
			bool bVisited = false;			// Check if the nodes is visited
			float fGlobalGoal;				// Distance to goal so far
			float fLocalGoal;				// Distance to goal if we took the alternative route
			int x;							// Nodes position in 2D space
			int y;
			vector<sNode*> vecNeighbours;	// Connections to neighbours
			sNode* parent;					// Node connecting to this node that offers shortest parent
		};

		sNode *nodes = nullptr;
		int MapWidth = 16;
		int MapHeight = 16;

		sNode *nodeStart = nullptr;
		sNode *nodeEnd = nullptr;
	protected:
		virtual bool OnUserCreate() {
			/* Create a 2D array of nodes - this is for convenience of rendering and construction
			// and is not required for the algorithm to work - the nodes could be placed anywhere
			// in any space, in multiple dimensions because we give each node its own coordinates */
			nodes = new sNode[MapWidth * MapHeight];
			for (int x = 0; x < MapWidth; x++)
				for (int y = 0; y < MapHeight; y++) {
					nodes[y * MapWidth + x].x = x;
					nodes[y * MapWidth + x].y = y;
					nodes[y * MapWidth + x].bObstacle = false;
					nodes[y * MapWidth + x].parent = nullptr;
					nodes[y * MapWidth + x].bVisited = false;
				}

			// Create connections - in this case nodes are on a regular grid
			if (this->pathmode == 0){
				for (int x = 0; x < MapWidth; x++)
					for (int y = 0; y < MapHeight; y++) {
						if (y > 0)
							nodes[y*MapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * MapWidth + (x + 0)]);
						if (y < MapHeight - 1)
							nodes[y*MapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * MapWidth + (x + 0)]);
						if (x > 0)
							nodes[y*MapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * MapWidth + (x - 1)]);
						if (x < MapWidth - 1)
							nodes[y*MapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * MapWidth + (x + 1)]);
					}
			}
			if (this->pathmode == 1){
				for (int x = 0; x < MapWidth; x++)
					for (int y = 0; y < MapHeight; y++) {
						if (y > 0)
							nodes[y*MapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * MapWidth + (x + 0)]);
						if (y < MapHeight - 1)
							nodes[y*MapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * MapWidth + (x + 0)]);
						if (x > 0)
							nodes[y*MapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * MapWidth + (x - 1)]);
						if (x < MapWidth - 1)
							nodes[y*MapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * MapWidth + (x + 1)]);

						//  connect diagonally
						if (y > 0 && x > 0)
							nodes[y*MapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * MapWidth + (x - 1)]);
						if (y < MapHeight - 1 && x>0)
							nodes[y*MapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * MapWidth + (x - 1)]);
						if (y > 0 && x < MapWidth - 1)
							nodes[y*MapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * MapWidth + (x + 1)]);
						if (y < MapHeight - 1 && x < MapWidth - 1)
							nodes[y*MapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * MapWidth + (x + 1)]);

					}
			}
			// Manually positioN the start and end markers so they are not nullptr
			nodeStart = &nodes[(MapHeight / 2) * MapWidth + 1];
			nodeEnd = &nodes[(MapHeight / 2) * MapWidth + MapWidth - 2];
			return true;
		}
		

		bool Solve_AStar(){
			auto start = std::chrono::high_resolution_clock::now();
			// Reset Navigation Graph - default all node states
			for (int x = 0; x < MapWidth; x++)
				for (int y = 0; y < MapHeight; y++){
					nodes[y*MapWidth + x].bVisited = false;
					nodes[y*MapWidth + x].fGlobalGoal = INFINITY;
					nodes[y*MapWidth + x].fLocalGoal = INFINITY;
					nodes[y*MapWidth + x].parent = nullptr;	// No parents
				}
			// For convenience
			auto distance = [](sNode* a, sNode* b){
				return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
			};
			
			// heuristic is automatically generated here
			auto heuristic = [distance](sNode* a, sNode* b){
				return distance(a, b);
			};

			// Setup starting conditions
			sNode *nodeCurrent = nodeStart;
			nodeStart->fLocalGoal = 0.0f;
			nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);

			// Add start node to not tested list - this will ensure it gets tested.
			// As the algorithm progresses, newly discovered nodes get added to this
			// list, and will themselves be tested later
			list<sNode*> listNotTestedNodes;
			listNotTestedNodes.push_back(nodeStart);

			// if the not tested list contains nodes, there may be better paths
			// which have not yet been explored. However, we will also stop 
			// searching when we reach the target - there may well be better
			// paths but this one will do - it wont be the longest.

			
			// Sort Untested nodes by global goal, so lowest is first
			// Front of listNotTestedNodes is potentially the lowest distance node. Our
			// list may also contain nodes that have been visited, so ditch these...
			// ...or abort because there are no valid nodes left to test
			while (!listNotTestedNodes.empty() && nodeCurrent != nodeEnd){
				listNotTestedNodes.sort([](const sNode* lhs, const sNode* rhs){return lhs->fGlobalGoal < rhs->fGlobalGoal;});
				while (!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
					listNotTestedNodes.pop_front();
				if (listNotTestedNodes.empty())
					break;
				nodeCurrent = listNotTestedNodes.front();
				nodeCurrent->bVisited = true; // We only explore a node once

				// Check each of this node's neighbours...
				// ... and only if the neighbour is not visited and is 
				// not an obstacle, add it to NotTested List
				for (auto nodeNeighbour : nodeCurrent->vecNeighbours){
					if (!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == 0)
						listNotTestedNodes.push_back(nodeNeighbour);
					
					// Calculate the neighbours potential lowest parent distance
					float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour);

					// If choosing to path through this node is a lower distance than what 
					// the neighbour currently has set, update the neighbour to use this node
					// as the path source, and set its distance scores as necessary
					if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal){
						nodeNeighbour->parent = nodeCurrent;
						nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;
						// The best path length to the neighbour being tested has changed, so
						// update the neig	hbour's score. The heuristic is used to globally guide
						// the path algorithm, so it knows if its getting better or worse. At some
						// point the algo will realise this path is worse and abandon it, and then go
						// and search along the next best path.
						nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, nodeEnd);
					}
				}
			}
			auto finish = std::chrono::high_resolution_clock::now();
			this->elapsedTotal = finish - start;
			return true;
		}

		bool Solve_Dijkstra() {
			// Reset Navigation Graph set all nodes to its default states
			auto start = std::chrono::high_resolution_clock::now();//place a clock counter here
			for (int x = 0; x < MapWidth; x++)
				for (int y = 0; y < MapHeight; y++) {
					nodes[y*MapWidth + x].bVisited = false;
					nodes[y*MapWidth + x].fGlobalGoal = INFINITY;
					nodes[y*MapWidth + x].fLocalGoal = INFINITY;
					nodes[y*MapWidth + x].parent = nullptr;	// No parents
				}
			// priotiry Queue
			auto distance = [](sNode* a, sNode* b) {
				return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
			};

			// Setup starting conditions
			sNode *nodeCurrent = nodeStart;
			nodeStart->fLocalGoal = 0.0f;
			

			/* Add start node to not tested list - this will ensure it gets tested.
			 As the algorithm progresses, newly discovered nodes get added to this
			 list, and will be tested later*/
			list<sNode*> listNotTestedNodes;
			listNotTestedNodes.push_back(nodeStart);

			// if the not tested list contains nodes, there may be better paths
			// which have not yet been explored. However, we will also stop 
			// searching when we reach the target 

			// Find absolutely shortest path // && nodeCurrent != nodeEnd)
			// Sort Untested nodes by global goal, so lowest is first(greedy algorithm)
			// Front of listNotTestedNodes is potentially the lowest distance node. Our
			// list may also contain nodes that have been visited, so ditch these...
			// ...or abort because there are no valid nodes left to test
			while (!listNotTestedNodes.empty() && nodeCurrent != nodeEnd) {
				listNotTestedNodes.sort([](const sNode* lhs, const sNode* rhs) {return lhs->fGlobalGoal < rhs->fGlobalGoal;});
				while (!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
					listNotTestedNodes.pop_front();
				if (listNotTestedNodes.empty())
					break;
				nodeCurrent = listNotTestedNodes.front();
				nodeCurrent->bVisited = true; // We only explore a node once

				// Check each of this node's neighbours...
				// ... and only if the neighbour is not visited and is 
				// not an obstacle, add it to NotTested List
				for (auto nodeNeighbour : nodeCurrent->vecNeighbours) {
					if (!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == 0)
						listNotTestedNodes.push_back(nodeNeighbour);

					// Calculate the neighbours potential lowest parent distance
					float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour);

					// If choosing to path through this node is a lower distance than what 
					// the neighbour currently has set, update the neighbour to use this node
					// as the path source, and set its distance scores as necessary
					if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal) {
						nodeNeighbour->parent = nodeCurrent;
						nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;
						// The best path length to the neighbour being tested has changed, so
						// update the neighbour's score. 
					}
				}
			}
			auto finish = std::chrono::high_resolution_clock::now();
			this->elapsedTotal = finish - start;
			return true;
		}

		void Update_testResult()
		{
			ofstream myfile;
			myfile.open("WOI_Result.txt");
			if (this->mode == 0) {
				myfile << "Dijkstra's Algorithm" << endl;
			}if (this->mode == 1) {
				myfile << "A* Algorithm" << endl;
			}
			myfile << "The Total Weight(Cost) is:" << this->shortestpath_results << " edges" << endl;
			myfile << "Elapsed Time:" << this->elapsedTotal.count() << "sec" << endl;
			myfile << "Total of visited nodes:" << this->visitednode <<"nodes" <<endl;
			myfile << "Total of unvisited nodes:" << this->MapHeight*this->MapWidth - this->visitednode << "nodes" << endl;
			myfile.close();
		}

		virtual bool OnUserUpdate(float fElapsedTime){
			int nNodeSize = 9;
			int nNodeBorder = 2;
			int path_result = 0;
			double elapsed_time = 0;
			int visitednodes = 0;
			// Use integer division to nicely get cursor position in node space
			int nSelectedNodeX = m_mousePosX / nNodeSize;
			int nSelectedNodeY = m_mousePosY / nNodeSize;
			
			//use Key 1 and 2 to switchmode (1) djikstra (2)A*star
			if (m_keys[VK_NUMPAD1].bPressed) {
				this->mode = 0;
				Solve_Dijkstra();
			}if (m_keys[VK_NUMPAD2].bPressed) {
				this->mode = 1;
				Solve_AStar();
			}if (m_keys[VK_NUMPAD4].bPressed) {
				this->pathmode = 0;
				this->OnUserCreate();
			}if (m_keys[VK_NUMPAD8].bPressed) {
				this->pathmode = 1;
				this->OnUserCreate();
			}if (m_keys[VK_NUMPAD9].bPressed) {
				this->Update_testResult();
			}


			// Use mouse to draw maze, shift and ctrl to place start and end
			if (m_mouse[0].bReleased){
				if (nSelectedNodeX >= 0 && nSelectedNodeX < MapWidth)
					if (nSelectedNodeY >= 0 && nSelectedNodeY < MapHeight){
						if (m_keys[VK_SHIFT].bHeld)
							nodeStart = &nodes[nSelectedNodeY * MapWidth + nSelectedNodeX];
						else if (m_keys[VK_CONTROL].bHeld)
							nodeEnd = &nodes[nSelectedNodeY * MapWidth + nSelectedNodeX];
						else
							nodes[nSelectedNodeY * MapWidth + nSelectedNodeX].bObstacle = !nodes[nSelectedNodeY * MapWidth + nSelectedNodeX].bObstacle;
						
						// Solve in "real-time" and get the elapsed total time 

						if(this->mode == 0){
							Solve_Dijkstra();
						}else if (this->mode == 1) {
							Solve_AStar();
						}
					}
			}

			// Draw Connections First - lines from this nodes position to its
			// connected neighbour node positions
			Fill(0, 0, ScreenWidth(), ScreenHeight(), L' ');
			for (int x = 0; x < MapWidth; x++)
				for (int y = 0; y < MapHeight; y++){
					for (auto n : nodes[y*MapWidth + x].vecNeighbours){
						DrawLine(x*nNodeSize + nNodeSize / 2, y*nNodeSize + nNodeSize / 2,
						n->x*nNodeSize + nNodeSize / 2, n->y*nNodeSize + nNodeSize / 2, PIXEL_SOLID, FG_DARK_BLUE);
					}
				}
			
			// Draw Nodes on top
			for (int x = 0; x < MapWidth; x++)
				for (int y = 0; y < MapHeight; y++){
					Fill(x*nNodeSize + nNodeBorder, y*nNodeSize + nNodeBorder,
						(x + 1)*nNodeSize - nNodeBorder, (y + 1)*nNodeSize - nNodeBorder,
						PIXEL_HALF, nodes[y * MapWidth + x].bObstacle ? FG_WHITE : FG_BLUE);

					if (nodes[y * MapWidth + x].bVisited) {
						Fill(x*nNodeSize + nNodeBorder, y*nNodeSize + nNodeBorder,
							(x + 1)*nNodeSize - nNodeBorder, (y + 1)*nNodeSize - nNodeBorder,
							PIXEL_SOLID, FG_BLUE);
							visitednodes += 1;
					}

					if (&nodes[y * MapWidth + x] == nodeStart)
						Fill(x*nNodeSize + nNodeBorder, y*nNodeSize + nNodeBorder, 
							(x + 1)*nNodeSize - nNodeBorder, (y + 1)*nNodeSize - nNodeBorder, 
							PIXEL_SOLID, FG_GREEN);

					if (&nodes[y * MapWidth + x] == nodeEnd)
						Fill(x*nNodeSize + nNodeBorder, y*nNodeSize + nNodeBorder, 
							(x + 1)*nNodeSize - nNodeBorder, (y + 1)*nNodeSize - nNodeBorder, 
							PIXEL_SOLID, FG_RED);
				}

		 		// Draw Path by starting ath the end, and following the parent node trail
				// back to the start - the start node will not have a parent path to follow
				if (nodeEnd != nullptr){
					sNode *p = nodeEnd;
					while (p->parent != nullptr){
						if (this->mode==0){//refer to dijkstra
							DrawLine(p->x*nNodeSize + nNodeSize / 2, p->y*nNodeSize + nNodeSize / 2,
								p->parent->x*nNodeSize + nNodeSize / 2, p->parent->y*nNodeSize + nNodeSize / 2,
								PIXEL_SOLID, FG_RED);
							// Set next node to this node's parent
							p = p->parent;
						}if (this->mode == 1){//refer to a*
							DrawLine(p->x*nNodeSize + nNodeSize / 2, p->y*nNodeSize + nNodeSize / 2,
								p->parent->x*nNodeSize + nNodeSize / 2, p->parent->y*nNodeSize + nNodeSize / 2,
								PIXEL_SOLID, FG_YELLOW);
							// Set next node to this node's parent
							p = p->parent;
						}
						path_result += 1;
					}
				}
			this->visitednode = visitednodes;
			this->shortestpath_results=path_result;
			return true;
		}
};
#endif
