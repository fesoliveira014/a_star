#ifndef _A_STAR_
#define _A_STAR_

#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <iterator>
#include <cstdlib>

#define INF 2147483647

// Structure used to store a point in the grid and its priority
// on the priority queue
struct vertex {
	int x;
	int y;

	int priority;

	vertex() {}
	vertex(int x, int y) : x(x), y(y) { priority = -1; }

	friend bool operator== (const vertex& a, const vertex& b) {
		return (a.x == b.x && a.y == b.y);
	}
};

// Hash function necessary for unordered map under C++11
struct VertexHash {
	std::size_t operator() (const vertex& a) const {
		std::size_t h1 = std::hash<int>()(a.x);
		std::size_t h2 = std::hash<int>()(a.y);
		return h1 ^ (h2 << 1);
	}
};

// Weight comparission functor for priority queue sorting
class VertexWeightComparisson {
public:
	bool operator() (const vertex a, const vertex b) const {
		return a.priority > b.priority;
	}
};

// Manhattan distance heuristic for A* algorithm
int Heuristic(const int x, const int y, const int targetX, const int targetY) {
	return abs(x - targetX) + abs(y - targetY);
}

// A* implementation. Given the start point a target point, finds the shortest path from
// start to target by looking at the neighbors and using the Manhattan as a heuristic,
// decides witch neighbor to go next. 
int FindPath(const int nStartX, const int nStartY,
	const int nTargetX, const int nTargetY,
	const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
	int* pOutBuffer, const int nOutBufferSize) {

	// data structures used to store the distance from a node
	// or point to the starting node and to store the node
	// used to reach the current node; priority queue used for
	// A*.
	std::unordered_map<vertex, int, VertexHash> distances;
	std::unordered_map<vertex, vertex, VertexHash> previous;
	std::priority_queue<vertex, std::vector<vertex>, VertexWeightComparisson> queue;

	// will use (-1.-1) as an invalid point
	const vertex none(-1, -1);

	vertex start(nStartX, nStartY);
	start.priority = 0;
	queue.push(start);

	vertex target(nTargetX, nTargetY);

	// 0 = top, 1 = right, 2 = bottom. 3 = left
	vertex neighbors[4];

	distances[start] = 0;
	previous[start] = none;

	while (!queue.empty()) {
		vertex current = queue.top();
		queue.pop();

#ifdef _DEBUG
		std::cout << "current: (" << current.x << ", " << current.y << ")" << std::endl;
#endif

		if (current == target) // found target
			break;

		////////////////////////////////
		// neighborhood construction
		///////////////////////////////
		// gets top neighbor
		if (current.y != 0)
			neighbors[0] = vertex(current.x, current.y - 1);
		else
			neighbors[0] = none; // top border

		// gets left neighbor
		if (current.x != nMapWidth - 1)
			neighbors[1] = vertex(current.x + 1, current.y);
		else
			neighbors[1] = none; // right border

		// gets bottom neighbor
		if (current.y != nMapHeight - 1)
			neighbors[2] = vertex(current.x, current.y + 1);
		else
			neighbors[2] = none; // bottom border

		// gets left neighbor
		if (current.x != 0)
			neighbors[3] = vertex(current.x - 1, current.y);
		else
			neighbors[3] = none; // left border
		///////////////////////////////

		for (vertex node : neighbors) {
			// checks if neighbor is border or if it's position is valid
			if (node == none || pMap[node.x + node.y * nMapWidth] == 0)
				continue;
#ifdef _DEBUG
			std::cout << "    neighour: (" << node.x << "," << node.y << "); pMap[" << node.x + node.y * nMapWidth << "]: " << unsigned int(pMap[node.x + node.y * nMapWidth]) << std::endl;
#endif
			int distance = distances[current] + 1;

			// checks if neighbor has not yet been visited or if the
			// the current distance is smaller than the one stored
			if (distances.find(node) == distances.end() || distance < distances[node]) {
				distances[node] = distance;
				node.priority = distance + Heuristic(node.x, node.y, target.x, target.y);
#ifdef _DEBUG
				std::cout << "        priority: " << node.priority << std::endl;
#endif
				queue.push(node);
				previous[node] = current;
			}
		}

#ifdef _DEBUG
		std::cout << std::endl;
#endif

	}

#ifdef _DEBUG
	std::cout << std::endl;
#endif

	///////////////////////////////
	// Path reconstruction
	///////////////////////////////
	std::vector<vertex> totalPath;
	vertex current = target;
	totalPath.push_back(target);

	while (previous.find(current) != previous.end()) {
		current = previous[current];
		if (current == none)
			break;
		totalPath.push_back(current);
	}
	///////////////////////////////

	// if the solution has a valid size, return it
	// else, return -1
	if (totalPath.size() > 1 && totalPath.size() <= nOutBufferSize) {
		std::cout << "path: ";
		int k = 0;
		for (int i = totalPath.size() - 1; i >= 0; i--) {
			if (i != 0)
			 	std::cout << "(" << totalPath[i].x << "," << totalPath[i].y << ") -> ";
			else
				std::cout << "(" << totalPath[i].x << "," << totalPath[i].y << ")";
			if(totalPath[i] == start)
				continue;
			pOutBuffer[k] = totalPath[i].x + totalPath[i].y * nMapWidth;
			k++;
		}

		std::cout << std::endl;
		std::cout << std::endl;
		return k;
	}

	std::cout << std::endl;

	return -1;


}

#endif