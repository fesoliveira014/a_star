#ifndef _A_STAR_
#define _A_STAR_

#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <iterator>
#include <cstdlib>
#include <algorithm>

#define INF 2147483647

// Structure used to store a point in the grid and its priority
// on the priority queueStart
struct vertex {
    int x;
    int y;

    int priority;

    vertex() {}
    vertex(int x, int y) : x(x), y(y) { priority = -1; }

    friend bool operator== (const vertex& a, const vertex& b) {
        return (a.x == b.x && a.y == b.y);
    }

    friend bool operator!= (const vertex& a, const vertex& b) {
        return !(a == b);
    }
};

const vertex none(-1, -1);

// Hash function necessary for unordered map under C++11
struct VertexHash {
    std::size_t operator() (const vertex& a) const {
        std::size_t h1 = std::hash<int>()(a.x);
        std::size_t h2 = std::hash<int>()(a.y);
        return h1 ^ (h2 << 1);
    }
};

// Weight comparission functor for priority queueStart sorting
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

void getNeighbors(vertex neighbors[], vertex node, const int nMapWidth, const int nMapHeight) {
    if (node.y != 0)
        neighbors[0] = vertex(node.x, node.y - 1);
    else
        neighbors[0] = none; // top border

    // gets left neighbor
    if (node.x != nMapWidth - 1)
        neighbors[1] = vertex(node.x + 1, node.y);
    else
        neighbors[1] = none; // right border

    // gets bottom neighbor
    if (node.y != nMapHeight - 1)
        neighbors[2] = vertex(node.x, node.y + 1);
    else
        neighbors[2] = none; // bottom border

    // gets left neighbor
    if (node.x != 0)
        neighbors[3] = vertex(node.x - 1, node.y);
    else
        neighbors[3] = none; // left border
}

// A* implementation. Given the start point a target point, finds the shortest path from
// start to target by looking at the neighborsStart and using the Manhattan as a heuristic,
// decides witch neighbor to go next. 
int FindPath(const int nStartX, const int nStartY,
    const int nTargetX, const int nTargetY,
    const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
    int* pOutBuffer, const int nOutBufferSize) {

    std::unordered_map<vertex, int, VertexHash> distancesStart;
    std::unordered_map<vertex, vertex, VertexHash> previousStart;
    std::priority_queue<vertex, std::vector<vertex>, VertexWeightComparisson> queueStart;

    std::unordered_map<vertex, int, VertexHash> distancesTarget;
    std::unordered_map<vertex, vertex, VertexHash> previousTarget;
    std::priority_queue<vertex, std::vector<vertex>, VertexWeightComparisson> queueTarget;

    vertex start(nStartX, nStartY);
    start.priority = 0;
    queueStart.push(start);

    vertex target(nTargetX, nTargetY);
    target.priority = 0;
    queueTarget.push(target);

    vertex neighborsStart[4];
    vertex neighborsTarget[4];

    distancesStart[start] = 0;
    previousStart[start] = none;

    distancesTarget[target] = 0;
    previousTarget[target] = none;

    vertex touch = none;

    int maxDistance = INF;

    while (!queueStart.empty() && !queueTarget.empty()) {
        vertex currentStart = queueStart.top();
        vertex currentTarget = queueTarget.top();

        if(touch != none) {
            int priorityFromStart = distancesStart[currentStart] + Heuristic(currentStart.x, currentStart.y, target.x, target.y);
            int priorityFromTarget = distancesTarget[currentStart] + Heuristic(currentTarget.x, currentTarget.y, start.x, start.y);

            if(maxDistance <= std::max(priorityFromStart, priorityFromTarget)) {
                break;
            }

        }

        
        queueStart.pop();
        queueTarget.pop();

        //std::cout << "currentStart: (" << currentStart.x << ", " << currentStart.y << ")" << std::endl;

        getNeighbors(neighborsStart, currentStart, nMapWidth, nMapHeight);  

        for (vertex node : neighborsStart) {
            // checks if neighbor is border or if it's position is valid
            if (node == none || pMap[node.x + node.y * nMapWidth] == 0)
                continue;

            // std::cout << "    neighour: (" << node.x << "," << node.y << "); pMap[" << node.x + node.y * nMapWidth << "]: " << int(pMap[node.x + node.y * nMapWidth]) << std::endl;
            int distance = distancesStart[currentStart] + 1;

            // checks if neighbor has not yet been visited or if the
            // the currentStart distance is smaller than the one stored
            if (distancesStart.find(node) == distancesStart.end() || distance < distancesStart[node]) {
                distancesStart[node] = distance;
                node.priority = distance + Heuristic(node.x, node.y, target.x, target.y);

                // std::cout << "        priority: " << node.priority << std::endl;

                queueStart.push(node);
                previousStart[node] = currentStart;

                if(previousTarget.find(node) != previousTarget.end()) {
                    if(maxDistance > node.priority) {
                        maxDistance = node.priority;
                        touch = node;
                    }
                }
            }
        }

        // std::cout << "currentTarget: (" << currentTarget.x << ", " << currentTarget.y << ")" << std::endl;
        getNeighbors(neighborsTarget, currentTarget, nMapWidth, nMapHeight);    

        for (vertex node : neighborsTarget) {
            // checks if neighbor is border or if it's position is valid
            if (node == none || pMap[node.x + node.y * nMapWidth] == 0)
                continue;

            // std::cout << "    neighour: (" << node.x << "," << node.y << "); pMap[" << node.x + node.y * nMapWidth << "]: " << int(pMap[node.x + node.y * nMapWidth]) << std::endl;
            int distance = distancesTarget[currentTarget] + 1;

            // checks if neighbor has not yet been visited or if the
            // the currentTarget distance is smaller than the one stored
            if (distancesTarget.find(node) == distancesTarget.end() || distance < distancesTarget[node]) {
                distancesTarget[node] = distance;
                node.priority = distance + Heuristic(node.x, node.y, start.x, start.y);

                // std::cout << "        priority: " << node.priority << std::endl;

                queueTarget.push(node);
                previousTarget[node] = currentTarget;

                if(previousStart.find(node) != previousStart.end()) {
                    if(maxDistance > node.priority) {
                        maxDistance = node.priority;
                        touch = node;
                    }
                }
            }
        }
        // std::cout << std::endl;

    }

    // std::cout << std::endl;

    ///////////////////////////////
    // Path reconstruction
    ///////////////////////////////
    std::vector<vertex> totalPath;
    vertex current = touch;
    totalPath.push_back(touch);

    while (previousStart.find(current) != previousStart.end()) {
        current = previousStart[current];
        if (current == none)
            break;
        totalPath.push_back(current);
    }

    std::reverse(totalPath.begin(), totalPath.end());
    current = touch;

    while (previousTarget.find(current) != previousTarget.end()) {
        current = previousTarget[current];
        if (current == none)
            break;
        totalPath.push_back(current);
    }

    std::reverse(totalPath.begin(), totalPath.end());

    ///////////////////////////////

    // if the solution has a valid size, return it
    // else, return -1
    if (totalPath.size() > 1 && totalPath.size() <= nOutBufferSize) {
        // std::cout << "path: ";
        int k = 0;
        for (int i = totalPath.size() - 1; i >= 0; i--) {
            // if (i != 0)
                // std::cout << "(" << totalPath[i].x << "," << totalPath[i].y << ") -> ";
            // else
                // std::cout << "(" << totalPath[i].x << "," << totalPath[i].y << ")";
            if(totalPath[i] == start)
                continue;
            pOutBuffer[k] = totalPath[i].x + totalPath[i].y * nMapWidth;
            k++;
        }

        // std::cout << std::endl;
        // std::cout << std::endl;
        return k;
    }

    // std::cout << std::endl;

    return -1;


}

#endif