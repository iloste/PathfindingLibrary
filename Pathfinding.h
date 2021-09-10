#pragma once
#include "PathfindingNode.h"
#include <stack>
#include <queue>
#include <vector>

namespace PathfindingLibrary
{
	using nodesVector_t = std::vector<std::shared_ptr<PathfindingNode>>;
	using nodesStack_t = std::stack<std::shared_ptr<PathfindingNode>>;
	using nodesQ_t = std::queue<std::shared_ptr<PathfindingNode>>;
	using node_t = std::shared_ptr<PathfindingNode>;

	class Pathfinding
	{
	public:

		nodesVector_t AStar(node_t origin, node_t destination);
		nodesVector_t Dijkstra(node_t origin, node_t destination);
		nodesVector_t BFS(node_t origin, node_t destination);
		nodesVector_t DFS(node_t origin, node_t destination);

		nodesVector_t DFSNodesSearched(node_t origin, node_t destination);
		nodesVector_t BFSNodesSearched(node_t origin, node_t destination);
		nodesVector_t DijkstraNodesSearched(node_t origin, node_t destination);
		nodesVector_t AStarNodesSearched(node_t origin, node_t destination);

	private:
		nodesVector_t getPath(node_t origin, node_t destination);

		void insertNodeViaPathCost(node_t node, nodesVector_t& vector);
		void insertNodeViaCombinedHeuristic(node_t  node, nodesVector_t& vector);
		void repositionNodeViaPathCost(node_t node, nodesVector_t& vector);
		void repositionNodeViaCombinedHeuristic(node_t node, nodesVector_t& vector);
	};


}