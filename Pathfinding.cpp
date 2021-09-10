#include "Pathfinding.h"

namespace PathfindingLibrary
{

	nodesVector_t Pathfinding::AStar(node_t origin, node_t destination)
	{
		nodesVector_t toSearch{};
		node_t start{ origin };
		node_t end{ destination };
		node_t currentNode{ nullptr };
		toSearch.push_back(start);

		while (toSearch.size() > 0)
		{
			currentNode = toSearch[0];
			toSearch.erase(toSearch.begin());
			currentNode->_inLine = false;

			if (currentNode == end)
			{
				return getPath(start, end);
			}
			node_t neighbour;

			for (size_t i = 0; i < currentNode->_neighbours.size(); i++)
			{
				neighbour = currentNode->_neighbours[i];
				if (neighbour != nullptr)
				{
					if (!neighbour->_visited && neighbour->_isWalkable)
					{
						if (!neighbour->_inLine)
						{
							int direction[2]{};
							direction[0] = end->_pos[0] - neighbour->_pos[0];
							direction[1] = end->_pos[1] - neighbour->_pos[1];
							neighbour->_distanceToDestination = sqrt((direction[0] * direction[0]) + (direction[1] * direction[1]));

							neighbour->_pathCost = currentNode->_pathCost + neighbour->_cost;
							neighbour->_combinedHeuristic = neighbour->_pathCost + neighbour->_distanceToDestination;
							neighbour->_parent = currentNode;
							neighbour->_inLine = true;
							insertNodeViaCombinedHeuristic(neighbour, toSearch);
						}
						else
						{
							// if new cost is smaller than the current cost
							if (currentNode->_pathCost + neighbour->_cost < neighbour->_pathCost)
							{
								neighbour->_pathCost = currentNode->_pathCost + neighbour->_cost;
								neighbour->_parent = currentNode;
								repositionNodeViaCombinedHeuristic(neighbour, toSearch);
							}
						}

					}
				}
			}
			// checked all neighbour nodes
			currentNode->_visited = true;
		}

		return nodesVector_t{};
	}

	nodesVector_t Pathfinding::Dijkstra(node_t origin, node_t destination)
	{
		nodesVector_t toSearch{};
		node_t start{ origin };
		node_t end{ destination };
		node_t currentNode{ nullptr };
		toSearch.push_back(start);

		while (toSearch.size() > 0)
		{
			currentNode = toSearch[0];
			toSearch.erase(toSearch.begin());
			currentNode->_inLine = false;

			if (currentNode == end)
			{
				return getPath(start, end);
			}
			node_t neighbour;

			for (size_t i = 0; i < currentNode->_neighbours.size(); i++)
			{
				neighbour = currentNode->_neighbours[i];
				if (neighbour != nullptr)
				{
					if (!neighbour->_visited && neighbour->_isWalkable)
					{
						if (!neighbour->_inLine)
						{
							neighbour->_pathCost = currentNode->_pathCost + neighbour->_cost;
							neighbour->_parent = currentNode;
							neighbour->_inLine = true;
							insertNodeViaPathCost(neighbour, toSearch);
						}
						else
						{
							// if new cost is smaller than the current cost
							if (currentNode->_pathCost + neighbour->_cost < neighbour->_pathCost)
							{
								neighbour->_pathCost = currentNode->_pathCost + neighbour->_cost;
								neighbour->_parent = currentNode;
								repositionNodeViaPathCost(neighbour, toSearch);
							}
						}
					}
				}
			}

			// checked all neighbour nodes
			currentNode->_visited = true;
		}

		return nodesVector_t{};
	}

	nodesVector_t Pathfinding::BFS(node_t origin, node_t destination)
	{
		node_t start{ origin };
		node_t end{ destination };

		nodesQ_t q = nodesQ_t();
		q.push(start);

		bool foundGoal{ false };
		node_t currentNode;

		while (!q.empty() && !foundGoal)
		{
			currentNode = q.front();
			q.pop();
			currentNode->_visited = true;
			currentNode->_inLine = false;

			for (size_t i = 0; i < currentNode->_neighbours.size(); i++)
			{
				if (currentNode->_neighbours[i] != nullptr)
				{
					if (!currentNode->_neighbours[i]->_visited && currentNode->_isWalkable)
					{
						currentNode->_neighbours[i]->_parent = currentNode;

						if (currentNode->_neighbours[i] == end)
						{
							return getPath(start, end);
						}
						else
						{
							if (currentNode->_neighbours[i]->_isWalkable && !currentNode->_neighbours[i]->_visited)
							{
								if (!currentNode->_neighbours[i]->_inLine)
								{
									q.push(currentNode->_neighbours[i]);
									currentNode->_neighbours[i]->_inLine = true;
								}
							}
						}
					}
				}
			}
		}

		return nodesVector_t{};
	}

	nodesVector_t Pathfinding::DFS(node_t origin, node_t destination)
	{
		node_t start{ origin };
		node_t end{ destination };

		nodesStack_t stack = nodesStack_t();
		stack.push(start);

		bool foundGoal{ false };
		node_t currentNode;

		while (!stack.empty() && !foundGoal)
		{
			currentNode = stack.top();
			stack.pop();
			currentNode->_visited = true;

			for (size_t i = 0; i < currentNode->_neighbours.size(); i++)
			{
				if (currentNode->_neighbours[i] != nullptr)
				{
					if (!currentNode->_neighbours[i]->_visited && currentNode->_isWalkable)
					{
						currentNode->_neighbours[i]->_parent = currentNode;

						if (currentNode->_neighbours[i] == end)
						{
							return getPath(start, end);
						}
						else
						{
							if (currentNode->_neighbours[i]->_isWalkable)
							{
								if (!currentNode->_neighbours[i]->_inLine)
								{
									stack.push(currentNode->_neighbours[i]);
									currentNode->_neighbours[i]->_inLine = true;
								}
							}
						}
					}
				}
			}
		}

		return nodesVector_t{};
	}


	nodesVector_t Pathfinding::DFSNodesSearched(node_t origin, node_t destination)
	{
		nodesVector_t nodesSearched{};
		node_t start{ origin };
		node_t end{ destination };

		nodesStack_t stack = nodesStack_t();
		stack.push(start);

		bool foundGoal{ false };
		node_t currentNode;

		while (!stack.empty() && !foundGoal)
		{
			currentNode = stack.top();
			stack.pop();
			currentNode->_visited = true;
			nodesSearched.push_back(currentNode);

			for (size_t i = 0; i < currentNode->_neighbours.size(); i++)
			{
				if (currentNode->_neighbours[i] != nullptr)
				{
					if (!currentNode->_neighbours[i]->_visited && currentNode->_isWalkable)
					{
						currentNode->_neighbours[i]->_parent = currentNode;

						if (currentNode->_neighbours[i] == end)
						{
							return nodesSearched;
						}
						else
						{
							if (currentNode->_neighbours[i]->_isWalkable)
							{
								if (!currentNode->_neighbours[i]->_inLine)
								{

									stack.push(currentNode->_neighbours[i]);
									currentNode->_neighbours[i]->_inLine = true;
								}
							}
						}
					}
				}
			}
		}

		return nodesSearched;
	}

	nodesVector_t Pathfinding::BFSNodesSearched(node_t origin, node_t destination)
	{
		nodesVector_t nodesSearched{};

		node_t start{ origin };
		node_t end{ destination };

		nodesQ_t q = nodesQ_t();
		q.push(start);

		bool foundGoal{ false };
		node_t currentNode;

		while (!q.empty() && !foundGoal)
		{
			currentNode = q.front();
			q.pop();
			currentNode->_visited = true;
			currentNode->_inLine = false;
			nodesSearched.push_back(currentNode);

			for (size_t i = 0; i < currentNode->_neighbours.size(); i++)
			{
				if (currentNode->_neighbours[i] != nullptr)
				{
					if (!currentNode->_neighbours[i]->_visited && currentNode->_isWalkable)
					{
						currentNode->_neighbours[i]->_parent = currentNode;

						if (currentNode->_neighbours[i] == end)
						{
							return nodesSearched;
						}
						else
						{
							if (currentNode->_neighbours[i]->_isWalkable && !currentNode->_neighbours[i]->_visited)
							{
								if (!currentNode->_neighbours[i]->_inLine)
								{

									q.push(currentNode->_neighbours[i]);
									currentNode->_neighbours[i]->_inLine = true;
								}
							}
						}
					}
				}
			}
		}

		return nodesSearched;
	}

	nodesVector_t Pathfinding::DijkstraNodesSearched(node_t origin, node_t destination)
	{
		nodesVector_t toSearch{};
		nodesVector_t searched{};
		node_t start{ origin };
		node_t end{ destination };
		node_t currentNode{ nullptr };
		toSearch.push_back(start);

		while (toSearch.size() > 0)
		{
			currentNode = toSearch[0];
			toSearch.erase(toSearch.begin());
			currentNode->_inLine = false;
			searched.push_back(currentNode);

			if (currentNode == end)
			{
				return searched;
			}

			node_t neighbour;

			for (size_t i = 0; i < currentNode->_neighbours.size(); i++)
			{
				neighbour = currentNode->_neighbours[i];
				if (neighbour != nullptr)
				{
					if (!neighbour->_visited && neighbour->_isWalkable)
					{
						if (!neighbour->_inLine)
						{
							neighbour->_pathCost = currentNode->_pathCost + neighbour->_cost;
							neighbour->_parent = currentNode;
							neighbour->_inLine = true;
							insertNodeViaPathCost(neighbour, toSearch);
						}
						else
						{
							// if new cost is smaller than the current cost
							if (currentNode->_pathCost + neighbour->_cost < neighbour->_pathCost)
							{
								neighbour->_pathCost = currentNode->_pathCost + neighbour->_cost;
								neighbour->_parent = currentNode;
								repositionNodeViaPathCost(neighbour, toSearch);
							}
						}

					}
				}
			}

			// checked all neighbour nodes
			currentNode->_visited = true;
		}
		return searched;
	}

	nodesVector_t Pathfinding::AStarNodesSearched(node_t origin, node_t destination)
	{
		nodesVector_t toSearch{};
		nodesVector_t searched{};
		node_t start{ origin };
		node_t end{ destination };
		node_t currentNode{ nullptr };
		toSearch.push_back(start);

		while (toSearch.size() > 0)
		{
			currentNode = toSearch[0];
			toSearch.erase(toSearch.begin());
			currentNode->_inLine = false;
			searched.push_back(currentNode);

			if (currentNode == end)
			{
				return searched;
			}

			node_t neighbour;

			for (size_t i = 0; i < currentNode->_neighbours.size(); i++)
			{
				neighbour = currentNode->_neighbours[i];
				if (neighbour != nullptr)
				{
					if (!neighbour->_visited && neighbour->_isWalkable)
					{
						if (!neighbour->_inLine)
						{
							int direction[2]{};
							direction[0] = end->_pos[0] - neighbour->_pos[0];
							direction[1] = end->_pos[1] - neighbour->_pos[1];
							neighbour->_distanceToDestination = sqrt((direction[0] * direction[0]) + (direction[1] * direction[1]));

							neighbour->_pathCost = currentNode->_pathCost + neighbour->_cost;
							neighbour->_combinedHeuristic = neighbour->_pathCost + neighbour->_distanceToDestination;
							neighbour->_parent = currentNode;
							neighbour->_inLine = true;
							insertNodeViaCombinedHeuristic(neighbour, toSearch);
						}
						else
						{
							// if new cost is smaller than the current cost
							if (currentNode->_pathCost + neighbour->_cost < neighbour->_pathCost)
							{
								neighbour->_pathCost = currentNode->_pathCost + neighbour->_cost;
								neighbour->_parent = currentNode;
								repositionNodeViaCombinedHeuristic(neighbour, toSearch);
							}
						}
					}
				}
			}
			// checked all neighbour nodes
			currentNode->_visited = true;
		}
		return searched;
	}








	nodesVector_t Pathfinding::getPath(node_t origin, node_t destination)
	{
		nodesVector_t path{};
		node_t node{ destination };

		while (node != nullptr && node != origin)
		{
			path.insert(path.begin(), node);
			node = node->_parent;
		}

		return path;
	}

	void Pathfinding::insertNodeViaPathCost(node_t node, nodesVector_t& vector)
	{
		if (vector.size() == 0 || node->_pathCost < vector[0]->_pathCost)
		{
			vector.insert(vector.begin(), node);
			return;
		}

		nodesVector_t::iterator it{ };

		for (size_t i = 1; i < vector.size(); i++)
		{
			if (node->_pathCost < vector[i]->_pathCost)
			{
				it = vector.begin() + i;
				vector.insert(it, node);
				return;
			}
		}

		vector.insert(vector.end(), node);
	}

	void Pathfinding::insertNodeViaCombinedHeuristic(node_t node, nodesVector_t& vector)
	{
		if (vector.size() == 0 || node->_combinedHeuristic < vector[0]->_combinedHeuristic)
		{
			vector.insert(vector.begin(), node);
			return;
		}

		nodesVector_t::iterator it{ };

		for (size_t i = 1; i < vector.size(); i++)
		{
			if (node->_combinedHeuristic < vector[i]->_combinedHeuristic)
			{
				it = vector.begin() + i;
				vector.insert(it, node);
				return;
			}
		}

		vector.insert(vector.end(), node);
	}

	void Pathfinding::repositionNodeViaPathCost(node_t node, nodesVector_t& vector)
	{
		for (size_t i = 0; i < vector.size(); i++)
		{
			if (vector[i] == node)
			{
				vector.erase(vector.begin() + i);
				break;
			}
		}

		insertNodeViaPathCost(node, vector);
	}

	void Pathfinding::repositionNodeViaCombinedHeuristic(node_t node, nodesVector_t& vector)
	{
		for (size_t i = 0; i < vector.size(); i++)
		{
			if (vector[i] == node)
			{
				vector.erase(vector.begin() + i);
				break;
			}
		}

		insertNodeViaCombinedHeuristic(node, vector);
	}





}