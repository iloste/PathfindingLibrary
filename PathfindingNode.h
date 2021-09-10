#pragma once
#include <memory>
#include <vector>

namespace PathfindingLibrary
{
	struct PathfindingNode
	{
		int _pos[2]{};
		bool _visited{ false };
		bool _isOrigin{ false };
		bool _isDestination{ false };
		bool _isWalkable{ true };
		bool _isWater{ false };
		bool _inLine{ false };
		float _cost{ 1 };
		float _pathCost{ 0 };
		float _distanceToDestination{ 0 };
		float _combinedHeuristic{ 0 };

		std::vector< std::shared_ptr<PathfindingNode>>_neighbours{};
		std::shared_ptr<PathfindingNode> _parent{};

		void resetConditions()
		{
			_visited = false;
			_isOrigin = false;
			_isDestination = false;
			_isWalkable = true;
			_isWater = false;
			_inLine = false;
			_cost = 1;
			_pathCost = 0;
			_distanceToDestination = 0;
			_combinedHeuristic = 0;
		}
	};


}