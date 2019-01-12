#include "pch.h"
#include "GraphNode.h"

GraphNode::GraphNode()
{
	this->key = 0;
	this->expanded = false;
}

GraphNode::GraphNode(std::string name, double key)
{
	this->name = name;
	this->key = key;
}
GraphNode::GraphNode(std::string name, double key, int g)
{
	this->name = name;
	this->key = key;
	this->g = g;
}
GraphNode::GraphNode(std::string name)
{
	this->name = name;
	this->key = 0;
	this->expanded = false;
}
GraphNode::GraphNode(std::vector<int> int_name)
{
	std::string tname;
	tname.push_back(int_name[0] + '0');
	tname.push_back(int_name[1] + '0');
	this->name = tname;
	this->key = 0;
	this->expanded = false;
}
GraphNode::~GraphNode()
{
}
bool GraphNode::operator ==(const GraphNode &obj) const
{
	if (name == obj.name)
	{
		return true;
	}
	else
	{
		return false;
	}
}

std::vector<int> GraphNode::getIndex()
{
	std::vector<int> result(2);
	result[0] = this->name.at(0) - '0';
	result[1] = this->name.at(1) - '0';
	return result;
}

double GraphNode::getH(GraphNode goal)
{
	return abs(goal.getIndex()[0] - this->getIndex()[0]) + abs(goal.getIndex()[1] - this->getIndex()[1]);
}

std:: vector<GraphNode> GraphNode :: generateNeibor(std::vector<std::vector<char>> current_map, std::vector<std::vector<bool>> expandMatrix, int row, int column)
{
	std::vector<GraphNode> neibors;
	std::vector<int> currentIndex = this->getIndex();
	std::vector<int> upIndex = currentIndex;
	std::vector<int> downIndex = currentIndex;
	std::vector<int> leftIndex = currentIndex;
	std::vector<int> rightIndex = currentIndex;
	upIndex[0]--;
	downIndex[0]++;
	leftIndex[1]--;
	rightIndex[1]++;

	if (upIndex[0] >= 0 && current_map[upIndex[0]][upIndex[1]] != 'x' && expandMatrix[upIndex[0]][upIndex[1]] == false)
	{
		GraphNode up(upIndex);
		neibors.push_back(up);
	}
	if (downIndex[0] <= row - 1 && current_map[downIndex[0]][downIndex[1]] != 'x' && expandMatrix[downIndex[0]][downIndex[1]] == false)
	{
		GraphNode down(downIndex);
		neibors.push_back(down);
	}
	if (leftIndex[1] >= 0 && current_map[leftIndex[0]][leftIndex[1]] != 'x' && expandMatrix[leftIndex[0]][leftIndex[1]] == false)
	{
		GraphNode left(leftIndex);
		neibors.push_back(left);
	}
	if (rightIndex[1] <= column - 1 && current_map[rightIndex[0]][rightIndex[1]] != 'x' && expandMatrix[rightIndex[0]][rightIndex[1]] == false)
	{
		GraphNode right(rightIndex);
		neibors.push_back(right);
	}
	return neibors;
}
std::vector<std::vector<char>> GraphNode:: exploreNewMap(std::vector<std::vector<char>> global_map, std::vector<std::vector<char>> current_map, int row, int column)
{
	std::vector<std::vector<char>> newMap = current_map;
	for (int i = 0; i < row; i++) 
	{
		for (int j = 0; j < column; j++)
		{
			if (newMap[i][j] == 's')
			{
				newMap[i][j] = '_';
			}
		}
	}
	newMap[this->getIndex()[0]][this->getIndex()[1]] = 's';
	if (this->getIndex()[0] - 1 >= 0 && global_map[this->getIndex()[0] - 1][this->getIndex()[1]] != 's')
	{
		newMap[this->getIndex()[0] - 1][this->getIndex()[1]] = global_map[this->getIndex()[0] - 1][this->getIndex()[1]];
	}
	if (this->getIndex()[0] + 1 <= row - 1 && global_map[this->getIndex()[0] + 1][this->getIndex()[1]] != 's')
	{
		newMap[this->getIndex()[0] + 1][this->getIndex()[1]] = global_map[this->getIndex()[0] + 1][this->getIndex()[1]];
	}
	if (this->getIndex()[1] - 1 >= 0 && global_map[this->getIndex()[0]][this->getIndex()[1] - 1] != 's')
	{
		newMap[this->getIndex()[0]][this->getIndex()[1] - 1] = global_map[this->getIndex()[0]][this->getIndex()[1] - 1];
	}
	if (this->getIndex()[1] + 1 < column - 1 && global_map[this->getIndex()[0]][this->getIndex()[1] + 1] != 's')
	{
		newMap[this->getIndex()[0]][this->getIndex()[1] + 1] = global_map[this->getIndex()[0]][this->getIndex()[1] + 1];
	}
	return newMap;
}
