#ifndef GRAPHNODE_H
#define GRAPHNODE_H
#include <string>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <vector>

class GraphNode
{
public:
	double key;
	int g;
	std:: string name;
	bool expanded;
	std::vector<int> parent;
	GraphNode();
	GraphNode(std::string name, double key);
	GraphNode(std::string name, double key, int g);
	GraphNode(std::string name);
	GraphNode(std::vector<int> int_name);
	~GraphNode();
	bool operator ==(const GraphNode &obj) const;
	std::vector<int> getIndex();
	double getH(GraphNode goal);
	std::vector<GraphNode> generateNeibor(std::vector<std::vector<char>> current_map, std::vector<std::vector<bool>> expandMatrix, int row, int column);
	std::vector<std::vector<char>> exploreNewMap(std::vector<std::vector<char>> global_map, std::vector<std::vector<char>> current_map, int row, int column);
};
namespace std
{
	template<>
	struct hash<GraphNode>
	{
		size_t operator()(const GraphNode &obj) const
		{
			return hash<string>()(obj.name);
		}
	};
}

//struct GraphNodeHasher
//{
//	size_t operator()(const GraphNode & obj) const
//	{
//		return std::hash<std::string>()(obj.name);
//	}
//};
#endif