#ifndef ASTAR_H
#define ASTAR_H
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <queue>
#include <functional>
#include <unordered_set>
#include "../Astar/GraphNode.h"
#include "../Astar/GraphNode.cpp"
using namespace std;
vector<int> getRowCol();
void printMatrix(vector<vector<char>>);
vector<vector<int>> getPath(vector<vector<vector<int>>> pathMatrix, GraphNode current);
vector<vector<int>> plan(GraphNode current, vector<vector<char>> global_map, vector<vector<char>> current_map, vector<vector<int>> &preGMatrix, vector<vector<bool>> &preExpandMatrix);
template<
	class T,
	class Container = std::vector<T>,
	class Compare = std::less<typename Container::value_type>
> class MyQueue : public std::priority_queue<T, Container, Compare>
{
public:
	typedef typename
		std::priority_queue<
		T,
		Container,
		Compare>::container_type::const_iterator const_iterator;

	bool remove(const T& value) {
		auto it = std::find(this->c.begin(), this->c.end(), value);
		if (it != this->c.end()) {
			this->c.erase(it);
			std::make_heap(this->c.begin(), this->c.end(), this->comp);
			return true;
		}
		else {
			return false;
		}
	}
	bool contains(const T&val) const
	{
		auto first = this->c.cbegin();
		auto last = this->c.cend();
		while (first != last) {
			if (*first == val) return true;
			++first;
		}
		return false;
	}
};
struct KeyCompare
{
	bool operator()(const GraphNode &i1, const GraphNode &i2) const
	{
		return i1.key > i2.key;
	}
};
#endif 