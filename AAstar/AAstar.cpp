#include "pch.h"
#include <iostream>
#include "AAstar.h"
using namespace std;
string map;
int row = getRowCol()[0];
int column = getRowCol()[1];
GraphNode start;
GraphNode goal;
int expandedNumber;
bool firstPlan = true;

int main()
{
	cout << "Adaptive A* go!" << endl;
	vector<vector<int>> preGMatrix(row, vector<int>(column));
	vector<vector<bool>> preExpandMatrix(row, vector<bool>(column));
	vector<vector<char>> start_map(row, vector<char>(column));
	vector<vector<char>> global_map(row, vector<char>(column));
	int c = 0;
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < column; j++)
		{
			global_map[i][j] = map.at(c++);
		}
	}
	start_map = global_map;
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < column; j++)
		{
			if (start_map[i][j] == 'x')
			{
				start_map[i][j] = '_';
			}
		}
	}

	// find start and goal node;
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < column; j++)
		{
			if (global_map[i][j] == 's')
			{
				string  s_string;
				s_string.push_back('0' + i);
				s_string.push_back('0' + j);
				start.name = s_string;
			}
			if (global_map[i][j] == 'g')
			{
				string  g_string;
				g_string.push_back('0' + i);
				g_string.push_back('0' + j);
				goal.name = g_string;
			}
		}
	}

	vector<vector<char>> current_map = start.exploreNewMap(global_map, start_map, row, column);
	GraphNode current = start; 
	vector<vector<bool> > initialExpandMatrix(row, vector<bool>(column));
	vector<GraphNode> aroundGoal = goal.generateNeibor(global_map, initialExpandMatrix, row, column);


	int NumberofPlan = 0;
	while (find(aroundGoal.begin(), aroundGoal.end(), current) == aroundGoal.end())
	{
		NumberofPlan += 1;
		cout << '\n' << "# of plan : " << NumberofPlan;

		vector<vector<int>> routeVector = plan(current, global_map, current_map, preGMatrix, preExpandMatrix);
		if (routeVector[0][0] == -1)
		{
			break;
		}
		for (size_t i = 0; i < routeVector.size(); i++)
		{
			std::string newName;
			newName.push_back(routeVector[i][0] + '0');
			newName.push_back(routeVector[i][1] + '0');
			current.name = newName;
			current_map = current.exploreNewMap(global_map, current_map, row, column);
			if (i != routeVector.size() - 1)
			{
				if (current_map[routeVector[i + 1][0]][routeVector[i + 1][1]] == 'x')
				{
					break;
				}
			}
		}
	}
	cout << '\n' << "Total expaneded nodes = " << expandedNumber << endl;

}
void printMatrix(vector<vector<char>> charMatrix)
{
	for (int i = 0; i < row; i++)
	{
		cout << endl;
		for (int j = 0; j < column; j++)
		{
			cout << charMatrix[i][j] << " ";
		}
	}
}
vector<vector<int>> getPath(vector<vector<vector<int>>> pathMatrix, GraphNode current)
{
	vector<vector<int>> path_vector;
	vector<int> g = goal.getIndex();
	while (g != current.getIndex())
	{
		path_vector.push_back(pathMatrix[g[0]][g[1]]);
		g = pathMatrix[g[0]][g[1]];
	}
	path_vector.pop_back();
	reverse(path_vector.begin(), path_vector.end());
	return path_vector;
}
vector<vector<int>> plan(GraphNode current, vector<vector<char>> global_map, vector<vector<char>> current_map, vector<vector<int>> &preGMatrixI, vector<vector<bool>> &preExpandMatrixI)
{
	vector<vector<bool> > expandMatrix(row, vector<bool>(column));
	vector<vector<int>> gMatrix(row, vector<int>(column));
	vector<vector<vector<int>>> pathMatrix(row, vector<vector<int>>(column, vector<int>(2)));
	MyQueue<GraphNode, vector<GraphNode>, KeyCompare> pq;
	pq.push(current);

	while (!expandMatrix[goal.getIndex()[0]][goal.getIndex()[1]])
	{
		if (pq.size() == 0)
		{
			cout << '\n' << "No route found" << endl;
			return { {-1, -1} };
		}
		GraphNode out = pq.top();
		pq.pop();
		expandMatrix[out.getIndex()[0]][out.getIndex()[1]] = true;
		if (out.generateNeibor(current_map, expandMatrix, row, column).size() == 0 && out.getIndex() != goal.getIndex())
		{
			continue;
		}
		for (GraphNode &neibor : out.generateNeibor(current_map, expandMatrix, row, column))
		{
			
			if (pq.contains(neibor) == false && expandMatrix[neibor.getIndex()[0]][neibor.getIndex()[1]] == false)
			{
				if (preExpandMatrixI[neibor.getIndex()[0]][neibor.getIndex()[1]] == false)
				{
					gMatrix[neibor.getIndex()[0]][neibor.getIndex()[1]] = gMatrix[out.getIndex()[0]][out.getIndex()[1]] + 1;
					neibor.key = gMatrix[neibor.getIndex()[0]][neibor.getIndex()[1]] + neibor.getH(goal);
					
				}
				else
				{
					gMatrix[neibor.getIndex()[0]][neibor.getIndex()[1]] = gMatrix[out.getIndex()[0]][out.getIndex()[1]] + 1;
					neibor.key = gMatrix[neibor.getIndex()[0]][neibor.getIndex()[1]] + preGMatrixI[goal.getIndex()[0]][goal.getIndex()[1]] - preGMatrixI[neibor.getIndex()[0]][neibor.getIndex()[1]];
				}
				pathMatrix[neibor.getIndex()[0]][neibor.getIndex()[1]] = out.getIndex();
				pq.push(neibor);
			}
			else if (pq.contains(neibor) == true)
			{
				if (neibor.key > gMatrix[out.getIndex()[0]][out.getIndex()[1]] + 1 + neibor.getH(goal))
				{
					if (preExpandMatrixI[neibor.getIndex()[0]][neibor.getIndex()[1]] == false)
					{
						gMatrix[neibor.getIndex()[0]][neibor.getIndex()[1]] = gMatrix[out.getIndex()[0]][out.getIndex()[1]] + 1;
						neibor.key = gMatrix[neibor.getIndex()[0]][neibor.getIndex()[1]] + neibor.getH(goal);

					}
					else
					{
						gMatrix[neibor.getIndex()[0]][neibor.getIndex()[1]] = gMatrix[out.getIndex()[0]][out.getIndex()[1]] + 1;
						neibor.key = gMatrix[neibor.getIndex()[0]][neibor.getIndex()[1]] + preGMatrixI[goal.getIndex()[0]][goal.getIndex()[1]] - preGMatrixI[neibor.getIndex()[0]][neibor.getIndex()[1]];
					}
					GraphNode newNeibor(neibor.name, neibor.key);
					pathMatrix[newNeibor.getIndex()[0]][newNeibor.getIndex()[1]] = out.getIndex();
					pq.remove(neibor);
					pq.push(newNeibor);
				}
			}
		}
	}
	preExpandMatrixI = expandMatrix;
	preGMatrixI = gMatrix;
	vector<vector<char>> path_map = current_map;
	vector<vector<int>> result = getPath(pathMatrix, current);
	for (int i = 0; i < result.size(); i++)
	{
		path_map[result[i][0]][result[i][1]] = 'o';
	}

	printMatrix(path_map);
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < column; j++)
		{
			if (expandMatrix[i][j] == true)
			{
				expandedNumber += 1;
			}
		}
	}
	if (firstPlan)
	{
		firstPlan = false;
	}
	return result;
}
vector<int> getRowCol()
{
	ifstream inf("C:\\Users\\cyc19\\source\\repos\\Astar\\Astar\\inputs\\14.txt");
	inf >> row;
	inf >> column;
	char a;
	while (inf >> a)
	{
		map.push_back(a);
	}
	
	return {row, column};
}


