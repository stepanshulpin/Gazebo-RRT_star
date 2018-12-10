#ifndef _RRT_STAR_HH_
#define _RRT_STAR_HH_
#include <ignition/math.hh>
#include "BoxExpanded.hh"

typedef std::vector<ignition::math::Vector2d> VectorOf2d;
typedef std::pair<double,int> CostPair;

class RRT_star
{
private:
	ignition::math::Vector2d minPlace;
	ignition::math::Vector2d maxPlace;
	ignition::math::Vector2d init;
	ignition::math::Vector2d goal;
	int N_steps;
	double p;
	double accuracy;
	std::vector<ignition::math::Line2<double>> lines;
	VectorOf2d vertexes;
	std::vector<int> parents;
	std::vector<double> costs;

public:
	RRT_star(ignition::math::Vector2d _minPlace, ignition::math::Vector2d _maxPlace, 
			ignition::math::Vector2d _init, ignition::math::Vector2d _goal, int _N_steps, 
			double _p, double _accuracy, std::vector<BoxExpanded> _obstacles);
	void getPath(VectorOf2d & path);
private:
	void nearestNeighbour(ignition::math::Vector2d & near, ignition::math::Vector2d rand);
	void findStoppingState(ignition::math::Vector2d & nev, ignition::math::Vector2d near, ignition::math::Vector2d rand);
	void nearestNeighbours(std::vector<int> & nearestInd, std::vector<double> & distanceToNew,ignition::math::Vector2d nev,double r);
	int nearestNeighbour(ignition::math::Vector2d rand);
	void sortNearestNeighbours(std::vector<double> & sortedCosts, std::vector<int> & prevSortInd,std::vector<int> nearestInd,std::vector<double> distanceToNew);
	void minCostParent(double & newCost, int & iParent,ignition::math::Vector2d nev, std::vector<int> nearestInd, std::vector<double> sortedCosts, std::vector<int> prevSortInd);
	void rewire(int tree_size,double newCost,std::vector<double> distanceToNew,std::vector<int> nearestInd,ignition::math::Vector2d nev);
	void optimalPathPlot(VectorOf2d & path);
	void optimalPathPlot(VectorOf2d & path, int nearInd);
	double searchRadius(int step, int N_steps);
	double generateState(double min, double max);
	template<class T>
	void printVec(std::vector<T> vec);
	bool crossLine(ignition::math::Line2<double> line1, ignition::math::Line2<double> line2, ignition::math::Vector2d & point);
};
#endif