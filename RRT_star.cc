#include "RRT_star.hh"
#include <random>  

RRT_star::RRT_star(ignition::math::Vector2d _minPlace, ignition::math::Vector2d _maxPlace, 
		ignition::math::Vector2d _init, ignition::math::Vector2d _goal, int _N_steps, 
		double _p, double _accuracy, std::vector<BoxExpanded> _obstacles)
{
	minPlace=_minPlace;
	maxPlace=_maxPlace;
	init=_init;
	goal=_goal;
	N_steps=_N_steps;
	p=_p;
	accuracy=_accuracy;
	for_each(_obstacles.begin(), _obstacles.end(), [&](BoxExpanded i){	
		std::vector<ignition::math::Line2<double>> linesFromBox(i.getLines());
		lines.insert(lines.end(),linesFromBox.begin(),linesFromBox.end());
	});	
}

void RRT_star::getPath(VectorOf2d & path)
{
	vertexes.push_back(init);
	parents.push_back(-1);
	costs.push_back(0);
	int step=0;
	int tree_size=0;
	while(step<N_steps)
	{
		std::cout<<"Step--->"<<step<<std::endl;
		ignition::math::Vector2d rand(generateState(minPlace.X(),maxPlace.X()),
				generateState(minPlace.Y(),maxPlace.Y()));
		ignition::math::Vector2d near;
		nearestNeighbour(near,rand);
		ignition::math::Vector2d nev;//new vertex
		findStoppingState(nev,near,rand);
		if(nev!=near)
		{
			tree_size++;
			double r=searchRadius(step,N_steps);
			std::vector<int> nearestInd;
			std::vector<double> distanceToNew;
			nearestNeighbours(nearestInd, distanceToNew,nev,r);
			if(!nearestInd.empty())
			{
				std::vector<double> sortedCosts;
				std::vector<int> prevSortInd;
				sortNearestNeighbours(sortedCosts,prevSortInd,nearestInd,distanceToNew);
				double newCost;
				int iParent;
				minCostParent(newCost,iParent,nev,nearestInd,sortedCosts,prevSortInd);
				vertexes.push_back(nev);
				parents.push_back(nearestInd[iParent]);
				costs.push_back(newCost);
				rewire(tree_size,newCost,distanceToNew,nearestInd,nev);
			}
			if(nev.Distance(goal)<accuracy)
			{
				optimalPathPlot(path);
				break;
			}
		}
		step++;
	}
}

void RRT_star::optimalPathPlot(VectorOf2d & path)
{
	int ix=vertexes.size()-1;
	path.push_back(vertexes[ix]);
	while(parents[ix]!=-1)
	{
		path.push_back(vertexes[parents[ix]]);
		ix=parents[ix];
	}
}

double RRT_star::generateState(double min, double max)
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(min, max);
	return dis(gen);
}

void RRT_star::nearestNeighbour(ignition::math::Vector2d & near, ignition::math::Vector2d rand)
{
	std::vector<double> dist;
	for_each(vertexes.begin(), vertexes.end(), [&](ignition::math::Vector2d i){
		dist.push_back(i.Distance(rand));
	});
	near = vertexes.at(std::distance(dist.begin(), std::min_element(dist.begin(), dist.end())));
}

void RRT_star::findStoppingState(ignition::math::Vector2d & nev, ignition::math::Vector2d near, ignition::math::Vector2d rand)
{	
	ignition::math::Line2<double> line(near,rand);
	VectorOf2d points;
	std::vector<double> dist;
	int isntCross=1;
	for_each(lines.begin(),lines.end(),[&](ignition::math::Line2<double> i){
		ignition::math::Vector2d point;
		int cross = i.Intersect(line,point);
		if(cross) 
		{
			isntCross=0;
			points.push_back(point);
			dist.push_back(point.Distance(near));
		}
	});
	if(isntCross) nev=rand;
	else
	{
		ignition::math::Vector2d crossVec = points.at(std::distance(dist.begin(), std::min_element(dist.begin(), dist.end())));
		ignition::math::Vector2d dif = near-crossVec;
		dif.Normalize();
		dif*=p;		
		nev=crossVec+dif;		
	}
}

void RRT_star::nearestNeighbours(std::vector<int> & nearestInd, std::vector<double> & distanceToNew,ignition::math::Vector2d nev,double r)
{
	int j=0;
	for_each(vertexes.begin(), vertexes.end(), [&](ignition::math::Vector2d i){
		double dist = i.Distance(nev);
		if(dist<r)
		{
			nearestInd.push_back(j);
		}
		distanceToNew.push_back(dist);
		j++;
	});
}
	

void RRT_star::sortNearestNeighbours(std::vector<double> & sortedCosts, std::vector<int> & prevSortInd,std::vector<int> nearestInd,std::vector<double> distanceToNew)
{
	std::vector<CostPair> costsPair;
	int j=0;
	for(auto &i:costs) 
	{
		if(std::find(nearestInd.begin(),nearestInd.end(),j)!=nearestInd.end())
		{
			costsPair.push_back(CostPair(i+distanceToNew[j],j));			
		}
		j++;
	}
	std::sort(costsPair.begin(),costsPair.end(),
	          [] (const auto& lhs, const auto& rhs) {
	    return lhs.first < rhs.first;
	});
	for_each(costsPair.begin(), costsPair
			.end(), [&](CostPair i){
		sortedCosts.push_back(i.first);
		prevSortInd.push_back(i.second);
	});
}
void RRT_star::minCostParent(double & newCost, int & iParent,ignition::math::Vector2d nev, std::vector<int> nearestInd, std::vector<double> sortedCosts, std::vector<int> prevSortInd)
{
	int foundColisionFreeParent=0;
	int j=0;
	while(!foundColisionFreeParent)
	{
		iParent=prevSortInd[j];
		ignition::math::Line2<double> line(nev,vertexes[iParent]);
		int isntCross=0;
		isntCross=std::all_of(lines.begin(),lines.end(),[&](ignition::math::Line2<double> i){
			int cross = i.Intersect(line);
			return !cross;
		});
		if(isntCross)
		{
			newCost=sortedCosts[j];
			foundColisionFreeParent=1;
		}
		else { j++; }
	}

}

void RRT_star::rewire(int tree_size,double newCost,std::vector<double> distanceToNew,std::vector<int> nearestInd,ignition::math::Vector2d nev)
{
	int j=0;
	for(const auto &i:vertexes) 
	{		
		if(std::find(nearestInd.begin(),nearestInd.end(),j)!=nearestInd.end())
		{
			if((newCost+distanceToNew[j])<costs[j]){
				int iNode = j;
				ignition::math::Line2<double> line(nev,vertexes[iNode]);
				int isntCross=0;
				isntCross=std::all_of(lines.begin(),lines.end(),[&](ignition::math::Line2<double> k){
					int cross = k.Intersect(line);
					return !cross;
				});
				if(isntCross)
				{
					parents[iNode]=tree_size;
					costs[iNode]=newCost+distanceToNew[iNode];
				}
			}
		}
		j++;
	}
}

double RRT_star::searchRadius(int step, int N_steps)
{
	double size=std::sqrt(std::pow((maxPlace.X()-minPlace.X()),2)+std::pow((maxPlace.Y()-minPlace.Y()),2));
	return size*(1-((double)step)/((double)N_steps));
}
template<class T>
void RRT_star::printVec(std::vector<T> vec)
{
	for(auto &i:vec) std::cout<<i<<"  ";
		std::cout<<std::endl;
}

