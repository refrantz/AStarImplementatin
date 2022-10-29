//problemA.cpp
//Simple path finding algorithm using A*
//Made by Renan Frantz

//sorry for taking a while to submit, this week was really busy
//thanks for the opportunity! I hope the AI overlords from Earth Custodianship would approve of this code :)

#include <vector>
#include <array>
#include <iostream>
#include <unordered_map>
#include <queue>

struct Point{
    int x;
    int y;

    //shouldn't have to be used, but map opertator[] requires default constructor for some cases
    Point(){
    }

    Point(int coordX, int coordY){
        x = coordX;
        y = coordY;
    }

    bool operator==(const Point &a) const{
        return (x == a.x) && (y == a.y);
    }

    bool operator!=(const Point &a) const{
        return (x != a.x) || (y != a.y);
    }

    //so that priority queue always uses the value from cost instead of some attribute from point
    bool operator<(const Point &a) const{
        return false;
    }

    bool operator>(const Point &a) const{
        return false;
    }

    Point operator+(const std::array<int,2> values) const{
        return Point(values[0] + x, values[1] + y);
    }
};

//decided upon the manhattan distance as a heuristic due to only being able to move through the grid vertically and horizontally, but not diagonally, also wanting to always find the shortest path
//this heuristic could be something else, normally could consider a heuristic that overestimates the path depending on application
//with a heuristic overestimating, the path wouldn't always be the shortest, but the algorithm would be faster, and for something like an AI in a game it's an interesting idea
//I mean, in reality, who always takes the shortest path? Many times we take a longer path if it means we have it memorized, is easier to follow/find etc.
inline double calculateManhattan(const Point a, const Point b);

//simple 2D hash function for the unordered map
template <> struct std::hash<Point> {
  size_t operator()(const Point& id) const noexcept {
    //using prime numbers to limit repeated results from hash in a grid where small changes result in different position
    return (53 + hash<int>()(id.x)) * (53 + hash<int>()(id.y));
  }
};

class MapClass {
    public:
        std::vector<int> Map;
        std::pair<int, int> MapDimensions;

        std::unordered_map<Point, double> costTo;
        std::unordered_map<Point, Point> pathFrom;

        MapClass(const std::vector<int>& M, std::pair<int, int> MDims){
            Map = M;
            MapDimensions = MDims;
        }

        std::vector<Point> getNeighbors(Point p){

            std::array<std::array<int,2>,4> dirs = {{{{1, 0}}, {{0, 1}}, {{-1, 0}}, {{0, -1}}}};
            std::vector<Point> result;

            for (auto dir : dirs){  
                Point candidateNeighbor = p+dir;

                //check boundaries and if is transversable terrain or not
                if((candidateNeighbor.x >= 0)
                && (candidateNeighbor.x < MapDimensions.first)
                && (candidateNeighbor.y >= 0)
                && (candidateNeighbor.y < MapDimensions.second)
                && (Map[candidateNeighbor.x+candidateNeighbor.y*MapDimensions.first])){

                    result.push_back(candidateNeighbor);

                }
            }

            return result;

        }

};

bool FindPath(std::pair<int, int> Start,
              std::pair<int, int> Target,
              const std::vector<int>& Map,
              std::pair<int, int> MapDimensions,
              std::vector<int>& OutPath);



int main(int argc, char **argv){

    std::vector<int> Map = {1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1};
    std::vector<int> OutPath;

    if(FindPath({0, 0}, {1, 2}, Map, {4, 3}, OutPath)){
        for(int index : OutPath){
            std::cout<<index<<std::endl;
        }
    }else{
        std::cout<<"Path not found"<<std::endl;
    }

}

//inline for better performance of simple function called frequently
inline double calculateManhattan(const Point a, const Point b){
    return abs(a.x - b.x) + abs(a.y - b.y);
}


bool FindPath(std::pair<int, int> Start, std::pair<int, int> Target, const std::vector<int>& Map, std::pair<int, int> MapDimensions, std::vector<int>& OutPath){

    MapClass mapObject(Map, MapDimensions);

    //frontier as priority queue pairing points and their costs (costs go first so they're prioritized when ordering)
    std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>, std::greater<std::pair<double, Point>>> frontier;

    Point start(Start.first, Start.second);
    Point target(Target.first, Target.second);

    mapObject.costTo.insert_or_assign(start,0);

    frontier.emplace(mapObject.costTo[start]+calculateManhattan(start, target),start);
    
    bool found = false;
    
    while(!frontier.empty()){

        Point currentPoint = frontier.top().second;

        //if target is on top of frontier, shortest path has been found
        if(currentPoint == target){
            found = true;
            break;
        }

        frontier.pop();

        for (Point neighbor : mapObject.getNeighbors(currentPoint)){
            
            double neighborCurrentCost = mapObject.costTo[currentPoint]+1;

            //if neighbor has not been seen or has stored cost higher than current cost add to frontier with this route and cost
            if(!mapObject.costTo.count(neighbor) || mapObject.costTo[neighbor] > neighborCurrentCost){

                mapObject.costTo.insert_or_assign(neighbor, neighborCurrentCost);
                mapObject.pathFrom.insert_or_assign(neighbor, currentPoint);
                frontier.emplace(neighborCurrentCost+calculateManhattan(neighbor, target),neighbor);

            }

        }

    }

    if(found){
        
        std::deque<Point> path;

        Point current = frontier.top().second;

        //follow path from target to start
        while (current != start){
            path.push_front(current);
            current = mapObject.pathFrom[current];
        }

        //turn points into indexes of map
        while (!path.empty()){
            OutPath.push_back(path.front().x + path.front().y * MapDimensions.first);
            path.pop_front();
        }
        
    }
    return found;

}