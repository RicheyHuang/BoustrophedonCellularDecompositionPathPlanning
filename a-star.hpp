//
// Created by huangxh on 19-9-16.
//

#ifndef BCD_PLANNER_A_STAR_H
#define BCD_PLANNER_A_STAR_H

#endif //BCD_PLANNER_A_STAR_H

#include <vector>
#include <map>
#include <deque>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

cv::Mat map;

class Point2D
{
public:
    Point2D(int x_pos, int y_pos)
    {
        x = x_pos;
        y = y_pos;
    }
    int x;
    int y;
};

struct MapPoint
{
    double cost = 0.0;
    double occupancy = 1.0;
    Point2D prev_position = Point2D(INT_MAX, INT_MAX);
    bool costComputed = false;
};

std::map<Point2D, MapPoint> cost_map;

std::vector<Point2D> GetNeighbors(Point2D position)
{
    std::vector<Point2D> neighbors = {
            Point2D(position.x-1, position.y-1),
            Point2D(position.x, position.y-1),
            Point2D(position.x+1, position.y-1),
            Point2D(position.x-1, position.y),
            Point2D(position.x+1, position.y),
            Point2D(position.x-1, position.y+1),
            Point2D(position.x, position.y+1),
            Point2D(position.x+1, position.y+1)
    };
    return neighbors;
}


void BuildOccupancyMap() // CV_32FC3
{
    for(int i = 0; i < map.rows; i++)
    {
        for(int j = 0; j < map.cols; j++)
        {
            if(map.at<cv::Vec3f>(i,j) == cv::Vec3f(255,255,255))
            {
                cost_map[Point2D(j,i)].occupancy = INT_MAX;
            }
            else
            {
                cost_map[Point2D(j,i)].occupancy = 1.0;
            }
        }
    }
}

void BuildCostMap(Point2D start)
{
    std::deque<Point2D> task_list = {start};
    cost_map[start].costComputed = true;

    std::vector<Point2D> neighbors;
    std::vector<Point2D> candidates;

    while(!task_list.empty())
    {
        candidates.clear();
        neighbors = GetNeighbors(task_list.front());
        for(auto neighbor:neighbors)
        {
            if(!cost_map[neighbor].costComputed)
            {
                candidates.emplace_back(neighbor);
                cost_map[neighbor].cost = cost_map[task_list.front()].cost + 1.0;
                cost_map[neighbor].prev_position = task_list.front();
                cost_map[neighbor].costComputed = true;
            }
        }
        task_list.pop_front();
        task_list.insert(task_list.end(), candidates.begin(), candidates.end());
    }

}

std::deque<Point2D> FindShortestPath(Point2D start, Point2D end)
{
    std::deque<Point2D> path = {end};

    Point2D curr_positon = Point2D(end.x, end.y);

    int prev_x = cost_map[curr_positon].prev_position.x;
    int prev_y = cost_map[curr_positon].prev_position.y;


    while(prev_x != start.x && prev_y != start.y)
    {
        path.emplace_front(Point2D(prev_x,prev_y));
        curr_positon = Point2D(prev_x, prev_y);
        prev_x = cost_map[curr_positon].prev_position.x;
        prev_y = cost_map[curr_positon].prev_position.y;
    }

    path.emplace_front(start);

    return path;
}

void ResetCostMap()
{
    for(int i = 0; i < map.rows; i++)
    {
        for(int j = 0; j < map.cols; j++)
        {
            cost_map[Point2D(j,i)].cost = 0.0;
            cost_map[Point2D(j,i)].prev_position = Point2D(INT_MAX, INT_MAX);
            cost_map[Point2D(j,i)].costComputed = false;
        }
    }
}
