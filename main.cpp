#include <iostream>
#include <vector>
#include <deque>
#include <list>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

cv::Mat map;

enum EventType
{
    IN,
    OUT,
    CEILING,
    FLOOR
};

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

typedef std::vector<Point2D> Polygon;
typedef std::vector<Polygon> PolygonList;
typedef std::vector<Point2D> Edge;
typedef std::vector<Edge> EdgeList;

class Event
{
public:
    Event(int x_pos, int y_pos, EventType type)
    {
        x = x_pos;
        y = y_pos;
        event_type = type;
    }

    int x;
    int y;
    Edge ceiling_edge;
    Edge floor_edge;
    EventType event_type;
};

class CellNode  // ceiling.size() == floor.size()
{
public:
    CellNode()
    {
        isVisited = false;
        isCleaned = false;
        parentIndex = INT_MAX;
    }
    bool isVisited;
    bool isCleaned;
    EdgeList ceiling;
    EdgeList floor;

    int parentIndex;
    std::deque<int> neighbor_indices;

    int cellIndex;
};


std::deque<CellNode> path;

std::vector<CellNode> cell_graph;

void WalkingThroughGraph(int cell_index)
{
    cell_graph[cell_index].isVisited = true;
    path.emplace_front(cell_graph[cell_index]);
    std::cout<< "cell: " <<cell_graph[cell_index].cellIndex<<std::endl;

    CellNode neighbor;
    int neighbor_idx;

    for(int i = 0; i < cell_graph[cell_index].neighbor_indices.size(); i++)
    {
        neighbor = cell_graph[cell_graph[cell_index].neighbor_indices[i]];
        neighbor_idx = cell_graph[cell_index].neighbor_indices[i];
        if(!neighbor.isVisited)
        {
            break;
        }
    }

    if(!neighbor.isVisited) // unvisited neighbor found
    {
        neighbor.parentIndex = cell_graph[cell_index].cellIndex;
        WalkingThroughGraph(neighbor_idx);
    }
    else  // unvisited neighbor not found
    {

        if (cell_graph[cell_index].parentIndex == INT_MAX) // cannot go on back-tracking
        {
            return;
        }
        else
        {
            WalkingThroughGraph(cell_graph[cell_index].parentIndex);
        }
    }
}

std::vector<Point2D> GetBoustrophedonPath(std::vector<Point2D> ceiling, std::vector<Point2D> floor, int robot_radius=0)
{
    std::vector<Point2D> path;

    int x=0, y=0, y_start=0, y_end=0;
    bool reverse = false;

    for(int i = (robot_radius + 1); i < ceiling.size() - (robot_radius + 1); i++)
    {
        x = ceiling[i].x;

        if(!reverse)
        {
            y_start = ceiling[i].y + (robot_radius + 1);
            y_end   = floor[i].y - (robot_radius + 1);

            for(y = y_start; y <= y_end; y++)
            {
                path.emplace_back(Point2D(x, y));
                std::cout<< x << "," << y << std::endl;
            }
            reverse = !reverse;
        }
        else
        {
            y_start = floor[i].y - (robot_radius + 1);
            y_end   = ceiling[i].y + (robot_radius + 1);

            for (y = y_start; y >= y_end; y--)
            {
                path.emplace_back(Point2D(x, y));
                std::cout<< x << "," << y << std::endl;
            }
            reverse = !reverse;
        }
    }

    return path;
}

bool operator<(const Point2D& p1, const Point2D& p2)
{
    return (p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y));
}

bool operator<(const Event& e1, const Event& e2)
{
    return (e1.x < e2.x || (e1.x == e2.x && e1.y < e2.y));
}


std::vector<Event> EventListGenerator(PolygonList polygons)
{
    std::vector<Event> event_list;

    Polygon polygon;
    int leftmost_idx, rightmost_idx;

    for(int i = 0; i < polygons.size(); i++)
    {
        polygon = polygons[i];
        leftmost_idx = std::distance(polygon.begin(),std::min_element(polygon.begin(),polygon.end()));
        rightmost_idx = std::distance(polygon.begin(), std::max_element(polygon.begin(),polygon.end()));

        event_list.emplace_back(Event(polygon[leftmost_idx].x, polygon[leftmost_idx].y, IN));
        event_list.emplace_back(Event(polygon[rightmost_idx].x, polygon[rightmost_idx].y, OUT));

        // 假设多边形为凸壳 且各个顶点按照逆时针的顺序排列
        if (leftmost_idx < rightmost_idx)
        {
            for(int m = 0; m < polygon.size(); m++)
            {
                if(leftmost_idx < m && m < rightmost_idx)
                {
                    event_list.emplace_back(Event(polygon[m].x, polygon[m].y, CEILING));
                }
                if(m < leftmost_idx || m >rightmost_idx)
                {
                    event_list.emplace_back(Event(polygon[m].x, polygon[m].y, FLOOR));
                }
            }
        }
        else if(leftmost_idx > rightmost_idx)
        {
            for(int n = 0; n < polygon.size(); n++)
            {
                if(rightmost_idx < n && n < leftmost_idx)
                {
                    event_list.emplace_back(Event(polygon[n].x, polygon[n].y, FLOOR));
                }
                if(n < rightmost_idx || n > leftmost_idx)
                {
                    event_list.emplace_back(Event(polygon[n].x, polygon[n].y, CEILING));
                }
            }
        }
    }
    std::sort(event_list.begin(),event_list.end());
    return event_list;
}


void ExecuteOpenOperation(int curr_cell_idx, Point2D in, Point2D c, Point2D f) // in event  add two new node
{
    Point2D last_ceil_point = cell_graph[curr_cell_idx].ceiling.back()[0];
    cv::LineIterator ceil_points(map, cv::Point(last_ceil_point.x, last_ceil_point.y), cv::Point(c.x, c.y), 8, true);
    ceil_points++;
    for(int i = 1; i < ceil_points.count; i++)
    {
        cell_graph[curr_cell_idx].ceiling.back().emplace_back(Point2D(ceil_points.pos().x, ceil_points.pos().y));
        ceil_points++;
    }

    Point2D last_floor_point = cell_graph[curr_cell_idx].floor.back()[0];
    cv::LineIterator floor_points(map, cv::Point(last_floor_point.x, last_floor_point.y), cv::Point(f.x, f.y), 8, true);
    floor_points++;
    for(int i = 1; i < floor_points.count; i++)
    {
        cell_graph[curr_cell_idx].floor.back().emplace_back(Point2D(floor_points.pos().x, floor_points.pos().y));
        floor_points++;
    }


    CellNode top_cell, bottom_cell;

    Edge top_cell_ceil = {c};
    top_cell.ceiling.emplace_back(top_cell_ceil);
    Edge top_cell_floor = {in};
    top_cell.floor.emplace_back(top_cell_floor);

    Edge bottom_cell_ceil = {in};
    bottom_cell.ceiling.emplace_back(bottom_cell_ceil);
    Edge bottom_cell_floor = {f};
    bottom_cell.floor.emplace_back(bottom_cell_floor);

    int top_cell_index = curr_cell_idx + 1;
    int bottom_cell_index = curr_cell_idx + 2;

    top_cell.cellIndex = top_cell_index;
    bottom_cell.cellIndex = bottom_cell_index;
    cell_graph.emplace_back(top_cell);
    cell_graph.emplace_back(bottom_cell);


    cell_graph[top_cell_index].neighbor_indices.emplace_back(curr_cell_idx);
    cell_graph[bottom_cell_index].neighbor_indices.emplace_front(curr_cell_idx);

    cell_graph[curr_cell_idx].neighbor_indices.emplace_front(top_cell_index);
    cell_graph[curr_cell_idx].neighbor_indices.emplace_front(bottom_cell_index);
}

// top cell和bottom cell按前后排列
void ExecuteCloseOperation(int top_cell_idx, int bottom_cell_idx, Point2D out, Point2D c, Point2D f) // out event  add one new node
{
    Point2D top_cell_last_ceil_point = cell_graph[top_cell_idx].ceiling.back()[0];
    Point2D top_cell_last_floor_point = cell_graph[top_cell_idx].floor.back()[0];
    cv::LineIterator top_cell_ceil_points(map, cv::Point(top_cell_last_ceil_point.x, top_cell_last_ceil_point.y), cv::Point(c.x, c.y), 8, true);
    cv::LineIterator top_cell_floor_points(map, cv::Point(top_cell_last_floor_point.x, top_cell_last_floor_point.y), cv::Point(out.x, out.y), 8, true);
    top_cell_ceil_points++;
    top_cell_floor_points++;
    for(int i = 1; i < top_cell_ceil_points.count; i++)
    {
        cell_graph[top_cell_idx].ceiling.back().emplace_back(Point2D(top_cell_ceil_points.pos().x, top_cell_ceil_points.pos().y));
        top_cell_ceil_points++;
    }
    for(int i = 1; i < top_cell_floor_points.count; i++)
    {
        cell_graph[top_cell_idx].floor.back().emplace_back(Point2D(top_cell_floor_points.pos().x, top_cell_floor_points.pos().y));
        top_cell_floor_points++;
    }

    Point2D bottom_cell_last_ceil_point = cell_graph[bottom_cell_idx].ceiling.back()[0];
    Point2D bottom_cell_last_floor_point = cell_graph[bottom_cell_idx].floor.back()[0];
    cv::LineIterator bottom_cell_ceil_points(map, cv::Point(bottom_cell_last_ceil_point.x, bottom_cell_last_ceil_point.y), cv::Point(out.x, out.y), 8, true);
    cv::LineIterator bottom_cell_floor_points(map, cv::Point(bottom_cell_last_floor_point.x, bottom_cell_last_floor_point.y), cv::Point(f.x, f.y), 8, true);
    bottom_cell_ceil_points++;
    bottom_cell_floor_points++;
    for(int i = 1; i < bottom_cell_ceil_points.count; i++)
    {
        cell_graph[bottom_cell_idx].ceiling.back().emplace_back(Point2D(bottom_cell_ceil_points.pos().x, bottom_cell_ceil_points.pos().y));
        bottom_cell_ceil_points++;
    }
    for(int i = 1; i < bottom_cell_floor_points.count; i++)
    {
        cell_graph[bottom_cell_idx].floor.back().emplace_back(Point2D(bottom_cell_floor_points.pos().x, bottom_cell_floor_points.pos().y));
        bottom_cell_floor_points++;
    }

    CellNode new_cell;

    Edge new_cell_ceil = {c};
    Edge new_floor_ceil = {f};
    new_cell.ceiling.emplace_back(new_cell_ceil);
    new_cell.floor.emplace_back(new_floor_ceil);

    int new_cell_idx = bottom_cell_idx + 1;
    new_cell.cellIndex = new_cell_idx;

    cell_graph.emplace_back(new_cell);


    cell_graph[new_cell_idx].neighbor_indices.emplace_back(top_cell_idx);
    cell_graph[new_cell_idx].neighbor_indices.emplace_back(bottom_cell_idx);

    cell_graph[top_cell_idx].neighbor_indices.emplace_front(new_cell_idx);
    cell_graph[bottom_cell_idx].neighbor_indices.emplace_back(new_cell_idx);

}

void ExecuteCeilOperation(int curr_cell_idx, const Point2D& ceil_point) // finish constructing last ceiling edge
{
    Point2D last_ceil_point = cell_graph[curr_cell_idx].ceiling.back()[0];

    cv::LineIterator ceil_points(map, cv::Point(last_ceil_point.x, last_ceil_point.y), cv::Point(ceil_point.x, ceil_point.y), 8, true);
    ceil_points++;

    for(int i = 1; i < ceil_points.count-1; i++)
    {
        cell_graph[curr_cell_idx].ceiling.back().emplace_back(Point2D(ceil_points.pos().x, ceil_points.pos().y));
        ceil_points++;
    }

    Edge new_ceil_points;
    new_ceil_points.emplace_back(ceil_point);
    cell_graph[curr_cell_idx].ceiling.emplace_back(new_ceil_points);
}

void ExecuteFloorOperation(int curr_cell_idx, const Point2D& floor_point) // finish constructing last floor edge
{
    Point2D last_floor_point = cell_graph[curr_cell_idx].floor.back()[0];

    cv::LineIterator floor_points(map, cv::Point(last_floor_point.x, last_floor_point.y), cv::Point(floor_point.x, floor_point.y), 8, true);
    floor_points++;

    for(int i = 1; i < floor_points.count-1; i++)
    {
        cell_graph[curr_cell_idx].floor.back().emplace_back(Point2D(floor_points.pos().x, floor_points.pos().y));
        floor_points++;
    }

    Edge new_floor_points;
    new_floor_points.emplace_back(floor_point);
    cell_graph[curr_cell_idx].floor.emplace_back(new_floor_points);
}


void InitializeCellDecomposition()
{
    CellNode cell_0;

    int cell_0_idx = 0;
    cell_0.cellIndex = cell_0_idx;

    Edge ceil = {Point2D(0,0)};
    Edge floor = {Point2D(0,map.rows-1)}; // 初始化最初的ceil和floor点

    cell_0.ceiling.emplace_back(ceil);
    cell_0.floor.emplace_back(floor);
    cell_graph.emplace_back(cell_0);
}

void FinishCellDecomposition()
{
    int last_cell_idx = cell_graph.size()-1;

    // 封闭最后的ceil点和floor点

    cv::LineIterator ceiling(map, cv::Point(cell_graph[last_cell_idx].ceiling.back()[0].x,cell_graph[last_cell_idx].ceiling.back()[0].y), cv::Point(map.cols-1, 0), 8, true);
    cv::LineIterator floor(map, cv::Point(cell_graph[last_cell_idx].floor.back()[0].x, cell_graph[last_cell_idx].floor.back()[0].y), cv::Point(map.cols-1, map.rows-1), 8, true);
    ceiling++;
    floor++;
    for(int i = 1; i < ceiling.count; i++)
    {
        cell_graph[last_cell_idx].ceiling.back().emplace_back(ceiling.pos().x, ceiling.pos().y);
        ceiling++;
    }
    for(int i = 1; i < floor.count; i++)
    {
        cell_graph[last_cell_idx].floor.back().emplace_back(floor.pos().x, floor.pos().y);
        floor++;
    }
}

void ExecuteCellDecomposition()
{


    return;
}



void drawing_test(const CellNode& cell)
{
    for(int i = 0; i < cell.ceiling.size(); i++)
    {
        for(int j = 0; j < cell.ceiling[i].size(); j++)
        {
            map.at<cv::Vec3f>(cell.ceiling[i][j].y, cell.ceiling[i][j].x) = cv::Vec3f(0, 0, 255);
        }
    }

    for(int i = 0; i < cell.floor.size(); i++)
    {
        for(int j = 0; j < cell.floor[i].size(); j++)
        {
            map.at<cv::Vec3f>(cell.floor[i][j].y, cell.floor[i][j].x) = cv::Vec3f(0, 0, 255);
        }
    }

    cv::line(map, cv::Point(cell.ceiling.front().front().x,cell.ceiling.front().front().y), cv::Point(cell.floor.front().front().x,cell.floor.front().front().y), cv::Scalar(0,0,255));
    cv::line(map, cv::Point(cell.ceiling.back().back().x,cell.ceiling.back().back().y), cv::Point(cell.floor.back().back().x,cell.floor.back().back().y), cv::Scalar(0,0,255));
}



int main() {

//    test GetBoustrophedonPath
//    std::vector<Point2D> ceil = {Point2D(0,0),Point2D(1,0),Point2D(2,0),Point2D(3,0),Point2D(4,0)};
//    std::vector<Point2D> floor = {Point2D(0,4),Point2D(1,4),Point2D(2,4),Point2D(3,4),Point2D(4,4)};
//
//    std::vector<Point2D> path = GetBoustrophedonPath(ceil, floor);
//    std::vector<Point2D>::iterator it = path.begin();

//    test DepthFirstSearch
//    CellNode cell1,cell2,cell3,cell4,cell5,cell6,cell7,cell8,cell9,cell10,cell11;
//    cell1.cellIndex=1;
//    cell2.cellIndex=2;
//    cell3.cellIndex=3;
//    cell4.cellIndex=4;
//    cell5.cellIndex=5;
//    cell6.cellIndex=6;
//    cell7.cellIndex=7;
//    cell8.cellIndex=8;
//    cell9.cellIndex=9;
//    cell10.cellIndex=10;
//    cell11.cellIndex=11;
//
//    cell1.neighbor_cells={&cell2,&cell3};
//    cell2.neighbor_cells={&cell4,&cell5,&cell1};
//    cell3.neighbor_cells={&cell6,&cell7,&cell1};
//    cell4.neighbor_cells={&cell8,&cell9,&cell2};
//    cell5.neighbor_cells={&cell2};
//    cell6.neighbor_cells={&cell3};
//    cell7.neighbor_cells={&cell10,&cell11,&cell3};
//    cell8.neighbor_cells={&cell4};
//    cell9.neighbor_cells={&cell4};
//    cell10.neighbor_cells={&cell7};
//    cell11.neighbor_cells={&cell7};
//
//    DepthFirstSearch(cell1);

//    test EventListGenerator
//    Polygon polygon = {Point2D(2,3), Point2D(4,3), Point2D(5,2), Point2D(3,1), Point2D(1,2)};
//    PolygonList polygons = {polygon};
//
//    std::vector<Event> event_list = EventListGenerator(polygons);
//
//    for(auto it : event_list)
//    {
//        std::cout<< "x:" << it.x << ", y:"<< it.y << ", type:" << it.event_type << std::endl;
//    }

//  test simple cell decomposition
    map = cv::Mat::zeros(400, 400, CV_32FC3);
    Point2D in = Point2D(100,200), c = Point2D(100,0), f = Point2D(100,399);
    Point2D out = Point2D(300,200), c_=Point2D(300,0), f_=Point2D(300,399);
    Point2D c_end = Point2D(399, 0), f_end = Point2D(399, 399);

    CellNode cell0;
    int cell0_idx = 0;
    cell0.cellIndex = cell0_idx;
    Edge ceil = {Point2D(0,0)}, floor = {Point2D(0,399)}; // 初始化最初的ceil和floor点
    cell0.ceiling.emplace_back(ceil);
    cell0.floor.emplace_back(floor);
    cell_graph.emplace_back(cell0);
    ExecuteOpenOperation(cell0_idx, in, c, f);
    int cell1_idx = cell_graph.size()-2;
    int cell2_idx = cell_graph.size()-1;
    ExecuteFloorOperation(cell1_idx, Point2D(200, 100));
    ExecuteCeilOperation(cell2_idx, Point2D(200, 300));
    ExecuteCloseOperation(cell1_idx, cell2_idx, out, c_, f_);
    int cell3_idx = cell_graph.size()-1;

    // 封闭最后的ceil点和floor点
    cv::LineIterator c_it(map, cv::Point(cell_graph[cell3_idx].ceiling.back()[0].x,cell_graph[cell3_idx].ceiling.back()[0].y), cv::Point(c_end.x, c_end.y), 8, true);
    cv::LineIterator f_it(map, cv::Point(cell_graph[cell3_idx].floor.back()[0].x, cell_graph[cell3_idx].floor.back()[0].y), cv::Point(f_end.x, f_end.y), 8, true);
    c_it++;
    f_it++;
    for(int i = 1; i < c_it.count; i++)
    {
        cell_graph[cell3_idx].ceiling.back().emplace_back(c_it.pos().x, c_it.pos().y);
        c_it++;
    }
    for(int i = 1; i < f_it.count; i++)
    {
        cell_graph[cell3_idx].floor.back().emplace_back(f_it.pos().x, f_it.pos().y);
        f_it++;
    }

    drawing_test(cell_graph[cell0_idx]);
    cv::imshow("cells", map);
    cv::waitKey(0);

    drawing_test(cell_graph[cell1_idx]);
    cv::imshow("cells", map);
    cv::waitKey(0);

    drawing_test(cell_graph[cell2_idx]);
    cv::imshow("cells", map);
    cv::waitKey(0);

    drawing_test(cell_graph[cell3_idx]);
    cv::imshow("cells", map);
    cv::waitKey(0);

    cv::imshow("cells", map);
    cv::waitKey(0);

    for(int i = 0; i < cell_graph[0].neighbor_indices.size(); i++)
    {
        std::cout<<"cell0's neighbor: cell "<<cell_graph[cell_graph[0].neighbor_indices[i]].cellIndex<<std::endl;
    }

    for(int i = 0; i < cell_graph[1].neighbor_indices.size(); i++)
    {
        std::cout<<"cell1's neighbor: cell "<<cell_graph[cell_graph[1].neighbor_indices[i]].cellIndex<<std::endl;
    }

    for(int i = 0; i < cell_graph[2].neighbor_indices.size(); i++)
    {
        std::cout<<"cell2's neighbor: cell "<<cell_graph[cell_graph[2].neighbor_indices[i]].cellIndex<<std::endl;
    }

    for(int i = 0; i < cell_graph[3].neighbor_indices.size(); i++)
    {
        std::cout<<"cell3's neighbor: cell "<<cell_graph[cell_graph[3].neighbor_indices[i]].cellIndex<<std::endl;
    }

    WalkingThroughGraph(cell0_idx);

    return 0;

}