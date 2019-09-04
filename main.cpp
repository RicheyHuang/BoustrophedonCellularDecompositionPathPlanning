#include <iostream>
#include <vector>
#include <deque>
#include <map>
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

const int TOPLEFT = 0;
const int BOTTOMLEFT = 1;
const int BOTTOMRIGHT = 2;
const int TOPRIGHT = 3;


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

typedef std::vector<Point2D> Polygon;   // contour points extracted from a blob, sorted by counter clockwise manner
typedef std::vector<Polygon> PolygonList;
typedef std::vector<Point2D> Edge;

class Event
{
public:
    Event(int obstacle_idx, int x_pos, int y_pos, EventType type)
    {
        obstacle_index = obstacle_idx;
        x = x_pos;
        y = y_pos;
        event_type = type;
        original_index_in_slice = INT_MAX;
        isUsed = false;
    }

    int x;
    int y;
    int original_index_in_slice;
    int obstacle_index;
    EventType event_type;

    bool isUsed;
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
    Edge ceiling;
    Edge floor;

    int parentIndex;
    std::deque<int> neighbor_indices;

    int cellIndex;
};


std::deque<CellNode> path;

std::vector<CellNode> cell_graph;

int unvisited_counter = INT_MAX;

void WalkingThroughGraph(int cell_index)
{
    if(!cell_graph[cell_index].isVisited)
    {
        cell_graph[cell_index].isVisited = true;
        unvisited_counter--;
    }
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
        cell_graph[neighbor_idx].parentIndex = cell_graph[cell_index].cellIndex;
        WalkingThroughGraph(neighbor_idx);
    }
    else  // unvisited neighbor not found
    {

        if (cell_graph[cell_index].parentIndex == INT_MAX) // cannot go on back-tracking
        {
            return;
        }
        else if(unvisited_counter == 0)
        {
            return;
        }
        else
        {
            WalkingThroughGraph(cell_graph[cell_index].parentIndex);
        }
    }
}

std::vector<Point2D> GetBoustrophedonPath(CellNode cell, int corner_indicator, int robot_radius=0)
{
    std::vector<Point2D> ceiling = cell.ceiling;
    std::vector<Point2D> floor = cell.floor;

    std::vector<Point2D> path;

    if(cell_graph[cell.cellIndex].isCleaned)
    {
        int c_size = cell.ceiling.size();
        int f_size = cell.floor.size();

        if(corner_indicator == TOPLEFT)
        {
            path.emplace_back(Point2D(ceiling[(robot_radius + 1)].x, ceiling[(robot_radius + 1)].y + (robot_radius + 1)));
        }
        if(corner_indicator == TOPRIGHT)
        {
            path.emplace_back(Point2D(cell.ceiling[c_size-1-(robot_radius + 1)].x, cell.ceiling[c_size-1-(robot_radius + 1)].y + (robot_radius + 1)));
        }
        if(corner_indicator == BOTTOMLEFT)
        {
            path.emplace_back(Point2D(cell.floor[robot_radius + 1].x, cell.floor[robot_radius + 1].y - (robot_radius + 1)));
        }
        if(corner_indicator == BOTTOMRIGHT)
        {
            path.emplace_back(Point2D(cell.floor[f_size-1-(robot_radius + 1)].x, cell.floor[f_size-1-(robot_radius + 1)].y - (robot_radius + 1)));
        }
    }
    else
    {
        if(corner_indicator == TOPLEFT)
        {
            int x=0, y=0, y_start=0, y_end=0;
            bool reverse = false;

            for(int i = (robot_radius + 1); i < ceiling.size() - (robot_radius + 1); i=i+robot_radius)
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = ceiling[i].y + (robot_radius + 1);
                    y_end   = floor[i].y - (robot_radius + 1);

                    for(y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x+j > ceiling.back().x - (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x+j, floor[i+j].y-(robot_radius + 1)));
                        }
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
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x+j > ceiling.back().x - (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x+j, ceiling[i+j].y+(robot_radius + 1)));
                        }
                    }

                    reverse = !reverse;
                }
            }
        }

        if(corner_indicator == TOPRIGHT)
        {
            int x=0, y=0, y_start=0, y_end=0;
            bool reverse = false;

            for(int i = ceiling.size()-1-(robot_radius + 1); i >= (robot_radius + 1); i=i-robot_radius)
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = ceiling[i].y + (robot_radius + 1);
                    y_end   = floor[i].y - (robot_radius + 1);

                    for(y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x-j < ceiling.front().x + (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x-j, floor[i-j].y-(robot_radius + 1)));
                        }
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
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x-j < ceiling.front().x + (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x-j, ceiling[i-j].y+(robot_radius + 1)));
                        }
                    }

                    reverse = !reverse;
                }
            }
        }

        if(corner_indicator == BOTTOMLEFT)
        {
            int x=0, y=0, y_start=0, y_end=0;
            bool reverse = false;

            for(int i = (robot_radius + 1); i < ceiling.size() - (robot_radius + 1); i=i+robot_radius)
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = floor[i].y - (robot_radius + 1);
                    y_end   = ceiling[i].y + (robot_radius + 1);

                    for(y = y_start; y >= y_end; y--)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x+j > ceiling.back().x - (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x+j, ceiling[i+j].y+(robot_radius + 1)));
                        }
                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = ceiling[i].y + (robot_radius + 1);
                    y_end   = floor[i].y - (robot_radius + 1);

                    for (y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x+j > ceiling.back().x - (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x+j, floor[i+j].y-(robot_radius + 1)));
                        }
                    }

                    reverse = !reverse;
                }
            }
        }

        if(corner_indicator == BOTTOMRIGHT)
        {
            int x=0, y=0, y_start=0, y_end=0;
            bool reverse = false;

            for(int i = ceiling.size()-1-(robot_radius + 1); i >= (robot_radius + 1); i=i-robot_radius)
            {
                x = ceiling[i].x;

                if(!reverse)
                {
                    y_start = floor[i].y - (robot_radius + 1);
                    y_end   = ceiling[i].y + (robot_radius + 1);

                    for(y = y_start; y >= y_end; y--)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x-j < ceiling.front().x + (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x-j, ceiling[i-j].y+(robot_radius + 1)));
                        }
                    }

                    reverse = !reverse;
                }
                else
                {
                    y_start = ceiling[i].y + (robot_radius + 1);
                    y_end   = floor[i].y - (robot_radius + 1);

                    for (y = y_start; y <= y_end; y++)
                    {
                        path.emplace_back(Point2D(x, y));
                    }

                    if(robot_radius != 0)
                    {
                        for(int j = 1; j <= robot_radius; j++)
                        {
                            if( x-j < ceiling.front().x + (robot_radius + 1))
                            {
                                break;
                            }
                            path.emplace_back(Point2D(x-j, floor[i-j].y-(robot_radius + 1)));
                        }
                    }

                    reverse = !reverse;
                }
            }
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

// 各个多边形按照其左顶点的x值从小到大的顺序进行排列
// 且对于每个多边形，其最左边和最右边都只有唯一的一个点
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

        event_list.emplace_back(Event(i, polygon[leftmost_idx].x, polygon[leftmost_idx].y, IN));
        event_list.emplace_back(Event(i, polygon[rightmost_idx].x, polygon[rightmost_idx].y, OUT));

        // 假设多边形为凸壳 且各个顶点按照逆时针的顺序排列
        if (leftmost_idx < rightmost_idx)
        {
            for(int m = 0; m < polygon.size(); m++)
            {
                if(leftmost_idx < m && m < rightmost_idx)
                {
                    event_list.emplace_back(Event(i, polygon[m].x, polygon[m].y, CEILING));
                }
                if(m < leftmost_idx || m >rightmost_idx)
                {
                    event_list.emplace_back(Event(i, polygon[m].x, polygon[m].y, FLOOR));
                }
            }
        }
        else if(leftmost_idx > rightmost_idx)
        {
            for(int n = 0; n < polygon.size(); n++)
            {
                if(rightmost_idx < n && n < leftmost_idx)
                {
                    event_list.emplace_back(Event(i, polygon[n].x, polygon[n].y, FLOOR));
                }
                if(n < rightmost_idx || n > leftmost_idx)
                {
                    event_list.emplace_back(Event(i, polygon[n].x, polygon[n].y, CEILING));
                }
            }
        }
    }
    std::sort(event_list.begin(),event_list.end());
    return event_list;
}

std::deque<std::deque<Event>> SliceListGenerator(std::vector<Event> event_list)
{
    std::deque<std::deque<Event>> slice_list;
    std::deque<Event> slice;
    int x = event_list.front().x;

    for(int i = 0; i < event_list.size(); i++)
    {
        if(event_list[i].x != x)
        {
            slice_list.emplace_back(slice);

            x = event_list[i].x;
            slice.clear();
            slice.emplace_back(event_list[i]);
        }
        else
        {
            slice.emplace_back(event_list[i]);
        }
    }
    slice_list.emplace_back(slice);

    return slice_list;
}

void ExecuteOpenOperation(int curr_cell_idx, Point2D in, Point2D c, Point2D f) // in event  add two new node
{

    CellNode top_cell, bottom_cell;

    top_cell.ceiling.emplace_back(c);
    top_cell.floor.emplace_back(in);

    bottom_cell.ceiling.emplace_back(in);
    bottom_cell.floor.emplace_back(f);

    int top_cell_index = cell_graph.size();
    int bottom_cell_index = cell_graph.size() + 1;

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
    CellNode new_cell;

    new_cell.ceiling.emplace_back(c);
    new_cell.floor.emplace_back(f);

    int new_cell_idx = cell_graph.size();
    new_cell.cellIndex = new_cell_idx;

    cell_graph.emplace_back(new_cell);


    cell_graph[new_cell_idx].neighbor_indices.emplace_back(top_cell_idx);
    cell_graph[new_cell_idx].neighbor_indices.emplace_back(bottom_cell_idx);

    cell_graph[top_cell_idx].neighbor_indices.emplace_front(new_cell_idx);
    cell_graph[bottom_cell_idx].neighbor_indices.emplace_back(new_cell_idx);

}

void ExecuteCeilOperation(int curr_cell_idx, const Point2D& ceil_point) // finish constructing last ceiling edge
{
    cell_graph[curr_cell_idx].ceiling.emplace_back(ceil_point);
}

void ExecuteFloorOperation(int curr_cell_idx, const Point2D& floor_point) // finish constructing last floor edge
{
    cell_graph[curr_cell_idx].floor.emplace_back(floor_point);
}


void InitializeCellDecomposition(Point2D first_in_pos)
{
    CellNode cell_0;

    int cell_0_idx = 0;
    cell_0.cellIndex = cell_0_idx;

    for(int i = 0; i < first_in_pos.x; i++)
    {
        cell_0.ceiling.emplace_back(Point2D(i,0));
        cell_0.floor.emplace_back(Point2D(i,map.rows-1));
    }


    cell_graph.emplace_back(cell_0);
}

void FinishCellDecomposition(Point2D last_out_pos)
{
    int last_cell_idx = cell_graph.size()-1;

    // 封闭最后的ceil点和floor点
    for(int i = last_out_pos.x + 1; i <= map.cols-1; i++)
    {
        cell_graph[last_cell_idx].ceiling.emplace_back(Point2D(i, 0));
        cell_graph[last_cell_idx].floor.emplace_back(Point2D(i, map.rows-1));
    }

    unvisited_counter = cell_graph.size();
}



void DrawCells(const CellNode& cell)
{
    for(int i = 0; i < cell.ceiling.size(); i++)
    {
        map.at<cv::Vec3f>(cell.ceiling[i].y, cell.ceiling[i].x) = cv::Vec3f(255, 0, 0);
    }

    for(int i = 0; i < cell.floor.size(); i++)
    {
        map.at<cv::Vec3f>(cell.floor[i].y, cell.floor[i].x) = cv::Vec3f(255, 0, 0);
    }

    cv::line(map, cv::Point(cell.ceiling.front().x,cell.ceiling.front().y), cv::Point(cell.floor.front().x,cell.floor.front().y), cv::Scalar(255, 0, 0));
    cv::line(map, cv::Point(cell.ceiling.back().x,cell.ceiling.back().y), cv::Point(cell.floor.back().x,cell.floor.back().y), cv::Scalar(255, 0, 0));
}


std::vector<int> cell_index_slice; // 按y从小到大排列


std::deque<Event> SortSliceEvent(const std::deque<Event>& slice)
{
    std::deque<Event> sorted_slice(slice);
    std::deque<Event> sub_slice;

    for(int i = 0; i < sorted_slice.size(); i++)
    {
        sorted_slice[i].original_index_in_slice = i;
    }

    for(int i = 0; i < sorted_slice.size(); i++)
    {
        if(sorted_slice[i].event_type == IN)
        {
            sub_slice.emplace_back(sorted_slice[i]);
            sorted_slice.erase(sorted_slice.begin()+i);
        }
        if(sorted_slice[i].event_type == OUT)
        {
            sub_slice.emplace_back(sorted_slice[i]);
            sorted_slice.erase(sorted_slice.begin()+i);
        }
    }
    std::sort(sub_slice.begin(), sub_slice.end());
    sorted_slice.insert(sorted_slice.begin(), sub_slice.begin(), sub_slice.end());

    return sorted_slice;
}

int CountCells(const std::deque<Event>& slice, int curr_idx)
{
    int cell_num = 0;
    for(int i = 0; i < curr_idx; i++)
    {
        if(slice[i].event_type==IN)
        {
            cell_num++;
        }
        if(slice[i].event_type==FLOOR)
        {
            cell_num++;
        }
    }
    return cell_num;
}


/***
 * in.y在哪个cell的ceil和floor中间，就选取哪个cell作为in operation的curr cell， in上面的点为c, in下面的点为f
 * 若out.y在cell A的上界A_c以下，在cell B的下界B_f以上，则cell A为out operation的top cell， cell B为out operation的bottom cell，out上面的点为c, out下面的点为f
 * cell A和cell B在cell slice中必需是紧挨着的，中间不能插有别的cell
 * 对于每个slice，从上往下统计有几个in或floor（都可代表某个cell的下界），每有一个就代表该slice上有几个cell完成上下界的确定。ceil或floor属于该slice上的哪个cell，就看之前完成了对几个cell
 * 的上下界的确定，则ceil和floor是属于该slice上的下一个cell。应等执行完该slice中所有的in和out事件后再统计cell数。
 */

/**
 * TODO: 1. in and out event in different obstacles in the same slice
 *       2. concave obstacle case
 *       3. several in/out event in the same obstacle
 * **/

void ExecuteCellDecomposition(std::deque<std::deque<Event>> slice_list)
{
    int curr_cell_idx = INT_MAX;
    int top_cell_idx = INT_MAX;
    int bottom_cell_idx = INT_MAX;

    int slice_x = INT_MAX;
    int event_y = INT_MAX;

    std::deque<Event> sorted_slice;

    std::vector<int> sub_cell_index_slices;

    int cell_counter = 0;

    cell_index_slice.emplace_back(0); // initialization

    for(int i = 0; i < slice_list.size(); i++)
    {
        slice_x = slice_list[i].front().x;
        slice_list[i].emplace_front(Event(INT_MAX, slice_x, 0, CEILING));       // add map upper boundary
        slice_list[i].emplace_back(Event(INT_MAX, slice_x, map.rows-1, FLOOR)); // add map lower boundary

        sorted_slice = SortSliceEvent(slice_list[i]);

        for(int j = 0; j < sorted_slice.size(); j++)
        {
            if(sorted_slice[j].event_type == IN)
            {
                if(sorted_slice.size() == 3)
                {
                    curr_cell_idx = cell_index_slice.back();
                    ExecuteOpenOperation(curr_cell_idx, Point2D(sorted_slice[j].x, sorted_slice[j].y),
                                                        Point2D(slice_list[i][sorted_slice[j].original_index_in_slice-1].x, slice_list[i][sorted_slice[j].original_index_in_slice-1].y),
                                                        Point2D(slice_list[i][sorted_slice[j].original_index_in_slice+1].x, slice_list[i][sorted_slice[j].original_index_in_slice+1].y));
                    cell_index_slice.pop_back();
                    cell_index_slice.emplace_back(curr_cell_idx+1);
                    cell_index_slice.emplace_back(curr_cell_idx+2);

                    slice_list[i][sorted_slice[j].original_index_in_slice].isUsed = true;
                    slice_list[i][sorted_slice[j].original_index_in_slice-1].isUsed = true;
                    slice_list[i][sorted_slice[j].original_index_in_slice+1].isUsed = true;
                }
                else
                {
                    event_y = sorted_slice[j].y;
                    for(int k = 0; k < cell_index_slice.size(); k++)
                    {
                        if(event_y > cell_graph[cell_index_slice[k]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                        {
                            curr_cell_idx = cell_index_slice[k];
                            ExecuteOpenOperation(curr_cell_idx, Point2D(sorted_slice[j].x, sorted_slice[j].y),
                                                                Point2D(slice_list[i][sorted_slice[j].original_index_in_slice-1].x, slice_list[i][sorted_slice[j].original_index_in_slice-1].y),
                                                                Point2D(slice_list[i][sorted_slice[j].original_index_in_slice+1].x, slice_list[i][sorted_slice[j].original_index_in_slice+1].y));
                            cell_index_slice.erase(cell_index_slice.begin()+k);
                            sub_cell_index_slices.clear();
                            sub_cell_index_slices = {int(cell_graph.size()-2), int(cell_graph.size()-1)};
                            cell_index_slice.insert(cell_index_slice.begin()+k, sub_cell_index_slices.begin(), sub_cell_index_slices.end());

                            slice_list[i][sorted_slice[j].original_index_in_slice].isUsed = true;
                            slice_list[i][sorted_slice[j].original_index_in_slice-1].isUsed = true;
                            slice_list[i][sorted_slice[j].original_index_in_slice+1].isUsed = true;

                            break;
                        }
                    }
                }
            }
            if(sorted_slice[j].event_type == OUT)
            {
                event_y = sorted_slice[j].y;
                for(int k = 1; k < cell_index_slice.size(); k++)
                {
                    if(event_y > cell_graph[cell_index_slice[k-1]].ceiling.back().y && event_y < cell_graph[cell_index_slice[k]].floor.back().y)
                    {
                        top_cell_idx = cell_index_slice[k-1];
                        bottom_cell_idx = cell_index_slice[k];
                        ExecuteCloseOperation(top_cell_idx, bottom_cell_idx,
                                              Point2D(sorted_slice[j].x, sorted_slice[j].y),
                                              Point2D(slice_list[i][sorted_slice[j].original_index_in_slice-1].x, slice_list[i][sorted_slice[j].original_index_in_slice-1].y),
                                              Point2D(slice_list[i][sorted_slice[j].original_index_in_slice+1].x, slice_list[i][sorted_slice[j].original_index_in_slice+1].y));
                        cell_index_slice.erase(cell_index_slice.begin()+k-1);
                        cell_index_slice.erase(cell_index_slice.begin()+k-1);
                        cell_index_slice.insert(cell_index_slice.begin()+k-1, int(cell_graph.size()-1));

                        slice_list[i][sorted_slice[j].original_index_in_slice].isUsed = true;
                        slice_list[i][sorted_slice[j].original_index_in_slice-1].isUsed = true;
                        slice_list[i][sorted_slice[j].original_index_in_slice+1].isUsed = true;

                        break;
                    }
                }

            }
            if(sorted_slice[j].event_type == CEILING)
            {
                cell_counter = CountCells(slice_list[i],sorted_slice[j].original_index_in_slice);
                curr_cell_idx = cell_index_slice[cell_counter];
                if(!slice_list[i][sorted_slice[j].original_index_in_slice].isUsed)
                {
                    ExecuteCeilOperation(curr_cell_idx, Point2D(sorted_slice[j].x, sorted_slice[j].y));
                }
            }
            if(sorted_slice[j].event_type == FLOOR)
            {
                cell_counter = CountCells(slice_list[i],sorted_slice[j].original_index_in_slice);
                curr_cell_idx = cell_index_slice[cell_counter];
                if(!slice_list[i][sorted_slice[j].original_index_in_slice].isUsed)
                {
                    ExecuteFloorOperation(curr_cell_idx, Point2D(sorted_slice[j].x, sorted_slice[j].y));
                }
            }
        }
    }
}


std::vector<Point2D> ComputeCellCornerPoints(CellNode cell, int robot_radius=0)
{
    int c_size = cell.ceiling.size();
    int f_size = cell.floor.size();

    // 按照TOPLEFT、BOTTOMLEFT、BOTTOMRIGHT、TOPRIGHT的顺序储存corner points（逆时针）
    std::vector<Point2D> corner_points = {
            Point2D(cell.ceiling[robot_radius + 1].x, cell.ceiling[robot_radius + 1].y + (robot_radius + 1)),
            Point2D(cell.floor[robot_radius + 1].x, cell.floor[robot_radius + 1].y - (robot_radius + 1)),
            Point2D(cell.floor[f_size-1-(robot_radius + 1)].x, cell.floor[f_size-1-(robot_radius + 1)].y - (robot_radius + 1)),
            Point2D(cell.ceiling[c_size-1-(robot_radius + 1)].x, cell.ceiling[c_size-1-(robot_radius + 1)].y + (robot_radius + 1))

    };
    return corner_points;
}



Point2D FindNextEntrance(Point2D curr_point, CellNode next_cell, int& corner_indicator, int robot_radius=0)
{

    double distance = DBL_MAX;
    Point2D next_entrance = Point2D(INT_MAX, INT_MAX);

    std::vector<Point2D> next_points = ComputeCellCornerPoints(next_cell, robot_radius);

    for(int i = 0; i < next_points.size(); i++)
    {
        if(std::sqrt(pow((next_points[i].x-curr_point.x),2)+pow((next_points[i].y-curr_point.y),2))<distance)
        {
            distance = std::sqrt(pow((next_points[i].x-curr_point.x),2)+pow((next_points[i].y-curr_point.y),2));
            next_entrance.x = next_points[i].x;
            next_entrance.y = next_points[i].y;
            corner_indicator = i;
        }
    }
    return next_entrance;
}


std::deque<Point2D> ExitAlongWall(Point2D start, Point2D end, int end_corner_indicator, CellNode cell, int robot_radius=0)
{
    int start_corner_indicator = INT_MAX;

    std::vector<Point2D> corner_points = ComputeCellCornerPoints(cell, robot_radius);
    for(int i = 0; i < corner_points.size(); i++)
    {
        if(corner_points[i].x==start.x && corner_points[i].y == start.y)
        {
            start_corner_indicator = i;
            break;
        }
    }

    std::vector<Point2D> left, bottom, right, top;  // 从上往下或从左往右
    for(int y = corner_points[0].y; y < corner_points[1].y; y++)
    {
        left.emplace_back(Point2D(corner_points[0].x, y));
    }
    for(int i = robot_radius+1; i < cell.ceiling.size()-(robot_radius+1); i++)
    {
        top.emplace_back(Point2D(cell.ceiling[i].x,cell.ceiling[i].y+(robot_radius+1)));
        bottom.emplace_back(Point2D(cell.floor[i].x,cell.floor[i].y-(robot_radius+1)));
    }
    for(int y = corner_points[3].y; y < corner_points[2].y; y++)
    {
        right.emplace_back(Point2D(corner_points[3].x, y));
    }


    std::deque<Point2D> path;

    if (start_corner_indicator == end_corner_indicator)
    {
        return path;
    }

    if(start_corner_indicator == TOPLEFT && end_corner_indicator == TOPRIGHT)
    {
        path.insert(path.begin(), top.begin(), top.end());
        return path;
    }
    if(start_corner_indicator == TOPLEFT && end_corner_indicator == BOTTOMLEFT)
    {
        path.insert(path.begin(), left.begin(), left.end());
        return path;
    }
    if(start_corner_indicator == TOPLEFT && end_corner_indicator == BOTTOMRIGHT)
    {
        path.insert(path.begin(),left.begin(), left.end());
        path.insert(path.end(), bottom.begin(), bottom.end());
        return path;
    }
    if(start_corner_indicator == TOPRIGHT && end_corner_indicator == TOPLEFT)
    {
        path.insert(path.begin(), top.rbegin(), top.rend());
        return path;
    }
    if(start_corner_indicator == TOPRIGHT && end_corner_indicator == BOTTOMLEFT)
    {
        path.insert(path.begin(), top.rbegin(), top.rend());
        path.insert(path.end(), left.begin(), left.end());
        return path;
    }
    if(start_corner_indicator == TOPRIGHT && end_corner_indicator == BOTTOMRIGHT)
    {
        path.insert(path.begin(), right.begin(), right.end());
        return path;
    }
    if(start_corner_indicator == BOTTOMLEFT && end_corner_indicator == TOPLEFT)
    {
        path.insert(path.begin(), left.rbegin(), left.rend());
        return path;
    }
    if(start_corner_indicator == BOTTOMLEFT && end_corner_indicator == TOPRIGHT)
    {
        path.insert(path.begin(), bottom.begin(), bottom.end());
        path.insert(path.end(), right.rbegin(), right.rend());
        return path;
    }
    if(start_corner_indicator == BOTTOMLEFT && end_corner_indicator == BOTTOMRIGHT)
    {
        path.insert(path.begin(), bottom.begin(), bottom.end());
        return path;
    }
    if(start_corner_indicator == BOTTOMRIGHT && end_corner_indicator == TOPLEFT)
    {
        path.insert(path.begin(), right.rbegin(), right.rend());
        path.insert(path.end(), top.rbegin(), top.rend());
        return path;
    }
    if(start_corner_indicator == BOTTOMRIGHT && end_corner_indicator == TOPRIGHT)
    {
        path.insert(path.begin(), right.rbegin(), right.rend());
        return path;
    }
    if(start_corner_indicator == BOTTOMRIGHT && end_corner_indicator == BOTTOMLEFT)
    {
        path.insert(path.begin(), bottom.rbegin(), bottom.rend());
        return path;
    }

}


std::deque<Point2D> FindLinkingPath(Point2D curr_exit, Point2D next_entrance, CellNode curr_cell, CellNode next_cell, int robot_radius=0)
{
    std::deque<Point2D> path;
    std::deque<Point2D> sub_path;

    int exit_corner_indicator = INT_MAX;
    Point2D exit = FindNextEntrance(next_entrance, curr_cell, exit_corner_indicator, robot_radius);
    sub_path = ExitAlongWall(curr_exit, exit, exit_corner_indicator, curr_cell, robot_radius);
    path.insert(path.begin(), sub_path.begin(), sub_path.end());

    int delta_x = next_entrance.x - exit.x;
    int delta_y = next_entrance.y - exit.y;

    int increment_x = 0;
    int increment_y = 0;

    if (delta_x != 0) {
        increment_x = delta_x / std::abs(delta_x);
    }
    if (delta_y != 0) {
        increment_y = delta_y / std::abs(delta_y);
    }

    int upper_bound = INT_MIN;
    int lower_bound = INT_MAX;

    if (exit.x >= curr_cell.ceiling.back().x - (robot_radius + 1))
    {
        upper_bound = (curr_cell.ceiling.end() - 1 - (robot_radius + 1))->y;
        lower_bound = (curr_cell.floor.end() - 1 - (robot_radius + 1))->y;
    }
    if (exit.x <= curr_cell.ceiling.front().x + (robot_radius + 1))
    {
        upper_bound = (curr_cell.ceiling.begin() + (robot_radius + 1))->y;
        lower_bound = (curr_cell.floor.begin() + (robot_radius + 1))->y;
    }

    if ((next_entrance.y >= upper_bound) && (next_entrance.y <= lower_bound)) {
        for (int y = exit.y; y != next_entrance.y; y += increment_y) {
            path.emplace_back(Point2D(exit.x, y));
        }
        for (int x = exit.x; x != next_entrance.x; x += increment_x) {
            path.emplace_back(Point2D(x, next_entrance.y));
        }
    } else {
        for (int x = exit.x; x != next_entrance.x; x += increment_x) {
            path.emplace_back(Point2D(x, exit.y));
        }
        for (int y = exit.y; y != next_entrance.y; y += increment_y) {
            path.emplace_back(Point2D(next_entrance.x, y));
        }
    }

    return path;
}

std::deque<Point2D> PathIninitialization(Point2D start, CellNode cell, int robot_radius=0)
{
    std::deque<Point2D> path;

    int index_offset = std::abs(start.x - cell.ceiling.front().x);

    for(int y = start.y; y >= cell.ceiling[index_offset].y+(robot_radius+1); y--)
    {
        path.emplace_back(Point2D(start.x, y));
    }

    for(int i = index_offset; i >= robot_radius+1; i--)
    {
        path.emplace_back(Point2D(cell.ceiling[i].x,cell.ceiling[i].y+(robot_radius+1)));
    }

    return path;
}


//struct MapPoint
//{
//    double cost = 0.0;
//    double occupancy = 1.0;
//    Point2D prev_position = Point2D(INT_MAX, INT_MAX);
//    bool costComputed = false;
//};
//
//std::map<Point2D, MapPoint> cost_map;
//
//std::vector<Point2D> GetNeighbors(Point2D position)
//{
//    std::vector<Point2D> neighbors = {
//            Point2D(position.x-1, position.y-1),
//            Point2D(position.x, position.y-1),
//            Point2D(position.x+1, position.y-1),
//            Point2D(position.x-1, position.y),
//            Point2D(position.x+1, position.y),
//            Point2D(position.x-1, position.y+1),
//            Point2D(position.x, position.y+1),
//            Point2D(position.x+1, position.y+1)
//    };
//    return neighbors;
//}
//
//
//void BuildOccupancyMap() // CV_32FC3
//{
//    for(int i = 0; i < map.rows; i++)
//    {
//        for(int j = 0; j < map.cols; j++)
//        {
//            if(map.at<cv::Vec3f>(i,j) == cv::Vec3f(255,255,255))
//            {
//                cost_map[Point2D(j,i)].occupancy = INT_MAX;
//            }
//            else
//            {
//                cost_map[Point2D(j,i)].occupancy = 1.0;
//            }
//        }
//    }
//}
//
//void BuildCostMap(Point2D start)
//{
//    std::deque<Point2D> task_list = {start};
//    cost_map[start].costComputed = true;
//
//    std::vector<Point2D> neighbors;
//    std::vector<Point2D> candidates;
//
//    while(!task_list.empty())
//    {
//        candidates.clear();
//        neighbors = GetNeighbors(task_list.front());
//        for(auto neighbor:neighbors)
//        {
//            if(!cost_map[neighbor].costComputed)
//            {
//                candidates.emplace_back(neighbor);
//                cost_map[neighbor].cost = cost_map[task_list.front()].cost + 1.0;
//                cost_map[neighbor].prev_position = task_list.front();
//                cost_map[neighbor].costComputed = true;
//            }
//        }
//        task_list.pop_front();
//        task_list.insert(task_list.end(), candidates.begin(), candidates.end());
//    }
//
//}
//
//std::deque<Point2D> FindShortestPath(Point2D start, Point2D end)
//{
//    std::deque<Point2D> path = {end};
//
//    Point2D curr_positon = Point2D(end.x, end.y);
//
//    int prev_x = cost_map[curr_positon].prev_position.x;
//    int prev_y = cost_map[curr_positon].prev_position.y;
//
//
//    while(prev_x != start.x && prev_y != start.y)
//    {
//        path.emplace_front(Point2D(prev_x,prev_y));
//        curr_positon = Point2D(prev_x, prev_y);
//        prev_x = cost_map[curr_positon].prev_position.x;
//        prev_y = cost_map[curr_positon].prev_position.y;
//    }
//
//    path.emplace_front(start);
//
//    return path;
//}
//
//void ResetCostMap()
//{
//    for(int i = 0; i < map.rows; i++)
//    {
//        for(int j = 0; j < map.cols; j++)
//        {
//            cost_map[Point2D(j,i)].cost = 0.0;
//            cost_map[Point2D(j,i)].prev_position = Point2D(INT_MAX, INT_MAX);
//            cost_map[Point2D(j,i)].costComputed = false;
//        }
//    }
//}


int main() {

    map = cv::Mat::zeros(400, 400, CV_32FC3);
    Polygon polygon1, polygon2;
    cv::LineIterator line1(map, cv::Point(200,300), cv::Point(300,200));
    cv::LineIterator line2(map, cv::Point(300,200), cv::Point(200,100));
    cv::LineIterator line3(map, cv::Point(200,100), cv::Point(100,200));
    cv::LineIterator line4(map, cv::Point(100,200), cv::Point(200,300));

    for(int i = 0; i < line1.count-1; i++)
    {
        polygon1.emplace_back(Point2D(line1.pos().x, line1.pos().y));
        line1++;
    }
    for(int i = 0; i < line2.count-1; i++)
    {
        polygon1.emplace_back(Point2D(line2.pos().x, line2.pos().y));
        line2++;
    }
    for(int i = 0; i < line3.count-1; i++)
    {
        polygon1.emplace_back(Point2D(line3.pos().x, line3.pos().y));
        line3++;
    }
    for(int i = 0; i < line4.count-1; i++)
    {
        polygon1.emplace_back(Point2D(line4.pos().x, line4.pos().y));
        line4++;
    }

    cv::LineIterator line5(map, cv::Point(300,350), cv::Point(350,300));
    cv::LineIterator line6(map, cv::Point(350,300), cv::Point(300,250));
    cv::LineIterator line7(map, cv::Point(300,250), cv::Point(250,300));
    cv::LineIterator line8(map, cv::Point(250,300), cv::Point(300,350));
    for(int i = 0; i < line5.count-1; i++)
    {
        polygon2.emplace_back(Point2D(line5.pos().x, line5.pos().y));
        line5++;
    }
    for(int i = 0; i < line6.count-1; i++)
    {
        polygon2.emplace_back(Point2D(line6.pos().x, line6.pos().y));
        line6++;
    }
    for(int i = 0; i < line7.count-1; i++)
    {
        polygon2.emplace_back(Point2D(line7.pos().x, line7.pos().y));
        line7++;
    }
    for(int i = 0; i < line8.count-1; i++)
    {
        polygon2.emplace_back(Point2D(line8.pos().x, line8.pos().y));
        line8++;
    }


    PolygonList polygons = {polygon1, polygon2};
    std::vector<Event> event_list = EventListGenerator(polygons);


    std::deque<std::deque<Event>> slice_list = SliceListGenerator(event_list);
    InitializeCellDecomposition(Point2D(slice_list.front().front().x, slice_list.front().front().y));
    ExecuteCellDecomposition(slice_list);
    FinishCellDecomposition(Point2D(slice_list.back().back().x, slice_list.back().back().y));

    Point2D start_point = Point2D(150, 100);
    int start_cell_index = 1;
    int robot_radius = 5;

    std::deque<Point2D> first_steps = PathIninitialization(start_point, cell_graph[start_cell_index], robot_radius);
    WalkingThroughGraph(start_cell_index);


    for(int i = 0; i < cell_graph.size(); i++)
    {
        std::cout<<"cell "<<i<<" 's ceiling points number:" << cell_graph[i].ceiling.size()<<std::endl;
        std::cout<<"cell "<<i<<" 's floor points number:" << cell_graph[i].floor.size()<<std::endl;
    }

    std::cout<<cell_graph.size()<<std::endl;


    for(int i = 0; i < cell_graph.size(); i++)
    {
        for(int j = 0; j < cell_graph[i].neighbor_indices.size(); j++)
        {
            std::cout<<"cell "<< i << "'s neighbor: cell "<<cell_graph[cell_graph[i].neighbor_indices[j]].cellIndex<<std::endl;
        }
    }

    std::vector<cv::Point> contour1 = {cv::Point(200,300), cv::Point(300,200), cv::Point(200,100), cv::Point(100,200)};
    std::vector<cv::Point> contour2 = {cv::Point(300,350), cv::Point(350,300), cv::Point(300,250), cv::Point(250,300)};
    std::vector<std::vector<cv::Point>> contours = {contour1, contour2};
    cv::fillPoly(map, contours, cv::Scalar(255, 255, 255));
    cv::circle(map, cv::Point(start_point.x, start_point.y), 3, cv::Scalar(0, 0, 255), -1);

    cv::namedWindow("trajectory", cv::WINDOW_NORMAL);
    cv::imshow("trajectory", map);
    cv::waitKey(0);

    for(int i = 0; i < cell_graph.size(); i++)
    {
        DrawCells(cell_graph[i]);
        cv::imshow("trajectory", map);
        cv::waitKey(500);
    }

    for(int i = 0; i < first_steps.size(); i++)
    {
        map.at<cv::Vec3f>(first_steps[i].y, first_steps[i].x) = cv::Vec3f(0, 0, 255);
        cv::imshow("trajectory", map);
        cv::waitKey(1);
    }

    std::vector<Point2D> sub_path;
    int corner_indicator = TOPLEFT;

    for(int i = path.size()-1; i >= 0; i--)
    {
        sub_path = GetBoustrophedonPath(path[i], corner_indicator, robot_radius);
        for(int j = 0; j < sub_path.size(); j++)
        {
            map.at<cv::Vec3f>(sub_path[j].y, sub_path[j].x) = cv::Vec3f(0, 255, 255);
            cv::imshow("trajectory", map);
            cv::waitKey(1);
        }

        cell_graph[path[i].cellIndex].isCleaned = true;

        if((i-1)>=0)
        {
            Point2D curr_exit = sub_path.back();
            Point2D next_entrance = FindNextEntrance(curr_exit, path[i - 1], corner_indicator, robot_radius);
            std::deque<Point2D> link_path = FindLinkingPath(curr_exit, next_entrance, path[i], path[i-1], robot_radius);
            for(int k = 0; k < link_path.size(); k++)
            {
                map.at<cv::Vec3f>(link_path[k].y, link_path[k].x) = cv::Vec3f(0, 255, 0);
                cv::imshow("trajectory", map);
                cv::waitKey(1);
            }
        }
    }
    cv::waitKey(0);

    return 0;
}