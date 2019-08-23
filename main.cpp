#include <iostream>
#include <vector>
#include <deque>


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



int main() {
    std::vector<Point2D> ceil = {Point2D(0,0),Point2D(1,0),Point2D(2,0),Point2D(3,0),Point2D(4,0)};
    std::vector<Point2D> floor = {Point2D(0,4),Point2D(1,4),Point2D(2,4),Point2D(3,4),Point2D(4,4)};

    std::vector<Point2D> path = GetBoustrophedonPath(ceil, floor);
    std::vector<Point2D>::iterator it = path.begin();

    return 0;
}