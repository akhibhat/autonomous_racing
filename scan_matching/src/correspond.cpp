#include "scan_matching/correspond.h"
#include "cmath"
#include "ros/ros.h"

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points, vector<vector<int>>& jump_table, vector<Correspondence>& c, float prob)
{
    c.clear();
    int last_best = -1;
    const int n = trans_points.size();
    const int m = old_points.size();
    float min_dist = 100000.00;
    int min_index = 0;
    int second_min_index = 0;

    // Do for each point
    for (int i=0; i<n; i++)
    {
        float min_dist = 10000000;
        int min_index = 0;
        for (int j=0; j<m; j++)
        {
            float dist = old_points[i].distToPoint2(&trans_points[j]);
            if (dist<min_dist)
            {
                min_dist = dist;
                min_index = j;
                second_min_index = j-1;
            }
        }

        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
    }
}

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points, vector<vector<int>>& jump_table, vector<Correspondence>& c, float prob)
{
    c.clear();
    int last_best = -1;
    const int n = trans_points.size();
    const int m = old_points.size();
    const int nrays = n;

    // Do for each point
    for (int i=0; i<n; ++i)
    {
        int best = 0;
        int second_best = 0;
        double best_dist = 10000000;
        int start_index = i; //int((trans_points[i].theta - old_points[0].theta) * (nrays/(2*3.14)));
    
        if (start_index <= 0)
        {
            start_index = 0;
        }
        else if (start_index >= m)
        {
            start_index = m-1;
        }
    
        int we_start_at;
        
        if (last_best != -1)
        {
            we_start_at = last_best + 1;
        }
        else
        {
            we_start_at = start_index;
        }

        int up = we_start_at + 1;
        int down = we_start_at;
    
        double last_dist_up = 10000000;
        double last_dist_down = 12000000;

        bool up_stopped = false;
        bool down_stopped = false;

        while (!(up_stopped && down_stopped))
        {
            bool now_up = !up_stopped && (last_dist_up < last_dist_down);

            if (now_up)
            {
                if (up >= m)
                {
                    up_stopped = true;
                    continue;
                }

                last_dist_up = trans_points[i].distToPoint2(&old_points[up]);

                if (last_dist_up < best_dist)
                {
                    best = up;
                    best_dist = last_dist_up;
                }

                if (up > start_index)
                {
                    double del_phi_up = old_points[up].theta - trans_points[i].theta;
                    double min_dist_up = trans_points[i].r * sin(del_phi_up);

                    if (pow(min_dist_up, 2) > best_dist)
                    {
                        up_stopped = true;
                        continue;
                    }
                    
                    if (old_points[up].r < trans_points[i].r)
                    {
                        up = jump_table[up][UP_SMALL];
                    }
                    else
                    {
                        up = jump_table[up][UP_BIG];
                    }
                }
                else
                {
                    up++;
                }
            }
            
            if (!now_up)
            {
                if (down <= -1)
                {
                    down_stopped = true;
                    continue;
                }

                last_dist_down = trans_points[i].distToPoint2(&old_points[down]);

                if (last_dist_down < best_dist)
                {
                    best = down;
                    best_dist = last_dist_down;
                }

                if (down < start_index)
                {
                    double del_phi_down = trans_points[i].theta - old_points[down].theta;
                    double min_dist_down = trans_points[i].r * sin(del_phi_down);

                    if (pow(min_dist_down, 2) > best_dist)
                    {
                        down_stopped = true;
                        continue;
                    }

                    if (old_points[down].r < trans_points[i].r)
                    {
                        down = jump_table[down][DOWN_BIG];
                    }
                    else
                    {
                        down = jump_table[down][DOWN_SMALL];
                    }
                }
                else
                {
                    down--;
                }
            }
        }

        last_best = best;

        second_best = best - 1;
//        if (best <= n/2)
//        {
//            second_best = best + 1;
//        }
//        else
//        {
//            second_best = best - 1;
//        }


        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[best], &old_points[second_best]));
    }
}

void computeJump(vector<vector<int>>& table, vector<Point>& points)
{
    table.clear();
    int n = points.size();

    for (int i=0; i<n; ++i)
    {
        vector<int> v = {n, n, -1, -1};
        for (int j=i+1; j<n; ++j)
        {
            if (points[j].r < points[i].r)
            {
                v[UP_SMALL] = j;
                break;
            }
        }

        for (int j=i+1; j<n; ++j)
        {
            if (points[j].r > points[i].r)
            {
                v[UP_BIG] = j;
                break;
            }
        }

        for (int j=i-1; j>=0; --j)
        {
            if (points[j].r < points[i].r)
            {
                v[DOWN_SMALL] = j;
                break;
            }
        }

        for (int j=i-1; j>=0; --j)
        {
            if (points[j].r > points[i].r)
            {
                v[DOWN_BIG] = j;
                break;
            }
        }

        table.push_back(v);
    }
}
