#pragma once

#include <vector>
#include "geometry_msgs/Point.h"
#include <Eigen/Geometry>
#include <cmath>
#include <Eigen/Dense>

using namespace std;

struct Point
{
    float r, theta;
    Point() : r(0), theta(0){};
    Point(float range, float angle): r(range), theta(angle){};

    float distToPoint(const Point* pt2) const
    {
        return sqrt(r*r + pt2->r*pt2->r - 2*r*pt2->r*cos(pt2->theta-theta));
    }

    float distToPoint2(const Point* pt2) const
    {
        return r*r + pt2->r*pt2->r - 2*r*pt2->r*cos(pt2->theta-theta);
    }

    float radialGap(const Point* pt2) const
    {
        return abs(r-pt2->r);
    }

    float getX()
    {
        return r*cos(theta);
    }

    float getY()
    {
        return r*sin(theta);
    }

    bool operator<(const Point& p)
    {
        return theta < p.theta;
    }

    bool operator>(const Point& p)
    {
        return theta > p.theta;
    }

    geometry_msgs::Point getPoint() const
    {
        geometry_msgs::Point p;
        p.x = r * cos(theta);
        p.y = r * sin(theta);
        p.z = 0.0;

        return p;
    }

    void wrapTheta()
    {
        while(theta > M_PI)
        {
            theta -= 2*M_PI;
        }
        while(theta < -M_PI)
        {
            theta += 2*M_PI;
        }
    }

    void rotate(float phi)
    {
        theta = theta + phi;
        wrapTheta();
    }

    void translate(float x, float y)
    {
        float new_x = getX() + x;
        float new_y = getY() + y;

        r = sqrt(new_x*new_x + new_y*new_y);
        theta = atan2(new_y, new_x);
    }

    Eigen::Vector2f getVector()
    {
        return Eigen::Vector2f(getX(), getY());
    }
};

struct Correspondence
{
    Point *p, *po, *pj1, *pj2;
    float pi_x, pi_y;
    Eigen::Vector2f v;

    Correspondence(Point *p, Point *po, Point *pj1, Point *pj2): p(p), po(po), pj1(pj1), pj2(pj2)
    {
        float p1_x = pj1->getX();
        float p1_y = pj1->getY();
        float p2_x = pj2->getX();
        float p2_y = pj2->getY();
        float p_x = p->getX();
        float p_y = p->getY();

        float d = (p1_x*p2_x + p1_y*p2_y + p1_x*p_x - p2_x*p_x + p1_y*p_y - p2_y*p_y - p1_x*p1_x - p1_y*p1_y)/(p1_x*p1_x - 2*p1_x*p2_x + p2_x*p2_x + p1_y*p1_y - 2*p1_y*p2_y + p2_y*p2_y);

        pi_x = p1_x + (p1_x - p2_x)*d;
        pi_y = p1_y + (p1_y - p2_y)*d;
        v << p1_y - p2_y, p2_x - p1_x;
    }

    Eigen::Vector2f getNormalNorm()
    {
        if (v.norm() > 0)
        {
            v = v/v.norm();
        }
        return v;
    }

    geometry_msgs::Point getPiGeo()
    {
        geometry_msgs::Point pi;
        pi.x = pi_x;
        pi.y = pi_y;

        return pi;
    }
};

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points, vector<vector<int>>& jump_table, vector<Correspondence>& c, float prob);

void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points, vector<vector<int>>& jump_table, vector<Correspondence>& c, float prob);

void computeJump(vector<vector<int>>& table, vector<Point>& points);
