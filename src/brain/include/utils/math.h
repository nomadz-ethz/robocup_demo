#pragma once

#include <cmath>
#include <vector>
#include "../types.h"

using namespace std;

// 角度转弧度
inline double deg2rad(double deg)
{
    return deg / 180.0 * M_PI;
}

// 弧度转角度
inline double rad2deg(double rad)
{
    return rad / M_PI * 180.0;
}

// 算术平均值
inline double mean(double x, double y)
{
    return (x + y) / 2;
}

// 把数字截断到一个范围内
inline double cap(double x, double upper_limit, double lower_limit)
{
    return max(min(x, upper_limit), lower_limit);
}

// 计算L2范数 (两个数的平方和开根号)
inline double norm(double x, double y)
{
    return sqrt(x * x + y * y);
}

// 计算L2范数 (两个数的平方和开根号)
inline double norm(vector<double> v)
{
    return sqrt(v[0] * v[0] + v[1] * v[1]);
}

// 把一个角度换算到 [-M_PI, M_PI) 区间.
inline double toPInPI(double theta)
{
    int n = static_cast<int>(fabs(theta / 2 / M_PI)) + 1;
    return fmod(theta + M_PI + 2 * n * M_PI, 2 * M_PI) - M_PI;
}

// 在任意直角坐标系中, 计算一个向量 v 与 x 轴的夹角 theta (rad), 取值范围: (-M_PI, M_PI)
inline double thetaToX(vector<double> v)
{
    vector<double> x = {1, 0};
    double ang = atan2(v[1], v[0]);
    return toPInPI(ang);
}

// 将一个平面坐标系 0 中的坐标, 转到坐标系 1 中, 坐标系 1 相对于 0 旋转 theta 角
inline Point2D transform(Point2D p0, double theta)
{
    Point2D p1;
    p1.x = p0.x * cos(theta) + p0.y * sin(theta);
    p1.y = -p0.x * sin(theta) + p0.y * cos(theta);
    return p1;
}

/**
 * @brief 将一个 source 坐标系(s) 中的 Pose (xs, ys, thetax) 转换到 target 坐标系(t)中 (xt, yt, thetat)
 *
 * @param xs, ys, thetas source 坐标系中一个 Pose 位置和朝向, theta 为弧度
 * @param xst, yst, thetast source 坐标系原点在 target 坐标系中的位置和朝向(st), theta 为弧度
 * @param xt, yt, thetat 输出该 Pose 在 target 坐标系中的位置和朝向, theta 为弧度
 */
inline void transCoord(const double &xs, const double &ys, const double &thetas, const double &xst, const double &yst, const double &thetast, double &xt, double &yt, double &thetat)
{
    thetat = toPInPI(thetas + thetast);
    xt = xst + xs * cos(thetast) - ys * sin(thetast);
    yt = yst + xs * sin(thetast) + ys * cos(thetast);
}