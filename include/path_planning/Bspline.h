#pragma once
#include<iostream>
#include <cmath>
#include <ctime>
#include<graphics.h>
#include<vector>
using namespace std;

constexpr auto Swidth = 1000;
constexpr auto Sheight = 1200;
constexpr auto deltaTIME = 50;

enum Type//B样条类型
{
	uniform,//均匀
	quniform//准均匀
};

class Point//点
{
public:
	double x;
	double y;
};

class Bspline //B样条曲线
{
public:
	Bspline(int _k, int _type, vector<Point> _p, bool _bDelayShow);
	~Bspline();
	void delay(int time); //延时函数，单位ms
	double BsplineBfunc(int i, int k, double uu);//计算每个u和每个i对应的B样条
	void creatBspline();//计算整个的B样条

public:
	int k;//阶数
	int n;//控制点数-1
	int type; //B样条类型
	vector<double> u;//自变量
	double delta_u = 0.01;//自变量间隔
	double uBegin;
	double uEnd;
	vector<Point> p;//控制点
	vector<Point> pTrack;//轨迹点
	bool bDelayShow = true;//是否显示曲线生成的过程
};

class BsplineTest//测试
{
public:
	void setPoint(vector<Point>& p);//定义点
	void myBsplineTest();//任意点测试
	void setPointHeart(vector<Point>& p1, vector<Point>& p2);//定义心形点
	void BsplineHeart();//画心
};