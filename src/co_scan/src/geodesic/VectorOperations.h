#pragma once
#include <vector>
using namespace std;
double Norm_Inf(const double *vec, int n);
double Norm_Inf(const vector<double>& vec);
double Norm_1(const double *vec, int n);
double Norm_1(const vector<double>& vec);
double LengthSquare(const double *vec, int n);
double LengthSquare(const vector<double>& vec);
double Length(const double *vec, int n);
double Length(const vector<double>& vec);
double Dot(const double *vec1, const double *vec2, int n);
double Dot(const vector<double>&vec1, const vector<double>&vec2);
double Angle(const double *vec1, const double *vec2, int n);
double Angle(const vector<double> &vec1, const vector<double>&vec2);	