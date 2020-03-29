#include "VectorOperations.h"
#include <algorithm>
#include <iterator> 
using namespace std;

double Norm_Inf(const double *vec, int n)
{
	double norm = -FLT_MAX;
	for (int i = 0; i < n; ++i)
		if (abs(vec[i]) > norm)
			norm = abs(vec[i]);
	return norm;
}

double Norm_Inf(const vector<double>& vec)
{
	double norm = -FLT_MAX;
	for (int i = 0; i < vec.size(); ++i)
		if (abs(vec[i]) > norm)
			norm = abs(vec[i]);
	return norm;
}

double Norm_1(const double *vec, int n)
{
	double norm = 0;
	for (int i = 0; i < n; ++i)
		norm += abs(vec[i]);
	return norm;
}

double Norm_1(const vector<double>& vec)
{
	double norm = 0;
	for (int i = 0; i < vec.size(); ++i)
		norm += abs(vec[i]);
	return norm;
}

double Dot(const double *vec1, const double *vec2, int n)
{
	double dot = 0;
	for (int i = 0; i < n; ++i)
	{
		dot += vec1[i] * vec2[i];
	}
	return dot;
}

double Dot(const vector<double>&vec1, const vector<double>&vec2)
{
	return Dot(&vec1[0], &vec2[0], vec1.size());
}

double LengthSquare(const double *vec, int n)
{
	return Dot(vec, vec, n);
}

double LengthSquare(const vector<double>& vec)
{
	return Dot(vec, vec);
}

double Length(const double *vec, int n)
{
	return sqrt(LengthSquare(vec, n));
}

double Length(const vector<double>& vec)
{
	return sqrt(LengthSquare(vec));
}

double Angle(const double *vec1, const double *vec2, int n)
{
	vector<double> vec1_cpy, vec2_cpy;
	copy(vec1, vec1 + n, back_inserter(vec1_cpy));
	copy(vec2, vec2 + n, back_inserter(vec2_cpy));
	double L1 = Norm_Inf(vec1, n);
	double L2 = Norm_Inf(vec2, n);
	for (int i = 0; i < n; ++i)
	{
		vec1_cpy[i] /= L1;
		vec2_cpy[i] /= L2;
	}
	return acos(Dot(vec1_cpy, vec2_cpy) / sqrt(Dot(vec1_cpy, vec1_cpy)) / sqrt(Dot(vec2_cpy, vec2_cpy)));
}

double Angle(const vector<double> &vec1, const vector<double>&vec2)
{
	return Angle(&vec1[0], &vec2[0], vec1.size());
}
