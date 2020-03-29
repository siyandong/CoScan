#pragma once
#include "RichModel.h"
#include <fstream>
using namespace std;
struct FacePoint
{
	int faceID;
	double lamda1, lamda2, lamda3;

	FacePoint(){}
	FacePoint(int faceID, double lamda1, double lamda2)
	: faceID(faceID), lamda1(lamda1), lamda2(lamda2) 
	{
		lamda3 = 1 - lamda1 - lamda2;
	}

	CPoint3D Get3DPoint(const CRichModel& model) const
	{
		return lamda1 * model.GetShiftVertex(model.Face(faceID)[0])
			+ lamda2 * model.GetShiftVertex(model.Face(faceID)[1])
			+ lamda3 * model.GetShiftVertex(model.Face(faceID)[2]);
	}
	pair<double, double> Get2DCoord(const CRichModel& model, int edgeIndex) const
	{
		double x, y;
		if (model.Edge(edgeIndex).indexOfLeftVert == model.Face(faceID)[0])
		{
			x = lamda1 * 0 + lamda2 * model.Edge(edgeIndex).length + lamda3 * model.Edge(edgeIndex).coordOfOppositeVert.first;
			y = lamda1 * 0 + lamda2 * 0 + lamda3 * model.Edge(edgeIndex).coordOfOppositeVert.second;
		}
		else if (model.Edge(edgeIndex).indexOfLeftVert == model.Face(faceID)[1])
		{
			x = lamda2 * 0 + lamda3 * model.Edge(edgeIndex).length + lamda1 * model.Edge(edgeIndex).coordOfOppositeVert.first;
			y = lamda2 * 0 + lamda3 * 0 + lamda1 * model.Edge(edgeIndex).coordOfOppositeVert.second;
		}
		else
		{
			x = lamda3 * 0 + lamda1 * model.Edge(edgeIndex).length + lamda2 * model.Edge(edgeIndex).coordOfOppositeVert.first;
			y = lamda3 * 0 + lamda1 * 0 + lamda2 * model.Edge(edgeIndex).coordOfOppositeVert.second;
		}
		return make_pair(x, y);
	}
};