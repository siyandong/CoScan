#pragma once
#include "RichModel.h"
#include <fstream>
using namespace std;
struct EdgePoint
{
	bool isVertex;
	int index;
	double proportion; //[0 --> left endpoint; 1 --> right endpoint]
	EdgePoint()
	{
		index = -1;
	}
	EdgePoint(int index) : index(index), isVertex(true){}
	EdgePoint(int index, double proportion) : index(index), proportion(proportion), isVertex(false) {}
	EdgePoint(const CRichModel& model, int leftVert, int rightVert, double proportion) : proportion(proportion), isVertex(false) 
	{
		index = model.GetEdgeIndexFromTwoVertices(leftVert, rightVert);
	}
	bool operator==(const EdgePoint& other) const
	{
		if (isVertex)
			return isVertex == other.isVertex && index == other.index;
		return isVertex == other.isVertex && index == other.index && proportion == other.proportion;
	}
	bool operator!=(const EdgePoint& other) const
	{
		return !(*this == other);
	}
	void Reverse(const CRichModel& model)
	{
		if (isVertex)
			return;
		index = model.Edge(index).indexOfReverseEdge;
		proportion = 1 - proportion;
	}
	double GetNumbering(const CRichModel& model, int faceID) const
	{
		if (isVertex)
		{
			for (int i = 0; i < 3; ++i)
				if (model.Face(faceID)[i] == index)
				return i;
		}
		EdgePoint copy(*this);
		if (model.Edge(copy.index).indexOfFrontFace != faceID)
			copy.Reverse(model);
		for (int i = 0; i < 3; ++i)
		{
			if (model.Edge(copy.index).indexOfLeftVert == model.Face(faceID)[i])
				return i + copy.proportion;
		}
		return -1;
	}
	CPoint3D Get3DPoint(const CRichModel& model) const
	{
		if (isVertex)
			return model.Vert(index);
		return (1 - proportion) * model.Vert(model.Edge(index).indexOfLeftVert)
			+ proportion * model.Vert(model.Edge(index).indexOfRightVert);
	}
	CPoint3D GetShiftPoint(const CRichModel& model) const
	{
		if (isVertex)
			return model.GetShiftVertex(index);
		return (1 - proportion) * model.GetShiftVertex(model.Edge(index).indexOfLeftVert)
			+ proportion * model.GetShiftVertex(model.Edge(index).indexOfRightVert);
	}
	bool operator <(const EdgePoint& other) const
	{
		if (isVertex == false && other.isVertex == true)
			return true;
		if (isVertex == true && other.isVertex == false)
			return false;
		if (index < other.index)
			return true;
		if (index > other.index)
			return false;
		if (proportion < other.proportion)
			return true;
		if (proportion > other.proportion)
			return false;
		return false;
	}
	friend ostream& operator <<(ostream& out, const EdgePoint& ep)
	{
		if (ep.isVertex)
			out << "v " << ep.index << " ";
		else
			out << "e " << ep.index << " " << ep.proportion << " ";
		return out;
	}
};
//
//void SavePathToObj(const CRichModel& model, const vector<EdgePoint>& pl, const char* filename);
//void SaveIsolineToObj(const CRichModel& model, const vector<EdgePoint>& isoline, const char* filename);
