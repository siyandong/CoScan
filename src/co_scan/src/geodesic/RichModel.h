// RichModel.h: interface for the CRichModel class.
//
//////////////////////////////////////////////////////////////////////

#pragma once
#include "BaseModel.h"
#include <cassert>
#define _USE_MATH_DEFINES
#include <math.h>
//#include <windows.h>
using namespace std;
struct EdgePoint;

string MyGetUserName();

class CRichModel : public CBaseModel 
{
public:
	struct CEdge
	{
		int indexOfLeftVert;
		int indexOfRightVert;
		int indexOfOppositeVert;
		int indexOfLeftEdge;
		int indexOfRightEdge;
		int indexOfReverseEdge;
		int indexOfFrontFace;
		double length;
		double angleOpposite;
		pair<double, double> coordOfOppositeVert;
		// |unitX   -unitY|
		// |unitY    unitX|
		pair<double, double> matrixRotatedToLeftEdge;
		pair<double, double> matrixRotatedToRightEdge;
		CEdge()
		{		
			indexOfOppositeVert = -1;	//key	
			indexOfLeftEdge = -1;
			indexOfRightEdge = -1;
			indexOfFrontFace = -1;
			angleOpposite = 2 * M_PI;
		}
	};

protected:
	void CreateEdgesFromVertsAndFaces();
	void CollectAndArrangeNeighs();
	void ComputeAnglesAroundVerts();
	void ComputePlanarCoordsOfIncidentVertForEdges();
	void ComputeNumOfHoles();
	void ComputeNumOfComponents();
public:
	CRichModel(const string &filename);
	CRichModel(const vector<CPoint3D> &verts, const vector<CBaseModel::CFace> &faces);
	void LoadModel();
	void PreprocessBaseModel();
	int SplitEdge(const EdgePoint& ep);
	void SplitBasedOnScalarField(const vector<double>& scalarField,
		double val,
		const string& fileWithLargerScalars,
		const string& fileWithSmallerScalars);
	void SavePathToObj(const vector<EdgePoint>& pl, const string& filename) const;
	//void SavePathToObj(const vector<CPoint3D>& pl, const string& filename) const;
	void SaveIsolineToObj(const vector<EdgePoint>& isoline, const string& filename) const;
	void SetEdgeLength(int leftVert, int rightVert, double newLength);
	void FinishChangingEdgeLengths();
	void PrintInfo(ostream& out) const;
	double AngleSum(int vertIndex) const;
	int GetSubindexToVert(int root, int neigh) const;
	const CEdge& Edge(int edgeIndex) const;	
	const vector<pair<int, double> >& Neigh(int root) const;		
	//inline double Curvature(int vertIndex) const;
	//compute the proportion by two points
	double ProportionOnEdgeByImage(int edgeIndex, const pair<double, double> &coord) const;
	//compute the proportion on the left edge
	double ProportionOnLeftEdgeByImage(int edgeIndex, const pair<double, double> &coord, double proportion) const;
	//compute the proportion on the right edge
	double ProportionOnRightEdgeByImage(int edgeIndex, const pair<double, double> &coord, double proportion) const;
	//compute the proportion by two points
	double ProportionOnEdgeByImage(int edgeIndex, double x1, double y1, double x2, double y2) const;
	pair<double, double> GetNew2DCoordinatesByRotatingAroundLeftChildEdge(int edgeIndex, const pair<double, double>& input2DCoordinates) const;	
	pair<double, double> GetNew2DCoordinatesByRotatingAroundRightChildEdge(int edgeIndex, const pair<double, double>& input2DCoordinates) const;
	pair<double, double> GetNew2DCoordinatesByReversingCurrentEdge(int edgeIndex, const pair<double, double>& input2DCoordinates) const;
	double DistanceToOppositeAngle(int edgeIndex, const pair<double, double>& coord) const;
	double DistanceToLeftVert(int edgeIndex, const pair<double, double>& coord) const;
	double DistanceToRightVert(int edgeIndex, const pair<double, double>& coord) const;
	int GetNumOfEdges() const;
	int GetNumOfValidDirectedEdges() const;
	int GetNumOfTotalUndirectedEdges() const;
	int GetNumOfGenera() const;
	int GetNumOfIsolated() const;
	int GetNumOfComponents() const;
	int GetNumOfBoundries() const;
	bool IsStronglyConvexVert(int index) const;
	bool IsWeaklyConvexVert(int index) const;
	bool isBoundaryVert(int index) const;
	bool IsClosedModel() const;
	bool IsExtremeEdge(int edgeIndex) const;
	bool IsStartEdge(int edgeIndex) const;	
	int GetEdgeIndexFromTwoVertices(int leftVert, int rightVert) const;
	pair<double, double> GetTwoSplitAngles(int root, EdgePoint pt1, EdgePoint pt2) const;
	int IntersectQuery(int faceID, const pair<EdgePoint, EdgePoint>& seg1, const pair<EdgePoint, EdgePoint>& seg2, EdgePoint& intersection) const;
	double GetMaxEdgeLength() const;
	CPoint3D GetBarycentricCoord(CPoint3D pt, int faceID) const;
protected:
	int m_nBoundries;
	int m_nIsolatedVerts;
	int m_nComponents;
	double m_maxEdgeLength;
	vector<CEdge> m_Edges;
	set<int> m_UselessEdges;
	vector<vector<pair<int, double> > > m_NeighsAndAngles;	
	//<strongconvex, weakconvex>
	vector<pair<bool, bool>> m_FlagsForCheckingConvexVerts;
};
