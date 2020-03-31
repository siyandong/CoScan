// ExactMethodForDGP.cpp: implementation of the CExactDGPMethod class.
//
//////////////////////////////////////////////////////////////////////
#include "ExactDGPMethod.h"
//#include <windows.h> // dsy: comment out.
//#include <gl/GL.h> // dsy: comment out.
//#include <gl/GLU.h> // dsy: comment out.
//#pragma comment(lib, "opengl32.lib") // dsy: comment out.
//#pragma comment(lib, "glu32.lib") // dsy: comment out.
#include <fstream>
#include <iterator>
#include <cassert>
#include "Parameters.h"
using namespace std;

// dsy - - -
#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif
/*template<class _BidIt> inline
void _Reverse(_BidIt _First, _BidIt _Last, bidirectional_iterator_tag)
{	
	// reverse elements in [_First, _Last), bidirectional iterators
	for (; _First != _Last && _First != --_Last; ++_First)
		::std:: iter_swap(_First, _Last);
}
template<class _BidIt> inline
void reverse(_BidIt _First, _BidIt _Last)
{	
	// reverse elements in [_First, _Last)
	//_DEBUG_RANGE(_First, _Last);
	_Reverse(_First._Unchecked(), _Unchecked(_Last), _Iter_cat(_First));
}*/
template<class _BidIt> inline
void reverse(_BidIt _First, _BidIt _Last)
{	
	// reverse elements in [_First, _Last), bidirectional iterators
	for (; _First != _Last && _First != --_Last; ++_First)
		::std:: iter_swap(_First, _Last);
}
// dsy - - -

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

void CExactDGPMethod::Initialize()
{
	CDistanceApproach::Initialize();		
	m_InfoAtVertices.resize(model.GetNumOfVerts());
}

CExactDGPMethod::CExactDGPMethod(const CRichModel& inputModel, int source) : CDistanceApproach(inputModel, source)
{
	m_nameOfAlgorithm = "Exact";
}
CExactDGPMethod::CExactDGPMethod(const CRichModel& inputModel, int source, int destination) : CDistanceApproach(inputModel, source, destination)
{
	m_nameOfAlgorithm = "Exact";
}

CExactDGPMethod::CExactDGPMethod(const CRichModel& inputModel, int source, double R) : CDistanceApproach(inputModel, source, R)
{
	m_nameOfAlgorithm = "Exact";
}

CExactDGPMethod::CExactDGPMethod(const CRichModel& inputModel, const set<int> &indexOfSourceVerts) : CDistanceApproach(inputModel, indexOfSourceVerts)
{
	m_nameOfAlgorithm = "Exact";
}

CExactDGPMethod::CExactDGPMethod(const CRichModel& inputModel, const set<int> &indexOfSourceVerts, double R) : CDistanceApproach(inputModel, indexOfSourceVerts, R)
{
	m_nameOfAlgorithm = "Exact";
}

CExactDGPMethod::CExactDGPMethod(const CRichModel& inputModel, const set<int> &indexOfSourceVerts, const set<int> &destinations) : CDistanceApproach(inputModel, indexOfSourceVerts, destinations)
{
	m_nameOfAlgorithm = "Exact";
}

CExactDGPMethod::CExactDGPMethod(const CRichModel& inputModel, const map<int, double> &indexOfSourceVerts) : CDistanceApproach(inputModel, indexOfSourceVerts)
{
	m_nameOfAlgorithm = "Exact";
}

CExactDGPMethod::CExactDGPMethod(const CRichModel& inputModel, const map<int, double> &indexOfSourceVerts, const set<int> &destinations) : CDistanceApproach(inputModel, indexOfSourceVerts, destinations)
{
	m_nameOfAlgorithm = "Exact";
}

vector<EdgePoint> CExactDGPMethod::BacktraceShortestPath(int end) const
{
	if (m_InfoAtVertices[end].birthTimeForCheckingValidity == -1
		|| m_InfoAtVertices[end].indexOfAncestor == -1)
	{
		assert(model.GetNumOfComponents() != 1 || model.Neigh(end).empty());
		return vector<EdgePoint>();
	}
	vector<EdgePoint> path;
	vector<int> vertexNodes;
	int index = end;
	vertexNodes.push_back(index);
	while (m_InfoAtVertices[index].indexOfDirectParent != -1)
	{
		int indexOfParent = m_InfoAtVertices[index].indexOfDirectParent;
		if (m_InfoAtVertices[index].fParentIsPseudoSource)
		{
			index = indexOfParent;
		}
		else
		{
			index = m_InfoAtVertices[index].indexOfRootVertOfDirectParent;
		}
		vertexNodes.push_back(index);
	};
	int indexOfSourceVert = index;

	for (int i = 0; i < (int)vertexNodes.size() - 1; ++i)
	{
		int lastVert = vertexNodes[i];
		path.push_back(EdgePoint(lastVert));
		if (m_InfoAtVertices[lastVert].fParentIsPseudoSource)
		{
			continue;
		}
		int parentEdgeIndex = m_InfoAtVertices[lastVert].indexOfDirectParent;
		int edgeIndex = model.Edge(parentEdgeIndex).indexOfReverseEdge;
		pair<double, double> coord(model.GetNew2DCoordinatesByReversingCurrentEdge(parentEdgeIndex, model.Edge(parentEdgeIndex).coordOfOppositeVert));
		
		double proportion = 1 - m_InfoAtVertices[lastVert].entryProp;
		while (true) 
		{
			path.push_back(EdgePoint(edgeIndex, proportion));
			if (model.Edge(edgeIndex).indexOfOppositeVert == vertexNodes[i + 1])
				break;
			double oldProprotion = proportion;
			proportion = model.ProportionOnLeftEdgeByImage(edgeIndex, coord, oldProprotion);
			if (abs(proportion - 1) < 1e-2)
			{
				vector<EdgePoint> path2 = BacktraceShortestPath(model.Edge(edgeIndex).indexOfOppositeVert);
				reverse(path.begin(), path.end());
				copy(path.begin(), path.end(), back_inserter(path2));
				return path2;
			}
			else if (proportion >= 0 && proportion <= 1)
			{
				proportion = max(proportion, 0);
				coord = model.GetNew2DCoordinatesByRotatingAroundLeftChildEdge(edgeIndex, coord);
				edgeIndex = model.Edge(edgeIndex).indexOfLeftEdge;
				//rightLen = disToAngle;				
			}
			else
			{
				if (model.IsExtremeEdge(edgeIndex))
				{
					int left = model.Edge(edgeIndex).indexOfLeftVert;
					int right = model.Edge(edgeIndex).indexOfRightVert;
					if (GetDistanceField()[left] < GetDistanceField()[right])
					{
						vector<EdgePoint> path2 = BacktraceShortestPath(left);
						reverse(path.begin(), path.end());
						copy(path.begin(), path.end(), back_inserter(path2));
						return path2;
					}
					else
					{
						vector<EdgePoint> path2 = BacktraceShortestPath(right);
						reverse(path.begin(), path.end());
						copy(path.begin(), path.end(), back_inserter(path2));
						return path2;
					}
				}
				else
				{
					proportion = model.ProportionOnRightEdgeByImage(edgeIndex, coord, oldProprotion);
					proportion = max(proportion, 0);
					proportion = min(proportion, 1);
					coord = model.GetNew2DCoordinatesByRotatingAroundRightChildEdge(edgeIndex, coord);
					edgeIndex = model.Edge(edgeIndex).indexOfRightEdge;
				}
			}
		};
	}
	path.push_back(EdgePoint(indexOfSourceVert));
	reverse(path.begin(), path.end());
	return path;
}

int CExactDGPMethod::GetAncestor(int vertex) const
{
	return m_InfoAtVertices[vertex].indexOfAncestor;
}

void CExactDGPMethod::CollectExperimentalResults()
{
	m_memory = ((double)model.GetNumOfVerts() * sizeof (InfoAtVertex)) / 1024 / 1024;
	for (int i = 0; i < m_scalarField.size(); ++i)
	{
		m_scalarField[i] = m_InfoAtVertices[i].disUptodate;
	}
	CDistanceApproach::CollectExperimentalResults();
}

void CExactDGPMethod::Dispose()
{
	//Do nothing...
}