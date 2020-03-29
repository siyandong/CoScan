#include "RichModel.h"
#include "EdgePoint.h"
#include <queue>
#include <cassert>
#include <math.h>
#include <iostream>
#define _USE_MATH_DEFINES
#include "Parameters.h"
#include "../../include/eigen/Eigen/Dense"
//#include <Windows.h>
using namespace std;


CRichModel::CRichModel(const string& filename) : CBaseModel(filename)
{
}

CRichModel::CRichModel(const vector<CPoint3D> &verts, const vector<CBaseModel::CFace> &faces) : CBaseModel("")
{
	m_Verts = verts;
	m_Faces = faces;
	PreprocessBaseModel();
}

void CRichModel::CreateEdgesFromVertsAndFaces()
{
	m_Edges.clear();
	m_Edges.reserve(2 * (GetNumOfVerts() + GetNumOfFaces() - 2));
	map<pair<int, int>, int> pondOfUndeterminedEdges;
	int szFaces = GetNumOfFaces();
	for (int i = 0; i < szFaces; ++i)
	{		
		int threeIndices[3];
		for (int j = 0; j < 3; ++j)
		{
			int post = (j + 1) % 3;
			int pre = (j + 2) % 3;
			
			int leftVert = Face(i)[pre];
			int rightVert = Face(i)[j];

			map<pair<int, int>, int>::const_iterator it = pondOfUndeterminedEdges.find(make_pair(leftVert, rightVert));
			if (it != pondOfUndeterminedEdges.end())
			{
				int posInEdgeList = it->second;
				if (m_Edges[posInEdgeList].indexOfOppositeVert != -1)
				{
					throw "Repeated edges!";
				}
				threeIndices[j] = posInEdgeList;
				m_Edges[posInEdgeList].indexOfOppositeVert = Face(i)[post];
				m_Edges[posInEdgeList].indexOfFrontFace = i;				
			}
			else
			{
				CEdge edge;
				edge.indexOfLeftVert = leftVert;
				edge.indexOfRightVert = rightVert;
				edge.indexOfFrontFace = i;
				edge.indexOfOppositeVert = Face(i)[post];
				edge.indexOfReverseEdge = (int)m_Edges.size() + 1;
				edge.length = (Vert(leftVert) - Vert(rightVert)).Len();
				m_Edges.push_back(edge);
				pondOfUndeterminedEdges[make_pair(leftVert, rightVert)] = threeIndices[j] = (int)m_Edges.size() - 1;

				edge.indexOfLeftVert = rightVert;
				edge.indexOfRightVert = leftVert;
				edge.indexOfReverseEdge = (int)m_Edges.size() - 1;
				edge.indexOfOppositeVert = -1;
				edge.indexOfFrontFace = -1;
				m_Edges.push_back(edge);
				pondOfUndeterminedEdges[make_pair(rightVert, leftVert)] = (int)m_Edges.size() - 1;
			}
		}
		for (int j = 0; j < 3; ++j)
		{
			m_Edges[threeIndices[j]].indexOfLeftEdge = Edge(threeIndices[(j + 2) % 3]).indexOfReverseEdge;
			m_Edges[threeIndices[j]].indexOfRightEdge = Edge(threeIndices[(j + 1) % 3]).indexOfReverseEdge;
		}
	}
	//m_Edges.swap(vector<CEdge>(m_Edges)); // dsy: comment out origin code.
}

void CRichModel::CollectAndArrangeNeighs()
{
	m_nIsolatedVerts = 0;
	vector<int> sequenceOfDegrees(GetNumOfVerts(), 0);	
	m_NeighsAndAngles.clear();
	m_NeighsAndAngles.resize(GetNumOfVerts());
	for (int i = 0; i < (int)m_NeighsAndAngles.size(); ++i)
	{
		m_NeighsAndAngles[i].resize(1, make_pair(-1, 0));
	}
	for (int i = 0; i < (int)GetNumOfEdges(); ++i)
	{
		const CEdge& edge = Edge(i);
		++sequenceOfDegrees[edge.indexOfLeftVert];
		int &indexOfStartEdge = m_NeighsAndAngles[edge.indexOfLeftVert][0].first;
		if (indexOfStartEdge == -1 || !IsStartEdge(indexOfStartEdge))
		{
			indexOfStartEdge = i;
		}
		else if (IsStartEdge(i))
		{
			m_NeighsAndAngles[edge.indexOfLeftVert].push_back(make_pair(i, 0));
		}
	}
	for (int i = 0; i < GetNumOfVerts(); ++i)
	{
		if (m_NeighsAndAngles[i][0].first == -1)
		{
			m_NeighsAndAngles[i].clear();
			m_nIsolatedVerts++;	
			continue;
		}
		vector<int> startEdges;
		for (int j = 0; j < (int)Neigh(i).size(); ++j)
		{
			startEdges.push_back(Neigh(i)[j].first);
		}	
		m_NeighsAndAngles[i].resize(sequenceOfDegrees[i], make_pair(0, 0));
		int num(0);
		for (int j = 0; j < (int)startEdges.size(); ++j)
		{
			int curEdge = startEdges[j];			
			while (1)
			{
				m_NeighsAndAngles[i][num].first = curEdge;
				++num;
				if (num >= sequenceOfDegrees[i])
					break;
				if (IsExtremeEdge(curEdge))
					break;
				curEdge = Edge(curEdge).indexOfLeftEdge;
				if (curEdge == startEdges[j])
				{
					break;
				}
			}
		}
		if (num != sequenceOfDegrees[i])
		{
			throw "Complex vertices";
		}
	}
}

void CRichModel::ComputeAnglesAroundVerts()
{	
	m_FlagsForCheckingConvexVerts.clear();
	m_FlagsForCheckingConvexVerts.resize(GetNumOfVerts());
	//for (int i = 0; i < (int)m_NeighsAndAngles.size(); ++i)
	//{
	//	m_NeighsAndAngles[i].resize(Neigh(i).size());
	//}
	for (int i = 0; i < (int)m_NeighsAndAngles.size(); ++i)
	{
		double angleSum(0);
		for (int j = 0; j < (int)m_NeighsAndAngles[i].size(); ++j)
		{
			if (IsExtremeEdge(Neigh(i)[j].first))
				m_NeighsAndAngles[i][j].second = 2 * M_PI + 0.1;
			else
			{
				int next = j + 1;
				if (next >= (int)m_NeighsAndAngles[i].size())
				{
					next = 0;
				}
				double l = Edge(Neigh(i)[j].first).length;
				double r = Edge(Neigh(i)[next].first).length;
				double b = Edge(Edge(Neigh(i)[j].first).indexOfRightEdge).length;				
				m_NeighsAndAngles[i][j].second = acos((l * l + r * r - b * b) / (2 * l * r));
				m_Edges[Edge(Edge(Neigh(i)[j].first).indexOfRightEdge).indexOfReverseEdge].angleOpposite = m_NeighsAndAngles[i][j].second;
			}
			angleSum += m_NeighsAndAngles[i][j].second;			
		}
		m_FlagsForCheckingConvexVerts[i].first = (angleSum < 2 * M_PI - 5 * AngleTolerance);
		m_FlagsForCheckingConvexVerts[i].second = (angleSum < 2 * M_PI + 5 * AngleTolerance);
	}

}

void CRichModel::ComputePlanarCoordsOfIncidentVertForEdges()
{
	for (int i = 0; i < GetNumOfEdges(); ++i)
	{
		if (IsExtremeEdge(i))
			continue;
		double bottom = Edge(i).length;
		double leftLen = Edge(Edge(i).indexOfLeftEdge).length;
		double squareOfLeftLen = leftLen * leftLen;
		double rightLen = Edge(Edge(i).indexOfRightEdge).length;
		double x = (squareOfLeftLen - rightLen * rightLen) / bottom + bottom;
		x /= 2.0;
		m_Edges[i].coordOfOppositeVert = make_pair(x, sqrt(max(0.0, squareOfLeftLen - x * x)));
	}
	for (int i = 0; i < GetNumOfEdges(); ++i)
	{
		if (IsExtremeEdge(i))
			continue;
		{
			int reverseEdge = m_Edges[m_Edges[i].indexOfLeftEdge].indexOfReverseEdge;
			pair<double, double> coord = GetNew2DCoordinatesByReversingCurrentEdge(reverseEdge, m_Edges[reverseEdge].coordOfOppositeVert);
			double scale = abs(coord.first) + abs(coord.second);
			coord.first /= scale;
			coord.second /= scale;
			double len = sqrt(coord.first * coord.first + coord.second * coord.second);
			m_Edges[i].matrixRotatedToLeftEdge = make_pair(coord.first / len, coord.second / len);
		}
		{
			int reverseEdge = m_Edges[m_Edges[i].indexOfRightEdge].indexOfReverseEdge;
			double rightX = m_Edges[reverseEdge].length;
			double rightY = 0;
			double leftX = m_Edges[reverseEdge].length - m_Edges[reverseEdge].coordOfOppositeVert.first;
			double leftY = -m_Edges[reverseEdge].coordOfOppositeVert.second;

			double detaX = rightX - leftX;
			double detaY = rightY - leftY;
			double scale = abs(detaX) + abs(detaY);
			detaX /= scale;
			detaY /= scale;
			double len = sqrt(detaX * detaX + detaY * detaY);
			m_Edges[i].matrixRotatedToRightEdge = make_pair(detaX / len, detaY / len);
		}
	}
}

void CRichModel::FinishChangingEdgeLengths()
{
	m_maxEdgeLength = 0;
	for (int i = 0; i < GetNumOfEdges(); ++i)
	{
		if (Edge(i).length > m_maxEdgeLength)
			m_maxEdgeLength = Edge(i).length;
	}
	
	//Perform the following 
	//when edge lengths are changed.
	//compute angles around a vertex
	ComputeAnglesAroundVerts();
	//planar unfolding
	ComputePlanarCoordsOfIncidentVertForEdges();
}

void CRichModel::PreprocessBaseModel()
{
	//build edges and compute lengths
	//cerr << "Line " << __LINE__ << endl;
	CreateEdgesFromVertsAndFaces();
	m_maxEdgeLength = 0;
	for (int i = 0; i < GetNumOfEdges(); ++i)
	{
		if (Edge(i).length > m_maxEdgeLength)
			m_maxEdgeLength = Edge(i).length;
	}
	//cerr << "Line " << __LINE__ << endl;
	//build edges incident to a vertex
	CollectAndArrangeNeighs();	
	//cerr << "Line " << __LINE__ << endl;
	//num of open boundaries
	ComputeNumOfHoles();
	//num of components
	//cerr << "Line " << __LINE__ << endl;
	ComputeNumOfComponents();
	//cerr << "Line " << __LINE__ << endl;
	//Perform the following 
	//when edge lengths are changed.
	//compute angles around a vertex
	ComputeAnglesAroundVerts();
	//planar unfolding
	//cerr << "Line " << __LINE__ << endl;
	ComputePlanarCoordsOfIncidentVertForEdges();
}

void CRichModel::LoadModel()
{
	CBaseModel::LoadModel();
	//build edges and compute lengths
	CreateEdgesFromVertsAndFaces();
	m_maxEdgeLength = 0;
	for (int i = 0; i < GetNumOfEdges(); ++i)
	{
		if (Edge(i).length > m_maxEdgeLength)
			m_maxEdgeLength = Edge(i).length;
	}
	//build edges incident to a vertex
	CollectAndArrangeNeighs();	
	//num of open boundaries
	ComputeNumOfHoles();
	//num of components
	ComputeNumOfComponents();

	//Perform the following 
	//when edge lengths are changed.
	//compute angles around a vertex
	ComputeAnglesAroundVerts();
	//planar unfolding
	ComputePlanarCoordsOfIncidentVertForEdges();
}


void CRichModel::ComputeNumOfHoles()
{
	m_nBoundries = 0;
	if (IsClosedModel())
	{
		return;
	}
	set<int> extremeEdges;
	for (int i = 0; i < (int)m_Edges.size(); ++i)
	{
		if (m_Edges[i].indexOfOppositeVert != -1)
			continue;
		extremeEdges.insert(i);
	}		

	while (!extremeEdges.empty())
	{
		++m_nBoundries;
		int firstEdge = *extremeEdges.begin();
		int edge = firstEdge;
		do
		{			
			extremeEdges.erase(edge);
			int root = Edge(edge).indexOfRightVert;
			int index = GetSubindexToVert(root, Edge(edge).indexOfLeftVert);
			edge  = Neigh(root)[(index - 1 + (int)Neigh(root).size()) % (int)Neigh(root).size()].first;		
		} while (edge != firstEdge && !extremeEdges.empty());
	}
}

void CRichModel::ComputeNumOfComponents()
{
	m_nComponents = 0;
	vector<bool> flags(GetNumOfVerts(), false);
	int cnt(0);
	while (cnt < GetNumOfVerts())
	{
		int v;
		for (int i = 0; i < (int)flags.size(); ++i)
		{
			if (!flags[i]) 
			{
				v = i;
				break;
			}
		}
		queue<int> Que;
		Que.push(v);
		while (!Que.empty())
		{
			int v = Que.front();
			Que.pop();
			if (flags[v])
				continue;
			flags[v] = true;
			cnt++;
			for (int i = 0; i < (int)Neigh(v).size(); ++i)
			{
				if (!flags[Edge(Neigh(v)[i].first).indexOfRightVert])
				{
					Que.push(Edge(Neigh(v)[i].first).indexOfRightVert);
				}
			}
		}
		m_nComponents++;
	}
}

void CRichModel::PrintInfo(ostream& out) const
{
	out << "Model info is as follows.\n";
	out << "Name: " << GetFileShortName() << endl;
	out << "VertNum = " << GetNumOfVerts() << endl;
	out << "FaceNum = " << GetNumOfFaces() << endl;
	out << "EdgeNum = " << GetNumOfEdges() << endl;
	out << "Scale = " << m_scale << endl;
	if (!IsClosedModel())
		out << "BoundaryNum = " << GetNumOfBoundries() << endl;
	out << "Genus = " << GetNumOfGenera() << endl;
	if (IsClosedModel())
		out << "It is a closed model.\n";
	else
		out << "It is an open model.\n";
	if (GetNumOfComponents() != 1)
		out << "It has " << GetNumOfComponents() << " components.\n";
}

void CRichModel::SetEdgeLength(int leftVert, int rightVert, double newLength)
{
	int edgeID = GetEdgeIndexFromTwoVertices(leftVert, rightVert);
	int reverseID = Edge(edgeID).indexOfReverseEdge;
	m_Edges[edgeID].length = newLength;
	m_Edges[reverseID].length = newLength;
}


double CRichModel::AngleSum(int vertIndex) const
{
	double angleSum(0);
	for (int j = 0; j < (int)m_NeighsAndAngles[vertIndex].size(); ++j)
	{		
		angleSum += m_NeighsAndAngles[vertIndex][j].second;			
	}
	return angleSum;
}

pair<double, double> CRichModel::GetTwoSplitAngles(int root, EdgePoint pt1, EdgePoint pt2) const
{
	if (!pt1.isVertex && pt2.isVertex)
	{
		pair<double, double> splitAngles = GetTwoSplitAngles(root, pt2, pt1);
		return make_pair(splitAngles.second, splitAngles.first);
	}
	if (!pt1.isVertex)
	{
		if (Edge(pt1.index).indexOfOppositeVert != root)
		{
			pt1.proportion = 1 - pt1.proportion;
			pt1.index = Edge(pt1.index).indexOfReverseEdge;
			assert(Edge(pt1.index).indexOfOppositeVert == root);
		}
	}
	if (!pt2.isVertex)
	{
		if (Edge(pt2.index).indexOfOppositeVert != root)
		{
			pt2.proportion = 1 - pt2.proportion;
			pt2.index = Edge(pt2.index).indexOfReverseEdge;
			assert(Edge(pt2.index).indexOfOppositeVert == root);
		}
	}

	double rightAngleSum(0);
	double leftAngleSum(0);
	if (pt1.isVertex && pt2.isVertex)
	{
		int index1 = GetSubindexToVert(root, pt1.index);
		int index2 = GetSubindexToVert(root, pt2.index);
		int index = index1;
		while (index != index2)
		{
			rightAngleSum += Neigh(root)[index].second;
			index = (index + 1) % Neigh(root).size();
		}
		index = index2;
		leftAngleSum = Neigh(root)[index].second;
		index = (index + 1) % Neigh(root).size();
		while (index != index1)
		{
			leftAngleSum += Neigh(root)[index].second;
			index = (index + 1) % Neigh(root).size();
		}		
	}
	else if (pt1.isVertex && !pt2.isVertex)
	{
		int index1 = GetSubindexToVert(root, pt1.index);
		int index2 = GetSubindexToVert(root, Edge(pt2.index).indexOfLeftVert);
		int index = index1;
		while (index != index2)
		{
			rightAngleSum += Neigh(root)[index].second;
			index = (index + 1) % Neigh(root).size();
		}
		int index3 = GetSubindexToVert(root, Edge(pt2.index).indexOfRightVert);
		index = index3;
		while (index != index1)
		{
			leftAngleSum += Neigh(root)[index].second;
			index = (index + 1) % Neigh(root).size();
		}

		if (pt2.proportion < 1e-2
			|| pt2.proportion > 1 - 1e-2
			|| Edge(pt2.index).angleOpposite < 5.0 * M_PI / 180)
		{
			double leftAngle2 = pt2.proportion * Edge(pt2.index).angleOpposite;
			double rightAngle2 = (1 - pt2.proportion) * Edge(pt2.index).angleOpposite;
			rightAngleSum += leftAngle2;
			leftAngleSum += rightAngle2;
		}
		else
		{
			double c = Edge(pt2.index).length * pt2.proportion;
			double a = Edge(Edge(pt2.index).indexOfLeftEdge).length;
			double detaX = c - Edge(pt2.index).coordOfOppositeVert.first;
			double detaY = 0 - Edge(pt2.index).coordOfOppositeVert.second;
			double b = sqrt(detaX * detaX + detaY * detaY);
			double leftAngle2 = acos((a * a + b * b - c * c)/(2.0 * a * b));
			double rightAngle2 = Edge(pt2.index).angleOpposite - leftAngle2;
			rightAngleSum += leftAngle2;
			leftAngleSum += rightAngle2;
		}
	}
	else
	{
		assert(!pt1.isVertex && !pt2.isVertex);
		int index1 = GetSubindexToVert(root, Edge(pt1.index).indexOfLeftVert);
		int index2 = GetSubindexToVert(root, Edge(pt1.index).indexOfRightVert);
		int index3 = GetSubindexToVert(root, Edge(pt2.index).indexOfLeftVert);
		int index4 = GetSubindexToVert(root, Edge(pt2.index).indexOfRightVert);
		if (index1 == index3)
		{
			double angleSum = 0;
			for (int i = 0; i < Neigh(root).size(); ++i)
				angleSum += Neigh(root)[i].second;
			if (abs(pt1.proportion - pt2.proportion) < 1e-2
				|| Edge(pt1.index).angleOpposite < 5.0 * M_PI / 180)
			{		
				if (pt1.proportion > pt2.proportion)
				{
					leftAngleSum = (pt1.proportion - pt2.proportion) * Edge(pt1.index).angleOpposite;
					rightAngleSum = angleSum - leftAngleSum;
				}
				else
				{
					rightAngleSum = (pt2.proportion - pt1.proportion) * Edge(pt1.index).angleOpposite;
					leftAngleSum = angleSum - rightAngleSum;
				}
			}
			else if (pt1.proportion > pt2.proportion)
			{
				double c = (pt1.proportion - pt2.proportion) * Edge(pt1.index).length;
				double detaX1 = pt1.proportion * Edge(pt1.index).length - Edge(pt1.index).coordOfOppositeVert.first;
				double detaY1 = 0 - Edge(pt1.index).coordOfOppositeVert.second;
				double a = sqrt(detaX1 * detaX1 + detaY1 * detaY1);
				double detaX2 = pt2.proportion * Edge(pt2.index).length - Edge(pt2.index).coordOfOppositeVert.first;
				double detaY2 = 0 - Edge(pt2.index).coordOfOppositeVert.second;
				double b = sqrt(detaX2 * detaX2 + detaY2 * detaY2);
				leftAngleSum = acos((a * a + b * b - c * c)/(2 * a * b));
				rightAngleSum = angleSum - leftAngleSum;
			}
			else
			{
				double c = (pt1.proportion - pt2.proportion) * Edge(pt1.index).length;
				double detaX1 = pt1.proportion * Edge(pt1.index).length - Edge(pt1.index).coordOfOppositeVert.first;
				double detaY1 = 0 - Edge(pt1.index).coordOfOppositeVert.second;
				double a = sqrt(detaX1 * detaX1 + detaY1 * detaY1);
				double detaX2 = pt2.proportion * Edge(pt2.index).length - Edge(pt2.index).coordOfOppositeVert.first;
				double detaY2 = 0 - Edge(pt2.index).coordOfOppositeVert.second;
				double b = sqrt(detaX2 * detaX2 + detaY2 * detaY2);
				rightAngleSum = acos((a * a + b * b - c * c)/(2 * a * b));
				leftAngleSum = angleSum - rightAngleSum;
			}
		}
		else
		{
			double leftAngle1, rightAngle1, leftAngle2, rightAngle2;
			if (pt1.proportion < 1e-2
				|| pt1.proportion > 1 - 1e-2
				|| Edge(pt1.index).angleOpposite < 5.0 * M_PI / 180)
			{
				leftAngle1 = pt1.proportion * Edge(pt1.index).angleOpposite;
				rightAngle1 = (1 - pt1.proportion) * Edge(pt1.index).angleOpposite;
			}
			else
			{
				double c = Edge(pt1.index).length * pt1.proportion;
				double a = Edge(Edge(pt1.index).indexOfLeftEdge).length;
				double detaX = c - Edge(pt1.index).coordOfOppositeVert.first;
				double detaY = 0 - Edge(pt1.index).coordOfOppositeVert.second;
				double b = sqrt(detaX * detaX + detaY * detaY);
				leftAngle1 = acos((a * a + b * b - c * c)/(2.0 * a * b));
				rightAngle1 = Edge(pt1.index).angleOpposite - leftAngle1;
			}
			if (pt2.proportion < 1e-2
				|| pt2.proportion > 1 - 1e-2
				|| Edge(pt2.index).angleOpposite < 5.0 * M_PI / 180)
			{
				leftAngle2 = pt2.proportion * Edge(pt2.index).angleOpposite;
				rightAngle2 = (1 - pt2.proportion) * Edge(pt2.index).angleOpposite;
			}
			else
			{
				double c = Edge(pt2.index).length * pt2.proportion;
				double a = Edge(Edge(pt2.index).indexOfLeftEdge).length;
				double detaX = c - Edge(pt2.index).coordOfOppositeVert.first;
				double detaY = 0 - Edge(pt2.index).coordOfOppositeVert.second;
				double b = sqrt(detaX * detaX + detaY * detaY);
				leftAngle2 = acos((a * a + b * b - c * c)/(2.0 * a * b));
				rightAngle2 = Edge(pt2.index).angleOpposite - leftAngle2;
			}
			leftAngleSum = leftAngle1 + rightAngle2;
			rightAngleSum = rightAngle1 + leftAngle2;
			int index = index2;
			while (index != index3)
			{
				rightAngleSum += Neigh(root)[index].second;
				index = (index + 1) % Neigh(root).size();
			}
			index = index4;
			while (index != index1)
			{
				leftAngleSum += Neigh(root)[index].second;
				index = (index + 1) % Neigh(root).size();
			}
		}
	}
	return make_pair(leftAngleSum, rightAngleSum);
}


int CRichModel::GetNumOfValidDirectedEdges() const
{
	return (int)m_Faces.size() * 3;
}

int CRichModel::GetNumOfTotalUndirectedEdges() const
{
	return (int)m_Edges.size() / 2;
}

int CRichModel::GetNumOfGenera() const
{
	return int(GetNumOfTotalUndirectedEdges() - (GetNumOfVerts() - m_nIsolatedVerts) - GetNumOfFaces() - GetNumOfBoundries()) / 2 + 1;
}

int CRichModel::GetNumOfComponents() const
{
	return m_nComponents;
}

int CRichModel::GetNumOfBoundries() const
{
	return m_nBoundries;
}

bool CRichModel::IsClosedModel() const
{
	return GetNumOfValidDirectedEdges() ==  GetNumOfEdges();
}

int CRichModel::GetNumOfIsolated() const
{
	return m_nIsolatedVerts;
}

int CRichModel::GetNumOfEdges() const
{
	return (int)m_Edges.size();
}

bool CRichModel::isBoundaryVert(int index) const
{
	return IsStartEdge(Neigh(index).front().first);
}

bool CRichModel::IsStronglyConvexVert(int index) const
{
	return m_FlagsForCheckingConvexVerts[index].first;
}

bool CRichModel::IsWeaklyConvexVert(int index) const
{
	return m_FlagsForCheckingConvexVerts[index].second;
}

bool CRichModel::IsExtremeEdge(int edgeIndex) const
{
	return Edge(edgeIndex).indexOfOppositeVert == -1;
}

bool CRichModel::IsStartEdge(int edgeIndex) const
{
	return Edge(Edge(edgeIndex).indexOfReverseEdge).indexOfOppositeVert == -1;
}

const CRichModel::CEdge& CRichModel::Edge(int edgeIndex) const
{
	return m_Edges[edgeIndex];
}

const vector<pair<int, double> >& CRichModel::Neigh(int root) const
{
	return m_NeighsAndAngles[root];
}

double CRichModel::ProportionOnEdgeByImage(int edgeIndex, const pair<double, double>& coord) const
{
	double res = Edge(edgeIndex).coordOfOppositeVert.first * coord.second - Edge(edgeIndex).coordOfOppositeVert.second * coord.first;
	return res / ((coord.second - Edge(edgeIndex).coordOfOppositeVert.second) * Edge(edgeIndex).length);
}

double CRichModel::ProportionOnEdgeByImage(int edgeIndex, double x1, double y1, double x2, double y2) const
{
	double res = x1 * y2 - x2 * y1;
	return res / ((y2 - y1) * Edge(edgeIndex).length);
}

double CRichModel::ProportionOnLeftEdgeByImage(int edgeIndex, const pair<double, double> &coord, double proportion) const
{
	double xBalance = proportion * Edge(edgeIndex).length;
	double res = Edge(edgeIndex).coordOfOppositeVert.first * coord.second - Edge(edgeIndex).coordOfOppositeVert.second * (coord.first - xBalance);
	return xBalance * coord.second / res;
}

double CRichModel::ProportionOnRightEdgeByImage(int edgeIndex, const pair<double, double> &coord, double proportion) const
{
	double part1 = Edge(edgeIndex).length * coord.second;
	double part2 = proportion * Edge(edgeIndex).length * Edge(edgeIndex).coordOfOppositeVert.second;
	double part3 = Edge(edgeIndex).coordOfOppositeVert.second * coord.first - Edge(edgeIndex).coordOfOppositeVert.first * coord.second;	
	return (part3 + proportion * part1 - part2) / (part3 + part1 - part2);
}

pair<double, double> CRichModel::GetNew2DCoordinatesByRotatingAroundLeftChildEdge(int edgeIndex, const pair<double, double>& input2DCoordinates) const
{
	return make_pair(Edge(edgeIndex).matrixRotatedToLeftEdge.first * input2DCoordinates.first - Edge(edgeIndex).matrixRotatedToLeftEdge.second * input2DCoordinates.second,
		Edge(edgeIndex).matrixRotatedToLeftEdge.second * input2DCoordinates.first + Edge(edgeIndex).matrixRotatedToLeftEdge.first * input2DCoordinates.second);
}

pair<double, double> CRichModel::GetNew2DCoordinatesByRotatingAroundRightChildEdge(int edgeIndex, const pair<double, double>& input2DCoordinates) const
{
	int reverseEdge = Edge(Edge(edgeIndex).indexOfRightEdge).indexOfReverseEdge;
	pair<double, double> coordOfLeftEnd = GetNew2DCoordinatesByReversingCurrentEdge(reverseEdge, Edge(reverseEdge).coordOfOppositeVert);	
	return make_pair(Edge(edgeIndex).matrixRotatedToRightEdge.first * input2DCoordinates.first - Edge(edgeIndex).matrixRotatedToRightEdge.second * input2DCoordinates.second + coordOfLeftEnd.first,
		Edge(edgeIndex).matrixRotatedToRightEdge.second * input2DCoordinates.first + Edge(edgeIndex).matrixRotatedToRightEdge.first * input2DCoordinates.second + coordOfLeftEnd.second);
}

pair<double, double> CRichModel::GetNew2DCoordinatesByReversingCurrentEdge(int edgeIndex, const pair<double, double>& input2DCoordinates) const
{
	return make_pair(Edge(edgeIndex).length - input2DCoordinates.first, - input2DCoordinates.second);
}

int CRichModel::GetSubindexToVert(int root, int neigh) const
{
	for (int i = 0; i < (int)Neigh(root).size(); ++i)
	{
		if (Edge(Neigh(root)[i].first).indexOfRightVert == neigh)
			return i;
	}
	return -1;
}

double CRichModel::DistanceToOppositeAngle(int edgeIndex, const pair<double, double>& coord) const
{
	double detaX = coord.first - Edge(edgeIndex).coordOfOppositeVert.first;
	double detaY = coord.second - Edge(edgeIndex).coordOfOppositeVert.second;
	return sqrt(detaX * detaX + detaY * detaY);
}

double CRichModel::DistanceToLeftVert(int edgeIndex, const pair<double, double>& coord) const
{
	double detaX = coord.first;
	double detaY = coord.second;
	return sqrt(detaX * detaX + detaY * detaY);
}

double CRichModel::DistanceToRightVert(int edgeIndex, const pair<double, double>& coord) const
{
	double detaX = coord.first - Edge(edgeIndex).length;
	double detaY = coord.second;
	return sqrt(detaX * detaX + detaY * detaY);
}

int CRichModel::GetEdgeIndexFromTwoVertices(int leftVert, int rightVert) const
{
	int subIndex = GetSubindexToVert(leftVert, rightVert);
	assert (subIndex != -1);
	return Neigh(leftVert)[subIndex].first;
}

int CRichModel::SplitEdge(const EdgePoint& ep)
{
	CPoint3D newVert = (1 - ep.proportion) * m_Verts[Edge(ep.index).indexOfLeftVert] 
		+ ep.proportion * m_Verts[Edge(ep.index).indexOfRightVert];
	m_Verts.push_back(newVert);
	int vertID = m_Verts.size() - 1;
	int edgeIndex = ep.index;
	m_UselessFaces.insert(Edge(edgeIndex).indexOfFrontFace);
	int reverseFaceIndex = Edge(Edge(edgeIndex).indexOfReverseEdge).indexOfFrontFace;
	if (reverseFaceIndex != -1)
		m_UselessFaces.insert(reverseFaceIndex);

	m_UselessEdges.insert(edgeIndex);
	if (reverseFaceIndex != -1)
		m_UselessEdges.insert(Edge(edgeIndex).indexOfReverseEdge);
	m_Faces.push_back(CBaseModel::CFace(Edge(edgeIndex).indexOfLeftVert, vertID, Edge(edgeIndex).indexOfOppositeVert));
	int f1 = (int)m_Faces.size() - 1;
	m_Faces.push_back(CBaseModel::CFace(vertID, m_Edges[edgeIndex].indexOfRightVert, m_Edges[edgeIndex].indexOfOppositeVert));
	int f2 = (int)m_Faces.size() - 1;
	if (reverseFaceIndex != -1)
		m_Faces.push_back(CBaseModel::CFace(m_Edges[m_Edges[edgeIndex].indexOfReverseEdge].indexOfLeftVert, vertID, m_Edges[m_Edges[edgeIndex].indexOfReverseEdge].indexOfOppositeVert));
	int f3 = (int)m_Faces.size() - 1;
	if (reverseFaceIndex != -1)
		m_Faces.push_back(CBaseModel::CFace(vertID, m_Edges[m_Edges[edgeIndex].indexOfReverseEdge].indexOfRightVert, m_Edges[m_Edges[edgeIndex].indexOfReverseEdge].indexOfOppositeVert));
	int f4 = (int)m_Faces.size() - 1;
	if (reverseFaceIndex == -1)
		f3 = f4 = -1;
	m_Edges.push_back(CRichModel::CEdge());
	int e1 = (int)m_Edges.size() - 1;
	m_Edges.push_back(CRichModel::CEdge());
	int e2 = (int)m_Edges.size() - 1;
	m_Edges.push_back(CRichModel::CEdge());
	int e3 = (int)m_Edges.size() - 1;
	m_Edges.push_back(CRichModel::CEdge());
	int e4 = (int)m_Edges.size() - 1;
	m_Edges.push_back(CRichModel::CEdge());
	int e5 = (int)m_Edges.size() - 1;
	if (reverseFaceIndex != -1)
		m_Edges.push_back(CRichModel::CEdge());
	int e6 = (int)m_Edges.size() - 1;
	if (reverseFaceIndex != -1)
		m_Edges.push_back(CRichModel::CEdge());
	int e7 = (int)m_Edges.size() - 1;
	m_Edges.push_back(CRichModel::CEdge());
	int e8 = (int)m_Edges.size() - 1;
	if (reverseFaceIndex == -1)
		e6 = e7 = -1;

	m_Edges[e1].indexOfFrontFace = f1;
	m_Edges[e1].indexOfLeftEdge = m_Edges[edgeIndex].indexOfLeftEdge;
	m_Edges[e1].indexOfLeftVert = m_Edges[edgeIndex].indexOfLeftVert;
	m_Edges[e1].indexOfOppositeVert = m_Edges[edgeIndex].indexOfOppositeVert;
	m_Edges[e1].indexOfReverseEdge = e8;
	m_Edges[e1].indexOfRightEdge = e3;
	m_Edges[e1].indexOfRightVert = vertID;

	m_Edges[e4].indexOfFrontFace = f2;
	m_Edges[e4].indexOfLeftEdge = e2;
	m_Edges[e4].indexOfLeftVert = vertID;
	m_Edges[e4].indexOfOppositeVert = m_Edges[edgeIndex].indexOfOppositeVert;
	m_Edges[e4].indexOfReverseEdge = e5;
	m_Edges[e4].indexOfRightEdge =  m_Edges[edgeIndex].indexOfRightEdge;
	m_Edges[e4].indexOfRightVert = m_Edges[edgeIndex].indexOfRightVert;

	m_Edges[e5].indexOfFrontFace = f3;
	m_Edges[e5].indexOfLeftEdge = m_Edges[m_Edges[edgeIndex].indexOfReverseEdge].indexOfLeftEdge;
	m_Edges[e5].indexOfLeftVert = m_Edges[m_Edges[edgeIndex].indexOfReverseEdge].indexOfLeftVert;
	m_Edges[e5].indexOfOppositeVert = m_Edges[m_Edges[edgeIndex].indexOfReverseEdge].indexOfOppositeVert;
	m_Edges[e5].indexOfReverseEdge = e4;
	m_Edges[e5].indexOfRightEdge =  e7;
	m_Edges[e5].indexOfRightVert = vertID;

	m_Edges[e8].indexOfFrontFace = f4;
	m_Edges[e8].indexOfLeftEdge = e6;
	m_Edges[e8].indexOfLeftVert = vertID;
	m_Edges[e8].indexOfOppositeVert = m_Edges[m_Edges[edgeIndex].indexOfReverseEdge].indexOfOppositeVert;
	m_Edges[e8].indexOfReverseEdge = e1;
	m_Edges[e8].indexOfRightEdge =  m_Edges[m_Edges[edgeIndex].indexOfReverseEdge].indexOfRightEdge;
	m_Edges[e8].indexOfRightVert = m_Edges[m_Edges[edgeIndex].indexOfReverseEdge].indexOfRightVert;

	m_Edges[e2].indexOfFrontFace = f1;
	m_Edges[e2].indexOfLeftEdge = m_Edges[e1].indexOfReverseEdge;
	m_Edges[e2].indexOfLeftVert = m_Edges[e1].indexOfRightVert;
	m_Edges[e2].indexOfOppositeVert = m_Edges[e1].indexOfLeftVert;
	m_Edges[e2].indexOfReverseEdge = e3;
	m_Edges[e2].indexOfRightEdge = m_Edges[e1].indexOfLeftEdge;
	m_Edges[e2].indexOfRightVert = m_Edges[e1].indexOfOppositeVert;

	m_Edges[e3].indexOfFrontFace = f2;
	m_Edges[e3].indexOfLeftEdge = m_Edges[e4].indexOfRightEdge;
	m_Edges[e3].indexOfLeftVert =  m_Edges[e4].indexOfOppositeVert;
	m_Edges[e3].indexOfOppositeVert =  m_Edges[e4].indexOfRightVert;
	m_Edges[e3].indexOfReverseEdge = e2;
	m_Edges[e3].indexOfRightEdge = m_Edges[e4].indexOfReverseEdge;
	m_Edges[e3].indexOfRightVert = m_Edges[e4].indexOfLeftVert;

	if (reverseFaceIndex != -1)
	{
		m_Edges[e6].indexOfFrontFace = f3;
		m_Edges[e6].indexOfLeftEdge = m_Edges[e5].indexOfReverseEdge;
		m_Edges[e6].indexOfLeftVert =  m_Edges[e5].indexOfRightVert;
		m_Edges[e6].indexOfOppositeVert =  m_Edges[e5].indexOfLeftVert;
		m_Edges[e6].indexOfReverseEdge = e7;
		m_Edges[e6].indexOfRightEdge = m_Edges[e5].indexOfLeftEdge;
		m_Edges[e6].indexOfRightVert = m_Edges[e5].indexOfOppositeVert;

		m_Edges[e7].indexOfFrontFace = f4;
		m_Edges[e7].indexOfLeftEdge = m_Edges[e8].indexOfRightEdge;
		m_Edges[e7].indexOfLeftVert =  m_Edges[e8].indexOfOppositeVert;
		m_Edges[e7].indexOfOppositeVert =  m_Edges[e8].indexOfRightVert;
		m_Edges[e7].indexOfReverseEdge = e6;
		m_Edges[e7].indexOfRightEdge = m_Edges[e8].indexOfReverseEdge;
		m_Edges[e7].indexOfRightVert = m_Edges[e8].indexOfLeftVert;
	}
	int newEdgeCnt = 8;
	if (reverseFaceIndex == -1)
		newEdgeCnt = 6;
	//for (int i = 0; i < newEdgeCnt; ++i)
	//{
	//	extraEdgePool[make_pair(newEdges[newEdges.size() - 1 - i].indexOfLeftVert, newEdges[newEdges.size() - 1 - i].indexOfRightVert)] = newEdges.size() - 1 - i;
	//}

	int leftEdge = m_Edges[m_Edges[e1].indexOfLeftEdge].indexOfReverseEdge;
	m_Edges[leftEdge].indexOfFrontFace = f1;
	m_Edges[leftEdge].indexOfLeftEdge = e3;
	m_Edges[leftEdge].indexOfLeftVert;
	m_Edges[leftEdge].indexOfOppositeVert = m_Edges[e1].indexOfRightVert;
	m_Edges[leftEdge].indexOfReverseEdge;
	m_Edges[leftEdge].indexOfRightEdge = m_Edges[e1].indexOfReverseEdge;
	m_Edges[leftEdge].indexOfRightVert;

	int rightEdge = m_Edges[m_Edges[e4].indexOfRightEdge].indexOfReverseEdge;
	m_Edges[rightEdge].indexOfFrontFace = f2;
	m_Edges[rightEdge].indexOfLeftEdge = m_Edges[e4].indexOfReverseEdge;
	m_Edges[rightEdge].indexOfLeftVert;
	m_Edges[rightEdge].indexOfOppositeVert = m_Edges[e4].indexOfLeftVert;
	m_Edges[rightEdge].indexOfReverseEdge;
	m_Edges[rightEdge].indexOfRightEdge = e2;
	m_Edges[rightEdge].indexOfRightVert;

	if (reverseFaceIndex != -1)
	{
		leftEdge = m_Edges[m_Edges[e5].indexOfLeftEdge].indexOfReverseEdge;
		m_Edges[leftEdge].indexOfFrontFace = f3;
		m_Edges[leftEdge].indexOfLeftEdge = e7;
		m_Edges[leftEdge].indexOfLeftVert;
		m_Edges[leftEdge].indexOfOppositeVert = m_Edges[e5].indexOfRightVert;
		m_Edges[leftEdge].indexOfReverseEdge;
		m_Edges[leftEdge].indexOfRightEdge = m_Edges[e5].indexOfReverseEdge;
		m_Edges[leftEdge].indexOfRightVert;

		rightEdge = m_Edges[m_Edges[e8].indexOfRightEdge].indexOfReverseEdge;
		m_Edges[rightEdge].indexOfFrontFace = f4;
		m_Edges[rightEdge].indexOfLeftEdge = m_Edges[e8].indexOfReverseEdge;
		m_Edges[rightEdge].indexOfLeftVert;
		m_Edges[rightEdge].indexOfOppositeVert = m_Edges[e8].indexOfLeftVert;
		m_Edges[rightEdge].indexOfReverseEdge;
		m_Edges[rightEdge].indexOfRightEdge = e6;
		m_Edges[rightEdge].indexOfRightVert;
	}
	return vertID;
}

void CRichModel::SavePathToObj(const vector<EdgePoint>& pl, const string& filename) const
{
	ofstream out(filename.c_str());
	//filename.substr(filename.rfind("\\") + 1, filename.rfind('.') - filename.rfind("\\") - 1);
	//filename.substr(filename.rfind("\\") + 1, filename.rfind('.') - filename.rfind("\\") - 1)
	out << "g " << filename.substr(filename.rfind("\\") + 1, filename.rfind('.') - filename.rfind("\\") - 1) << endl;
	if (!pl.empty())
	{	
		for (int i = 0; i < pl.size(); ++i)
		{
			CPoint3D pt = pl[i].GetShiftPoint(*this);
			out << "v " << pt.x << " " << pt.y << " " << pt.z << endl;
		}

		out << "l ";
		for (int i = 0; i < pl.size(); ++i)
		{
			out << i + 1 << " ";
		}
		out << endl;
	}
	out.close();
}

//void CRichModel::SavePathToObj(const vector<CPoint3D>& pl, const string& filename) const
//{
//	ofstream out(filename.c_str());
//	out << "g 3D_Curve" << endl;
//	if (!pl.empty())
//	{	
//		for (int i = 0; i < pl.size(); ++i)
//		{
//			CPoint3D pt = pl[i];
//			out << "v " << pt.x << " " << pt.y << " " << pt.z << endl;
//		}
//
//		out << "l ";
//		for (int i = 0; i < pl.size(); ++i)
//		{
//			out << i + 1 << " ";
//		}
//		out << endl;
//	}
//	out.close();
//}

void CRichModel::SaveIsolineToObj(const vector<EdgePoint>& isoline, const string& filename) const
{
	ofstream out(filename.c_str());
	out << "g " << filename.substr(filename.rfind("\\") + 1, filename.rfind('.') - filename.rfind("\\") - 1) << endl;
	if (!isoline.empty())
	{
		for (int i = 0; i < isoline.size(); ++i)
		{
			CPoint3D pt = isoline[i].GetShiftPoint(*this);
			out << "v " << pt.x << " " << pt.y << " " << pt.z << endl;
		}

		for (int i = 0; i < isoline.size() / 2; ++i)
		{
			out << "l " << i * 2 + 1 << " " << 2 * i + 2 << endl;
		}
	}
	out.close();
}

void CRichModel::SplitBasedOnScalarField(const vector<double>& scalarField,
		double val,
		const string& fileWithLargerScalars,
		const string& fileWithSmallerScalars)
{
	vector<EdgePoint> eps;
	for (int i = 0; i < GetNumOfEdges(); ++i)
	{
		if (Edge(i).indexOfLeftVert > 
			Edge(i).indexOfRightVert)
			continue;
		if (scalarField[Edge(i).indexOfLeftVert] >= val
			== scalarField[Edge(i).indexOfRightVert] >= val)
			continue;
		double prop = (val - scalarField[Edge(i).indexOfLeftVert]) 
			/ (scalarField[Edge(i).indexOfRightVert] - scalarField[Edge(i).indexOfLeftVert]);
		if (prop < 1e-4 || prop > 1 - 1e-4)
			continue;
		eps.push_back(EdgePoint(i, prop));
	}
	int oldVertNum = GetNumOfVerts();
	for (int i = 0; i < eps.size(); ++i)
		SplitEdge(eps[i]);

	ofstream outLarge(fileWithLargerScalars.c_str());
	ofstream outSmall(fileWithSmallerScalars.c_str());
	for (int i = 0; i < GetNumOfVerts(); ++i)
	{
		outLarge << "v " << Vert(i).x
			<< " " << Vert(i).y
			<< " " << Vert(i).z << endl;
		outSmall << "v " << Vert(i).x
			<< " " << Vert(i).y
			<< " " << Vert(i).z << endl;
	}

	for (int i = 0; i < GetNumOfFaces(); ++i)
	{
		if (m_UselessFaces.find(i) != m_UselessFaces.end())
			continue;
		int v1 = Face(i)[0];
		int v2 = Face(i)[1];
		int v3 = Face(i)[2];
		int cnt(0);
		double average(0);
		if (v1 < oldVertNum)
		{
			average += scalarField[v1];
			cnt++;
		}
		if (v2 < oldVertNum)
		{
			average += scalarField[v2];
			cnt++;
		}
		if (v3 < oldVertNum)
		{
			average += scalarField[v3];
			cnt++;
		}
		average /= cnt;
		if (average < val)
		{
			outSmall << "f " << Face(i)[0] + 1 << " " << Face(i)[1] + 1 << " " << Face(i)[2] + 1 << endl;
		}
		else
		{
			outLarge << "f " << Face(i)[0] + 1 << " " << Face(i)[1] + 1 << " " << Face(i)[2] + 1 << endl;
		}
	}

	outLarge.close();
	outSmall.close();
}

int CRichModel::IntersectQuery(int faceID, const pair<EdgePoint, EdgePoint>& seg1, const pair<EdgePoint, EdgePoint>& seg2, EdgePoint& intersection) const
{
	set<pair<double, string>> sortingSet;
	sortingSet.insert(make_pair(seg1.first.GetNumbering(*this, faceID), "seg1"));
	sortingSet.insert(make_pair(seg1.second.GetNumbering(*this, faceID), "seg1"));
	sortingSet.insert(make_pair(seg2.first.GetNumbering(*this, faceID), "seg2"));
	sortingSet.insert(make_pair(seg2.second.GetNumbering(*this, faceID), "seg2"));
	vector<pair<double, string>> sortingVec(sortingSet.begin(), sortingSet.end());
	for (int i = 0; i < 4; ++i)
	{
		int nxt = (i + 1) % 4;
		if (sortingVec[i].first == sortingVec[nxt].first
			&& sortingVec[i].second != sortingVec[nxt].second)
		{
			if (seg1.first == seg2.first || seg1.first == seg2.second)
				intersection = seg1.first;
			else
				intersection = seg1.second;
			return -1;
		}
	}
	for (int i = 0; i < 4; ++i)
	{
		int nxt = (i + 1) % 4;
		if (sortingVec[i].second == sortingVec[nxt].second)
			return 0;
	}
	return 1;
}

double CRichModel::GetMaxEdgeLength() const
{
	return m_maxEdgeLength;
}

//string GetUserName()
//{
//	const int MAX_BUFFER_LEN = 500;
//	char  szBuffer[MAX_BUFFER_LEN] = "";
//	DWORD dwNameLen;
//
//	dwNameLen = MAX_BUFFER_LEN;
//	::GetUserName(szBuffer, &dwNameLen);
//	return szBuffer;
//}

CPoint3D CRichModel::GetBarycentricCoord(CPoint3D pt, int faceID) const
{	
	//if (MyGetUserName() == "xinshiqing")
	//{
	//	cout << "-----begin---------\n";
	//	cout << "[" << Vert(Face(faceID)[0]).x << " " << Vert(Face(faceID)[0]).y << " " << Vert(Face(faceID)[0]).z << ";" << endl;
	//	cout << Vert(Face(faceID)[1]).x << " " << Vert(Face(faceID)[1]).y << " " << Vert(Face(faceID)[1]).z << ";" << endl;
	//	cout << Vert(Face(faceID)[2]).x << " " << Vert(Face(faceID)[2]).y << " " << Vert(Face(faceID)[2]).z << ";" << endl;
	//	cout << pt.x << " " << pt.y << " " << pt.z << "];" << endl;
	//}

	Eigen::MatrixXd M(4, 3);
	M << Vert(Face(faceID)[0]).x, Vert(Face(faceID)[1]).x, Vert(Face(faceID)[2]).x,
		Vert(Face(faceID)[0]).y, Vert(Face(faceID)[1]).y, Vert(Face(faceID)[2]).y,
		Vert(Face(faceID)[0]).z, Vert(Face(faceID)[1]).z, Vert(Face(faceID)[2]).z,
		1, 1, 1;
	Eigen::VectorXd right(4);
	right << pt.x, pt.y, pt.z, 1;
	auto M_trans = M.transpose();
	//if (MyGetUserName() == "xinshiqing")
	//{
	//	cerr << "--------------------\n";
	//	cerr << M_trans << endl;
	//	cerr << "--------------------\n";
	//	cerr << M << endl;
	//}
	
	right = M_trans * right;
	//if (MyGetUserName() == "xinshiqing")
	//{
	//	cerr << "--------------------\n";
	//	cerr << M << endl;
	//}
	M = M_trans * M;
	//if (MyGetUserName() == "xinshiqing")
	//{
	//	cerr << "--------------------\n";
	//	cerr << M << endl;
	//}

	Eigen::Vector3d res = M.inverse() * right;

	
	return CPoint3D(res(0), res(1), res(2));
}