#include "DistanceApproach.h"
//#include <Windows.h>
#include <cfloat> // dsy: DBL_MAX
#include <iostream>
using namespace std;
CDistanceApproach::CDistanceApproach(const CRichModel& model, int source) : model(model)
{
	m_nameOfAlgorithm = "Abstract";
	m_sources[source] = 0;	
	m_radius = DBL_MAX;
}
CDistanceApproach::CDistanceApproach(const CRichModel& model, int source, int destination) : model(model)
{
	m_nameOfAlgorithm = "Abstract";
	m_sources[source] = 0;	
	m_destinations.insert(destination);
	m_radius = DBL_MAX;
}
CDistanceApproach::CDistanceApproach(const CRichModel& model, int source, double R) : model(model), m_radius(R)
{
	m_nameOfAlgorithm = "Abstract";
	m_sources[source] = 0;	
}
CDistanceApproach::CDistanceApproach(const CRichModel& model, const map<int, double>& sources) : model(model), m_sources(sources)
{
	m_nameOfAlgorithm = "Abstract";
	m_radius = DBL_MAX;
}
CDistanceApproach::CDistanceApproach(const CRichModel& model, const map<int, double>& sources, const set<int> &destinations) : model(model), m_sources(sources), m_destinations(destinations)
{
	m_nameOfAlgorithm = "Abstract";
	m_radius = DBL_MAX;
}
CDistanceApproach::CDistanceApproach(const CRichModel& model, const set<int>& sources) : model(model)
{
	m_nameOfAlgorithm = "Abstract";
	for (set<int>::const_iterator it = sources.begin(); it != sources.end(); ++it)
		m_sources[*it] = 0;
	m_radius = DBL_MAX;
}
CDistanceApproach::CDistanceApproach(const CRichModel& model, const set<int>& sources, double R) : model(model), m_radius(R)
{
	m_nameOfAlgorithm = "Abstract";
	for (set<int>::const_iterator it = sources.begin(); it != sources.end(); ++it)
		m_sources[*it] = 0;
}
CDistanceApproach::CDistanceApproach(const CRichModel& model, const set<int>& sources, const set<int>& destinations) : model(model), m_destinations(destinations)
{
	m_nameOfAlgorithm = "Abstract";
	for (set<int>::const_iterator it = sources.begin(); it != sources.end(); ++it)
		m_sources[*it] = 0;
	m_radius = DBL_MAX;
}

const vector<double>& CDistanceApproach::GetDistanceField() const
{
	return m_scalarField;
}
//vector<double> CDistanceApproach::GetNormalizedDistanceField() const
//{
//	vector<double> scalarValues(m_scalarField);
//	for (int i = 0; i < scalarValues.size(); ++i)
//	{
//		scalarValues[i] /= m_maxDisValue;
//	}
//	return scalarValues;
//}

//vector<double> CDistanceApproach::GetDistanceFieldDividedBy(double denominator) const
//{
//	vector<double> scalarValues(m_scalarField);
//	for (int i = 0; i < scalarValues.size(); ++i)
//	{
//		scalarValues[i] /= denominator;
//	}
//	return scalarValues;
//}

double CDistanceApproach::GetMaxDistance() const
{
	return m_maxDisValue;
}

vector<double> CDistanceApproach::DiffDistanceField(const vector<double>& field1, 
		const vector<double>& field2)
{
	vector<double> result(field1.size());
	for (int i = 0; i < field1.size(); ++i)
		result[i] = abs(field1[i] - field2[i]);
	return result;
}

vector<EdgePoint> CDistanceApproach::BacktraceIsoline(double val) const
{
	vector<EdgePoint> isoline;
	for (int i = 0; i < model.GetNumOfFaces(); ++i)
	{
		int high, middle, low, total(0);
		double highestDis(-DBL_MAX);
		double lowestDis(DBL_MAX);
	
		for (int j = 0; j < 3; ++j)
		{			
			if (m_scalarField[model.Face(i).verts[j]] > highestDis)
			{
				high = model.Face(i).verts[j];
				highestDis = m_scalarField[model.Face(i).verts[j]];
			}
			if (m_scalarField[model.Face(i).verts[j]] < lowestDis)
			{
				low = model.Face(i).verts[j];
				lowestDis = m_scalarField[model.Face(i).verts[j]];
			}
			total += model.Face(i).verts[j];
		}

		if (high == low)
		{
			continue;
		}
		middle = total - high - low;
		if (highestDis <= val)
			continue;
		if (lowestDis >= val)
			continue;
		//CPoint3D ptLow = model.Vert(low);
		//CPoint3D ptMiddle = model.Vert(middle);
		//CPoint3D ptHigh = model.Vert(high);
		if (val < m_scalarField[middle])
		{			
			double prop = (val - m_scalarField[low]) / (m_scalarField[middle] - m_scalarField[low]);
			//CPoint3D pt = CRichModel::CombineTwoNormalsTo(ptLow, 1 - prop, ptMiddle, prop);
			//outFile << 0 << " " << low << " " << middle << " " << prop << endl;
			isoline.push_back(EdgePoint(model.GetEdgeIndexFromTwoVertices(low, middle), prop));
			prop = (val - m_scalarField[low]) / (m_scalarField[high] - m_scalarField[low]);
			//pt = CRichModel::CombineTwoNormalsTo(ptLow, 1 - prop, ptHigh, prop);
			//outFile << 0 << " " << low << " " << high << " " << prop << endl;
			isoline.push_back(EdgePoint(model.GetEdgeIndexFromTwoVertices(low, high), prop));
		}
		else
		{ 
			double prop = (val - m_scalarField[middle]) / (m_scalarField[high] - m_scalarField[middle]);
			//CPoint3D pt = CRichModel::CombineTwoNormalsTo(ptMiddle, 1 - prop, ptHigh, prop);			
			//outFile << 0 << " " << middle << " " << high << " " << prop << endl;
			isoline.push_back(EdgePoint(model.GetEdgeIndexFromTwoVertices(middle, high), prop));
			prop = (val - m_scalarField[low]) / (m_scalarField[high] - m_scalarField[low]);
			//pt = CRichModel::CombineTwoNormalsTo(ptLow, 1 - prop, ptHigh, prop);
			//outFile << 0 << " " << low << " " << high << " " << prop << endl;
			isoline.push_back(EdgePoint(model.GetEdgeIndexFromTwoVertices(low, high), prop));
		}	
	}
	return isoline;
}

string CDistanceApproach::GetAlgorithmName() const
{
	return m_nameOfAlgorithm;
}

void CDistanceApproach::Execute()
{
	Initialize();
	//m_nTotalMilliSeconds = GetTickCount(); // dsy: comment out.
	Propagate();
	//m_nTotalMilliSeconds = GetTickCount() - m_nTotalMilliSeconds; // dsy: comment out.
	CollectExperimentalResults();
	Dispose();	
}

void CDistanceApproach::Initialize()
{
	m_scalarField.resize(model.GetNumOfVerts(), DBL_MAX);
	m_maxLenOfQueue = 0;
	m_depthOfResultingTree = 0;
	m_maxDisValue = -1;
	m_memory = 0;
	//m_nTotalMilliSeconds = 0; // dsy: comment out.
}

void CDistanceApproach::CollectExperimentalResults()
{
	m_maxDisValue = -1;	
	for (int i = 0; i < model.GetNumOfVerts(); ++i)
		if (m_scalarField[i] > m_maxDisValue
			&& m_scalarField[i] < FLT_MAX)
			m_maxDisValue = m_scalarField[i];
}

void CDistanceApproach::OutputExperimentalResults() const
{
	cout << "Experimental results are as follows:\n";
	cout << "Algorithm: " << m_nameOfAlgorithm << endl;
	cout << "Memory = " << m_memory << " Mega-bytes.\n";
	//cout << "Timing = " << m_nTotalMilliSeconds << " ms.\n"; // dsy: comment out.
	cout << "MaxDepth = " << m_depthOfResultingTree << " levels.\n";
	cout << "MaxLenOfQue = " << m_maxLenOfQueue << " elements.\n";
}