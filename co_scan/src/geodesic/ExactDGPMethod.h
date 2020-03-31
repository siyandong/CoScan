// ExactMethodForDGP.h: interface for the CExactDGPMethod class.
//
//////////////////////////////////////////////////////////////////////

#pragma once
#include "EdgePoint.h"
#include "RichModel.h"
#include <list>
#include <float.h>
#include "DistanceApproach.h"
using namespace std;


class CExactDGPMethod : public CDistanceApproach
{
protected:
	struct InfoAtVertex
	{
		bool fParentIsPseudoSource;
		char birthTimeForCheckingValidity;		
		int indexOfDirectParent;
		int indexOfRootVertOfDirectParent;
		int levelOnSequenceTree;
		int indexOfAncestor;
		double disUptodate;
		double entryProp;

		InfoAtVertex()
		{
			fParentIsPseudoSource = true;
			levelOnSequenceTree = -1;
			birthTimeForCheckingValidity = -1;
			indexOfDirectParent = -1;
			indexOfAncestor = -1;
			disUptodate = DBL_MAX;
		}		
		EdgePoint GetEntryPoint()
		{
			EdgePoint ep;
			ep.isVertex = fParentIsPseudoSource;
			ep.index = indexOfDirectParent;
			ep.proportion = entryProp;
			return ep;
		}
	};
	struct QuoteInfoAtVertex
	{
		char birthTime;
		int indexOfVert;
		double disUptodate;
		bool operator<(const QuoteInfoAtVertex& another) const
		{
			return disUptodate > another.disUptodate;
		}
		QuoteInfoAtVertex(){}
		QuoteInfoAtVertex(char birthTime, int indexOfVert, double disUptodate)
		{
			this->birthTime = birthTime;
			this->indexOfVert = indexOfVert;
			this->disUptodate = disUptodate;
		}
	};
	

protected:	
	virtual void Initialize();
	virtual void Dispose();
	virtual void Propagate() = 0;
	virtual void CollectExperimentalResults();
public:
	vector<InfoAtVertex> m_InfoAtVertices;
	CExactDGPMethod(const CRichModel& inputModel, int source);
	CExactDGPMethod(const CRichModel& inputModel, int source, int destination);
	CExactDGPMethod(const CRichModel& inputModel, int source, double R);
	CExactDGPMethod(const CRichModel& inputModel, const map<int, double> &indexOfSourceVerts);
	CExactDGPMethod(const CRichModel& inputModel, const map<int, double> &indexOfSourceVerts, const set<int> &destinations);
	CExactDGPMethod(const CRichModel& inputModel, const set<int> &indexOfSourceVerts);
	CExactDGPMethod(const CRichModel& inputModel, const set<int> &indexOfSourceVerts, double R);
	CExactDGPMethod(const CRichModel& inputModel, const set<int> &indexOfSourceVerts, const set<int> &destinations);
	virtual vector<EdgePoint> BacktraceShortestPath(int end) const;
	virtual int GetAncestor(int vIndex) const;
};






