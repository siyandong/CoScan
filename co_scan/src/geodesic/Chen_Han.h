// PreviousCH.h: interface for the CChen_Han class.
//
//////////////////////////////////////////////////////////////////////

#pragma once
#include "ExactDGPMethod.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <queue>
//#include <minmax.h>
#include <iostream>
#include "Parameters.h"
using namespace std;

class CChen_Han : public CExactDGPMethod
{
protected:
	struct InfoAtAngle
	{
		char birthTime;
		double disUptodate;
		double entryProp;
		InfoAtAngle()
		{
			birthTime = -1;
			disUptodate = DBL_MAX;
		}
	};
	struct Window
	{
		bool fIsOnLeftSubtree;
		bool fBrachParentIsPseudoSource;	
		bool fDirectParentEdgeOnLeft; //may removed
		bool fDirectParenIsPseudoSource; //may removed
		char birthTimeOfParent;
		int indexOfBrachParent;
		int indexOfRootVertex;
		int indexOfCurEdge;
		int levelOnSequenceTree;//may removed
		int indexOfAncestor;//may removed
		double disToRoot;
		double proportions[2];
		double entryPropOfParent;
		pair<double, double> coordOfPseudoSource;
		Window()
		{
			indexOfRootVertex = -1;
			indexOfCurEdge = -1;
			indexOfAncestor = -1;
		}
		bool Invalid() const
		{
			return indexOfRootVertex == -1
				&& indexOfCurEdge == -1
				&& indexOfAncestor == -1;
		}
		int GetDirectParentEdge(const CRichModel& model) const
		{
			assert(!fDirectParenIsPseudoSource);
			if (fDirectParentEdgeOnLeft)
			{
				return model.Edge(model.Edge(model.Edge(indexOfCurEdge).indexOfReverseEdge).indexOfRightEdge).indexOfReverseEdge;
			}
			else
			{
				return model.Edge(model.Edge(model.Edge(indexOfCurEdge).indexOfReverseEdge).indexOfLeftEdge).indexOfReverseEdge;
			}
		}		
	};//at least 64 bytes.
	struct QuoteWindow
	{
		Window* pWindow;
		double disUptodate;
		bool operator<(const QuoteWindow& another) const
		{
			return disUptodate > another.disUptodate;
		}
	};
protected:	
	queue<QuoteWindow> m_QueueForWindows;
	queue<QuoteInfoAtVertex> m_QueueForPseudoSources;
	vector<InfoAtAngle> m_InfoAtAngles;
	 __int64 m_nMaxLenOfWindowQueue;
	 __int64 m_nMaxLenOfPseudoSourceQueue;
	 __int64 m_nCountOfWindows;
protected:
	virtual void Initialize();
	virtual void Dispose();
	virtual void Propagate();
	virtual void CollectExperimentalResults();

	virtual void AddIntoQueueOfPseudoSources(const QuoteInfoAtVertex& quoteOfPseudoSource);
	virtual void AddIntoQueueOfWindows(QuoteWindow& quoteW);
	virtual bool UpdateTreeDepthBackWithChoice();

protected:	
	bool IsTooNarrowWindow(const Window& w) const;		
	void ComputeChildrenOfPseudoSource(int indexOfParentVertex);
	void ComputeChildrenOfPseudoSourceFromPseudoSource(int indexOfParentVertex);
	void ComputeChildrenOfPseudoSourceFromWindow(int indexOfParentVertex);
	void CreateIntervalChildOfPseudoSource(int source, int subIndexOfIncidentEdge, double propL = 0, double propR = 1);
	void FillVertChildOfPseudoSource(int source, int subIndexOfVert);
	void ComputeChildrenOfSource();
	void ComputeChildrenOfSource(int indexOfSourceVert, double dis);
	void ComputeTheOnlyLeftChild(const Window& w);
	void ComputeTheOnlyLeftTrimmedChild(const Window& w);
	void ComputeLeftTrimmedChildWithParent(const Window& w);
	void ComputeTheOnlyRightChild(const Window& w);
	void ComputeTheOnlyRightTrimmedChild(const Window& w);
	void ComputeRightTrimmedChildWithParent(const Window& w);
	void ComputeChildrenOfWindow(QuoteWindow& quoteParentWindow);
public:
	CChen_Han(const CRichModel& model, int source);
	CChen_Han(const CRichModel& model, int source, int destination);
	CChen_Han(const CRichModel& model, int source, double R);
	CChen_Han(const CRichModel& model, const map<int, double>& sources);
	CChen_Han(const CRichModel& model, const map<int, double>& sources, const set<int> &destinations);	
	CChen_Han(const CRichModel& model, const set<int>& sources);	
	CChen_Han(const CRichModel& model, const set<int>& sources, double R);
	CChen_Han(const CRichModel& model, const set<int>& sources, const set<int>& destinations);
	__int64 GetTotalNumOfWindows() const{return m_nCountOfWindows;}
	__int64 GetMaxLenOfWindowQueue() const{return m_nMaxLenOfWindowQueue;}
	__int64 GetMaxLenOfPseudoSourceQueue() const{return m_nMaxLenOfPseudoSourceQueue;}
	void OutputExperimentalResults() const;
};
