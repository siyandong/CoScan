// ICHWithFurtherPriorityQueue.h: interface for the CXin_Wang class.
//
//////////////////////////////////////////////////////////////////////

#pragma once
#include "ICH_WindowFiltering.h"
class CXin_Wang : public CICH_WindowFiltering  
{	
protected:
	vector<int> m_fixedSequence;
	priority_queue<QuoteWindow> m_QueueForWindows;
	priority_queue<QuoteInfoAtVertex> m_QueueForPseudoSources;
protected:
	virtual void Dispose();
	virtual void Propagate();
protected:
	void AddIntoQueueOfPseudoSources(const QuoteInfoAtVertex& quoteOfPseudoSource);
	void AddIntoQueueOfWindows(QuoteWindow& quoteW);
	bool UpdateTreeDepthBackWithChoice();
	double GetMinDisOfWindow(const Window& w) const;
public:
	CXin_Wang(const CRichModel& model, int source);
	CXin_Wang(const CRichModel& model, int source, int destination);
	CXin_Wang(const CRichModel& model, int source, double R);
	CXin_Wang(const CRichModel& model, const map<int, double>& sources);
	CXin_Wang(const CRichModel& model, const map<int, double>& sources, const set<int> &destinations);	
	CXin_Wang(const CRichModel& model, const set<int>& sources);	
	CXin_Wang(const CRichModel& model, const set<int>& sources, double R);
	CXin_Wang(const CRichModel& model, const set<int>& sources, const set<int>& destinations);
	vector<int> GetFixedVertSequence() const;
};
