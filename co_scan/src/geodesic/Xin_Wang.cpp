// ICHWithFurtherPriorityQueue.cpp: implementation of the CXin_Wang class.
//
//////////////////////////////////////////////////////////////////////

#include "Xin_Wang.h"
#include <iostream>
using namespace std;

// dsy
int64_t max(int64_t a, const int b)
{
	if (a>b)
		return a;
	return (int64_t)b;
}

CXin_Wang::CXin_Wang(const CRichModel& model, int source) : CICH_WindowFiltering(model, source)
{
	m_nameOfAlgorithm = "ICH2";
}
CXin_Wang::CXin_Wang(const CRichModel& model, int source, int destination) : CICH_WindowFiltering(model, source, destination)
{
	m_nameOfAlgorithm = "ICH2";
}

CXin_Wang::CXin_Wang(const CRichModel& model, int source, double R) 
	: CICH_WindowFiltering(model, source, R)
{
	m_nameOfAlgorithm = "ICH2";
}

CXin_Wang::CXin_Wang(const CRichModel& model, const map<int, double>& sources) : CICH_WindowFiltering(model, sources)
{
	m_nameOfAlgorithm = "ICH2";
}

CXin_Wang::CXin_Wang(const CRichModel& model, const map<int, double>& sources, const set<int> &destinations) : CICH_WindowFiltering(model, sources, destinations)
{
	m_nameOfAlgorithm = "ICH2";
}

CXin_Wang::CXin_Wang(const CRichModel& model, const set<int>& sources) : CICH_WindowFiltering(model, sources)
{
	m_nameOfAlgorithm = "ICH2";
}

CXin_Wang::CXin_Wang(const CRichModel& model, const set<int>& sources, double R) 
	: CICH_WindowFiltering(model, sources, R)
{
	m_nameOfAlgorithm = "ICH2";
}

CXin_Wang::CXin_Wang(const CRichModel& model, const set<int>& sources, const set<int>& destinations) : CICH_WindowFiltering(model, sources, destinations)
{
	m_nameOfAlgorithm = "ICH2";
}


void CXin_Wang::Dispose()
{
	while (!m_QueueForWindows.empty())
	{
		delete m_QueueForWindows.top().pWindow;
		m_QueueForWindows.pop();
	}
	m_QueueForPseudoSources = priority_queue<QuoteInfoAtVertex>();
}

void CXin_Wang::Propagate()
{
	set<int> tmpDestinations(m_destinations);
	for (map<int, double>::const_iterator it = m_sources.begin();
		it != m_sources.end(); ++it)
	{
		tmpDestinations.erase(it->first);
		m_fixedSequence.push_back(it->first);
	}
	ComputeChildrenOfSource();
	bool fFromQueueOfPseudoSources = UpdateTreeDepthBackWithChoice();

	while (!m_QueueForPseudoSources.empty() || !m_QueueForWindows.empty())
	{	
		if (m_QueueForWindows.size() > m_nMaxLenOfWindowQueue)
			m_nMaxLenOfWindowQueue = (int)m_QueueForWindows.size();
		if (m_QueueForPseudoSources.size() > m_nMaxLenOfPseudoSourceQueue)
			m_nMaxLenOfPseudoSourceQueue = m_QueueForPseudoSources.size();
		if (m_QueueForWindows.size() + m_QueueForPseudoSources.size() > m_maxLenOfQueue)
			m_maxLenOfQueue = m_QueueForWindows.size() + m_QueueForPseudoSources.size();
		if (fFromQueueOfPseudoSources) //pseudosource
		{
			int indexOfVert = m_QueueForPseudoSources.top().indexOfVert;			
			//cout << "Vert = " <<  indexOfVert << endl;
			m_QueueForPseudoSources.pop();	
			if (m_InfoAtVertices[indexOfVert].disUptodate > m_radius)
				break;
			tmpDestinations.erase(indexOfVert);	
			m_fixedSequence.push_back(indexOfVert);
			if (!m_destinations.empty() && tmpDestinations.empty())
				return;	
			if (!model.IsStronglyConvexVert(indexOfVert))	
				ComputeChildrenOfPseudoSource(indexOfVert);	
		}
		else			
		{
			QuoteWindow quoteW = m_QueueForWindows.top();			
			m_QueueForWindows.pop();
			if (quoteW.disUptodate > m_radius)
				break;
			ComputeChildrenOfWindow(quoteW);		
			delete quoteW.pWindow;
		}
		fFromQueueOfPseudoSources = UpdateTreeDepthBackWithChoice();
	}
}


double CXin_Wang::GetMinDisOfWindow(const Window& w) const
{
	double projProp = w.coordOfPseudoSource.first / model.Edge(w.indexOfCurEdge).length;
	if (projProp <= w.proportions[0])
	{
		double detaX = w.coordOfPseudoSource.first - w.proportions[0] * model.Edge(w.indexOfCurEdge).length;
		return w.disToRoot + sqrt(detaX * detaX + w.coordOfPseudoSource.second * w.coordOfPseudoSource.second);
	}
	if (projProp >= w.proportions[1])
	{
		double detaX = w.coordOfPseudoSource.first - w.proportions[1] * model.Edge(w.indexOfCurEdge).length;
		return w.disToRoot + sqrt(detaX * detaX + w.coordOfPseudoSource.second * w.coordOfPseudoSource.second);
	}
	return w.disToRoot - w.coordOfPseudoSource.second;
}

void CXin_Wang::AddIntoQueueOfPseudoSources(const QuoteInfoAtVertex& quoteOfPseudoSource)
{
	//if (model.IsStronglyConvexVert(quoteOfPseudoSource.indexOfVert))
	//	return;
	m_QueueForPseudoSources.push(quoteOfPseudoSource);
}

void CXin_Wang::AddIntoQueueOfWindows(QuoteWindow& quoteW)
{
	if (!CheckValidityWithXinWangFiltering(*quoteW.pWindow))
	{
		delete quoteW.pWindow;
		return;
	}
	quoteW.disUptodate = GetMinDisOfWindow(*quoteW.pWindow);
	m_QueueForWindows.push(quoteW);
	++m_nCountOfWindows;
}

bool CXin_Wang::UpdateTreeDepthBackWithChoice()
{
	while (!m_QueueForPseudoSources.empty()
		&& m_QueueForPseudoSources.top().birthTime 
		!= m_InfoAtVertices[m_QueueForPseudoSources.top().indexOfVert].birthTimeForCheckingValidity)
		m_QueueForPseudoSources.pop();

	while (!m_QueueForWindows.empty())
	{
		const QuoteWindow& quoteW = m_QueueForWindows.top();

		if (quoteW.pWindow->fBrachParentIsPseudoSource)
		{
			if (quoteW.pWindow->birthTimeOfParent != 
				m_InfoAtVertices[quoteW.pWindow->indexOfBrachParent].birthTimeForCheckingValidity)
			{
				delete quoteW.pWindow;
				m_QueueForWindows.pop();
			}
			else
				break;
		}
		else
		{
			if (quoteW.pWindow->birthTimeOfParent ==
				m_InfoAtAngles[quoteW.pWindow->indexOfBrachParent].birthTime)
				break;
			else if (quoteW.pWindow->fIsOnLeftSubtree ==
				(quoteW.pWindow->entryPropOfParent < m_InfoAtAngles[quoteW.pWindow->indexOfBrachParent].entryProp))
				break;
			else
			{
				delete quoteW.pWindow;
				m_QueueForWindows.pop();				
			}
		}
	}

	bool fFromQueueOfPseudoSources(false);		
	if (m_QueueForWindows.empty())
	{	
		if (!m_QueueForPseudoSources.empty())
		{
			const InfoAtVertex& infoOfHeadElemOfPseudoSources = m_InfoAtVertices[m_QueueForPseudoSources.top().indexOfVert];
			m_depthOfResultingTree = max(m_depthOfResultingTree, 
				infoOfHeadElemOfPseudoSources.levelOnSequenceTree);
			fFromQueueOfPseudoSources = true;
		}
	}
	else 
	{
		if (m_QueueForPseudoSources.empty())
		{
			const Window& infoOfHeadElemOfWindows = *m_QueueForWindows.top().pWindow;
			m_depthOfResultingTree = max(m_depthOfResultingTree,
				infoOfHeadElemOfWindows.levelOnSequenceTree);
			fFromQueueOfPseudoSources = false;
		}
		else
		{
			const QuoteInfoAtVertex& headElemOfPseudoSources = m_QueueForPseudoSources.top();
			const QuoteWindow& headElemOfWindows = m_QueueForWindows.top();
			if (headElemOfPseudoSources.disUptodate <= 
				headElemOfWindows.disUptodate)
			{
				m_depthOfResultingTree = max(m_depthOfResultingTree,
					m_InfoAtVertices[headElemOfPseudoSources.indexOfVert].levelOnSequenceTree);
				fFromQueueOfPseudoSources = true;
			}
			else
			{
				m_depthOfResultingTree = max(m_depthOfResultingTree,
					headElemOfWindows.pWindow->levelOnSequenceTree);
				fFromQueueOfPseudoSources = false;
			}
		}
	}	
	return fFromQueueOfPseudoSources;
}

vector<int> CXin_Wang::GetFixedVertSequence() const
{
	return m_fixedSequence;
}