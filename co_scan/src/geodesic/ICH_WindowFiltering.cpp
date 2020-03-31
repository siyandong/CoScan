// ImprovedCHWithEdgeValve.cpp: implementation of the CICH_WindowFiltering class.
//

#include "ICH_WindowFiltering.h"


CICH_WindowFiltering::CICH_WindowFiltering(const CRichModel& model, int source) : CChen_Han(model, source)
{
	m_nameOfAlgorithm = "ICH1";
}

CICH_WindowFiltering::CICH_WindowFiltering(const CRichModel& model, int source, int destination) : CChen_Han(model, source, destination)
{
	m_nameOfAlgorithm = "ICH1";
}

CICH_WindowFiltering::CICH_WindowFiltering(const CRichModel& model, int source, double R) : CChen_Han(model, source, R)
{
	m_nameOfAlgorithm = "ICH1";
}

CICH_WindowFiltering::CICH_WindowFiltering(const CRichModel& model, const map<int, double>& sources) : CChen_Han(model, sources)
{
	m_nameOfAlgorithm = "ICH1";
}

CICH_WindowFiltering::CICH_WindowFiltering(const CRichModel& model, const map<int, double>& sources, const set<int> &destinations) : CChen_Han(model, sources, destinations)
{
	m_nameOfAlgorithm = "ICH1";
}

CICH_WindowFiltering::CICH_WindowFiltering(const CRichModel& model, const set<int>& sources) : CChen_Han(model, sources)
{
	m_nameOfAlgorithm = "ICH1";
}

CICH_WindowFiltering::CICH_WindowFiltering(const CRichModel& model, const set<int>& sources, double R) : CChen_Han(model, sources, R)
{
	m_nameOfAlgorithm = "ICH1";
}

CICH_WindowFiltering::CICH_WindowFiltering(const CRichModel& model, const set<int>& sources, const set<int>& destinations) : CChen_Han(model, sources, destinations)
{
	m_nameOfAlgorithm = "ICH1";
}

bool CICH_WindowFiltering::CheckValidityWithXinWangFiltering(Window& w) const
{
	if (w.fDirectParenIsPseudoSource)
		return true;
	const CRichModel::CEdge& edge = model.Edge(w.indexOfCurEdge);
	//TRACE
	//out << setw(10) << setiosflags(ios_base::fixed) << min(10000, m_InfoAtVertices[edge.indexOfLeftVert].disUptodate) << "\t";
	//out << setw(10) << setiosflags(ios_base::fixed) << min(10000, m_InfoAtVertices[edge.indexOfRightVert].disUptodate) << "\t";
	//out << setw(10) << setiosflags(ios_base::fixed) << min(10000, m_InfoAtVertices[model.Edge(edge.indexOfReverseEdge).indexOfOppositeVert].disUptodate) << "\n";
	//out.flush();
	int leftVert = edge.indexOfLeftVert;
	double detaX = w.coordOfPseudoSource.first - w.proportions[1] * edge.length;
	double rightLen = sqrt(detaX * detaX + w.coordOfPseudoSource.second * w.coordOfPseudoSource.second);
	if (m_InfoAtVertices[leftVert].disUptodate < 10000 * model.GetScale() && m_InfoAtVertices[leftVert].disUptodate + w.proportions[1] * edge.length
		< w.disToRoot + rightLen)
	{
		return false;
	}
	int rightVert = edge.indexOfRightVert;
	detaX = w.coordOfPseudoSource.first - w.proportions[0] * edge.length;
	double leftLen = sqrt(detaX * detaX + w.coordOfPseudoSource.second * w.coordOfPseudoSource.second);
	if (m_InfoAtVertices[rightVert].disUptodate < 10000  * model.GetScale() && m_InfoAtVertices[rightVert].disUptodate + (1 - w.proportions[0]) * edge.length
		< w.disToRoot + leftLen)
	{
		return false;
	}
	const CRichModel::CEdge& oppositeEdge = model.Edge(edge.indexOfReverseEdge);
	double xOfVert = edge.length - oppositeEdge.coordOfOppositeVert.first;
	double yOfVert = -oppositeEdge.coordOfOppositeVert.second;
	if (m_InfoAtVertices[oppositeEdge.indexOfOppositeVert].disUptodate < 10000 * model.GetScale())	
	{
		if (w.fDirectParentEdgeOnLeft)
		{
			double deta = w.disToRoot + leftLen - m_InfoAtVertices[oppositeEdge.indexOfOppositeVert].disUptodate;
			if (deta <= 0)
				return true;
			detaX = xOfVert - w.proportions[0] * edge.length;
			if (detaX * detaX + yOfVert * yOfVert < deta * deta)
				return false;
		}
		else
		{
			double deta = w.disToRoot + rightLen - m_InfoAtVertices[oppositeEdge.indexOfOppositeVert].disUptodate;
			if (deta <= 0)
				return true;
			detaX = xOfVert - w.proportions[1] * edge.length;
			if (detaX * detaX + yOfVert * yOfVert < deta * deta)
				return false;
		}	
	}
	return true;
}

void CICH_WindowFiltering::AddIntoQueueOfWindows(QuoteWindow& quoteW)
{
	if (!CheckValidityWithXinWangFiltering(*quoteW.pWindow))
	{
		delete quoteW.pWindow;
		return;
	}
	m_QueueForWindows.push(quoteW);
	++m_nCountOfWindows;
}