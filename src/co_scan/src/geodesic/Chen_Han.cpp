// PreviousCH.cpp: implementation of the CChen_Han class.
//
//////////////////////////////////////////////////////////////////////
#include "Chen_Han.h"

// dsy
#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif
// dsy

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CChen_Han::CChen_Han(const CRichModel& model, int source) : CExactDGPMethod(model, source)
{
	m_nameOfAlgorithm = "CH";
}
CChen_Han::CChen_Han(const CRichModel& model, int source, int destination) : CExactDGPMethod(model, source, destination)
{
	m_nameOfAlgorithm = "CH";
}
CChen_Han::CChen_Han(const CRichModel& model, int source, double R) : CExactDGPMethod(model, source, R)
{
	m_nameOfAlgorithm = "CH";
}

CChen_Han::CChen_Han(const CRichModel& model, const map<int, double>& sources) : CExactDGPMethod(model, sources)
{
	m_nameOfAlgorithm = "CH";
}

CChen_Han::CChen_Han(const CRichModel& model, const map<int, double>& sources, const set<int> &destinations) : CExactDGPMethod(model, sources, destinations)
{
	m_nameOfAlgorithm = "CH";
}

CChen_Han::CChen_Han(const CRichModel& model, const set<int>& sources) : CExactDGPMethod(model, sources)
{
	m_nameOfAlgorithm = "CH";
}

CChen_Han::CChen_Han(const CRichModel& model, const set<int>& sources, double R) : CExactDGPMethod(model, sources, R)
{
	m_nameOfAlgorithm = "CH";
}

CChen_Han::CChen_Han(const CRichModel& model, const set<int>& sources, const set<int>& destinations) : CExactDGPMethod(model, sources, destinations)
{
	m_nameOfAlgorithm = "CH";
}


void CChen_Han::Initialize()
{
	CExactDGPMethod::Initialize();
	m_nMaxLenOfWindowQueue = 0;
	m_nMaxLenOfPseudoSourceQueue = 0;
	m_nCountOfWindows = 0;
	m_InfoAtAngles.resize(model.GetNumOfEdges());
}

void CChen_Han::Dispose()
{
	m_QueueForWindows = queue<QuoteWindow>();
	m_QueueForPseudoSources = queue<QuoteInfoAtVertex>();
}

void CChen_Han::Propagate()
{
	ComputeChildrenOfSource();
	bool fFromQueueOfPseudoSources = UpdateTreeDepthBackWithChoice();
	while (m_depthOfResultingTree < model.GetNumOfFaces() && !(m_QueueForPseudoSources.empty() && m_QueueForWindows.empty()))
	{		
		if (m_QueueForWindows.size() > m_nMaxLenOfWindowQueue)
			m_nMaxLenOfWindowQueue = (int)m_QueueForWindows.size();
		if (m_QueueForPseudoSources.size() > m_nMaxLenOfPseudoSourceQueue)
			m_nMaxLenOfPseudoSourceQueue = (int)m_QueueForPseudoSources.size();
		if (m_QueueForWindows.size() + m_QueueForPseudoSources.size() > m_maxLenOfQueue)
			m_maxLenOfQueue = m_QueueForWindows.size() + m_QueueForPseudoSources.size();
		if (fFromQueueOfPseudoSources) //pseudosource
		{				
			int indexOfVert = m_QueueForPseudoSources.front().indexOfVert;
			m_QueueForPseudoSources.pop();			
			ComputeChildrenOfPseudoSource(indexOfVert);			
		}
		else			
		{
			QuoteWindow quoteW = m_QueueForWindows.front();
			m_QueueForWindows.pop();
			ComputeChildrenOfWindow(quoteW);		
			delete quoteW.pWindow;
		}
		fFromQueueOfPseudoSources = UpdateTreeDepthBackWithChoice();
	}
}

void CChen_Han::CollectExperimentalResults()
{
	m_memory = ((double)model.GetNumOfVerts() * sizeof (InfoAtVertex)
		+ (double)model.GetNumOfEdges() * sizeof (Window)
		+ (double)m_maxLenOfQueue * sizeof(Window *)) / 1024 / 1024;
	for (int i = 0; i < m_scalarField.size(); ++i)
	{
		m_scalarField[i] = m_InfoAtVertices[i].disUptodate;
	}
	CDistanceApproach::CollectExperimentalResults();
}

void CChen_Han::AddIntoQueueOfPseudoSources(const QuoteInfoAtVertex& quoteOfPseudoSource)
{
	if (model.IsStronglyConvexVert(quoteOfPseudoSource.indexOfVert))
		return;
	m_QueueForPseudoSources.push(quoteOfPseudoSource);
}

void CChen_Han::AddIntoQueueOfWindows(QuoteWindow& quoteW)
{
	m_QueueForWindows.push(quoteW);
	++m_nCountOfWindows;
}

bool CChen_Han::UpdateTreeDepthBackWithChoice()
{
	while (!m_QueueForPseudoSources.empty()
		&& m_QueueForPseudoSources.front().birthTime != m_InfoAtVertices[m_QueueForPseudoSources.front().indexOfVert].birthTimeForCheckingValidity)
		m_QueueForPseudoSources.pop();

	while (!m_QueueForWindows.empty())
	{
		const QuoteWindow& quoteW = m_QueueForWindows.front();
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
			const InfoAtVertex& infoOfHeadElemOfPseudoSources = m_InfoAtVertices[m_QueueForPseudoSources.front().indexOfVert];
			m_depthOfResultingTree = max(m_depthOfResultingTree, 
				infoOfHeadElemOfPseudoSources.levelOnSequenceTree);
			fFromQueueOfPseudoSources = true;
		}
	}
	else 
	{
		if (m_QueueForPseudoSources.empty())
		{
			const Window& infoOfHeadElemOfWindows = *m_QueueForWindows.front().pWindow;
			m_depthOfResultingTree = max(m_depthOfResultingTree,
				infoOfHeadElemOfWindows.levelOnSequenceTree);
			fFromQueueOfPseudoSources = false;
		}
		else
		{
			const InfoAtVertex& infoOfHeadElemOfPseudoSources = m_InfoAtVertices[m_QueueForPseudoSources.front().indexOfVert];
			const Window& infoOfHeadElemOfWindows = *m_QueueForWindows.front().pWindow;
			if (infoOfHeadElemOfPseudoSources.levelOnSequenceTree <= 
				infoOfHeadElemOfWindows.levelOnSequenceTree)
			{
				m_depthOfResultingTree = max(m_depthOfResultingTree,
					infoOfHeadElemOfPseudoSources.levelOnSequenceTree);
				fFromQueueOfPseudoSources = true;
			}
			else
			{
				m_depthOfResultingTree = max(m_depthOfResultingTree,
					infoOfHeadElemOfWindows.levelOnSequenceTree);
				fFromQueueOfPseudoSources = false;
			}
		}
	}	
	return fFromQueueOfPseudoSources;
}

void CChen_Han::OutputExperimentalResults() const
{
	cout << "Experimental results are as follows:\n";
	cout << "Algorithm: " << m_nameOfAlgorithm << endl;
	cout << "Memory = " << m_memory << " Mega-bytes.\n";
	cout << "Timing = " << m_nTotalMilliSeconds << " ms.\n";
	cout << "MaxDepth = " << m_depthOfResultingTree << " levels.\n";
	cout << "MaxLenOfQue = " << m_maxLenOfQueue << " elements.\n";
	cout << "TotalWindowNum = " << m_nCountOfWindows << endl;
}


bool CChen_Han::IsTooNarrowWindow(const Window& w) const
{
	return w.proportions[1] - w.proportions[0] < 1e-5;
		//|| w.proportions[1] - w.proportions[0] >= 1 + LengthTolerance;
}

void CChen_Han::ComputeChildrenOfSource(int indexOfSourceVert, double dis)
{
	//if (!IsReachable(indexOfSourceVert))
	//	return;
	//m_InfoAtVertices[indexOfSourceVert].fParentIsPseudoSource;
	++m_InfoAtVertices[indexOfSourceVert].birthTimeForCheckingValidity;
	//m_InfoAtVertices[indexOfSourceVert].indexOfParent;
	//m_InfoAtVertices[indexOfSourceVert].indexOfRootVertOfParent;
	m_InfoAtVertices[indexOfSourceVert].levelOnSequenceTree = 0;
	m_InfoAtVertices[indexOfSourceVert].indexOfAncestor = indexOfSourceVert;
	m_InfoAtVertices[indexOfSourceVert].disUptodate = dis;
	//m_InfoAtVertices[indexOfSourceVert].entryProp;	

	int degree = (int)model.Neigh(indexOfSourceVert).size();
	for (int i = 0; i < degree; ++i) // vertex-nodes
	{
		FillVertChildOfPseudoSource(indexOfSourceVert, i);
	}
	
	for (int i = 0; i < degree; ++i)
	{
		CreateIntervalChildOfPseudoSource(indexOfSourceVert, i);	
	}
}

void CChen_Han::ComputeChildrenOfSource()
{
	for (map<int, double>::const_iterator it = m_sources.begin(); 
		it != m_sources.end(); ++it)
	{
		assert(it->first < model.GetNumOfVerts());
		//if (IsReachable(it->first))
		ComputeChildrenOfSource(it->first, it->second);
	}
}

void CChen_Han::ComputeChildrenOfPseudoSourceFromPseudoSource(int indexOfParentVertex)
{
	int degree = (int)model.Neigh(indexOfParentVertex).size();
	const vector<pair<int, double> >& neighs = model.Neigh(indexOfParentVertex);
	int indexOfParentOfParent = m_InfoAtVertices[indexOfParentVertex].indexOfDirectParent;
	int subIndex = model.GetSubindexToVert(indexOfParentVertex, indexOfParentOfParent);
	double angleSumPlus(0);
	int indexPlus;
	for (indexPlus = subIndex; indexPlus != (subIndex - 1 + degree) % degree; indexPlus = (indexPlus + 1) % degree)
	{
		angleSumPlus += neighs[indexPlus].second;
		if (angleSumPlus > M_PI - 3 * AngleTolerance)
			break;
	}
	double angleSumMinus = 0;
	int indexMinus;
	for (indexMinus = (subIndex - 1 + degree) % degree; 
	indexMinus == (subIndex - 1 + degree) % degree || indexMinus != (indexPlus - 1 + degree) % degree; 
	indexMinus = (indexMinus - 1 + degree) % degree)
	{
		angleSumMinus += neighs[indexMinus].second;
		if (angleSumMinus > M_PI - 3 * AngleTolerance)
			break;
	}
	if (indexMinus == (indexPlus - 1 + degree) % degree)
		return;
	//vertices;
	for (int i = 0; i < degree; ++i)
	//for (int i = (indexPlus + 1) % degree; i != (indexMinus + 1) % degree; i = (i + 1) % degree)
	{
		FillVertChildOfPseudoSource(indexOfParentVertex, i);
	}
	
	//windows
	double propPlus = 0;
	double propMinus = 1;
#if 1
	double devirationAngle = 20. * M_PI / 180.0;
	double anglePlus = model.Neigh(indexOfParentVertex)[indexPlus].second - (angleSumPlus - M_PI);
	
	if (model.Edge(model.Neigh(indexOfParentVertex)[indexPlus].first).indexOfOppositeVert != -1)
	{
		if (model.Neigh(indexOfParentVertex)[indexPlus].second 
			< 2 * devirationAngle)
		{
			propPlus = 0;
		}
		else if (angleSumPlus - M_PI < devirationAngle
			|| abs(anglePlus - model.Neigh(indexOfParentVertex)[indexPlus].second) 
			< devirationAngle
			|| model.Neigh(indexOfParentVertex)[indexPlus].second
			> M_PI - devirationAngle)
		{
			propPlus = (model.Neigh(indexOfParentVertex)[indexPlus].second - (angleSumPlus - M_PI))
				/ model.Neigh(indexOfParentVertex)[indexPlus].second
				- devirationAngle / model.Neigh(indexOfParentVertex)[indexPlus].second;
			//if (propPlus >= 1)
			//{
			//	cerr << "propPlus " << __LINE__ << ": " << propPlus << endl;
			//}
		}
		else 
		{
			double angleTmp = 
				model.Edge(model.Edge(model.Edge(model.Neigh(indexOfParentVertex)[indexPlus].first).indexOfLeftEdge).indexOfReverseEdge).angleOpposite;
			propPlus = sin(anglePlus) / sin(anglePlus + angleTmp) * model.Edge(model.Neigh(indexOfParentVertex)[indexPlus].first).length / model.Edge(model.Edge(model.Neigh(indexOfParentVertex)[indexPlus].first).indexOfRightEdge).length
				- devirationAngle / model.Neigh(indexOfParentVertex)[indexPlus].second;
			//if (propPlus >= 1)
			//{
			//	cerr << "propPlus " << __LINE__ << ": "  << propPlus << endl;
			//}
		}
		propPlus = max(0, propPlus);
		propPlus = min(1, propPlus);
	}

	double angleMinus = angleSumMinus - M_PI;
	if (model.Edge(model.Neigh(indexOfParentVertex)[indexMinus].first).indexOfOppositeVert != -1)
	{
		if (model.Neigh(indexOfParentVertex)[indexMinus].second 
			< 2 * devirationAngle)
		{
			propMinus = 1;
		}
		else if (angleSumMinus - M_PI < devirationAngle
			|| abs(angleSumMinus - M_PI - model.Neigh(indexOfParentVertex)[indexMinus].second) 
			< devirationAngle
			|| model.Neigh(indexOfParentVertex)[indexMinus].second
			> M_PI - devirationAngle)
		{
			propMinus = (angleSumMinus - M_PI) / model.Neigh(indexOfParentVertex)[indexMinus].second
				+ devirationAngle / model.Neigh(indexOfParentVertex)[indexMinus].second;
			//if (propMinus <= 0)
			//{
			//	cerr << "propMinus " << __LINE__ << ": "  << propMinus << endl;
			//}
		}
		else
		{
			double angleTmp = 
				model.Edge(model.Edge(model.Edge(model.Neigh(indexOfParentVertex)[indexMinus].first).indexOfLeftEdge).indexOfReverseEdge).angleOpposite;
			propMinus = sin(angleMinus) / sin(angleMinus + angleTmp) * model.Edge(model.Neigh(indexOfParentVertex)[indexMinus].first).length / model.Edge(model.Edge(model.Neigh(indexOfParentVertex)[indexMinus].first).indexOfRightEdge).length
				+ devirationAngle / model.Neigh(indexOfParentVertex)[indexMinus].second;
			//if (propMinus <= 0)
			//{
			//	cerr << "propMinus " << __LINE__ << ": "  << propMinus << endl;
			//}
		}
		propMinus = max(0, propMinus);
		propMinus = min(1, propMinus);
	}
#endif

	for (int i = indexPlus; i != (indexMinus + 1) % degree; i = (i + 1) % degree)
	{
		if (model.Edge(model.Neigh(indexOfParentVertex)[i].first).indexOfOppositeVert == -1)
			continue;
		double propL = 0;
		double propR = 1;
		if (indexPlus == i)
		{
			propR = 1 - propPlus;//
		}
		if (indexMinus == i)
		{
			propL = 1 - propMinus;//
		}
		
		CreateIntervalChildOfPseudoSource(indexOfParentVertex, i, propL, propR);
	}	
}

void CChen_Han::ComputeChildrenOfPseudoSourceFromWindow(int indexOfParentVertex)
{
	int degree = (int)model.Neigh(indexOfParentVertex).size();
	const vector<pair<int, double> >& neighs = model.Neigh(indexOfParentVertex);	
	int indexOfParentOfParent = m_InfoAtVertices[indexOfParentVertex].indexOfDirectParent;
	int leftVert = model.Edge(indexOfParentOfParent).indexOfLeftVert;
	int rightVert = model.Edge(indexOfParentOfParent).indexOfRightVert;
	int subIndexLeft = model.GetSubindexToVert(indexOfParentVertex, leftVert);
	int subIndexRight = (subIndexLeft + 1) % degree;
	double x1 = m_InfoAtVertices[indexOfParentVertex].entryProp * model.Edge(indexOfParentOfParent).length;
	double y1 = 0;
	double x2 = model.Edge(indexOfParentOfParent).length;
	double y2 = 0;
	x1 -= model.Edge(indexOfParentOfParent).coordOfOppositeVert.first;
	y1 -= model.Edge(indexOfParentOfParent).coordOfOppositeVert.second;
	x2 -= model.Edge(indexOfParentOfParent).coordOfOppositeVert.first;
	y2 -= model.Edge(indexOfParentOfParent).coordOfOppositeVert.second;

	double anglePlus = acos((x1 * x2 + y1 * y2) / sqrt((x1 * x1 + y1 * y1) * (x2 * x2 + y2 * y2)));
	double angleSumR(anglePlus);
	int indexPlus;
	for (indexPlus = subIndexRight; indexPlus != subIndexLeft; indexPlus = (indexPlus + 1) % degree)
	{
		angleSumR += neighs[indexPlus].second;
		if (angleSumR > M_PI - 4 * AngleTolerance)
			break;
	}
	double angleSumL = neighs[subIndexLeft].second - anglePlus;
	int indexMinus;
	for (indexMinus = (subIndexLeft - 1 + degree) % degree; indexMinus != (indexPlus - 1 + degree) % degree; indexMinus = (indexMinus - 1 + degree) % degree)
	{
		angleSumL += neighs[indexMinus].second;
		if (angleSumL > M_PI - 4 * AngleTolerance)
			break;
	}
	if (indexMinus == (indexPlus - 1 + degree) % degree)
		return;
	for (int i = 0; i < degree; ++i)//modified on 2013-10-18
	//for (int i = (indexPlus + 1) % degree; i != (indexMinus + 1) % degree; i = (i + 1) % degree)
	{
		FillVertChildOfPseudoSource(indexOfParentVertex, i);
	}	
	//windows
	double propMinus = 1;//
	double propPlus = 0;//	
	
#if 1
	double devirationAngle = 20. * M_PI / 180.;
	int minusEdge = model.Neigh(indexOfParentVertex)[indexMinus].first;
	if (model.Edge(minusEdge).indexOfOppositeVert != -1)
	{
		double angleRemaining = angleSumL - M_PI;
		if (model.Neigh(indexOfParentVertex)[indexMinus].second 
			< 2 * devirationAngle)
		{
			propMinus = 1;
		}
		else if (angleSumL - M_PI < devirationAngle
			|| abs(angleRemaining - model.Neigh(indexOfParentVertex)[indexMinus].second)
			< devirationAngle
			|| model.Neigh(indexOfParentVertex)[indexMinus].second 
			> M_PI - devirationAngle)
		{
			propMinus = angleRemaining / model.Neigh(indexOfParentVertex)[indexMinus].second 
				+ devirationAngle / model.Neigh(indexOfParentVertex)[indexMinus].second;
			//if (propMinus <= 0)
			//{
			//	cerr << "propMinus " << __LINE__ << ": "  << propMinus << "\n";
			//	assert(0);
			//}
		}		
		else
		{
			propMinus = sin(angleRemaining) * model.Edge(minusEdge).length
				/ sin(angleRemaining + 
				model.Edge(model.Edge(model.Edge(minusEdge).indexOfLeftEdge).indexOfReverseEdge).angleOpposite)
				/ model.Edge(model.Edge(minusEdge).indexOfRightEdge).length
				+ devirationAngle / model.Neigh(indexOfParentVertex)[indexMinus].second;
			//if (propMinus <= 0)
			//{
			//	cerr << "propMinus " << __LINE__ << ": "  << propMinus << "\n";
			//	assert(0);
			//}
		}

		propMinus = max(0, propMinus);
		propMinus = min(1, propMinus);
	}
	
	int rightEdge = model.Neigh(indexOfParentVertex)[indexPlus].first;
	if (model.Edge(rightEdge).indexOfOppositeVert != -1)
	{
		double angleRemaining = model.Neigh(indexOfParentVertex)[indexPlus].second - (angleSumR - M_PI);
		if (model.Neigh(indexOfParentVertex)[indexPlus].second 
			< 2 * devirationAngle)
		{
			propPlus = 0;
		}
		else if (angleSumR - M_PI < devirationAngle
			|| abs(angleRemaining) < devirationAngle
			|| model.Neigh(indexOfParentVertex)[indexPlus].second 
			> M_PI - devirationAngle)
		{
			propPlus = angleRemaining / model.Neigh(indexOfParentVertex)[indexPlus].second
				- devirationAngle / model.Neigh(indexOfParentVertex)[indexPlus].second;
			//if (propPlus >= 1)
			//{
			//	cerr << "propPlus " << __LINE__ << ": "  << propPlus << "\n";
			//	assert(0);
			//}
		}
		else
		{
			propPlus = sin(angleRemaining) 
				* model.Edge(rightEdge).length 
				/ sin(angleRemaining + model.Edge(model.Edge(model.Edge(rightEdge).indexOfLeftEdge).indexOfReverseEdge).angleOpposite) 
				/ model.Edge(model.Edge(rightEdge).indexOfRightEdge).length
				- devirationAngle / model.Neigh(indexOfParentVertex)[indexPlus].second;
			//if (propPlus >= 1)
			//{
			//	cerr << "propPlus " << __LINE__ << ":= " << propPlus << "\n";
			//	assert(0);
			//}
		}

		propPlus = max(0, propPlus);
		propPlus = min(1, propPlus);
	}

#endif
	for (int i = indexPlus; i != (indexMinus + 1) % degree; i = (i + 1) % degree)
	{
		if (model.Edge(model.Neigh(indexOfParentVertex)[i].first).indexOfOppositeVert == -1)
			continue;
		double propL = 0;
		double propR = 1;

		if (indexPlus == i)
		{
			propR = 1 - propPlus;//
		}
		if (indexMinus == i)
		{
			propL = 1 - propMinus;//
		}
		CreateIntervalChildOfPseudoSource(indexOfParentVertex, i, propL, propR);
	}
}

void CChen_Han::ComputeChildrenOfWindow(QuoteWindow& quoteParentWindow)
{	
	const Window& w = *quoteParentWindow.pWindow;
	const CRichModel::CEdge& edge = model.Edge(w.indexOfCurEdge);
	double entryProp = model.ProportionOnEdgeByImage(w.indexOfCurEdge, w.coordOfPseudoSource);	

	if (entryProp >= w.proportions[1] 
		|| entryProp >= 1 - LengthTolerance)
	{
		ComputeTheOnlyLeftChild(w);
		return;
	}
	
	if (entryProp <= w.proportions[0]
		|| entryProp <= LengthTolerance)
	{
		ComputeTheOnlyRightChild(w);
		return;
	}
	double disToAngle = model.DistanceToOppositeAngle(w.indexOfCurEdge, w.coordOfPseudoSource);
	int incidentVertex = edge.indexOfOppositeVert;
	bool fLeftChildToCompute(false), fRightChildToCompute(false);
	bool fWIsWinning(false);
	double totalDis = w.disToRoot + disToAngle;

	if (m_InfoAtAngles[w.indexOfCurEdge].birthTime == -1)
	{
		fLeftChildToCompute = fRightChildToCompute = true;
		fWIsWinning = true;
	}
	else
	{
		if (totalDis < m_InfoAtAngles[w.indexOfCurEdge].disUptodate
			- 2 * LengthTolerance)
		{
			fLeftChildToCompute = fRightChildToCompute = true;
			fWIsWinning = true;
		}
		else if (totalDis < m_InfoAtAngles[w.indexOfCurEdge].disUptodate
			+ 2 * LengthTolerance)
		{
			fLeftChildToCompute = fRightChildToCompute = true;
			fWIsWinning = false;
		}
		else
		{
			fLeftChildToCompute = entryProp < m_InfoAtAngles[w.indexOfCurEdge].entryProp;
			fRightChildToCompute = !fLeftChildToCompute;
			fWIsWinning = false;
		}
		
	}
	if (!fWIsWinning)
	{		
		if (fLeftChildToCompute)
		{
			ComputeTheOnlyLeftTrimmedChild(w);				
		}
		if (fRightChildToCompute)
		{
			ComputeTheOnlyRightTrimmedChild(w);				
		}
		return;
	}

	m_InfoAtAngles[w.indexOfCurEdge].disUptodate = totalDis;
	m_InfoAtAngles[w.indexOfCurEdge].entryProp = entryProp;
	++m_InfoAtAngles[w.indexOfCurEdge].birthTime;

	ComputeLeftTrimmedChildWithParent(w);
	ComputeRightTrimmedChildWithParent(w);	
	if (//IsReachable(incidentVertex) &&
		totalDis < m_InfoAtVertices[incidentVertex].disUptodate - LengthTolerance)
	{
		m_InfoAtVertices[incidentVertex].fParentIsPseudoSource = false;
		++m_InfoAtVertices[incidentVertex].birthTimeForCheckingValidity;
		m_InfoAtVertices[incidentVertex].indexOfDirectParent = w.indexOfCurEdge;
		m_InfoAtVertices[incidentVertex].indexOfRootVertOfDirectParent = w.indexOfRootVertex;
		m_InfoAtVertices[incidentVertex].levelOnSequenceTree = w.levelOnSequenceTree + 1;
		m_InfoAtVertices[incidentVertex].indexOfAncestor = w.indexOfAncestor;
		m_InfoAtVertices[incidentVertex].disUptodate = totalDis;
		m_InfoAtVertices[incidentVertex].entryProp = entryProp;
		
		//if (!model.IsStronglyConvexVert(incidentVertex))
			AddIntoQueueOfPseudoSources(QuoteInfoAtVertex(m_InfoAtVertices[incidentVertex].birthTimeForCheckingValidity,
			incidentVertex, totalDis));
	}
}

void CChen_Han::ComputeChildrenOfPseudoSource(int indexOfParentVertex)
{
	if (m_InfoAtVertices[indexOfParentVertex].fParentIsPseudoSource)
		ComputeChildrenOfPseudoSourceFromPseudoSource(indexOfParentVertex);
	else
		ComputeChildrenOfPseudoSourceFromWindow(indexOfParentVertex);
}

void CChen_Han::CreateIntervalChildOfPseudoSource(int source, int subIndexOfIncidentEdge, double propL/* = 0*/, double propR/* = 1*/)
{
	int indexOfIncidentEdge = model.Neigh(source)[subIndexOfIncidentEdge].first;
	if (model.IsExtremeEdge(indexOfIncidentEdge))
		return;
	const CRichModel::CEdge& edge = model.Edge(indexOfIncidentEdge);
	int edgeIndex = edge.indexOfRightEdge;
	if (model.IsExtremeEdge(edgeIndex))
		return;
	QuoteWindow quoteW;
	quoteW.pWindow = new Window;
	quoteW.pWindow->proportions[0] = propL;
	quoteW.pWindow->proportions[1] = propR;
	if (IsTooNarrowWindow(*quoteW.pWindow))
	{
		delete quoteW.pWindow;
		return;
	}
	//quoteW.pWindow->fIsOnLeftSubtree;
	quoteW.pWindow->fBrachParentIsPseudoSource = true;
	//quoteW.pWindow->fDirectParentEdgeOnLeft;
	quoteW.pWindow->fDirectParenIsPseudoSource = true;
	quoteW.pWindow->birthTimeOfParent = m_InfoAtVertices[source].birthTimeForCheckingValidity;
	quoteW.pWindow->indexOfBrachParent = source;
	quoteW.pWindow->indexOfRootVertex = source;
	quoteW.pWindow->indexOfCurEdge = edgeIndex;
	quoteW.pWindow->levelOnSequenceTree = m_InfoAtVertices[source].levelOnSequenceTree + 1;
	quoteW.pWindow->indexOfAncestor = m_InfoAtVertices[source].indexOfAncestor;
	quoteW.pWindow->disToRoot = m_InfoAtVertices[source].disUptodate;	
	quoteW.pWindow->entryPropOfParent;
	int reverseEdge = model.Edge(edgeIndex).indexOfReverseEdge;
	quoteW.pWindow->coordOfPseudoSource = model.GetNew2DCoordinatesByReversingCurrentEdge(reverseEdge,
		model.Edge(reverseEdge).coordOfOppositeVert);
	AddIntoQueueOfWindows(quoteW);
}

void CChen_Han::FillVertChildOfPseudoSource(int source, int subIndexOfVert)
{
	const CRichModel::CEdge& edge = model.Edge(model.Neigh(source)[subIndexOfVert].first);
	int index = edge.indexOfRightVert;	
	//if (!IsReachable(index))
	//	return;
	double dis = m_InfoAtVertices[source].disUptodate + edge.length;
	if (dis >= m_InfoAtVertices[index].disUptodate - LengthTolerance)
		return;
	m_InfoAtVertices[index].fParentIsPseudoSource = true;
	++m_InfoAtVertices[index].birthTimeForCheckingValidity;
	m_InfoAtVertices[index].indexOfDirectParent = source;
	//m_InfoAtVertices[index].indexOfRootVertOfParent;
	m_InfoAtVertices[index].levelOnSequenceTree = m_InfoAtVertices[source].levelOnSequenceTree + 1;
	m_InfoAtVertices[index].indexOfAncestor = m_InfoAtVertices[source].indexOfAncestor;
	m_InfoAtVertices[index].disUptodate = dis;
	//m_InfoAtVertices[index].entryProp;		
	//if (!model.IsStronglyConvexVert(index))
		AddIntoQueueOfPseudoSources(QuoteInfoAtVertex(m_InfoAtVertices[index].birthTimeForCheckingValidity,
		index, dis));
}


void CChen_Han::ComputeTheOnlyLeftChild(const Window& w)
{	
	if (model.IsExtremeEdge(model.Edge(w.indexOfCurEdge).indexOfLeftEdge))
		return;
	QuoteWindow quoteW;
	quoteW.pWindow = new Window;
	quoteW.pWindow->proportions[0] = model.ProportionOnLeftEdgeByImage(w.indexOfCurEdge,
		w.coordOfPseudoSource, w.proportions[0]) - 1e-4;
	quoteW.pWindow->proportions[0] = max(0, quoteW.pWindow->proportions[0]);
	quoteW.pWindow->proportions[1] = model.ProportionOnLeftEdgeByImage(w.indexOfCurEdge,
		w.coordOfPseudoSource, w.proportions[1]) + 1e-4;
	quoteW.pWindow->proportions[1] = min(1, quoteW.pWindow->proportions[1]);
	if (IsTooNarrowWindow(*quoteW.pWindow))
	{
		delete quoteW.pWindow;
		return;
	}
	quoteW.pWindow->fBrachParentIsPseudoSource = w.fBrachParentIsPseudoSource;
	quoteW.pWindow->fDirectParenIsPseudoSource = false;
	quoteW.pWindow->fDirectParentEdgeOnLeft = true;
	quoteW.pWindow->indexOfCurEdge = model.Edge(w.indexOfCurEdge).indexOfLeftEdge;
	quoteW.pWindow->disToRoot = w.disToRoot;

	quoteW.pWindow->coordOfPseudoSource = model.GetNew2DCoordinatesByRotatingAroundLeftChildEdge(w.indexOfCurEdge, w.coordOfPseudoSource);

	quoteW.pWindow->fIsOnLeftSubtree = w.fIsOnLeftSubtree;
	quoteW.pWindow->levelOnSequenceTree = w.levelOnSequenceTree + 1;	
	quoteW.pWindow->indexOfAncestor = w.indexOfAncestor;
	quoteW.pWindow->entryPropOfParent = w.entryPropOfParent;
	quoteW.pWindow->birthTimeOfParent = w.birthTimeOfParent;
	quoteW.pWindow->indexOfBrachParent = w.indexOfBrachParent;
	quoteW.pWindow->indexOfRootVertex = w.indexOfRootVertex;

	AddIntoQueueOfWindows(quoteW);
}

void CChen_Han::ComputeTheOnlyRightChild(const Window& w)
{
	if (model.IsExtremeEdge(model.Edge(w.indexOfCurEdge).indexOfRightEdge))
		return;
	QuoteWindow quoteW;
	quoteW.pWindow = new Window;
	quoteW.pWindow->proportions[0] = model.ProportionOnRightEdgeByImage(w.indexOfCurEdge,
		w.coordOfPseudoSource, w.proportions[0]) - 1e-4;
	quoteW.pWindow->proportions[0] = max(0, quoteW.pWindow->proportions[0]);
	quoteW.pWindow->proportions[1] = model.ProportionOnRightEdgeByImage(w.indexOfCurEdge,
		w.coordOfPseudoSource, w.proportions[1]) + 1e-4;
	quoteW.pWindow->proportions[1] = min(1, quoteW.pWindow->proportions[1]);
	if (IsTooNarrowWindow(*quoteW.pWindow))
	{
		delete quoteW.pWindow;
		return;
	}
	quoteW.pWindow->fBrachParentIsPseudoSource = w.fBrachParentIsPseudoSource;
	quoteW.pWindow->fDirectParenIsPseudoSource = false;
	quoteW.pWindow->fDirectParentEdgeOnLeft = false;
	quoteW.pWindow->indexOfCurEdge = model.Edge(w.indexOfCurEdge).indexOfRightEdge;
	quoteW.pWindow->disToRoot = w.disToRoot;
	quoteW.pWindow->coordOfPseudoSource = model.GetNew2DCoordinatesByRotatingAroundRightChildEdge(w.indexOfCurEdge, w.coordOfPseudoSource);

	quoteW.pWindow->levelOnSequenceTree = w.levelOnSequenceTree + 1;	
	quoteW.pWindow->indexOfAncestor = w.indexOfAncestor;
	quoteW.pWindow->birthTimeOfParent = w.birthTimeOfParent;
	quoteW.pWindow->indexOfBrachParent = w.indexOfBrachParent;
	quoteW.pWindow->indexOfRootVertex = w.indexOfRootVertex;
	quoteW.pWindow->fIsOnLeftSubtree = w.fIsOnLeftSubtree;
	quoteW.pWindow->entryPropOfParent = w.entryPropOfParent;

	AddIntoQueueOfWindows(quoteW);
}

void CChen_Han::ComputeTheOnlyLeftTrimmedChild(const Window& w)
{
	if (model.IsExtremeEdge(model.Edge(w.indexOfCurEdge).indexOfLeftEdge))
		return;
	QuoteWindow quoteW;
	quoteW.pWindow = new Window;
	quoteW.pWindow->proportions[0] = model.ProportionOnLeftEdgeByImage(w.indexOfCurEdge,
		w.coordOfPseudoSource, w.proportions[0]) - 1e-4;
	quoteW.pWindow->proportions[0] = max(0, quoteW.pWindow->proportions[0]);
	quoteW.pWindow->proportions[1] = 1;
	if (IsTooNarrowWindow(*quoteW.pWindow))
	{
		delete quoteW.pWindow;
		return;
	}
	quoteW.pWindow->fBrachParentIsPseudoSource = w.fBrachParentIsPseudoSource;
	quoteW.pWindow->fDirectParenIsPseudoSource = false;
	quoteW.pWindow->fDirectParentEdgeOnLeft = true;
	quoteW.pWindow->indexOfCurEdge = model.Edge(w.indexOfCurEdge).indexOfLeftEdge;
	quoteW.pWindow->disToRoot = w.disToRoot;
	quoteW.pWindow->coordOfPseudoSource = model.GetNew2DCoordinatesByRotatingAroundLeftChildEdge(w.indexOfCurEdge, w.coordOfPseudoSource);

	quoteW.pWindow->levelOnSequenceTree = w.levelOnSequenceTree + 1;	
	quoteW.pWindow->indexOfAncestor = w.indexOfAncestor;
	quoteW.pWindow->birthTimeOfParent = w.birthTimeOfParent;
	quoteW.pWindow->indexOfBrachParent = w.indexOfBrachParent;
	quoteW.pWindow->indexOfRootVertex = w.indexOfRootVertex;
	quoteW.pWindow->fIsOnLeftSubtree = w.fIsOnLeftSubtree;
	quoteW.pWindow->entryPropOfParent = w.entryPropOfParent;

	AddIntoQueueOfWindows(quoteW);	
}

void CChen_Han::ComputeTheOnlyRightTrimmedChild(const Window& w)
{
	if (model.IsExtremeEdge(model.Edge(w.indexOfCurEdge).indexOfRightEdge))
		return;
	QuoteWindow quoteW;
	quoteW.pWindow = new Window;
	quoteW.pWindow->proportions[0] = 0;
	quoteW.pWindow->proportions[1] = model.ProportionOnRightEdgeByImage(w.indexOfCurEdge,
		w.coordOfPseudoSource, w.proportions[1]) + 1e-4;
	quoteW.pWindow->proportions[1] = min(1, quoteW.pWindow->proportions[1]);
	if (IsTooNarrowWindow(*quoteW.pWindow))
	{
		delete quoteW.pWindow;
		return;
	}
	quoteW.pWindow->fBrachParentIsPseudoSource = w.fBrachParentIsPseudoSource;
	quoteW.pWindow->fDirectParenIsPseudoSource = false;
	quoteW.pWindow->fDirectParentEdgeOnLeft = false;
	quoteW.pWindow->indexOfCurEdge = model.Edge(w.indexOfCurEdge).indexOfRightEdge;
	quoteW.pWindow->disToRoot = w.disToRoot;
	quoteW.pWindow->coordOfPseudoSource = model.GetNew2DCoordinatesByRotatingAroundRightChildEdge(w.indexOfCurEdge, w.coordOfPseudoSource);

	quoteW.pWindow->levelOnSequenceTree = w.levelOnSequenceTree + 1;	
	quoteW.pWindow->indexOfAncestor = w.indexOfAncestor;
	quoteW.pWindow->birthTimeOfParent = w.birthTimeOfParent;
	quoteW.pWindow->indexOfBrachParent = w.indexOfBrachParent;
	quoteW.pWindow->indexOfRootVertex = w.indexOfRootVertex;
	quoteW.pWindow->fIsOnLeftSubtree = w.fIsOnLeftSubtree;
	quoteW.pWindow->entryPropOfParent = w.entryPropOfParent;

	AddIntoQueueOfWindows(quoteW);	
}

void CChen_Han::ComputeLeftTrimmedChildWithParent(const Window& w)
{
	if (model.IsExtremeEdge(model.Edge(w.indexOfCurEdge).indexOfLeftEdge))
		return;
	QuoteWindow quoteW;
	quoteW.pWindow = new Window;
	quoteW.pWindow->proportions[0] = model.ProportionOnLeftEdgeByImage(w.indexOfCurEdge,
		w.coordOfPseudoSource, w.proportions[0]) - 1e-4;
	quoteW.pWindow->proportions[0] = max(0, quoteW.pWindow->proportions[0]);
	quoteW.pWindow->proportions[1] = 1;
	if (IsTooNarrowWindow(*quoteW.pWindow))
	{
		delete quoteW.pWindow;
		return;
	}
	quoteW.pWindow->fBrachParentIsPseudoSource = false;
	quoteW.pWindow->fDirectParenIsPseudoSource = false;
	quoteW.pWindow->fDirectParentEdgeOnLeft = true;
	quoteW.pWindow->indexOfCurEdge = model.Edge(w.indexOfCurEdge).indexOfLeftEdge;
	quoteW.pWindow->disToRoot = w.disToRoot;
	quoteW.pWindow->coordOfPseudoSource = model.GetNew2DCoordinatesByRotatingAroundLeftChildEdge(w.indexOfCurEdge, w.coordOfPseudoSource);

	quoteW.pWindow->levelOnSequenceTree = w.levelOnSequenceTree + 1;	
	quoteW.pWindow->indexOfAncestor = w.indexOfAncestor;
	quoteW.pWindow->birthTimeOfParent = m_InfoAtAngles[w.indexOfCurEdge].birthTime;
	quoteW.pWindow->indexOfBrachParent = w.indexOfCurEdge;
	quoteW.pWindow->indexOfRootVertex = w.indexOfRootVertex;
	quoteW.pWindow->fIsOnLeftSubtree = true;
	quoteW.pWindow->entryPropOfParent = m_InfoAtAngles[w.indexOfCurEdge].entryProp;

	AddIntoQueueOfWindows(quoteW);
}


void CChen_Han::ComputeRightTrimmedChildWithParent(const Window& w)
{
	if (model.IsExtremeEdge(model.Edge(w.indexOfCurEdge).indexOfRightEdge))
		return;
	QuoteWindow quoteW;
	quoteW.pWindow = new Window;
	quoteW.pWindow->proportions[0] = 0;
	quoteW.pWindow->proportions[1] = model.ProportionOnRightEdgeByImage(w.indexOfCurEdge,
		w.coordOfPseudoSource, w.proportions[1]) + 1e-4;
	quoteW.pWindow->proportions[1] = min(1, quoteW.pWindow->proportions[1]);
	//quoteW.pWindow->proportions[1] = max(quoteW.pWindow->proportions[1], quoteW.pWindow->proportions[0]);
	if (IsTooNarrowWindow(*quoteW.pWindow))
	{
		delete quoteW.pWindow;
		return;
	}
	quoteW.pWindow->fBrachParentIsPseudoSource = false;
	quoteW.pWindow->fDirectParenIsPseudoSource = false;
	quoteW.pWindow->fDirectParentEdgeOnLeft = false;
	quoteW.pWindow->indexOfCurEdge = model.Edge(w.indexOfCurEdge).indexOfRightEdge;
	quoteW.pWindow->disToRoot = w.disToRoot;
	quoteW.pWindow->coordOfPseudoSource = model.GetNew2DCoordinatesByRotatingAroundRightChildEdge(w.indexOfCurEdge, w.coordOfPseudoSource);

	quoteW.pWindow->fIsOnLeftSubtree = false;
	quoteW.pWindow->birthTimeOfParent = m_InfoAtAngles[w.indexOfCurEdge].birthTime;
	quoteW.pWindow->indexOfBrachParent = w.indexOfCurEdge;
	quoteW.pWindow->indexOfRootVertex = w.indexOfRootVertex;
	quoteW.pWindow->levelOnSequenceTree = w.levelOnSequenceTree + 1;	
	quoteW.pWindow->indexOfAncestor = w.indexOfAncestor;
	quoteW.pWindow->entryPropOfParent = m_InfoAtAngles[w.indexOfCurEdge].entryProp;

	AddIntoQueueOfWindows(quoteW);
}

//bool CChen_Han::IsReachable(int v) const
//{
//	return true;
//}