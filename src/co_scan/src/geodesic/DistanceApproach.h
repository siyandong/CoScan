#pragma once
#include "EdgePoint.h"
#include "RichModel.h"
#define __int64 int64_t
class CDistanceApproach
{
protected:
	vector<double> m_scalarField;
	double m_maxDisValue;
	double m_radius;
	const CRichModel& model;
	map<int, double> m_sources;	
	set<int> m_destinations;
	string m_nameOfAlgorithm;
	 __int64 m_maxLenOfQueue;
	 __int64 m_depthOfResultingTree;
	
	 __int64 m_nTotalMilliSeconds;
protected:
	virtual void Initialize() = 0;
	virtual void Dispose() = 0;
	virtual void Propagate() = 0;
	virtual void CollectExperimentalResults() = 0;
public:
	double m_memory;
	CDistanceApproach(const CRichModel& model, int source);
	CDistanceApproach(const CRichModel& model, int source, int destination);
	CDistanceApproach(const CRichModel& model, int source, double R);
	CDistanceApproach(const CRichModel& model, const map<int, double>& sources);
	CDistanceApproach(const CRichModel& model, const map<int, double>& sources, const set<int> &destinations);	
	CDistanceApproach(const CRichModel& model, const set<int>& sources);	
	CDistanceApproach(const CRichModel& model, const set<int>& sources, double R);
	CDistanceApproach(const CRichModel& model, const set<int>& sources, const set<int>& destinations);
public:
	virtual void Execute();	
	virtual vector<EdgePoint> BacktraceShortestPath(int end) const = 0;
	vector<EdgePoint> BacktraceIsoline(double val) const;
	virtual int GetAncestor(int vIndex) const = 0;	
	double GetMaxDistance() const;
	virtual  __int64 GetMaxLenOfQueue() const{return m_maxLenOfQueue;}
	virtual  __int64 GetMaxPropagationLevels()const {return m_depthOfResultingTree;}
	string GetAlgorithmName() const;
	 __int64 GetRunTime() const{return m_nTotalMilliSeconds;}
	double GetMemoryCost() const {return m_memory;}
	virtual void OutputExperimentalResults() const;
	const vector<double>& GetDistanceField() const;
	//vector<double> GetNormalizedDistanceField() const;
	//vector<double> GetDistanceFieldDividedBy(double denominator) const;
	static vector<double> DiffDistanceField(const vector<double>& field1, 
		const vector<double>& field2);	
};

template<class T>
pair<double, vector<EdgePoint>> GetShortestPathBetween(const CRichModel& model, int source, int end)
{
	set<int> sources;
	sources.insert(source);
	set<int> destinations;
	destinations.insert(end);
	
	T alg(model, sources, destinations);
	alg.Execute();
	return make_pair(alg.GetDistanceField()[end], alg.BacktraceShortestPath(end));
}

template<class T>
double GetShortestDistanceBetween(const CRichModel& model, int source, int end)
{
	set<int> sources;
	sources.insert(source);
	set<int> destinations;
	destinations.insert(end);
	
	T alg(model, sources, destinations);
	alg.Execute();
	return alg.GetDistanceField()[end];
}
