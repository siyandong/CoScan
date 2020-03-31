// difference between polygons

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "global.h"

using namespace std;

// exact kernel
typedef CGAL::Exact_predicates_exact_constructions_kernel		EK;
typedef EK::Point_2												EPoint_2;
typedef CGAL::Polygon_2<EK>										EPolygon_2;
typedef CGAL::Polygon_with_holes_2<EK>							EPolygon_with_holes_2;
typedef std::list<EPolygon_with_holes_2>						EPwh_list_2;

// main
int main(int argc, char* argv[])
{
/*
	if (argc == 5)
	{
		//cerr<<"program		   "<<argv[0]<<endl;
		//cerr<<"domainFilePath  "<<argv[1]<<endl;
		//cerr<<"holeFilePath    "<<argv[2]<<endl;
		//cerr<<"resultFilePath  "<<argv[3]<<endl;
		//cerr<<"communicateFile "<<argv[4]<<endl;
		domainFilePath = argv[1];
		holeFilePath = argv[2];
		resultFilePath = argv[3];
		communicateFile = argv[4];
	}
//*/
	cerr<<"diff between polygons...";
	// set up
	ifstream ifs(communicateFile.c_str());
	int process_num = 0;
	ifs >> process_num;
	ifs.clear();
	ifs.close();
	// load polygons
	EPolygon_with_holes_2 eDomain;
	ifs.open(domainFilePath.c_str());
	ifs >> eDomain;
	ifs.clear();
	ifs.close();
	EPolygon_2 eHole;
	ifs.open(holeFilePath.c_str());
	ifs >> eHole;
	ifs.clear();
	ifs.close();
	// add noise or not. avoid numerical error.
	if (process_num == -1)
	{
		// noise domain outer boundary
		{
			srand((unsigned)time(NULL));
			vector<EPoint_2> temp;
			for (int i = 0; i < eDomain.outer_boundary().size(); i++)
				temp.push_back(EPoint_2(eDomain.outer_boundary()[i].x() + (double)rand() / RAND_MAX / 10, eDomain.outer_boundary()[i].y() + (double)rand() / RAND_MAX / 10));
			eDomain.outer_boundary() = EPolygon_2(temp.begin(), temp.end());
		}
		// noise hole
		{
			srand((unsigned)time(NULL));
			vector<EPoint_2> temp;
			for (int i = 0; i < eHole.size(); i++)
				temp.push_back(EPoint_2(eHole[i].x() + (double)rand() / RAND_MAX / 10, eHole[i].y() + (double)rand() / RAND_MAX / 10));
			eHole = EPolygon_2(temp.begin(), temp.end());
		}
	}
	// compute difference
	EPwh_list_2 eList;
	CGAL::difference(eDomain, eHole, back_inserter(eList));
	// result
	if (eList.size() == 0)
	{
		ofstream ofs(communicateFile.c_str());
		ofs << 2 << endl;
		ofs.clear();
		ofs.close();
		return -1;
	}
	// get domain result
	EPolygon_with_holes_2 ePass;
	if (eList.size() != 1) // if domain become not continious
	{
		auto j = eList.begin();
		CGAL::Lazy_exact_nt<CGAL::Quotient<CGAL::Gmpq> > max_area = 0;
		for (auto i = eList.begin(); i != eList.end(); i++)
		{
			if ((i->outer_boundary().area())*(i->outer_boundary().area()) > max_area*max_area)
			{
				max_area = i->outer_boundary().area();
				j = i;
			}
		}
		ePass = EPolygon_with_holes_2(*j);
	}
	else // domain continious
	{
		ePass = EPolygon_with_holes_2(*eList.begin());
	}
	// save result and feedback
	ofstream ofs(communicateFile.c_str());
	ofs << 1 << endl;
	ofs.clear();
	ofs.close();
	ofs.open(resultFilePath.c_str());
	ofs << ePass;
	ofs.clear();
	ofs.close();
	// end.
	cerr << "done." << endl;
	return 0;
}