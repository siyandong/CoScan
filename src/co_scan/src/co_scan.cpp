// std
#include <stdio.h> 
#include <stdlib.h> 
// ros
#include "ros/ros.h"

// processing scanning data
#include "data_engine.h"
// navigation
#include "navigation.h"

using namespace std;

// robot number
int rbt_num = 3;

// main
int main(int argc, char **argv)
{
    // set up ros env
    ros::init(argc, argv, "co_scan");
    ros::NodeHandle n;

    // set up global variables
    {
        g_rbtTrajectories.resize(rbt_num);
        g_camTrajectories.resize(rbt_num);
        // test
        g_scene_boundary.clear();
        g_scene_boundary.push_back(cv::Point(560, 478));
        g_scene_boundary.push_back(cv::Point(560, 710));
        g_scene_boundary.push_back(cv::Point(672, 710));
        g_scene_boundary.push_back(cv::Point(672, 478));
    }

    // data engine
    DataEngine de(rbt_num);
    de.initialize();

    // navigation
    Navigation nav(de, 8);
/*
    // test geodesic
    {
        nav.test();
        exit(-1);
    }
//*/

    // progressive scanning
    cerr << "scanning surroundings..." << endl;
    de.SetUpSurroundings(); 
    //de.showStatement();
    while(1)
    {
PLAN:
        // motion planning
        nav.processCurrentScene();
        nav.OMT_TSP();
        nav.Trajectory_Optimization();
        g_plan_iteration++;
        goto SCAN;
        
        // todo
        goto END;
SCAN:
        // ask robot to move and scan 
        nav.moveRobotsAndScan();
        // todo
        goto PLAN;
    }
    // finished.
END:
    de.showCellMap();
    // finished.
    printf("program done.\n");
    return 0;
}