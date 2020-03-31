#include "data_engine.h"
#include <iostream>

using namespace std;

typedef unsigned char uchar;

bool send_success = false;
bool reply_success = false;

// initialization: connect socket.
bool DataEngine::create_connection()
{
    sockClient = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in serveraddr;
    bzero(&serveraddr, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(PORT);
    inet_pton(AF_INET, server_ip, &serveraddr.sin_addr);

    printf("connecting server...");
    if(connect(sockClient, (struct sockaddr*)&serveraddr, sizeof(serveraddr)) == -1)
    {
        printf("failed.\n");
        return false;
    }
    printf("connected.\n");

    return true;
}
//*/

// send thread
void socketSendThread(int sockClient, char* data, int data_lengh)
{
    cerr<<"command message "<<data<<endl;
    int rtn_num = send(sockClient, data, data_lengh, 0);
    cerr << "socket_send_thread: send command to server ... ";
    send_success = true;
    sleep(1);
    while (!reply_success)
    {
        cerr<<"command message "<<data<<endl;
        int rtn_num = send(sockClient, data, data_lengh, 0);
        cerr << "socket_send_thread: send command to server ... ";
        sleep(1);
    }
    return;
}

// stop a connection thread 
void stopThread(int sockClient)
{
    // send command
    char sendData[1];
    sendData[0] = 'e';
    //send(sockClient, sendData, 1, 0); // send command
    thread send_task(socketSendThread, sockClient, sendData, 1); // send command
    send_task.detach(); // execute the thread
}

// get rgbd
bool DataEngine::getRGBDFromServer()
{

// test
    sleep(0.1);
    stopThread(sockClient);
    create_connection();

getRGBDFromGazebo_begin:
    cerr << "getting rgbd data..." << endl;
    // command: ask for rgbd data
    char sendData[1];
    sendData[0] = '3';
    //send(sockClient, sendData, 1, 0); // send command
    send_success = false;
    reply_success = false;
    thread send_task(socketSendThread, sockClient, sendData, 1); // send command
    send_task.detach();
    while(!send_success) sleep(0.1);
    // recive data
    // rgb
    {
        // rcv package
        int data_len = (480 * 640 * 3 * (sizeof(uchar)) * rbt_num);
        char* rcvData = new char[data_len];
        int n = 0;
        int count = 0;
        char pkgData[MAXRECV];
        n = 0;
        count = 0;
        while (1)
        {
            // break point
            if (count == data_len) // 2018-07-26
                break; // 2018-07-26

            // for last package
            if (count + MAXRECV >= data_len) // next package is the last package
            {
                n = recv(sockClient, pkgData, data_len - count, 0); // recive #(data_len - count) data
                //cout<<"last packeg len "<<n<<endl;
                int rst = n;
                while (rst < data_len - count){
                    memcpy(&rcvData[count], &pkgData, rst);
                    count += rst;
                    //rst = data_len - count;
                    //rst = recv(sockClient, pkgData, rst, 0); // recive #(data_len - count) data
                    rst = recv(sockClient, pkgData, data_len - count, 0); // recive #(data_len - count) data, 2018-07-26
                }
                // rst >= data_len - count
                if (rst > data_len - count)
                {
                    cout << "error: recieve more than data_len." << endl;
                    //getchar();
                    memcpy(&rcvData[count], &pkgData, data_len - count);
                    count += rst;
                    break;
                }
                memcpy(&rcvData[count], &pkgData, rst);
                count += rst;
                break;
            }

            // recv
            n = recv(sockClient, pkgData, MAXRECV, 0);
            //cout<<"recved "<< n <<" byte"<<endl;
            if (n>0) reply_success = true;
            
            // check for errors
            if (n <= 0)
            {
                cout << "n = " << n << ", data_len = " << data_len << ", count = " << count << endl;
                if (n == -1)
                {
                    //int err = errno;
                    //cout << "error code: " << err << endl; // err: 10060 
                    delete [] rcvData;
                    // reset connection
                    stopThread(sockClient);
                    create_connection();
                    goto getRGBDFromGazebo_begin;
                }
                break;
            }
            else
            {
                if (n == -1) // test 2018-09-10
                {
                    cout << "before memcpy..." << endl;
                    //getchar();
                }
                memcpy(&rcvData[count], &pkgData, n); // error? most time works well, sometimes results in access vialation reading location. 2018-09-10 update: access vialation reading location because of n == -1.
                count += n;
                if (n == -1) // test 2018-09-10
                {
                    cout << "after memcpy, count = " << count << ", n = " << n << endl;
                    //getchar();
                }
            }
        }
        //cout<<"successfully recved rgb package."<<endl;

        // convert package data to rgb images
        int ind = 0;
        for (int id = 0; id < rbt_num; id++)
        {
            for (int i = 0; i < 480; i++)
            {
                for (int j = 0; j < 640; j++)
                {
                    memcpy(&m_rgb[id].ptr<cv::Vec3b>(i)[j][2], &rcvData[ind], sizeof(uchar));
                    ind += sizeof(uchar);
                    memcpy(&m_rgb[id].ptr<cv::Vec3b>(i)[j][1], &rcvData[ind], sizeof(uchar));
                    ind += sizeof(uchar);
                    memcpy(&m_rgb[id].ptr<cv::Vec3b>(i)[j][0], &rcvData[ind], sizeof(uchar));
                    ind += sizeof(uchar);
                }
            }
        }
        delete [] rcvData;
    }

    // depth
    {
        // rcv package
        int data_len = (480 * 640 * (sizeof(short)) * rbt_num);
        char* rcvData = new char[data_len];
        int n = 0;
        int count = 0;
        char pkgData[MAXRECV];
        n = 0;
        count = 0;
        while (1){
            if (count + MAXRECV >= data_len){
                n = recv(sockClient, pkgData, data_len - count, 0);
                int rst = n;
                while (rst < data_len - count){
                    memcpy(&rcvData[count], &pkgData, rst);
                    count += rst;
                    rst = data_len - count;
                    rst = recv(sockClient, pkgData, rst, 0);
                }
                memcpy(&rcvData[count], &pkgData, rst);
                count += rst;
                break;
            }
            n = recv(sockClient, pkgData, MAXRECV, 0);
            memcpy(&rcvData[count], &pkgData, n);
            count += n;
        }
        // cpy to depth
        int ind = 0;
        for (int id = 0; id < rbt_num; id++)
        {
            for (int i = 0; i < 480; i++)
            {
                for (int j = 0; j < 640; j++)
                {
                    memcpy(&m_depth[id].ptr<ushort>(i)[j], &rcvData[ind], sizeof(short));
                    ind += sizeof(short);
                }
            }
        }
        delete [] rcvData;
    }

/*
    // test show img
    for (int id = 0; id < rbt_num; id++)
    {
      cv::imshow("get rgb", m_rgb[id]);
      cv::imshow("get depth", m_depth[id]);
      cv::waitKey(0);
    }
//*/

    cerr << "done." << endl;
    return true;
}

// get pose
bool DataEngine::getPoseFromServer()
{
    // set up: rcv data
    int data_len = (7 * sizeof(float) * rbt_num);
    char* rcvData = new char[data_len];
    char pkgData[MAXRECV];

// test
    sleep(0.1);
    stopThread(sockClient);
    create_connection();

getPoseFromLinux_begin:

    cerr << "getting pose data ... " << endl;

    // command: ask for pose data
    char sendData[1];
    sendData[0] = '1';
    //send(sockClient, sendData, 1, 0); // send command
    send_success = false;
    reply_success = false;
    thread send_task(socketSendThread, sockClient, sendData, 1); // send command
    send_task.detach(); // execute the thread
    while(!send_success) sleep(0.1);
    int n = 0; // recv return code
    int count = 0; // recved data length
    // start to recv data
    while (1)
    {
        // the last package
        if (count + MAXRECV >= data_len)
        {
            n = recv(sockClient, pkgData, data_len - count, 0);
            //cout << "the last pkg： data_len - count: " << data_len - count << ", n: " << n << endl;
            if (n>0) reply_success = true;
            if (n < 0)
            {
                if (count == data_len) // finish transfer
                    break;
                cout << "count = " << count << endl;
                cerr << "recv return < 0, input to retry..." << endl;
                //int err = WSAGetLastError();
                //cout << "error code: " << err << endl; // err: 10060 
                // reset connection
                stopThread(sockClient);
                create_connection();
                goto getPoseFromLinux_begin;
            }
            int rst = n;
            while (rst < data_len - count){
                memcpy(&rcvData[count], &pkgData, rst);
                count += rst;
                rst = data_len - count;
                rst = recv(sockClient, pkgData, rst, 0);
            }
            memcpy(&rcvData[count], &pkgData, rst);
            count += rst;
            break;
        }
        // packages before the last package. recv #MAXRECV data, record into rcvData.
        n = recv(sockClient, pkgData, MAXRECV, 0);
        memcpy(&rcvData[count], &pkgData, n);
        count += n;
    }
    // cpy to pose
    int ind = 0;
    for (int id = 0; id < rbt_num; id++)
    {
        for (int i = 0; i < 7; i++)
        {
            memcpy(&m_pose[id][i], &rcvData[ind], sizeof(float));
            ind += sizeof(float);
        }
    }
    // test
    for (int rid = 0; rid < rbt_num; rid++)
    {
        cerr<<"pose "<<rid<<":"<<endl;
        for (int vid = 0; vid < 7; ++vid)
        {
            cerr<<m_pose[rid][vid]<<" ";
        }
        cerr<<endl;
        if (m_pose[rid][3]==m_pose[rid][4]&&m_pose[rid][4]==m_pose[rid][5]&&m_pose[rid][5]==m_pose[rid][6])
        {
            cerr<<m_pose[rid][0]<<" "<<m_pose[rid][1]<<" "<<m_pose[rid][2]<<" "<<m_pose[rid][3]<<" "<<m_pose[rid][4]<<" "<<m_pose[rid][5]<<" "<<m_pose[rid][6]<<" "<<endl;
            //getchar();getchar();getchar();
            stopThread(sockClient);
            create_connection();
            goto getPoseFromLinux_begin;
        }
        if (__isnan(m_pose[rid][0]))
        {
            cerr<<m_pose[rid][0]<<" "<<m_pose[rid][1]<<" "<<m_pose[rid][2]<<" "<<m_pose[rid][3]<<" "<<m_pose[rid][4]<<" "<<m_pose[rid][5]<<" "<<m_pose[rid][6]<<" "<<endl;
            //getchar();getchar();getchar();
            stopThread(sockClient);
            create_connection();
            goto getPoseFromLinux_begin;
        }
    }
    delete [] rcvData;
    cerr << "done." << endl;
    return true;
}

bool DataEngine::rcvRGBDFromServer()
{
    cerr << "waitting for rgbd data ... ";
    // recive data
    // rgb
    {
        // rcv data
        int data_len = (480 * 640 * 3 * (sizeof(uchar)) * rbt_num);
        char* rcvData = new char[data_len];
        int n = 0;
        int count = 0;
        char pkgData[MAXRECV];
        // rbt0
        n = 0;
        count = 0;
        while (1){
            //if (count == data_len)
            //  break;
            if (count + MAXRECV >= data_len){
                n = recv(sockClient, pkgData, data_len - count, 0);
                //cout << "the last pkg： data_len - count: " << data_len - count << ", n: " << n << endl;
                int rst = n;
                while (rst < data_len - count){
                    memcpy(&rcvData[count], &pkgData, rst);
                    count += rst;
                    rst = data_len - count;
                    rst = recv(sockClient, pkgData, rst, 0);
                }
                memcpy(&rcvData[count], &pkgData, rst);
                count += rst;
                break;
            }
            n = recv(sockClient, pkgData, MAXRECV, 0);
            //if (count + n <= data_len)
            memcpy(&rcvData[count], &pkgData, n);
            //else
            //  memcpy(&rcvData[count], &pkgData, data_len - count);
            count += n;
        }
        // cpy to rgb
        int ind = 0;
        for (int id = 0; id < rbt_num; id++)
        {
            for (int i = 0; i < 480; i++)
            {
                for (int j = 0; j < 640; j++)
                {
                    memcpy(&m_rgb[id].ptr<cv::Vec3b>(i)[j][2], &rcvData[ind], sizeof(uchar));
                    ind += sizeof(uchar);
                    memcpy(&m_rgb[id].ptr<cv::Vec3b>(i)[j][1], &rcvData[ind], sizeof(uchar));
                    ind += sizeof(uchar);
                    memcpy(&m_rgb[id].ptr<cv::Vec3b>(i)[j][0], &rcvData[ind], sizeof(uchar));
                    ind += sizeof(uchar);
                }
            }
        }
        delete [] rcvData;
    }

    //{// test show img
    //  for (int rid = 0; rid < rbt_num; rid++)
    //  {
    //      char name[10];
    //      sprintf(name, "rgb_%d", rid);
    //      cv::imshow(name, rgb[rid]);
    //      cv::waitKey(0);
    //  }
    //}

    // depth
    {
        // rcv data
        int data_len = (480 * 640 * (sizeof(short)) * rbt_num);
        char* rcvData = new char[data_len];
        int n = 0;
        int count = 0;
        char pkgData[MAXRECV];
        // rbt0
        n = 0;
        count = 0;
        while (1){
            //if (count == data_len)
            //  break;
            if (count + MAXRECV >= data_len){
                n = recv(sockClient, pkgData, data_len - count, 0);
                //cout << "the last pkg： data_len - count: " << data_len - count << ", n: " << n << endl;
                int rst = n;
                while (rst < data_len - count){
                    memcpy(&rcvData[count], &pkgData, rst);
                    count += rst;
                    rst = data_len - count;
                    rst = recv(sockClient, pkgData, rst, 0);
                }
                memcpy(&rcvData[count], &pkgData, rst);
                count += rst;
                break;
            }
            n = recv(sockClient, pkgData, MAXRECV, 0);
            //if (count + n <= data_len)
            memcpy(&rcvData[count], &pkgData, n);
            //else
            //  memcpy(&rcvData[count], &pkgData, data_len - count);
            count += n;
        }
        // cpy to depth
        int ind = 0;
        for (int id = 0; id < rbt_num; id++)
        {
            for (int i = 0; i < 480; i++)
            {
                for (int j = 0; j < 640; j++)
                {
                    memcpy(&m_depth[id].ptr<ushort>(i)[j], &rcvData[ind], sizeof(short));
                    ind += sizeof(short);
                }
            }
        }
        delete [] rcvData;
    }

    //// test show img
    //for (int id = 0; id < rbt_num; id++)
    //{
    //  cv::imwrite("depth.png", depth[id]);
    //  cv::imshow("get rgb", rgb[id]);
    //  cv::imshow("get depth", depth[id]);
    //  cv::waitKey(0);
    //}

    cerr << "done" << endl;
    return true;
}

bool DataEngine::rcvPoseFromServer()
{
    cerr << "waitting for pose data ... ";
    // rcv data
    int data_len = (7 * sizeof(float) * rbt_num);
    char* rcvData = new char[data_len];
    int n = 0;
    int count = 0;
    char pkgData[MAXRECV];
    while (1){
        if (count + MAXRECV >= data_len){
            n = recv(sockClient, pkgData, data_len - count, 0);
            //cout << "the last pkg： data_len - count: " << data_len - count << ", n: " << n << endl;
            int rst = n;
            while (rst < data_len - count){
                memcpy(&rcvData[count], &pkgData, rst);
                count += rst;
                rst = data_len - count;
                rst = recv(sockClient, pkgData, rst, 0);
            }
            memcpy(&rcvData[count], &pkgData, rst);
            count += rst;
            break;
        }
        /*if (count + MAXRECV >= data_len){
        n = recv(sockClient, pkgData, data_len - count, 0);
        cout << "the last pkg： data_len - count: " << data_len - count << ", n: " << n << endl;
        memcpy(&rcvData[count], &pkgData, n);
        count += n;
        break;
        }*/
        n = recv(sockClient, pkgData, MAXRECV, 0);
        //cout << ""
        //if (count + n <= data_len)
        memcpy(&rcvData[count], &pkgData, n);
        //else
        //  memcpy(&rcvData[count], &pkgData, data_len - count);
        count += n;
    }
    //cout << "final count: " << count << endl;
    // cpy to pose
    int ind = 0;
    for (int id = 0; id < rbt_num; id++)
    {
        for (int i = 0; i < 7; i++)
        {
            memcpy(&m_pose[id][i], &rcvData[ind], sizeof(float));
            ind += sizeof(float);
        }
        /*cout << "pose" << id << ": " << endl;
        for (int i = 0; i < 7; i++){
        cout << pose[id][i] << " ";
        }
        cout << endl;*/
    }

    delete [] rcvData;
    cerr << "done" << endl;
    return true;
}

// ask to set up surroundings, use to initialization
void DataEngine::scanSurroundingsCmd()
{
    // ask for data
    char sendData[1];
    sendData[0] = '5';
    send(sockClient, sendData, 1, 0);
}

// move to views
bool DataEngine::socket_move_to_views(vector<vector<double>> poses)
{
// test
    sleep(0.1);
    stopThread(sockClient);
    create_connection();
    // ask to set poses 
    char sendData[1];
    sendData[0] = 'm';
    send(sockClient, sendData, 1, 0);
    sleep(0.1);
    // send poses data 
    int data_len = rbt_num * 7 * sizeof(float);
    char* poseData = new char[data_len];
    int ind = 0;
    for (int rid = 0; rid < rbt_num; rid++)
    {
        for (int i = 0; i < 7; i++)
        {
            float pass = (float)poses[rid][i];
            memcpy(&poseData[ind], &pass, sizeof(float));
            ind += sizeof(float);
        }
    }
    int rtn_num = send(sockClient, poseData, data_len, 0);
    // free 
    delete [] poseData;
    return true;
}

// set up scan envir, not finished
void DataEngine::SetUpSurroundings()
{
    scanSurroundingsCmd();
    for (int i = 0; i < 6; i++)
    {
        rcvPoseFromServer();
        rcvRGBDFromServer();
        fuseScans2MapAndTree(); // insert scans multi-robot to tree
        projectOctree2Map(); // project to 2d
    }
}

// coordinate system transfer
pair<Eigen::MatrixXd, Eigen::Vector3d> DataEngine::coord_trans_7f_rt(vector<float> pose)
{
    Eigen::Quaternion<double> q((double)pose[6], (double)pose[3], (double)pose[4], (double)pose[5]);
    Eigen::MatrixXd r = q.toRotationMatrix();
    Eigen::Vector3d t(pose[0], pose[1], pose[2]);
    return pair<Eigen::MatrixXd, Eigen::Vector3d>(r, t);
}

// coordinate system transfer
iro::SE2 DataEngine::coord_trans_7f_se2(vector<float> pose)
{
    pair<Eigen::MatrixXd, Eigen::Vector3d> rt = coord_trans_7f_rt(pose);
    Eigen::MatrixXd r = rt.first;
    Eigen::Vector3d t = rt.second;
    double theta = asin(pose[5]) * 2;
    Eigen::Vector3d origin(0, 0, 0); // gazebo coordinate
    origin = r*origin + t;
    //rbtpose[rbt_id].coordinate = cv::Point2f(-origin[1], origin[0]);
    // to img coordinate, then to OT coordinate
    double nodesize = map_cellsize;
    double scale = nodesize / map_cellsize;
    // u行v列
    double v = ((-origin[1] - xmin_oc_g) / nodesize)*scale;
    double u = ((zmax_oc_g - origin[0]) / nodesize)*scale;
    if (u < 0)
        u = 0;
    if (v < 0)
        v = 0;
    if(__isnan(v)||__isnan(u)||__isnan(theta))
    {
        cerr<<"se2 nan: "<<v<<" "<<u<<" "<<theta<<endl;
        cerr<<pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" "<<pose[3]<<" "<<pose[4]<<" "<<pose[5]<<" "<<pose[6]<<" "<<endl;
        getchar();getchar();getchar();
        exit(-1);
    }
    return iro::SE2(v, -u, theta);
}

// coordinate transformation: oc2cell return (row, col)
Eigen::Vector2i DataEngine::coord_trans_oc2cell(Eigen::Vector3d p_oc)
{
    Eigen::Vector2i result = Eigen::Vector2i::Zero(2, 1);
    result[1] = (int)((p_oc[0] - xmin_oc_g) / map_cellsize);//列
    result[0] = (int)((zmax_oc_g - p_oc[2]) / map_cellsize);//行
    if (result[0] < 0) result[0] = 0;
    if (result[0] >= map_rows) result[0] = map_rows - 1;
    if (result[1] < 0) result[1] = 0;
    if (result[1] >= map_cols) result[1] = map_cols - 1;
    return result;
}

// coordinate transformation: oc2cell return (row, col)
Eigen::Vector2i DataEngine::coord_trans_oc2cell(Eigen::Vector3d p_oc, double node_size, int scale)
{
    Eigen::Vector2i result = Eigen::Vector2i::Zero(2, 1);
    result[1] = (int)((p_oc[0] - xmin_oc_g) / node_size)*scale;//列
    result[0] = (int)((zmax_oc_g - p_oc[2]) / node_size)*scale;//行
    return result;
}

// coordinate system transfer: se22gazebo
vector<double> DataEngine::coord_trans_se22gazebo(iro::SE2 pose)
{
    // set up
    vector<double> result;
    result.resize(7);
    // convert
    double v = pose.translation().x();
    double u = -pose.translation().y();
    if (v < 10 || u < 10)
    {
        cerr << "error: out of boundary" << endl;
        getchar();
    }
    double octree_x = v * map_cellsize + xmin_oc_g;
    double octree_z = zmax_oc_g - u * map_cellsize;
    double theta = pose.rotation().angle();
    Eigen::Quaternion<double> q(cos(theta / 2), 0.0, 0.0, sin(theta / 2));
    Eigen::MatrixXd r = q.toRotationMatrix();
    Eigen::Vector3d t(octree_z, -octree_x, g_camera_height);
    result[0] = octree_z;
    result[1] = -octree_x;
    result[2] = g_camera_height;
    result[3] = 0.0;
    result[4] = 0.0;
    result[5] = sin(theta / 2);
    result[6] = cos(theta / 2);
    return result;
}

// project octree 2 map, not finished
void DataEngine::projectOctree2Map()
{
    // label free cells in cellmap according to octree
    for (octomap::OcTree::leaf_iterator it = m_recon3D.m_tree->begin_leafs(), end = m_recon3D.m_tree->end_leafs(); it != end; ++it)
    {
        // out boundary 
        if (fabs(it.getY()) > project_max_height)
            continue;
        // coordinate 
        double nodesize = it.getSize();
        int scale = nodesize / map_cellsize;
        int v = (int)((it.getX() - xmin_oc_g) / nodesize)*scale;
        int u = (int)((zmax_oc_g - it.getZ()) / nodesize)*scale;
        if (u < 0)
            u = 0;
        if (v < 0)
            v = 0;
        if (m_recon2D.m_cellmap[u][v].isScanned) // 防止离太近导致occupied被错误的标记为free
            continue;
        m_recon2D.m_cellmap[u][v].coordinate.x = it.getX();
        m_recon2D.m_cellmap[u][v].coordinate.y = it.getZ();
        // free 
        if (m_recon3D.m_tree->search(it.getKey())->getOccupancy() < 0.5 && -it.getY() < free_voxel_porject_height)
        { // 该voxel为free
            //if (!m_recon2D.m_cellmap[u][v].isScanned) // new free
            //    explore_counter++;
            m_recon2D.m_cellmap[u][v].isScanned = true;
            m_recon2D.m_cellmap[u][v].isOccupied = false;
            m_recon2D.m_cellmap[u][v].isFree = true;
            if (scale != 1)
            {
                for (int i = 0; i < scale; i++)
                {
                    for (int j = 0; j < scale; j++)
                    {
                        if (!m_recon2D.m_cellmap[u + i][v + j].isOccupied)
                        {
                            m_recon2D.m_cellmap[u + i][v + j].isScanned = true;
                            m_recon2D.m_cellmap[u + i][v + j].coordinate.x = it.getX();
                            m_recon2D.m_cellmap[u + i][v + j].coordinate.y = it.getZ();
                            m_recon2D.m_cellmap[u + i][v + j].isOccupied = false;
                            m_recon2D.m_cellmap[u + i][v + j].isFree = true;
                        }
                    }
                }
            }
        }
    }
    // label occupied cells in cellmap according to octree
    for (octomap::OcTree::leaf_iterator it = m_recon3D.m_tree->begin_leafs(), end = m_recon3D.m_tree->end_leafs(); it != end; ++it)
    {
        // out boundary
        if (fabs(it.getY()) > project_max_height || fabs(it.getY()) < project_min_height)
            continue;
        // coordinate
        double nodesize = it.getSize();
        int scale = nodesize / map_cellsize;
        int v = (int)((it.getX() - xmin_oc_g) / nodesize)*scale;
        int u = (int)((zmax_oc_g - it.getZ()) / nodesize)*scale;
        if (u < 0)
            u = 0;
        if (v < 0)
            v = 0;
        m_recon2D.m_cellmap[u][v].coordinate.x = it.getX();
        m_recon2D.m_cellmap[u][v].coordinate.y = it.getZ();
        // occupied
        //if (tree->search(it.getKey())->getOccupancy() > 0.5 && it.getY() < -0.2)
        if (m_recon3D.m_tree->search(it.getKey())->getOccupancy() > 0.5)
        { // 该voxel为occupied
            // need new iteration check
            { 
                //if (!cellmap[u][v].isScanned) // new occupied
                //    explore_counter++;
                //if (cellmap[u][v].isScanned && cellmap[u][v].isFree) // new collision
                //    explore_counter += 2;
            }
            m_recon2D.m_cellmap[u][v].isScanned = true;
            m_recon2D.m_cellmap[u][v].isOccupied = true;
            m_recon2D.m_cellmap[u][v].isFree = false;
            if (scale != 1)
            {
                for (int i = 0; i < scale; i++)
                {
                    for (int j = 0; j < scale; j++)
                    {
                        m_recon2D.m_cellmap[u + i][v + j].isScanned = true;
                        m_recon2D.m_cellmap[u + i][v + j].coordinate.x = it.getX();
                        m_recon2D.m_cellmap[u + i][v + j].coordinate.y = it.getZ();
                        m_recon2D.m_cellmap[u + i][v + j].isOccupied = true;
                        m_recon2D.m_cellmap[u + i][v + j].isFree = false;
                    }
                }
            }
        }
    }
    // label free cells in cellmap that octomap not record
    for (int cid = 0; cid < m_free_space_contours2d.size(); cid++)
    {
        if (m_free_space_contours2d[cid].empty()) continue;
        for (int r = 0; r < map_rows; r++)
        {
            for (int c = 0; c < map_cols; c++)
            {
                if (!m_recon2D.m_cellmap[r][c].isScanned)
                {
                    if (cv::pointPolygonTest(m_free_space_contours2d[cid], cv::Point(c, r), false) >= 0)
                    { // inside
                        m_recon2D.m_cellmap[r][c].isScanned = true;
                        m_recon2D.m_cellmap[r][c].isFree = true;
                        m_recon2D.m_cellmap[r][c].isOccupied = false;
                    }
                }
            }
        }
    }

    // fill small holes in known region
    fillTrivialHolesKnownRegion();

    // scene boundary constraint. avoid robots move out of boundary to infinite unknown space.
    if (!g_scene_boundary.empty())
    {
        for (int r = 0; r < map_rows; r++)
        {
            for (int c = 0; c < map_cols; c++)
            {
                // if a scanned cell is out of boundary, set to 'scanned & occupied'
                if (m_recon2D.m_cellmap[r][c].isScanned)
                {
                    if (cv::pointPolygonTest(g_scene_boundary, cv::Point(c, r), false) <= 0) // out_of_boundary or on_boundary
                    {
                        m_recon2D.m_cellmap[r][c].isScanned = false;
                        m_recon2D.m_cellmap[r][c].isFree = false;
                        m_recon2D.m_cellmap[r][c].isOccupied = true;
                        //cerr<<"voxel out of boundary."<<endl; getchar(); getchar(); getchar();
                    }
                    /*
                    else if (cv::pointPolygonTest(g_scene_boundary, cv::Point(c, r), true) < 2) // add and then commented. 20200311.
                    {
                        m_recon2D.m_cellmap[r][c].isScanned = false;
                        m_recon2D.m_cellmap[r][c].isFree = false;
                        m_recon2D.m_cellmap[r][c].isOccupied = true;
                        //cerr<<"voxel out of boundary."<<endl; getchar(); getchar(); getchar();
                    }
                    //*/
                }
            }
        }
    }

// todo: uncertainty 2d 

    return;
}
//*/

// find free contours that octomap cant handle, not finished
void DataEngine::findExtraFreeSpace(int rid, cv::Mat depth, vector<float> pose)
{
    // set up 
    cv::Mat dpth = depth.clone();

    // convert pose to rt 
    pair<Eigen::MatrixXd, Eigen::Vector3d> rt = coord_trans_7f_rt(pose);
    Eigen::MatrixXd r = rt.first;
    Eigen::Vector3d t = rt.second;

    // depth_img 2 vector: use to find free space
    cv::Mat depth_vec = cv::Mat::zeros(1, dpth.cols, CV_8UC1);
    for (int c = 0; c < dpth.cols; c++)
    {
        bool value_valid = false;
        for (int r = 0; r < dpth.rows; r++)
        {
            //if (depth.ptr<ushort>(r)[c] <= scan_max_range && depth.ptr<ushort>(r)[c] != 0)
            if (dpth.ptr<ushort>(r)[c] != 0)
            {
                value_valid = true;
                break;
            }
        }
        if (value_valid)
            depth_vec.ptr<uchar>(0)[c] = 255;
    }
    vector<int> free_indexes;
    for (int crt = 0; crt < depth_vec.cols; crt++)
    {
        if (depth_vec.ptr<uchar>(0)[crt] == 0)
            free_indexes.push_back(crt);
    }
    vector<pair<int, int>> free_spaces;
    //cerr << "free_indexes.size() = " << free_indexes.size() << endl;
    if (free_indexes.size() > 1)
    {
        //cout << "free_indexes.size() > 1" << endl;
        int begin_index = -1;
        int end_index = -1;
        for (int crt = 0; crt < free_indexes.size(); crt++)
        {
            if (crt == 0){ // the first point
                begin_index = free_indexes[crt];
                continue;
            }
            int last = crt - 1;
            if (free_indexes[crt] - free_indexes[last] > 1)
            { // at this time, last is the end point of this group，crt is the beg point of next group
                end_index = free_indexes[last];
                if (begin_index != end_index)
                    free_spaces.push_back(pair<int, int>(begin_index, end_index));
                begin_index = free_indexes[crt];
            }
            if (crt == free_indexes.size() - 1){ // the last point
                end_index = free_indexes[crt];
                if (begin_index != end_index)
                    free_spaces.push_back(pair<int, int>(begin_index, end_index));
            }
        }
    }
    //cerr << "free_spaces.size() = " << free_spaces.size() << endl;
    if (!free_spaces.empty())
    {
        //cout << "!free_spaces.empty()" << endl;
        // free space contour
        for (int sid = 0; sid < free_spaces.size(); sid++)
        {
            vector<cv::Point> space_contour;
            double nodesize = map_cellsize;
            int scale = nodesize / map_cellsize;
            // right end point 
            {
                // r [0] c [free_spaces[sid].second];
                ushort d = scan_max_range;
                // compute coordinate, octomap camera coord system
                double z = double(d) / camera_factor;
                double x = (free_spaces[sid].second - camera_cx) * z / camera_fx;
                double y = (0 - camera_cy) * z / camera_fy;
                Eigen::Vector3d p;
                p[0] = z, p[1] = -x, p[2] = -y; // convert to gazebo camera coord system
                p = r*p + t; // convert to gazebo world coord system
                -p[1], -p[2], p[0]; // convert to octomap world coord system
                int v = (int)((-p[1] - xmin_oc_g) / nodesize)*scale;
                int u = (int)((zmax_oc_g - p[0]) / nodesize)*scale;
                if (u < 0)
                    u = 0;
                if (v < 0)
                    v = 0;
                // u v
                space_contour.push_back(cv::Point(v, u));
            }
            // left end point
            {  
                // r [0] c [free_spaces[sid].first];
                ushort d = scan_max_range;
                double z = double(d) / camera_factor;
                double x = (free_spaces[sid].first - camera_cx) * z / camera_fx;
                double y = (0 - camera_cy) * z / camera_fy;
                Eigen::Vector3d p;
                p[0] = z, p[1] = -x, p[2] = -y; 
                p = r*p + t; 
                int v = (int)((-p[1] - xmin_oc_g) / nodesize)*scale;
                int u = (int)((zmax_oc_g - p[0]) / nodesize)*scale;
                if (u < 0)
                    u = 0;
                if (v < 0)
                    v = 0;
                space_contour.push_back(cv::Point(v, u));
            }
            // camera point 
            { 
                double z = 0;
                double x = 0;
                double y = 0;
                Eigen::Vector3d p;
                p[0] = z, p[1] = -x, p[2] = -y;
                p = r*p + t; 
                int v = (int)((-p[1] - xmin_oc_g) / nodesize)*scale;
                int u = (int)((zmax_oc_g - p[0]) / nodesize)*scale;
                if (u < 0)
                    u = 0;
                if (v < 0)
                    v = 0;
                space_contour.push_back(cv::Point(v, u));
            }
            m_free_space_contours2d[rid] = space_contour; // save
        }
/*
        // draw contours
        cerr<<"show contours 2d"<<endl;
        cv::Mat pc_resultImage = cv::Mat::zeros(map_rows, map_cols, CV_8U);
        cv::drawContours(pc_resultImage, m_free_space_contours2d, -1, cv::Scalar(255, 0, 255));
        cv::imshow("space_contours", pc_resultImage);
        cv::waitKey(0);
//*/
    }
    return;
}
//*/

// compute ideal frustum
vector<cv::Point> DataEngine::loadIdealFrustum(Eigen::MatrixXd r, Eigen::Vector3d t)
{
    vector<cv::Point> result;
    // right end point
    {
        // r [0] c [free_spaces[sid].second];
        ushort d = scan_max_range;
        double z = double(d) / camera_factor;
        double x = (map_cols - 1 - camera_cx) * z / camera_fx;
        double y = (0 - camera_cy) * z / camera_fy;
        Eigen::Vector3d p_cam;
        p_cam[0] = x; p_cam[1] = y; p_cam[2] = z;
        Eigen::Vector3d p_gz;
        p_gz[0] = p_cam.z(), p_gz[1] = -p_cam.x(), p_gz[2] = -p_cam.y(); // 转到gazebo的相机坐标系
        p_gz = r*p_gz + t; // 转到gazebo的世界坐标系
        Eigen::Vector3d p_oc;
        p_oc[0] = -p_gz[1]; p_oc[1] = -p_gz[2]; p_oc[2] = p_gz[0]; // 转到octomap的世界坐标系，在octomap中的坐标
        double node_size = map_cellsize;
        int scale = node_size / map_cellsize;
        Eigen::Vector2i rc = coord_trans_oc2cell(p_oc, node_size, scale);
        int v = rc[1];//列
        int u = rc[0];//行
        if (u < 0)
            u = 0;
        if (v < 0)
            v = 0;
        // u行v列
        result.push_back(cv::Point(v, u));
    }
    // left end point
    {
        // r [0] c [free_spaces[sid].first];
        ushort d = scan_max_range;
        double z = double(d) / camera_factor;
        double x = (0 - camera_cx) * z / camera_fx;
        double y = (0 - camera_cy) * z / camera_fy;
        Eigen::Vector3d p;
        p[0] = z, p[1] = -x, p[2] = -y; // 转到gazebo的相机坐标系
        p = r*p + t; // 转到gazebo的世界坐标系
        -p[1], -p[2], p[0]; // 转到octomap的世界坐标系
        double nodesize = map_cellsize;
        int scale = nodesize / map_cellsize;
        int v = (int)((-p[1] - xmin_oc_g) / nodesize)*scale;
        int u = (int)((zmax_oc_g - p[0]) / nodesize)*scale;
        if (u < 0)
            u = 0;
        if (v < 0)
            v = 0;
        // u行v列
        result.push_back(cv::Point(v, u));
    }
    // camera end point
    {
        double z = 0;
        double x = 0;
        double y = 0;
        Eigen::Vector3d p;
        p[0] = z, p[1] = -x, p[2] = -y; // 转到gazebo的相机坐标系
        p = r*p + t; // 转到gazebo的世界坐标系
        -p[1], -p[2], p[0]; // 转到octomap的世界坐标系
        double nodesize = map_cellsize;
        int scale = nodesize / map_cellsize;
        int v = (int)((-p[1] - xmin_oc_g) / nodesize)*scale;
        int u = (int)((zmax_oc_g - p[0]) / nodesize)*scale;
        if (u < 0)
            u = 0;
        if (v < 0)
            v = 0;
        // u行v列
        result.push_back(cv::Point(v, u));
    }
    return result;
}

// fill trivial holes in known region
void DataEngine::fillTrivialHolesKnownRegion()
{
    cv::Mat cellmap_mat = cv::Mat::zeros(map_rows, map_cols, CV_8UC3);
    for (int r = 0; r < cellmap_mat.rows; r++)
    {
        for (int c = 0; c < cellmap_mat.cols; c++)
        {
            if (m_recon2D.m_cellmap[r][c].isScanned){
                if (m_recon2D.m_cellmap[r][c].isFree)
                    cellmap_mat.ptr<cv::Vec3b>(r)[c][1] = 255;
                else
                    cellmap_mat.ptr<cv::Vec3b>(r)[c][2] = 255;
            }
        }
    }
    // resize to fill hole
    cv::Mat trans_mat(map_rows * 2, map_cols * 2, CV_8UC3);
    cv::resize(cellmap_mat, trans_mat, trans_mat.size(), 0, 0);
    // color correct
    for (int r = 0; r < trans_mat.rows; r++)
    {
        for (int c = 0; c < trans_mat.cols; c++)
        {
            if (trans_mat.ptr<cv::Vec3b>(r)[c][1] > trans_mat.ptr<cv::Vec3b>(r)[c][2]){ // free
                trans_mat.ptr<cv::Vec3b>(r)[c][2] = 0;
                trans_mat.ptr<cv::Vec3b>(r)[c][1] = 255;
                trans_mat.ptr<cv::Vec3b>(r)[c][0] = 0;
            }
            else if (trans_mat.ptr<cv::Vec3b>(r)[c][1] < trans_mat.ptr<cv::Vec3b>(r)[c][2]){ // occupied
                trans_mat.ptr<cv::Vec3b>(r)[c][2] = 255;
                trans_mat.ptr<cv::Vec3b>(r)[c][1] = 0;
                trans_mat.ptr<cv::Vec3b>(r)[c][0] = 0;
            }
        }
    }
    cv::resize(trans_mat, cellmap_mat, cellmap_mat.size(), 0, 0);
    // color correct
    for (int r = 0; r < cellmap_mat.rows; r++)
    {
        for (int c = 0; c < cellmap_mat.cols; c++)
        {
            if (cellmap_mat.ptr<cv::Vec3b>(r)[c][1] > cellmap_mat.ptr<cv::Vec3b>(r)[c][2]){ // free
                cellmap_mat.ptr<cv::Vec3b>(r)[c][2] = 0;
                cellmap_mat.ptr<cv::Vec3b>(r)[c][1] = 255;
                cellmap_mat.ptr<cv::Vec3b>(r)[c][0] = 0;
            }
            else if (cellmap_mat.ptr<cv::Vec3b>(r)[c][1] < cellmap_mat.ptr<cv::Vec3b>(r)[c][2]){ // occupied
                cellmap_mat.ptr<cv::Vec3b>(r)[c][2] = 255;
                cellmap_mat.ptr<cv::Vec3b>(r)[c][1] = 0;
                cellmap_mat.ptr<cv::Vec3b>(r)[c][0] = 0;
            }
        }
    }
    // fill holes inside frustum
    for (int rid = 0; rid < rbt_num; rid++)
    {
        if (m_frustum_contours[rid].size() == 0)
        {
            cerr << "empty frustum." << endl;
            getchar(); getchar(); getchar();
            continue;
        }
        // find bbx
        int minX = 999999;
        int minY = 999999;
        int maxX = 0;
        int maxY = 0;
        for (int pid = 0; pid < m_frustum_contours[rid].size(); pid++)
        {
            if (m_frustum_contours[rid][pid].x > maxX)
                maxX = m_frustum_contours[rid][pid].x;
            if (m_frustum_contours[rid][pid].y > maxY)
                maxY = m_frustum_contours[rid][pid].y;
            if (m_frustum_contours[rid][pid].x < minX)
                minX = m_frustum_contours[rid][pid].x;
            if (m_frustum_contours[rid][pid].y < minY)
                minY = m_frustum_contours[rid][pid].y;
        }
        // extract iner cells
        cv::Mat inerCellMap = cv::Mat::zeros(map_rows, map_cols, CV_8UC1);
        for (int r = minY; r <= maxY; r++)
        {
            for (int c = minX; c <= maxX; c++)
            {
                if (m_recon2D.m_cellmap[r][c].isScanned) // scanned cell
                {
                    if (cv::pointPolygonTest(m_frustum_contours[rid], cv::Point(c, r), false) >= 0) // inside the robot view
                    {
                        inerCellMap.ptr<uchar>(r)[c] = 255;
                    }
                }
            }
        }
        // contours
        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(inerCellMap, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);// tree, simple
        for (int cid = 0; cid < contours.size(); cid++)
        {
            if (contours[cid].size() <= 4) continue;
            for (int r = minY; r <= maxY; r++)
            {
                for (int c = minX; c <= maxX; c++)
                {
                    if (!m_recon2D.m_cellmap[r][c].isScanned) // maybe a hole.
                    {
                        if (cv::pointPolygonTest(contours[cid], cv::Point(c, r), false) >= 0) // inside the known region
                        {
                            if (cellmap_mat.ptr<cv::Vec3b>(r)[c][1] == 255) // free 
                            {
                                m_recon2D.m_cellmap[r][c].isScanned = true;
                                m_recon2D.m_cellmap[r][c].isFree = true;
                                m_recon2D.m_cellmap[r][c].isOccupied = false;
                            }
                            else if (cellmap_mat.ptr<cv::Vec3b>(r)[c][2] == 255) // occupied 
                            {
                                m_recon2D.m_cellmap[r][c].isScanned = true;
                                m_recon2D.m_cellmap[r][c].isFree = false;
                                m_recon2D.m_cellmap[r][c].isOccupied = true;
                            }
                        } // inside the known region
                    } // maybe a hole
                } // c
            } // r
        } // known region contour
    } // frustum iter
//*/
    return;
}

// fuse scans by multi-robot, not finished
void DataEngine::fuseScans2MapAndTree()
{
    cerr<<"fuseScans2MapAndTree..."<<endl;
    for (int rid = 0; rid < rbt_num; ++rid)
    {
        // update octree
        cerr<<"insertAFrame2Tree..."<<endl;
        insertAFrame2Tree(m_depth[rid], m_pose[rid]);
        // find extra free space that octree cant record
        findExtraFreeSpace(rid, m_depth[rid], m_pose[rid]);
        
        // todo: uncertainty

        // todo: surface normal

        // update robot poses
        m_pose2d[rid] = coord_trans_7f_se2(m_pose[rid]);
        // update robot viewports
        pair<Eigen::MatrixXd, Eigen::Vector3d> rt = coord_trans_7f_rt(m_pose[rid]);
        m_frustum_contours[rid] = loadIdealFrustum(rt.first, rt.second);
    }
    
    // todo: keyframe count

    return;
}

// insert a frame to octree
void DataEngine::insertAFrame2Tree(cv::Mat & depth, vector<float> pose)
{
    double tb = clock(); // timing
    // check nan
    for (int i = 0; i < pose.size(); ++i)
    {
        if (__isnan(pose[i]))
        {
            cerr<<"pose nan:"<<endl;
            cerr<<pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" "<<pose[3]<<" "<<pose[4]<<" "<<pose[5]<<" "<<pose[6]<<" "<<endl;
            getchar();getchar();getchar();
            exit(-1);
        }
    }
    if (pose[3]==pose[4] && pose[4]==pose[5] && pose[5]==pose[6])
    {
        cerr<<"invalid pose"<<endl;
        cerr<<pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" "<<pose[3]<<" "<<pose[4]<<" "<<pose[5]<<" "<<pose[6]<<" "<<endl;
        getchar();getchar();getchar();
        exit(-1);
    }

    // set up
    octomap::Pointcloud* pc = new octomap::Pointcloud();
    for (int r = 0; r < depth.rows; r++)
    {
        for (int c = 0; c < depth.cols; c++)
        {
            if (depth.ptr<ushort>(r)[c] > scan_max_range) // cut off
                depth.ptr<ushort>(r)[c] = 0;
            // depth error? infinite was set to 1000? 
            if (depth.ptr<ushort>(r)[c] == 1000) // 1000 mm
                depth.ptr<ushort>(r)[c] = 0;
            // depth error? infinite was set to 1000? 
/*
            if (method_num == 30) // add noise
            {
                if (depth.ptr<ushort>(r)[c] > 100 && depth.ptr<ushort>(r)[c] < 200)
                {
                    //cerr << "origin " << depth.ptr<ushort>(r)[c] << endl;
                    short noise = (rand() % 100 - 50);
                    //cerr << "noise " << noise << endl;
                    depth.ptr<ushort>(r)[c] = (ushort)depth.ptr<ushort>(r)[c] + noise;
                    //cerr << "noised " << depth.ptr<ushort>(r)[c] << endl;
                }
                else if (depth.ptr<ushort>(r)[c] > 2000)
                {
                    short noise = (rand() % 200 - 100); // +-100mm
                    //cerr << "noise " << noise << endl;
                    depth.ptr<ushort>(r)[c] = (ushort)depth.ptr<ushort>(r)[c] + noise;
                }
            }
//*/
        }
    }
    // convert pose to rt
    Eigen::Quaternion<double> q((double)pose[6], (double)pose[3], (double)pose[4], (double)pose[5]);
    Eigen::MatrixXd r = q.toRotationMatrix();
    Eigen::Vector3d t(pose[0], pose[1], pose[2]);
/*
    // check nan
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            if(__isnan(r(i, j)))
            {
                cerr<<"pose nan"<<endl;
                cerr<<"r"<<endl<<r<<endl;
                cerr<<pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" "<<pose[3]<<" "<<pose[4]<<" "<<pose[5]<<" "<<pose[6]<<" "<<endl;
                getchar();getchar();getchar();
                exit(-1);
            }
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        if (__isnan(t[i]))
        {
            cerr<<"pose nan"<<endl;
            cerr<<"t"<<endl<<t.transpose()<<endl;
            cerr<<pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" "<<pose[3]<<" "<<pose[4]<<" "<<pose[5]<<" "<<pose[6]<<" "<<endl;
            getchar();getchar();getchar();
            exit(-1);
        }
    }
//*/
    // convert depth img to point cloud in octomap global coordinate 
    for (int m = 0; m < depth.rows; m++){
        for (int n = 0; n < depth.cols; n++){
            // (m,n) d
            ushort d = depth.ptr<ushort>(m)[n];
            if (d == 0)
                continue;
            if (d > scan_max_range) //test: remove some far pixels max value 4913
                continue;
            // 3d position in octomap local(camera) coordinate
            double z = double(d) / camera_factor;
            double x = (n - camera_cx) * z / camera_fx;
            double y = (m - camera_cy) * z / camera_fy;
            //cout << "camera local coord " << x << ", " << y << ", " << z << endl;
            Eigen::Vector3d p;
            p[0] = z, p[1] = -x, p[2] = -y; // gazebo local(camera) coordinate
            p = r*p + t; // gazebo world coordinate
            if (__isnan(p[0])||__isnan(p[1])||__isnan(p[2]))
            {
                cerr<<"point nan in pc"<<endl;
                getchar();getchar();getchar();
                continue;
            }
//*/
            pc->push_back(-p[1], -p[2], p[0]); // convert to octomap world coordinate, then insert to pc
        }
    }
    Eigen::Vector3d origin(0, 0, 0);
    origin = r*origin + t; //-origin[1], -origin[2], origin[0]; // octomap world coordinate
/*
    if (__isnan(origin[0])||__isnan(origin[1])||__isnan(origin[2]))
    {
        cerr<<"pose nan"<<endl;
        getchar();getchar();getchar();
        exit(-1);
    }
    else
//*/
    {
        // inset into tree
        //m_recon3D.m_tree->insertPointCloudRays(pc, octomap::point3d(-origin[1], -origin[2], origin[0])); // openmp inside their code.
        m_recon3D.m_tree->insertPointCloud(pc, octomap::point3d(-origin[1], -origin[2], origin[0])); // cause error. cause error? it works well 2018-12-26.
        // update tree occupancy
        m_recon3D.m_tree->updateInnerOccupancy();
    }
    delete pc;
    double te = clock(); // timing
    cerr << "inserted a frame to octree, timing " << (te - tb)/CLOCKS_PER_SEC << " s" << endl;
    return;
}

// vis
cv::Mat DataEngine::visCellMap()
{
    cv::Mat vis_mat = cv::Mat::zeros(map_rows, map_cols, CV_8UC3);
    for (int r = 0; r < vis_mat.rows; r++)
    {
        for (int c = 0; c < vis_mat.cols; c++)
        {
            if (m_recon2D.m_cellmap[r][c].isScanned)
            {
                if (m_recon2D.m_cellmap[r][c].isFree)
                    vis_mat.ptr<cv::Vec3b>(r)[c][1] = 255;
                else
                    vis_mat.ptr<cv::Vec3b>(r)[c][2] = 255;
            }
        }
    }
    return vis_mat;
}

// show
cv::Mat DataEngine::showCellMap()
{
    cv::Mat vis_mat = visCellMap();
    cv::imshow("2d", vis_mat);
    cv::waitKey(0);
    return vis_mat;
}

// show
cv::Mat DataEngine::showStatement()
{
    // recon
    cv::Mat vis_mat = visCellMap();
    // robot
    for (int rid = 0; rid < rbt_num; ++rid)
    {
        cv::circle(vis_mat, cv::Point(round(m_pose2d[rid].translation().x()), round(-m_pose2d[rid].translation().y())), 1, CV_RGB(255, 255, 255));
        cv::circle(vis_mat, cv::Point(round(m_pose2d[rid].translation().x()), round(-m_pose2d[rid].translation().y())), 3, CV_RGB(0, 0, 255));
    }
    cv::imshow("statement2d", vis_mat);
    cv::waitKey(0);
    return vis_mat;
}