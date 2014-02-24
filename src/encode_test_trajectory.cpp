#include "dmp/encode_test_trajectory.h"

using namespace std;

namespace encode_test_trajectory {

/**
 * @brief This converts a normal trajectory of points into a DMPTraj
 * @param trajectory trajectory of points
 * @param dt time difference
 * @return
 */
dmp::DMPTraj createTrajectoryFromPoints(vector<double>& trajectory, double dt){
    //ROS_INFO("Entering createTrajectoryFromPoints");


    dmp::DMPTraj DMPtrajectory = dmp::DMPTraj();

    for(int index=0; index<trajectory.size(); index++){
        dmp::DMPPoint point = dmp::DMPPoint();
        point.positions.push_back(trajectory.at(index));
        DMPtrajectory.points.push_back(point);
        DMPtrajectory.times.push_back(dt*index);
    }

    //ROS_INFO("Leaving createTrajectoryFromPoints");
    return DMPtrajectory;
}

}


int main(int argc, char** argv)
{


    //Start-up
    ros::init(argc, argv, "encode_test_trajectory");
    ros::NodeHandle n;

    //Initialize values for test
    double dt = 0.01;
    double K = 150.0;
    double D = 25.0;
    int num_bases = 15;
    double traj[] = {0,0.0099998,0.019999,0.029996,0.039989,0.049979,0.059964,0.069943,0.079915,0.089879,0.099833,0.10978,0.11971,0.12963,0.13954,0.14944,0.15932,0.16918,0.17903,0.18886,0.19867,0.20846,0.21823,0.22798,0.2377,0.2474,0.25708,0.26673,0.27636,0.28595,0.29552,0.30506,0.31457,0.32404,0.33349,0.3429,0.35227,0.36162,0.37092,0.38019,0.38942,0.39861,0.40776,0.41687,0.42594,0.43497,0.44395,0.45289,0.46178,0.47063,0.47943,0.48818,0.49688,0.50553,0.51414,0.52269,0.53119,0.53963,0.54802,0.55636,0.56464,0.57287,0.58104,0.58914,0.5972,0.60519,0.61312,0.62099,0.62879,0.63654,0.64422,0.65183,0.65938,0.66687,0.67429,0.68164,0.68892,0.69614,0.70328,0.71035,0.71736,0.72429,0.73115,0.73793,0.74464,0.75128,0.75784,0.76433,0.77074,0.77707,0.78333,0.7895,0.7956,0.80162,0.80756,0.81342,0.81919,0.82489,0.8305,0.83603,0.84147,0.84683,0.85211,0.8573,0.8624,0.86742,0.87236,0.8772,0.88196,0.88663,0.89121,0.8957,0.9001,0.90441,0.90863,0.91276,0.9168,0.92075,0.92461,0.92837,0.93204,0.93562,0.9391,0.94249,0.94578,0.94898,0.95209,0.9551,0.95802,0.96084,0.96356,0.96618,0.96872,0.97115,0.97348,0.97572,0.97786,0.97991,0.98185,0.9837,0.98545,0.9871,0.98865,0.9901,0.99146,0.99271,0.99387,0.99492,0.99588,0.99674,0.99749,0.99815,0.99871,0.99917,0.99953,0.99978,0.99994,1,0.99996,0.99982,0.99957,0.99923,0.99879,0.99825,0.99761,0.99687,0.99602,0.99508,0.99404,0.9929,0.99166,0.99033,0.98889,0.98735,0.98572,0.98399,0.98215,0.98022,0.9782,0.97607,0.97385,0.97153,0.96911,0.96659,0.96398,0.96128,0.95847,0.95557,0.95258,0.94949,0.9463,0.94302,0.93965,0.93618,0.93262,0.92896,0.92521,0.92137,0.91744,0.91341,0.9093,0.90509,0.90079,0.89641,0.89193,0.88736,0.88271,0.87796,0.87313,0.86821,0.86321,0.85812,0.85294,0.84768,0.84233,0.8369,0.83138,0.82578,0.8201,0.81434,0.8085,0.80257,0.79657,0.79048,0.78432,0.77807,0.77175,0.76535,0.75888,0.75233,0.74571,0.73901,0.73223,0.72538,0.71846,0.71147,0.70441,0.69728,0.69007,0.6828,0.67546,0.66806,0.66058,0.65304,0.64543,0.63776,0.63003,0.62223,0.61437,0.60645,0.59847,0.59043,0.58233,0.57417,0.56596,0.55768,0.54936,0.54097,0.53253,0.52404,0.5155,0.50691,0.49826,0.48957,0.48082,0.47203,0.46319,0.45431,0.44537,0.4364,0.42738,0.41832,0.40921,0.40007,0.39088,0.38166,0.3724,0.3631,0.35376,0.34439,0.33499,0.32555,0.31608,0.30657,0.29704,0.28748,0.27789,0.26827,0.25862,0.24895,0.23925,0.22953,0.21978,0.21002,0.20023,0.19042,0.1806,0.17075,0.16089,0.15101,0.14112,0.13121,0.12129,0.11136,0.10142,0.091465,0.081502,0.071532,0.061554,0.05157,0.041581,0.031587,0.021591,0.011592,0.0015927};

    //Convert to vector
    vector<double> trajectory(traj, traj + sizeof(traj)/sizeof(double));


    //Convert to DMP trajectory
    dmp::DMPTraj DMPtrajectory = encode_test_trajectory::createTrajectoryFromPoints(trajectory, dt);

    //Get client for LfD
    ros::ServiceClient lfdclient = n.serviceClient<dmp::LearnDMPFromDemo>("learn_dmp_from_demo");

    //Prepare request to lfd service
    dmp::LearnDMPFromDemo lfdsrv;
    lfdsrv.request.demo = DMPtrajectory;
    lfdsrv.request.k_gains.push_back(K);
    lfdsrv.request.d_gains.push_back(D);
    lfdsrv.request.num_bases = num_bases;

    //Call service for LfD
    if (lfdclient.call(lfdsrv)) {
        ROS_INFO("Result LfD: %d", (bool)lfdsrv.response.tau);
    }
    else {
        ROS_ERROR("Failed to call service learn_dmp_from_demo");
    }

    //Get client for setting active DMP
    ros::ServiceClient ADMPclient = n.serviceClient<dmp::SetActiveDMP>("set_active_dmp");

    //Prepare request to set active DMP
    dmp::SetActiveDMP ADMPsrv;
    ADMPsrv.request.dmp_list = lfdsrv.response.dmp_list;

    //Call service for setting active DMP
    if (ADMPclient.call(ADMPsrv)) {
        ROS_INFO("Setting active succeeded: %d", (bool)ADMPsrv.response.success);
    }
    else {
        ROS_ERROR("Failed to call service set_active_dmp");
    }

    //Get client for making a new plan
    ros::ServiceClient newplanclient = n.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");

    //Prepare request for getting a DMP plan
    dmp::GetDMPPlan DMPPlansrv;
    double x_0 = 0.0;
    double x_dot_0 = 0.0;
    double t_0 = 0.0;
    double goal = 0.0;
    double goal_tresh = 0.02;
    double seg_length = lfdsrv.response.tau;
    double tau = lfdsrv.response.tau;
    double integrate_iter = 1;

    //Set value for getting a DMP plan
    DMPPlansrv.request.x_0.push_back(x_0);
    DMPPlansrv.request.x_dot_0.push_back(x_dot_0);
    DMPPlansrv.request.t_0 = t_0;
    DMPPlansrv.request.goal.push_back(goal);
    DMPPlansrv.request.goal_tresh.push_back(goal_tresh);
    DMPPlansrv.request.seg_length = seg_length;
    DMPPlansrv.request.tau = tau;
    DMPPlansrv.request.dt = dt;
    DMPPlansrv.request.integrate_iter = integrate_iter;

    //Call service for getting a planned trajectory
    if (newplanclient.call(DMPPlansrv)) {
        ROS_INFO("At goal: %d", (bool)DMPPlansrv.response.at_goal);
    }
    else {
        ROS_ERROR("Failed to call service get_dmp_plan");
    }

    //Print plan
    ROS_INFO("The resulting plan of size %ld is (time, position, velocity:", DMPPlansrv.response.plan.points.size());
    for(int i=0; i<DMPPlansrv.response.plan.points.size(); i++) {
        ROS_INFO(", %f, %f, %f",DMPPlansrv.response.plan.times.at(i),DMPPlansrv.response.plan.points.at(i).positions.at(0),DMPPlansrv.response.plan.points.at(i).velocities.at(0));
    }

    return 0;
}
