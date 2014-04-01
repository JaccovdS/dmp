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
    double K = 100.0;
    double D = 20.0;
    double intersection_height = 0.5;
    int num_bases = 15;
    double traj[] = {0,0.031411,0.062791,0.094108,0.12533,0.15643,0.18738,0.21814,0.24869,0.27899,0.30902,0.33874,0.36812,0.39715,0.42578,0.45399,0.48175,0.50904,0.53583,0.56208,0.58779,0.61291,0.63742,0.66131,0.68455,0.70711,0.72897,0.75011,0.77051,0.79016,0.80902,0.82708,0.84433,0.86074,0.87631,0.89101,0.90483,0.91775,0.92978,0.94088,0.95106,0.96029,0.96858,0.97592,0.98229,0.98769,0.99211,0.99556,0.99803,0.99951,1,0.99951,0.99803,0.99556,0.99211,0.98769,0.98229,0.97592,0.96858,0.96029,0.95106,0.94088,0.92978,0.91775,0.90483,0.89101,0.87631,0.86074,0.84433,0.82708,0.80902,0.79016,0.77051,0.75011,0.72897,0.70711,0.68455,0.66131,0.63742,0.61291,0.58779,0.56208,0.53583,0.50904,0.48175,0.45399,0.42578,0.39715,0.36812,0.33874,0.30902,0.27899,0.24869,0.21814,0.18738,0.15643,0.12533,0.094108,0.062791,0.031411,0
};


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
    lfdsrv.request.intersection_height = intersection_height;

    //Call service for LfD
    if (lfdclient.call(lfdsrv)) {
        ROS_INFO("Result LfD: %d", (bool)lfdsrv.response.tau);
    }
    else {
        ROS_ERROR("Failed to call service learn_dmp_from_demo");
    }

    //Get client for setting active DMP
    ros::ServiceClient ADMPclient = n.serviceClient<dmp::SetActiveDMP>("set_active_dmp");

    //Perturb weights with some random numbers
    //Print weights for testing purposes
    dmp::DMPData data =  lfdsrv.response.dmp_list.at(0);
    ROS_INFO("The weights are:");
    for(int i = 0; i<data.weights.size(); i++){
        //randomly add noise
        if(i == 1){
            //random number in [-1,1]
            //data.weights.at(i) += (0.5-(float)(rand()%2))/2.0;
        }
        ROS_INFO("%f ", data.weights.at(i));
    }

    //Put perturbed weights back in the dmp list
    lfdsrv.response.dmp_list.at(0) = data;

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
    double goal = 1.0;
    double goal_tresh = 0.05;
    // The length of the requested plan segment in seconds. Set to -1 if plan until goal is desired.
    double seg_length = lfdsrv.response.tau;
    // A time constant to set the length of DMP replay in seconds until 95% phase convergence
    double tau = seg_length;
    double integrate_iter = 1;

    //Set value for getting a DMP plan
    DMPPlansrv.request.x_0.push_back(x_0);
    DMPPlansrv.request.x_dot_0.push_back(x_dot_0);
    DMPPlansrv.request.t_0 = t_0;
    DMPPlansrv.request.goal.push_back(goal);
    DMPPlansrv.request.goal_tresh.push_back(goal_tresh);
    DMPPlansrv.request.seg_length = seg_length;
    DMPPlansrv.request.intersection_height = intersection_height;

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
    int last = DMPPlansrv.response.plan.points.size()-1;
    for(int i=0; i<DMPPlansrv.response.plan.points.size(); i++) {
               ROS_INFO(", %f, %f, %f",DMPPlansrv.response.plan.times.at(i),DMPPlansrv.response.plan.points.at(i).positions.at(0),DMPPlansrv.response.plan.points.at(i).velocities.at(0));
    }
//    ROS_INFO(", %f, %f, %f",DMPPlansrv.response.plan.times.at(last),DMPPlansrv.response.plan.points.at(last).positions.at(0),DMPPlansrv.response.plan.points.at(last).velocities.at(0));
    return 0;
}
