#ifndef ENCODE_TEST_TRAJECTORY_H
#define ENCODE_TEST_TRAJECTORY_H

#include "ros/ros.h"
#include "dmp/dmp.h"
#include <initializer_list>

using namespace std;

namespace encode_test_trajectory {

class encode_test_trajectory
{
public:

private:
    dmp::DMPTraj createTrajectoryFromPoints(vector<double>& trajectory, double dt);
};

}



#endif // ENCODE_TEST_TRAJECTORY_H
