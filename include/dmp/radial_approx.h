/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Scott Niekum
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/**
  * \author Scott Niekum
  */

#ifndef RADIAL_APPROX_H_
#define RADIAL_APPROX_H_

#include "dmp/function_approx.h"
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <vector>

namespace dmp{

/// Class for linear function approximation with the univariate Radial basis
class RadialApprox : public FunctionApprox{
public:
    RadialApprox(int num_bases, double min_value, double max_value, double intersection_height);
    RadialApprox(const std::vector<double> &w, double min_value, double max_value, double intersection_height);
    virtual ~RadialApprox();

    /**\brief Evaluate the function approximator at point x
     * \param x The point at which to evaluate
     * \return The scalar value of the function at x
     */
    virtual double evalAt(double x);

    /**\brief Computes the least squares weights given a set of data points
     * \param X A vector of the domain values of the points
     * \param Y A vector of the target values of the points
     */
    virtual void leastSquaresWeights(double *X, double *Y, int n_pts);

private:
    /**\brief Calculate the Radial basis features at point x
     * \param x The point at which to get features
     */
    void calcFeatures(double x);

    /**\brief Calculate the Moore-Penrose pseudoinverse of a matrix using SVD
     * \param mat The matrix to pseudoinvert
     * \return The pseudoinverted matrix
     */
    Eigen::MatrixXd pseudoinverse(Eigen::MatrixXd mat);

    /**
     * @brief Calculate the centers and the widths of the gaussian functions
     * @param min_value minimum value of the function that is approximated
     * @param max_value maximum value of the function that is approximated
     * @param intersection_height the height at which the basis functions intersect
     * @param centers the calculated centers
     * @param widths the calculated widths
     * This function is derived from the LWR approximator of https://github.com/stulp/dmpbbo/
     * in order to make the determination of the widths and centers more convenient
     */
    void calcCentersAndWidths(double min_value, double max_value, double intersection_height, double *centers, double *widths);

    double *features;  //Storage for a set of features
    double *centers;   //Centers of RBFs
    double *widths;    //Widths of RBFs
    double min_value;
    double max_value;
    double intersection_height;

};

}

#endif /* RADIAL_APPROX_H_ */
