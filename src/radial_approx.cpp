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
  * edited by Jacco van der Spek
  */


#include "dmp/radial_approx.h"
#include<stdio.h>
using namespace Eigen;
using namespace std;

namespace dmp{

RadialApprox::RadialApprox(int num_bases, double min_value, double max_value, double intersection_height)
{
    n_bases = num_bases;
    min_value = min_value;
    max_value = max_value;
    intersection_height = intersection_height;
    features = new double[n_bases];
    centers = new double[n_bases];
    widths = new double[n_bases];
    weights.resize(n_bases);
    calcCentersAndWidths(min_value, max_value, intersection_height, centers, widths);
    for(int i=0; i<n_bases; i++){
        features[i] = 0;
    }

}


//RadialApprox::RadialApprox(const vector<double> &w, double base_width, double alpha)
//{
//    weights = w;
//    n_bases = w.size();
//    features = new double[n_bases];
//        centers = new double[n_bases];
//        widths = new double[n_bases];
//        for(int i=0; i<n_bases; i++){
//            features[i] = 0;
//            centers[i] = exp((-alpha*i)/n_bases);  //((double)i)/((double)n_bases);  //exp((-alpha*i)/n_bases);
//            widths[i] =base_width * exp((-alpha*i)/n_bases);// base_width; //base_width * exp((-alpha*i)/n_bases);
//        }
//}

RadialApprox::RadialApprox(const vector<double> &w, double min_value, double max_value, double intersection_height){
    weights = w;
    n_bases = w.size();
    features = new double[n_bases];
    centers =  new double[n_bases];
    widths =  new double[n_bases];
    calcCentersAndWidths(min_value,max_value,intersection_height,centers,widths);
    for(int i=0; i<n_bases; i++){
        features[i] = 0;
    }
}


RadialApprox::~RadialApprox()
{
    delete[] features;
    delete[] centers;
    delete[] widths;
}


double RadialApprox::evalAt(double x)
{
    calcFeatures(x);

    double wsum = 0;
    for(int i=0; i<n_bases; i++){
        wsum += features[i] * weights[i];
    }
    //cout << wsum << endl;
    return wsum;
}

/*
 *Note that this function only works if the input parameters are vectors.
 */
void RadialApprox::leastSquaresWeights(double *X, double *Y, int n_pts)
{
    MatrixXd D_mat = MatrixXd(n_pts,n_bases);
    MatrixXd inputs = MatrixXd(n_pts,1);
    MatrixXd targets = MatrixXd(n_pts,1);
    MatrixXd Gamma;

    //Calculate the design matrix
    for(int i=0; i<n_pts; i++){
        inputs(i,0) = X[i];
        targets(i,0) = Y[i];
        calcFeatures(X[i]);
        for(int j=0; j<n_bases; j++){
            D_mat(i,j) = features[j];
        }
    }

    //Calculate the least squares
    for(int basis=0; basis<n_bases; basis++){
        VectorXd gamma = D_mat.col(basis);
        Gamma = gamma.asDiagonal();
        VectorXd beta_num = inputs.transpose()*Gamma*targets;
        //cout << beta_num(0) << endl;
        VectorXd beta_den = inputs.transpose()*Gamma*inputs;
        weights[basis] = beta_num(0)/beta_den(0);
    }

}


void RadialApprox::calcFeatures(double x)
{
    double sum = 0;
    for(int i=0; i<n_bases; i++){
        features[i] = exp((-0.5*(x-centers[i])*(x-centers[i]))/(widths[i]*widths[i]));
        sum += features[i];
    }
    for(int i=0; i<n_bases; i++){
        features[i] /= sum;
    }
}


MatrixXd RadialApprox::pseudoinverse(MatrixXd mat){
    //Numpy uses 1e-15 by default.  I use 1e-10 just to be safe.
    double precisionCutoff = 1e-10;

    //Compute the SVD of the matrix
    JacobiSVD<MatrixXd> svd(mat, ComputeThinU | ComputeThinV);
    MatrixXd U = svd.matrixU();
    MatrixXd V = svd.matrixV();
    MatrixXd S = svd.singularValues();

    //Psuedoinvert the diagonal matrix of singular values
    MatrixXd S_plus = MatrixXd::Zero(n_bases, n_bases);
    for(int i=0; i<n_bases; i++){
        if(S(i) > precisionCutoff){  //Cutoff to avoid huge inverted values for numerical stability
            S_plus(i,i) = 1.0/S(i);
        }
    }

    //Compute psuedoinverse of orginal matrix
    return V * S_plus * U.transpose();
}

void RadialApprox::calcCentersAndWidths(double min_value, double max_value, double intersection_height, double *centers, double *widths){

    //Compute centers based on min and max
    VectorXd local_centers = VectorXd::LinSpaced(n_bases,min_value,max_value);
    VectorXd local_widths(n_bases);

    //Case there is just one basis function; it has to be normal width
    if(n_bases == 1){
        local_widths[0] = 1.0;
    }
    //Compute width for each basis function based on intersection height
    else{
        /*This comment is litteraly from
         *https://github.com/stulp/dmpbbo/blob/master/src/functionapproximators/MetaParametersLWR.cpp
         *It explains how the widths are determined
         *
         *Consider two neighbouring basis functions, exp(-0.5(x-c0)^2/w^2) and exp(-0.5(x-c1)^2/w^2)
         *Assuming the widths are the same for both, they are certain to intersect at x = 0.5(c0+c1)
         *And we want the activation at x to be 'intersection'. So y = exp(-0.5(x-c0)^2/w^2)
         *   intersection = exp(-0.5((0.5(c0+c1))-c0)^2/w^2)
         *   intersection = exp(-0.5((0.5*c1-0.5*c0)^2/w^2))
         *   intersection = exp(-0.5((0.5*(c1-c0))^2/w^2))
         *   intersection = exp(-0.5(0.25*(c1-c0)^2/w^2))
         *   intersection = exp(-0.125((c1-c0)^2/w^2))
         *   w = sqrt((c1-c0)^2/-8*ln(intersection))
         */
        for(int cntr =0; cntr<n_bases-1; cntr++){
            double w = sqrt(pow(local_centers[cntr+1]-local_centers[cntr],2.0)/(-8.0*log(intersection_height)));
            local_widths[cntr] = w;
        }
        //can't compute last width because there is no intersection with a "next" basis function
        //so take the same width as the previous one
        local_widths[n_bases-1] = local_widths[n_bases-2];
    }
    //Put the centers and widths in the correct arrays
    for(int cntr = 0; cntr<n_bases; cntr++){
//        cout << "local_center " << cntr << " = " << local_centers[cntr] << endl;
//        cout << "local_width " << cntr << " = " << local_widths[cntr] << endl;
        centers[cntr] = local_centers[cntr];
        widths[cntr] = local_widths[cntr];
    }
}

}


