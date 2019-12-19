#include <mex.h>
#include <Eigen/Eigen>
#include "localize.h"

// Takes an mxArray and returns its contents as an eigen matrix by reference
void input_matrix(const mxArray* p, Eigen::MatrixXd& mat) {
    int m = mxGetM(p);
    int n = mxGetN(p);

    mat.resize(m,n);

    double *data = mxGetPr(p);

    for(int j = 0; j < n; j++) {
        for(int i = 0; i < m; i++) {
            mat(i,j) = data[m*j + i];
        }
    }
}

// Should be called as:
// [P, inliers] = CameraPoseRANSAC_Mex(X,x,tol,nIters), where X is a 3xN matrix containing the 3D points, and
// x is a 3xN matrix containing the image points as unit vectors and tol is the largest allowed
// angular deviation (in radians) for a correspondence to be considered an inlier, and nIters is the max number of iterations.
// The output P is the calculated camera matrix as well as an integer designating the number of inliers.
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // Validate the input
    if(nrhs != 4)
        mexErrMsgTxt("Invalid input! The function takes three inputs: \n\t3d structure X as a 3xN matrix, \n\ta 3xN matrix representing image points as unit vectors, \n\tthe maximum allowed angle deviation for a correspondence to be considered an inlier,\n\tnumber of RANSAC iterations.");
    if(nlhs != 2)
        mexErrMsgTxt("Invalid output! The function returns two outputs: P representing the triangulated camera matrix and an integer designating the number of inliers for P. ");

    Eigen::MatrixXd x;
    Eigen::MatrixXd X;
    double tol = mxGetScalar(prhs[2]);
    int nIter = int(mxGetScalar(prhs[3]));

    input_matrix(prhs[0],X);
    input_matrix(prhs[1],x);

    if(x.rows() != 3 || X.rows() != 3 || x.cols() != X.cols())
        mexErrMsgTxt("Input matrices must both have three rows and the same number of columns!");

    // Inputs and outputs are ok! Calculate camera pose!
    int numInliers = 0;
    Eigen::Matrix<double,3,4> P = Eigen::MatrixXd::Zero(3,4);
    bool success = cameraPoseRANSAC(x,X,nIter,tol,P,numInliers);
    if (!success)
        mexWarnMsgTxt("Could not find a solution to the 3 point problem.");

    // Create output
    plhs[0] = mxCreateDoubleMatrix(P.rows(),4,mxREAL);
    double* outputMatrix = mxGetPr(plhs[0]);
    int index = 0;
    for (int j = 0; j < P.cols(); j++) {
        for (int i = 0; i < P.rows(); i++) {
            outputMatrix[index] = P(i,j);
            index++;
        }
    }

    plhs[1] = mxCreateDoubleMatrix(1,1,mxREAL);
    double* output = mxGetPr(plhs[1]);
    output[0] = static_cast<double>(numInliers);
}
