#include <Eigen/Core>
#include <limits>

// TODO: HW2
// Assignment 2, Part 2.1. 
// Fill the values inside linear weight matrix W of shape (n, m)
// V is a vertices matrix in shape (n, 3), each row is the position of a vertex in mesh
// C is a control points matrix in shape (m, 3), each row is the position of a control point
void ComputeLinearSkinningWeights(
	const Eigen::MatrixXd& V, 
	const Eigen::MatrixXd& C, 
	Eigen::MatrixXd& W) 
{
	W.resize(V.rows(), C.rows());
	/* Implement your code here. */
}