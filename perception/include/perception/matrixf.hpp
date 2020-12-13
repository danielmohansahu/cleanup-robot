pragma once

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>

using matrix = std::vector<std::vector<double>>;

class MatrixF{
public:
	double getDeterminant(const matrix vect);
	matrix getTranspose(const matrix matrix1);
	matrix getCofactor(const matrix vect);
	matrix getInverse(const matrix &vect);
	matrix substract(const matrix& A, const matrix& B);
	matrix multiply(const matrix& A, const matrix& B);
	matrix req_multiply(matrix A, matrix B, matrix C, matrix D);
};