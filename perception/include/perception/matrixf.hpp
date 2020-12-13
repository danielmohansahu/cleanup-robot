#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>

// using matrix = std::vector<std::vector<double>>;

#define matrix std::vector<std::vector<double>>

class MatrixF{
public:
	matrix substract(const matrix& A, const matrix& B);
	matrix multiply(const matrix& A, const matrix& B);
	matrix req_multiply(matrix A, matrix B, matrix C, matrix D);
};