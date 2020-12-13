#include <perception/matrixf.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>

using matrix = std::vector<std::vector<double>>;
using namespace std;


double MatrixF::getDeterminant(const matrix vect) {
    if(vect.size() != vect[0].size()) {
        throw std::runtime_error("Matrix is not quadratic");
    } 
    int dimension = vect.size();

    if(dimension == 0) {
        return 1;
    }

    if(dimension == 1) {
        return vect[0][0];
    }

    //Formula for 2x2-matrix
    if(dimension == 2) {
        return vect[0][0] * vect[1][1] - vect[0][1] * vect[1][0];
    }

    double result = 0;
    int sign = 1;
    for(int i = 0; i < dimension; i++) {

        //Submatrix
        matrix subVect(dimension - 1, std::vector<double> (dimension - 1));
        for(int m = 1; m < dimension; m++) {
            int z = 0;
            for(int n = 0; n < dimension; n++) {
                if(n != i) {
                    subVect[m-1][z] = vect[m][n];
                    z++;
                }
            }
        }

        //recursive call
        result = result + sign * vect[0][i] * getDeterminant(subVect);
        sign = -sign;
    }

    return result;
}


matrix MatrixF::getTranspose(const matrix matrix1) {

    //Transpose-matrix: height = width(matrix), width = height(matrix)
    matrix solution(matrix1[0].size(), std::vector<double> (matrix1.size()));

    //Filling solution-matrix
    for(size_t i = 0; i < matrix1.size(); i++) {
        for(size_t j = 0; j < matrix1[0].size(); j++) {
            solution[j][i] = matrix1[i][j];
        }
    }
    return solution;
}

matrix MatrixF::getCofactor(const std::vector<std::vector<double>> vect) {
    if(vect.size() != vect[0].size()) {
        throw std::runtime_error("Matrix is not quadratic");
    } 

    matrix solution(vect.size(), std::vector<double> (vect.size()));
    matrix subVect(vect.size() - 1, std::vector<double> (vect.size() - 1));

    for(std::size_t i = 0; i < vect.size(); i++) {
        for(std::size_t j = 0; j < vect[0].size(); j++) {

            int p = 0;
            for(size_t x = 0; x < vect.size(); x++) {
                if(x == i) {
                    continue;
                }
                int q = 0;

                for(size_t y = 0; y < vect.size(); y++) {
                    if(y == j) {
                        continue;
                    }

                    subVect[p][q] = vect[x][y];
                    q++;
                }
                p++;
            }
            solution[i][j] = pow(-1, i + j) * getDeterminant(subVect);
        }
    }
    return solution;
}

matrix MatrixF::getInverse(const matrix &vect) {
    if(getDeterminant(vect) == 0) {
        throw std::runtime_error("Determinant is 0");
    } 

    double d = 1.0/getDeterminant(vect);
    std::vector<std::vector<double>> solution(vect.size(), std::vector<double> (vect.size()));

    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect.size(); j++) {
            solution[i][j] = vect[i][j]; 
        }
    }

    solution = getTranspose(getCofactor(solution));

    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect.size(); j++) {
            solution[i][j] *= d;
        }
    }

    return solution;
}

matrix MatrixF::substract(const matrix& A, const matrix& B) {
    matrix solution(B.size(), std::vector<double> (B.size()));
    for (int i = 0; i < A.size(); i++){
        for (int j = 0; j < B.size(); j++){
                solution[i][j]=A[i][j]-B[i][j];
            }
    }

    return solution;
}

matrix MatrixF::multiply(const matrix& A, const matrix& B) {
    // std::vector <std::vector<T>> c(n, std::vector<T>(p, 0));
    matrix solution(B.size(), std::vector<double> (B.size()));
    for (int i = 0; i < A.size(); i++){
        for (int j = 0; j < B.size(); j++){
            for (int k = 0; k < A.size();k++)
                solution[i][j] += A[i][k] * B[k][j];
        }
    }
    return solution;
}

matrix MatrixF::req_multiply(matrix A, matrix B, matrix C, matrix D) {
    matrix part1 = multiply(A,B);
    matrix req_part1 {
        {part1[0][0]},
        {part1[1][0]},
        {part1[2][0]}
    };
    matrix part2 = substract(part1, C);
    matrix req_part2 {
        {part2[0][0]},
        {part2[1][0]},
        {part2[2][0]}
    };
    matrix part3 = multiply(req_part2, D);
    matrix solution {
        {part1[0][0]},
        {part1[1][0]},
        {part1[2][0]}
    };

    return solution;
}