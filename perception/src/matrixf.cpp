#include <perception/matrixf.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>

// using matrix = std::vector<std::vector<double>>;
#define matrix std::vector<std::vector<double>>
using namespace std;

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