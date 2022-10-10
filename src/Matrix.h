/**
 * @file Matrix.h
 * @author Shivam Patel (shivam.patel94@rutgers.edu)
 * @brief This file contains the method declarations for a matrix class
 * @version 1.0
 * @date 2022-09-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef Matrix_H
#define Matrix_H

#include <cstddef>;

using namespace std;

class Matrix
{
private:
    size_t n;
    size_t n_rows;
    size_t n_columns;
    float** matrix;
public:
    Matrix(size_t r, size_t c);
    Matrix(float** arg);
    ~Matrix(void);
    Matrix getTranspose();
    static Matrix identity(size_t n);
};

#endif Matrix_H
