#include <Matrix.h>
#include <cstddef>

Matrix::Matrix(size_t r, size_t c)
{
    n_rows = r;
    n_columns = c;

    matrix = new float*[n_rows];

    for(unsigned int i = 0; i < n_rows; i++)
    {
        matrix[i] = new float[n_columns];
    }

}

Matrix::Matrix(float** arg)
{
    matrix = arg;
}

Matrix::~Matrix()
{
    for (unsigned int i = 0; i < n_rows; i++)
    {
        delete [] matrix[i];
    }

    delete[] matrix;
    
}