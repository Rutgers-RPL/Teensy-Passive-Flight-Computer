#include <Matrix.h>
#include <cstddef>

Matrix::Matrix(){}

Matrix::Matrix(size_t r, size_t c)
{
    n_rows = r;
    n_columns = c;

    matrix = new float*[n_rows];

    for(size_t i = 0; i < n_rows; i++)
    {
        matrix[i] = new float[n_columns];
        for(int j = 0; j < n_columns; j++)
            matrix[i][j] = 0;
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

float Matrix::get(size_t r, size_t c)
{
    return matrix[r][c];
}

Matrix Matrix::getTranspose()
{
    float** transposition = new float*[n_columns];
    for(size_t i = 0; i < n_columns; i++)
    {
        transposition[i] = new float[n_rows];

        for(size_t j = 0; j < n_rows; j++)
        {
            transposition[i][j] = matrix[j][i];
        }
    }

    return Matrix(transposition);
}

Matrix Matrix::operator*(const Matrix& m)
{
    {
        if(n_columns != m.n_rows)
        {
            return NULL;
        }
        else
        {
            
            float** ans = new float*[n_rows];

            for(size_t i = 0; i < n_rows; i++)
            {
                ans[i] = new float[n_columns];
            }

            for(int i = 0; i < n_rows; i++)
            {
                for(int j = 0; i < m.n_columns; j++)
                {
                    ans[i][j] = 0;

                    for(int k = 0; k < m.n_rows; k++)
                    {
                        ans[i][j] += matrix[i][k] * m.matrix[k][j];
                    }
                }
            }
        }
    }


}

Matrix Matrix::operator*(const int& i)
{
    Matrix result(matrix);

    for(int i = 0; i < n_rows; i++)
    {
        for(int j = 0; j < n_columns; j++)
        {
            result.matrix[i][j] *= i;
        }
    }

    return result;
}

Matrix Matrix::operator*(const float& f)
{
    Matrix result(matrix);

    for(int i = 0; i < n_rows; i++)
    {
        for(int j = 0; j < n_columns; j++)
        {
            result.matrix[i][j] *= f;
        }
    }

    return result;
}

Matrix Matrix::operator+(const Matrix& m)
{
    if(m.n_columns != n_columns || m.n_rows != n_rows)
        return NULL;

    Matrix result(matrix);

    for(int i = 0; i < n_rows; i++)
    {
        for(int j = 0; j < n_columns; j++)
        {
            result.matrix[i][j] =  m.matrix[i][j] + matrix[i][j];
        }
    }

    return result;
}

Matrix Matrix::identity(size_t n)
{
    float** id = new float*[n];

    for(int i = 0; i < n; i++)
    {
        id[i] = new float[n];
        id[i][i] = 1;
    }

    return Matrix(id);
}