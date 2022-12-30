#include "matrix.h"

// Constructor: create a matrix in the heap
Matrix::Matrix(int r, int c)
:rowSize(r), colSize(c)
{
    ptr = new int*[rowSize];
    for (int i = 0; i < rowSize; i++)
    {
        ptr[i] = new int[colSize];
    }
}
// Destructor: deletes memory locations in th heap
Matrix::~Matrix()
{
    for (int i = 0; i < rowSize; i++)
    {
        delete[]ptr[i];
    }
    delete[]ptr;
}
// The setup fills the cells with random values between 1 and 5
void Matrix::setup()
{
    for (int i = 0; i < rowSize; i++)
    {
        for (int j = 0; j < colSize; j++)
        {
            ptr[i][j] = rand()%5 + 1;
        }
    }
}
// The add function adds second the host and creates result;
void Matrix::add(const Matrix& second, Matrix& result)const
{
    assert(second.rowSize == rowSize && second.colSize == colSize);
    assert(result.rowSize == rowSize && result.colSize == colSize);

    for (int i = 0; i < rowSize; i++)
    {
        for (int j = 0; j < second.colSize; j++)
        {
            result.ptr[i][j] = ptr[i][j] + second.ptr[i][j];
        }
    }
}

// The sub function adds second the host and creates result;
void Matrix::subtract(const Matrix& second, Matrix& result)const
{
    assert(second.rowSize == rowSize && second.colSize == colSize);
    assert(result.rowSize == rowSize && result.colSize == colSize);

    for (int i = 0; i < rowSize; i++)
    {
        for (int j = 0; j < second.colSize; j++)
        {
            result.ptr[i][j] = ptr[i][j] - second.ptr[i][j];
        }
    }
}
// The multiply function adds second the host and creates result;
void Matrix::multiply(const Matrix& second, Matrix& result)const
{
    assert(colSize == second.rowSize);
    assert(result.rowSize = rowSize);
    assert(result.colSize = second.colSize);

    for (int i = 0; i < rowSize; i++)
    {
        for (int j = 0; j < second.colSize; j++)
        {
            result.ptr[i][j] = 0;
            for (int k = 0; k < colSize; k++)
            {
                result.ptr[i][j] += ptr[i][k] * second.ptr[k][j];
            }
        }
    }
}
// The print function prints the values of cells
void Matrix::print()const
{
    for (int i = 0; i < rowSize; i++)
    {
        for (int j = 0; j < colSize; j++)
        {
            cout << setw(5) < ptr[i][j];
        }
        cout << endl;
    }
    cout << endl;
}