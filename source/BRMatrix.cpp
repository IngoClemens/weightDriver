// ---------------------------------------------------------------------
//
//  BRMatrix.cpp
//
//  Created by ingo on 12/7/16.
//  Copyright (c) 2021 Ingo Clemens. All rights reserved.
//
// ---------------------------------------------------------------------

#include "BRMatrix.h"

#include "math.h"

BRMatrix::BRMatrix()
{
    mat.resize(0);
    rows = 0;
    cols = 0;
}


BRMatrix::BRMatrix(const BRMatrix &inMat)
{
    mat = inMat.mat;
    rows = inMat.getRowSize();
    cols = inMat.getColSize();
}


BRMatrix::~BRMatrix() {}


// ---------------------------------------------------------------------
// size
// ---------------------------------------------------------------------

void BRMatrix::setSize(unsigned rowSize, unsigned colSize)
{
    mat.resize(rowSize);
    for (unsigned i = 0; i < mat.size(); i ++)
        mat[i].resize(colSize);

    rows = rowSize;
    cols = colSize;
}


unsigned BRMatrix::getRowSize() const
{
    return this->rows;
}


unsigned BRMatrix::getColSize() const
{
    return this->cols;
}


// ---------------------------------------------------------------------
// get vector
// ---------------------------------------------------------------------

std::vector<double> BRMatrix::getRowVector(unsigned row)
{
    std::vector<double> vec;
    vec.resize(cols);

    for (unsigned i = 0; i < cols; i ++)
        vec[i] = this->mat[row][i];

    return vec;
}


std::vector<double> BRMatrix::getColumnVector(unsigned col)
{
    std::vector<double> vec;
    vec.resize(rows);

    for (unsigned i = 0; i < rows; i ++)
        vec[i] = this->mat[i][col];

    return vec;
}


double* BRMatrix::getColumnVector(double *vec, int col)
{
    for (unsigned i = 0; i < rows; i++)
        vec[i] = this->mat[i][(unsigned)col];

    return vec;
}


// ---------------------------------------------------------------------
// operator
// ---------------------------------------------------------------------

BRMatrix& BRMatrix::operator=(const BRMatrix &inMat)
{
    unsigned i, j;

    if (&inMat == this)
        return *this;

    unsigned rowSize = inMat.getRowSize();
    unsigned colSize = inMat.getColSize();

    mat.resize(rowSize);
    for (i = 0; i < mat.size(); i ++)
        mat[i].resize(colSize);

    for (i = 0; i < rowSize; i ++)
    {
        for (j = 0; j < colSize; j ++)
            mat[i][j] = inMat(i, j);
    }

    rows = rowSize;
    cols = colSize;

    return *this;
}


BRMatrix BRMatrix::operator*(const BRMatrix &inMat)
{
    unsigned i, j, k;

    unsigned rowSize = inMat.getRowSize();
    unsigned colSize = inMat.getColSize();

    BRMatrix result;
    result.setSize(rowSize, colSize);

    for (i = 0; i < rowSize; i ++)
    {
        for (j = 0; j < colSize; j ++)
        {
            for (k = 0; k < rowSize; k ++)
                result(i, j) += this->mat[i][k] * inMat(k, j);
        }
    }

    return result;
}


BRMatrix& BRMatrix::operator*=(const BRMatrix &inMat)
{
    BRMatrix result = (*this) * inMat;
    (*this) = result;
    return *this;
}


BRMatrix BRMatrix::transpose()
{
    unsigned i, j;

    BRMatrix result;
    result.setSize(rows, cols);

    for (i = 0; i < rows; i ++)
    {
        for (j = 0; j < cols; j ++)
            result(i, j) = this->mat[j][i];
    }

    return result;
}


// ---------------------------------------------------------------------
// access elements
// ---------------------------------------------------------------------

double& BRMatrix::operator()(const unsigned &row, const unsigned &col)
{
    return this->mat[row][col];
}


const double& BRMatrix::operator()(const unsigned &row, const unsigned &col) const
{
    return this->mat[row][col];
}


// ---------------------------------------------------------------------
// gaussian elimination
// ---------------------------------------------------------------------

bool BRMatrix::solve(std::vector<double> y, double w[])
{
    // Make sure that the matrix is square.
    if (rows != cols)
        return false;

    unsigned int i, j, k;
    int x;
    unsigned size = rows;
    double maxVal = 0.0;
    unsigned pivot = 0;
    bool swap = false;
    double mult = 0.0;
    double sum = 0.0;

    for (i = 0; i < size; i++)
    {
        // Find the row with the largest absolute value in the first
        // column and store the pivot index.
        maxVal = this->mat[i][i];
        pivot = i;
        swap = false;
        for (j = i + 1; j < size; j++)
        {
            if (fabs(maxVal) < fabs(this->mat[j][i]))
            {
                maxVal = this->mat[j][i];
                pivot = j;
                swap = true;
            }
        }

        // Perform the row interchange if necessary.
        if (swap)
        {
            double value = 0.0;
            for (j = 0; j < size; j++)
            {
                w[j] = this->mat[pivot][j];
                this->mat[pivot][j] = this->mat[i][j];
                this->mat[i][j] = w[j];
            }

            // Swap the order of the values.
            value = y[pivot];
            y[pivot] = y[i];
            y[i] = value;
        }

        // Check if the matrix is singular.
        if (fabs(this->mat[i][i]) < 0.0001)
            return false;

        // Perform the forward elimination.
        for (j = i + 1; j < size; j++)
        {
            mult = this->mat[j][i] / this->mat[i][i];
            for (k = 0; k < size; k++)
                this->mat[j][k] -= mult * this->mat[i][k];
            y[j] -= mult * y[i];
        }
    }

    // Perform the back substitution.
    for (x = (int)size - 1; x >= 0; x--)
    {
        sum = 0.0;
        for (j = (unsigned)x + 1; j < size; j++)
            sum += this->mat[(unsigned)x][j] * w[j];
        w[(unsigned)x] = (y[(unsigned)x] - sum) / this->mat[(unsigned)x][(unsigned)x];
    }

    return true;
}


// ---------------------------------------------------------------------
// Helper functions which output the values of the matrix in a formatted
// string.
// ---------------------------------------------------------------------

void BRMatrix::show(MString node, MString dataName)
{
    unsigned int i, j;

    MString s(node + " : " + dataName + ":\n");

    for (i = 0; i < rows; i++)
    {
        for (j = 0; j < cols; j++)
            s += MString(" ") + mat[i][j];

        s += MString("\n");
    }

    MGlobal::displayInfo(s);
}


void BRMatrix::showVector(std::vector<double> v, MString name)
{
    unsigned int i;
    size_t size = v.size();

    MString s(name + ":");

    for (i = 0; i < size; i++)
        s += MString(" ") + v[i];

    MGlobal::displayInfo(s);
}

// ---------------------------------------------------------------------
// MIT License
//
// Copyright (c) 2021 Ingo Clemens, brave rabbit
// weightDriver is under the terms of the MIT License
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Author: Ingo Clemens    www.braverabbit.com
// ---------------------------------------------------------------------
