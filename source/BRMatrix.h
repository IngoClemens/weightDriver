// ---------------------------------------------------------------------
//
//  BRMatrix.h
//
//  Created by ingo on 12/7/16.
//  Copyright (c) 2021 Ingo Clemens. All rights reserved.
//
// ---------------------------------------------------------------------

#ifndef __SHAPESTools__BRMatrix__
#define __SHAPESTools__BRMatrix__

#include <vector>

#include <maya/MGlobal.h>

class BRMatrix
{
public:
    BRMatrix();
    BRMatrix(const BRMatrix &inMat);
    virtual ~BRMatrix();

    void setSize(unsigned rowSize, unsigned colCount);
    unsigned getRowSize() const;
    unsigned getColSize() const;

    std::vector<double> getRowVector(unsigned row);
    std::vector<double> getColumnVector(unsigned col);
    double* getColumnVector(double *vec, int col);

    BRMatrix& operator=(const BRMatrix &inMat);
    BRMatrix operator*(const BRMatrix &inMat);
    BRMatrix& operator*=(const BRMatrix &inMat);
    BRMatrix transpose();

    double& operator()(const unsigned &row, const unsigned &col);
    const double& operator()(const unsigned &row, const unsigned &col) const;

    bool solve(std::vector<double> y, double w[]);

    void show(MString node, MString dataName);
    void showVector(std::vector<double> v, MString name);

private:
    std::vector<std::vector<double> > mat;
    unsigned rows;
    unsigned cols;
};

#endif

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
