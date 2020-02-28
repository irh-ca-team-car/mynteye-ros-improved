#ifndef CMatrix_hpp
#define CMatrix_hpp


#include <iostream>
#include <random>
#include <cstdio>
#include <cstring>


class CMatrix {
private:
	double* m_ptValues;
	int m_totalSize;
	int m_rows;
	int m_columns;
public:
	CMatrix(int rows, int columns); // base ctor.
	CMatrix(const CMatrix& rhs); // copy ctor.
	CMatrix& operator=(const CMatrix& rhs); // assign. ctor.
	~CMatrix(); // dtor.
	double& operator()(int row, int column);
	double& operator()(int row, int column) const;
	CMatrix& operator+=(double scalar);
	CMatrix operator+(double scalar);
	CMatrix& operator-=(double scalar);
	CMatrix operator-(double scalar);
	CMatrix& operator*=(double scalar);
	CMatrix operator*(double scalar);
	CMatrix& operator*=(const CMatrix& rhs);
	CMatrix operator*(const CMatrix& rhs);
	CMatrix& operator+=(const CMatrix& rhs);
	CMatrix operator+(const CMatrix& rhs);
	CMatrix operator-();

	void reshape(int newRows, int newColumns);
	void show(); //used for dev. only
	void range(int start, int defaultStep = 1);
	void fill(int value);
};

#endif /* CMatrix_hpp */