#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <array>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <unordered_map>
#include <concepts>
#include <cmath>
#include <sstream>
#include <string>

namespace csmutil {
    class dim_mismatch : std::exception {
    public:
        explicit dim_mismatch(const std::string& msg) {m_msg = msg;};
        const char* what() const noexcept override {return m_msg.c_str();};
    private:
        std::string m_msg;
    };
    
    class uninvertible : std::exception {
    public:
        explicit uninvertible(const std::string& msg) {m_msg = msg;};
    const char* what() const noexcept override {return m_msg.c_str();};
    private:
        std::string m_msg;
    };

    template<size_t numRows, size_t numCols>
    class Matrixd {
    private:
        static const inline size_t m_rows = numRows > 0 ? numRows : 1;
        static const inline size_t m_cols = numCols > 0 ? numCols : 1;
        std::array<std::array<double, m_cols>, m_rows> m_matrix;
    public:
        Matrixd() = default;

        Matrixd(std::initializer_list<double> data) {
            if(data.size() != m_rows*m_cols)  {
                std::ostringstream oss;
                oss << "Incorrect number of arguments in constructor for matrix size " << numRows << "x" << numCols;
                throw std::out_of_range(oss.str());
            }
            int i = 0;
            int j = 0;
            for(double e : data) {
                set(i,j,e);
                j = (j+1) % m_cols;
                if(j == 0) i++;
            }
        };

        Matrixd(const Matrixd& mat) {
            m_matrix = mat.m_matrix;
        }

        double at(const int row, const int col) const {return m_matrix[row][col];};
        void set(const int row, const int col, const double value) {m_matrix[row][col] = value;};

        size_t getNumRows() const {return m_rows;};
        size_t getNumCols() const {return m_cols;};

        Matrixd<numRows, numCols> switchRows(int row1, int row2) {
            Matrixd<numRows, numCols> toRet = *this;
            std::array<double, numCols> rowCPY = toRet[row2];
            toRet[row2] = toRet[row1];
            toRet[row1] = rowCPY;
            return toRet;
        }

        void switchRowsInPlace(int row1, int row2) {
            std::array<double, numCols> rowCPY = m_matrix[row2];
            m_matrix[row2] = m_matrix[row1];
            m_matrix[row1] = rowCPY;
        }

        Matrixd<numRows, numCols> addRows(int row1, std::array<double, numCols> row2) {
            Matrixd<numRows, numCols> toRet = *this;
            for(int j = 0; j < numCols; j++) {
                toRet.m_matrix[row1][j] += row2[j];
            }
            return toRet;
        }

        void addRowsInPlace(int row1, std::array<double, numCols> row2) {
            for(int j = 0; j < numCols; j++) {
                m_matrix[row1][j] += row2[j];
            }
        }

        Matrixd<numRows, numCols> multRow(int row, double val) {
            Matrixd<numRows, numCols> toRet = *this;
            for(int j = 0; j < numCols; j++) {
                toRet.m_matrix[row][j] *= val;
            }
            return toRet;
        }

        void multRowInPlace(int row, double val) {
            for(int j = 0; j < numCols; j++) {
                m_matrix[row][j] *= val;
            }
        }

        auto submatrix(int row, int col) requires requires {numRows > 1 && numCols > 1;} { // yes, i am aware that this is stupid, NO I am not going to change it :) [it's gcc's fault anyways >:( ) ]
            if(numRows <= 1 || numCols <= 1) throw std::out_of_range("Incorrect size of matrix");
            Matrixd<m_rows - 1, m_cols - 1> toRet = Matrixd<m_rows - 1, m_cols - 1>();
            bool skippedCol = false;
            bool skippedRow = false;
            for(int i = 0; i < numRows; i++) {
                skippedCol = false;
                if(i == row) {
                    skippedRow = true;
                    continue;
                }
                for(int j = 0; j < numCols; j++) {
                    if(j == col) {
                        skippedCol = true;
                        continue;
                    }

                    toRet.set(i - skippedRow, j - skippedCol, at(i, j));
                }
            }
            return toRet;
        }

        Matrixd<numCols, numRows> transpose() const {
            Matrixd<numCols, numRows> toRet;
            for(int i = 0; i < m_rows; i++) {
                for(int j = 0; j < m_cols; j++) {
                    toRet.set(j,i, at(i,j));
                }
            }
            return toRet;
        };

        Matrixd<numRows, numCols> cofactorMatrix() {
            if(m_rows != m_cols) throw dim_mismatch("Cannot get cofactor matrix of non-square matrix");
            Matrixd<numRows, numCols> toRet = Matrixd<numRows, numCols>();
            for(int i = 0; i < numRows; i++) {
                for(int j = 0; j < numCols; j++) {
                    toRet.set(i,j, cofactor(i,j));
                }
            }
            return toRet;
        }

        // can return a value of -0.0. Don't worry about it, IEEE-754 states that 0.0 == -0.0 :)
        double cofactor(int row, int col) {
            int sign = pow(-1, (row+col % 2));
            return sign * minor(row, col);
        };

        double minor(int row, int col) {
            return (submatrix(row, col)).determinant();
        }

        double determinant() {
            if(m_rows != m_cols) throw dim_mismatch("Cannot take determinant of non-square matrix");
            switch(m_rows) {
                case 1: // by definition
                    return m_matrix[0][0];
                case 2: // by definition
                    return m_matrix[0][0] * m_matrix[1][1] - m_matrix[0][1]  * m_matrix[1][0]; 
                case 3: // nice special case
                    return (m_matrix[0][0]*m_matrix[1][1] *m_matrix[2][2] + m_matrix[0][1]*m_matrix[1][2] *m_matrix[2][0] + m_matrix[0][2]*m_matrix[1][0]*m_matrix[2][1])
                            - (m_matrix[0][2]*m_matrix[1][1]*m_matrix[2][0] + m_matrix[0][1]*m_matrix[1][0]*m_matrix[2][2] + m_matrix[0][0]*m_matrix[1][2]*m_matrix[2][1]);
                default: // recursive implementation
                    double result = 0.0;
                    for(int j = 0; j < m_cols; j++) { // expand matrix along first row
                        result += at(0, j) * cofactor(0, j); // multiply element by cofactor of that element
                    }
                    return result; // done :)
            }
            return 0;
        };

        Matrixd<numRows, numCols> invert() {
            if(m_rows != m_cols) throw dim_mismatch("Cannot invert non-square matrix");
            if(determinant() == 0) throw uninvertible("Cannot invert matrix with determinant == 0");

            Matrixd toRet = Matrixd();
            for(int i = 0; i < m_rows; i++) {
                for(int j = 0; j < m_cols; j++) {

                }
            }
            return toRet;
        };

        // naive implementation
        // not the best for matrices >> 4
        template<size_t V>
        Matrixd<numRows, V> operator*(const Matrixd<numRows, V>& md) const {
            Matrixd<numRows, V> toRet = Matrixd<numRows, V>();
            for(int i = 0; i < m_rows; i++) {
                for(int j = 0; j < md.m_cols; j++) {
                    for(int k = 0; k < md.m_rows; k++) {
                        toRet.set(i,j, toRet.at(i,j) + (at(i,k) * md.at(k,j)));
                    }
                }
            }
            return toRet;
        }

        template<size_t V>
        Matrixd<numRows, V>& operator*=(const Matrixd<numRows, V>& md) const {
            *this = *this * md;
            return *this;
        };

        Matrixd<numRows, numCols> operator+(const Matrixd<numRows, numCols>& md) const {
            Matrixd<numRows, numCols> toRet = Matrixd<numRows, numCols>();
            for(int i = 0; i < m_rows; i++) {
                for(int j = 0; j < m_cols; j++) {
                    toRet.set(i,j, at(i,j) + md.at(i,j));
                }
            }
            return toRet;
        };

        Matrixd<numRows, numCols>& operator+=(const Matrixd<numRows, numCols>& md) {
            *this = *this + md;
            return *this;
        };

        Matrixd<numRows, numCols> operator-(const Matrixd<numRows, numCols>& md) const {
            Matrixd<numRows, numCols> toRet = Matrixd<numRows, numCols>();
            for(int i = 0; i < m_rows; i++) {
                for(int j = 0; j < m_cols; j++) {
                    toRet.set(i,j, at(i,j) - md.at(i,j));
                }
            }
            return toRet;
        };

        Matrixd<numRows, numCols>& operator-=(const Matrixd<numRows, numCols>& md) {
            *this = *this - md;
            return *this;
        };

        Matrixd<numRows, numCols>& operator=(const Matrixd<numRows, numCols>& md) {
            if(this != &md) {
                for(int i = 0; i < m_rows; i++) {
                    m_matrix[i] = md.m_matrix[i];
                }
            }
            return *this;
        };

        bool operator==(const Matrixd<numRows, numCols>& md) const {
            for(int i = 0; i < m_rows; i++) {
                for(int j = 0; j < m_rows; j++) {
                    if(md.at(i,j) != at(i,j)) return false;
                }
            }
            return true;
        }

        friend std::ostream& operator<<(std::ostream& os, const Matrixd<numRows, numCols>& mat4) {
            os << '[';
            
            for(int i = 0; i < mat4.m_rows; i++) {
                for(int j = 0; j < mat4.m_cols; j++) {
                    os << mat4.at(i, j);
                    if(j != (mat4.m_cols-1)) os << ", ";
                }
                if(i != (mat4.m_rows-1)) os << '\n';
            }

            os << ']';
            return os;
        };
    template<size_t U, size_t V>
    friend class Matrixd;
    };

    typedef csmutil::Matrixd<4,4> Matrix4d;
    typedef csmutil::Matrixd<3,3> Matrix3d;
};

#endif // MATRIX_HPP