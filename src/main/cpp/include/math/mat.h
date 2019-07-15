#pragma once

#include <cstddef>
#include <utility>
#include <stdexcept>

namespace rpf {

    template <typename T>
    class Mat {
    public:
        Mat(std::size_t m, std::size_t n) : m(m), n(n) {
            contents = new T[m * n]();
        }

        ~Mat() {
            delete[] contents;
        }

        inline T* operator[](std::size_t row) {
            return contents + row * n;
        }
        inline const T* operator[](std::size_t row) const {
            return contents + row * n;
        }

        void eliminate() {
            // If there are more rows than columns, don't do anything
            if(n < m) {
                throw std::invalid_argument("Cannot eliminate: There are more rows than columns!");
            }
            // Forward elimination
            for(std::size_t i = 0; i < m; i ++) {
                // Pivot is 0, so try to swap it with another row below
                if((*this)[i][i] == 0) {
                    // Find a row with a nonezero value in this column
                    std::size_t j;
                    for(j = i + 1; j < m; j ++) {
                        // If found, swap the rows
                        if((*this)[j][i] != 0) {
                            row_swap(i, j);
                            break;
                        }
                    }
                    // If nothing is found, the matrix is singular
                    if(i == m) {
                        throw std::invalid_argument("Cannot eliminate: The matrix is singular!");
                    }
                }

                double pivot = (*this)[i][i];
                // Now the pivot should be nonzero
                // Eliminate this column in all rows below
                for(std::size_t j = i + 1; j < m; j ++) {
                    row_add(j, i, -((*this)[j][i] / pivot));
                }
            }
            // Back substitution
            for(std::size_t i = m; i --> 0;) {
                // Make the pivot 1
                row_mult(i, 1 / (*this)[i][i]);
                // Eliminate this column in all rows above
                if(i != 0) {
                    for(std::size_t j = i; j --> 0;) {
                        row_add(j, i, -(*this)[j][i]);
                    }
                }
            }
        }

    protected:
        std::size_t m;
        std::size_t n;

        T *contents;

        void row_swap(std::size_t a, std::size_t b) {
            for(std::size_t i = 0; i < n; i ++) {
                std::swap(contents[i + a * n], contents[i + b * n]);
            }
        }
        template <typename U>
        void row_mult(std::size_t row, U scalar) {
            for(std::size_t i = 0; i < n; i ++) {
                contents[i + row * n] *= scalar;
            }
        }
        template <typename U>
        void row_add(std::size_t a, std::size_t b, U scalar = 1) {
            for(std::size_t i = 0; i < n; i ++) {
                contents[i + a * n] += contents[i + b * n] * scalar;
            }
        }
    };
}
