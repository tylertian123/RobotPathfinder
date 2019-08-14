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

        Mat(std::initializer_list<std::initializer_list<T>> values) {
            m = values.size();
            n = values.begin()->size();

            contents = new T[m * n]();

            std::size_t i = 0;
            for(const auto &row : values) {
                if(row.size() != n) {
                    throw std::invalid_argument("Matrix initializer lists must be rectangular!");
                }
                std::copy(row.begin(), row.end(), (*this)[i++]);
            }
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

        Mat<T>& operator=(std::initializer_list<std::initializer_list<T>> values) {
            if(values.size() != m || values.begin()->size() != n) {
                throw std::invalid_argument("Matrix dimensions aren't equal");
            }

            std::size_t i = 0;
            for(const auto &row : values) {
                if(row.size() != n) {
                    throw std::invalid_argument("Matrix initializer lists must be rectangular!");
                }
                std::copy(row.begin(), row.end(), (*this)[i++]);
            }

            return *this;
        }

        void eliminate() {
            // If there are more rows than columns, don't do anything
            if(n < m) {
                throw std::invalid_argument("Cannot eliminate: There are more rows than columns!");
            }

            for(std::size_t i = 0, j = 0; i < m && j < n; j ++) {
                // Zero pivot
                if((*this)[i][j] == 0) {
                    std::size_t k = i;
                    // Swap with row below if found
                    for(; k < m; k ++) {
                        if((*this)[k][j] != 0) {
                            row_swap(i, k);
                            break;
                        }
                    }
                    // If search finished without anything being found the matrix is singular
                    if(k == m) {
                        throw std::invalid_argument("Cannot eliminate: The matrix is singular!");
                    }
                }
                // Make the pivot 1
                row_mult(i, 1 / (*this)[i][j]);
                // Eliminate the column
                for(std::size_t k = 0; k < m; k ++) {
                    if(i == k) {
                        continue;
                    }

                    row_add(k, i, -(*this)[k][j]);
                }

                i++;
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
