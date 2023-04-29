#pragma once

#include <Eigen/Core>
#include <iostream>
#include <nlohmann/json.hpp>

#define TO_JSON(e, x) j[#x] = e.x;
#define FROM_JSON(e, x) e.x = j.value(#x, decltype(e.x)());

namespace Eigen {

template <typename ScalarType, int rows, int cols>
void to_json(nlohmann::json &jsonObject, const Matrix<ScalarType, rows, cols> &matrix) {
    jsonObject = nlohmann::json::array({});
    for (int r = 0; r < matrix.rows(); ++r) {
        nlohmann::json jsonArray;
        for (int c = 0; c < matrix.cols(); ++c) {
            jsonArray.push_back(matrix(r, c));
        }
        jsonObject.push_back(jsonArray);
    }
}

template <typename ScalarType, int rows, int cols>
void from_json(const nlohmann::json &jsonObject, Matrix<ScalarType, rows, cols> &matrix) {
    nlohmann::json jsonArray;
    if (jsonObject.is_array()) {
        if (jsonObject.empty()) {
            return;
        }
        jsonArray = jsonObject;
    } else if (jsonObject.is_number()) {
        jsonArray.push_back(jsonObject);
    } else {
        throw nlohmann::detail::type_error::create(0, "");
    }

    nlohmann::json jsonArrayOfArrays;
    if (jsonArray.front().is_array())  // provided matrix
    {
        jsonArrayOfArrays = jsonArray;
    } else  // provided vector
    {
        if (rows == 1)  // expected row vector
        {
            jsonArrayOfArrays.push_back(jsonArray);
        } else if (cols == 1)  // expected column vector
        {
            for (unsigned int i = 0; i < jsonArray.size(); ++i) {
                jsonArrayOfArrays.push_back({jsonArray.at(i)});
            }
        } else  // expected matrix
        {
            std::cerr << "Expected a matrix, received a vector." << std::endl;
            throw nlohmann::detail::type_error::create(0, "");
        }
    }

    const unsigned int providedRows = jsonArrayOfArrays.size();
    const unsigned int providedCols = jsonArrayOfArrays.front().size();
    if ((rows >= 0 && int(providedRows) != rows) || (cols >= 0 && int(providedCols) != cols)) {
        std::cerr << "Expected matrix of size " << rows << "x" << cols << ", received matrix of size " << providedRows << "x" << providedCols << "."
                  << std::endl;
        throw nlohmann::detail::type_error::create(0, "");
    }

    matrix.resize(providedRows, providedCols);
    for (unsigned int r = 0; r < providedRows; ++r) {
        if (jsonArrayOfArrays.at(r).size() != providedCols) {
            std::cerr << "Unconsistent matrix size: some rows have different "
                         "number of columns."
                      << std::endl;
            throw nlohmann::detail::type_error::create(0, "");
        }
        for (unsigned int c = 0; c < providedCols; ++c) {
            matrix(r, c) = jsonArrayOfArrays.at(r).at(c);
        }
    }
}

}  // namespace Eigen
