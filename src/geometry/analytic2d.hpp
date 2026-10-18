#pragma once

#include <core/tuple.hpp>

namespace mv::geometry {

void distance_pp(const core::Tuple& row1, const core::Tuple& column1, const core::Tuple& row2,
                 const core::Tuple& column2, core::Tuple* distance);
void distance_pl(const core::Tuple& row, const core::Tuple& column, const core::Tuple& row1,
                 const core::Tuple& column1, const core::Tuple& row2, const core::Tuple& column2,
                 core::Tuple* distance);
void intersection_lines(const core::Tuple& row_a1, const core::Tuple& column_a1,
                        const core::Tuple& row_a2, const core::Tuple& column_a2,
                        const core::Tuple& row_b1, const core::Tuple& column_b1,
                        const core::Tuple& row_b2, const core::Tuple& column_b2, core::Tuple* row,
                        core::Tuple* column, core::Tuple* is_overlapping);
void angle_ll(const core::Tuple& row_a1, const core::Tuple& column_a1, const core::Tuple& row_a2,
              const core::Tuple& column_a2, const core::Tuple& row_b1, const core::Tuple& column_b1,
              const core::Tuple& row_b2, const core::Tuple& column_b2, core::Tuple* angle);

}  // namespace mv::geometry
