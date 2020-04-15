// This file is part of Heimer.
// Copyright (C) 2019 Jussi Lind <jussi.lind@iki.fi>
//
// Heimer is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// Heimer is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Heimer. If not, see <http://www.gnu.org/licenses/>.

#ifndef LAYOUT_OPTIMIZER_HPP
#define LAYOUT_OPTIMIZER_HPP

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "mind_map_data.hpp"
#include "node.hpp"

class Graph;

class LayoutOptimizer
{
public:
    LayoutOptimizer(MindMapDataPtr mindMapData);

    void initialize(double aspectRatio, double minEdgeLength);

    void optimize();

    void extract();

private:
    double calculateCost() const;

    struct Cell;

    struct Row;

    using CellVector = std::vector<std::shared_ptr<Cell>>;

    struct Change
    {
        enum class Type
        {
            Move,
            Swap
        };

        Type type;

        std::shared_ptr<Cell> sourceCell;

        std::shared_ptr<Cell> targetCell;

        std::shared_ptr<Row> sourceRow;

        std::shared_ptr<Row> targetRow;

        size_t sourceIndex = 0;

        size_t targetIndex = 0;
    };

    void doChange(const LayoutOptimizer::Change & change);

    void undoChange(LayoutOptimizer::Change change);

    Change planChange();

    MindMapDataPtr m_mindMapData;

    struct Rect
    {
        int x = 0;

        int y = 0;

        int w = 0;

        int h = 0;
    };

    struct Cell
    {
        inline double x() const
        {
            return rect.x + rect.w / 2;
        }

        inline double y() const
        {
            return rect.y + rect.h / 2;
        }

        inline double distance(Cell & other)
        {
            const auto dx = x() - other.x();
            const auto dy = y() - other.y();
            return sqrt(dx * dx + dy * dy);
        }

        inline double overlapCost(Cell & inner, Cell & outer)
        {
            const auto x1 = inner.x() - x();
            const auto y1 = inner.y() - y();
            const auto x2 = outer.x() - x();
            const auto y2 = outer.y() - y();

            if (std::fabs(x1 * y2 - x2 * y1) < 0.001) {
                const auto l1 = x1 * x1 + y1 * y1;
                const auto l2 = x2 * x2 + y2 * y2;
                return l1 < l2 ? l1 : l2;
            }

            return 0;
        }

        inline double getConnectionCost(const CellVector & connections)
        {
            double cost = 0;
            for (auto && cell : connections) {
                cost += distance(*cell);
            }

            return cost;
        }

        inline double getOverlapCost(const CellVector & connections)
        {
            double cost = 0;
            for (size_t i = 0; i < connections.size(); i++) {
                const auto outer = connections.at(i);
                for (size_t j = i + 1; j < connections.size(); j++) {
                    const auto inner = connections.at(j);
                    if (outer != inner) {
                        cost += overlapCost(*outer, *inner);
                    }
                }
            }

            return cost;
        }

        inline double getOutCost()
        {
            double cost = getOverlapCost(out) + getOverlapCost(in);
            for (auto && dependency : out) {
                cost += dependency->getOverlapCost(dependency->out);
                cost += dependency->getOverlapCost(dependency->in);
            }
            for (auto && dependency : in) {
                cost += dependency->getOverlapCost(dependency->out);
                cost += dependency->getOverlapCost(dependency->in);
            }

            return cost + getConnectionCost(out);
        }

        inline double getCompoundCost()
        {
            double outCost = getOutCost();

            outCost += getOverlapCost(in);
            for (auto && dependency : in) {
                outCost += dependency->getOverlapCost(dependency->out);
                outCost += dependency->getOverlapCost(dependency->in);
            }

            return outCost + getConnectionCost(in);
        }

        inline void popRect()
        {
            rect = stash;
        }

        inline void pushRect()
        {
            stash = rect;
        }

        CellVector in;

        CellVector out;

        NodeBasePtr node;

        Rect rect;

        Rect stash;
    };

    struct Row
    {
        CellVector cells;

        Rect rect;
    };

    using RowVector = std::vector<std::shared_ptr<Row>>;

    struct Layout
    {
        double minEdgeLength = 0;

        CellVector all;

        RowVector rows;
    };

    std::unique_ptr<Layout> m_layout;

    std::mt19937 m_engine;
};

#endif // LAYOUT_OPTIMIZER_HPP
