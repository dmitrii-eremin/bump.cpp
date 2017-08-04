#include "bump.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

namespace Bump
{
    /// ------------------------------------------
    /// -- Auxiliary functions
    /// ------------------------------------------
    namespace Aux
    {
        inline std::int8_t sign(const Number &value)
        {
            if (value > 0)
            {
                return 1;
            }
            if (value < 0)
            {
                return -1;
            }
            return 0;
        }

        inline Number nearest(
            const Number &value, 
            const Number &first, 
            const Number &second
        )
        {
            return std::abs(first - value) < std::abs(second - value) ? first : second;
        }
    }

    /// ------------------------------------------
    /// -- Rectangle functions
    /// ------------------------------------------
    namespace Rect
    {
        std::tuple<Number, Number> getNearestCorner(
            const Number &x, const Number &y,
            const Number &w, const Number &h,
            const Number &px, const Number &py
        )
        {
            const Number nearestX = Aux::nearest(px, x, x + w);
            const Number nearestY = Aux::nearest(py, y, y + h);
            return std::make_tuple(nearestX, nearestY);
        }

        // This is a generalized implementation of the liang - barsky algorithm, which also returns
        // the normals of the sides where the segment intersects.
        // Returns nil if the segment never touches the rect
        // Notice that normals are only guaranteed to be accurate when initially ti1, ti2 == -math.huge, math.huge
        std::tuple<Number, Number, Number, Number, Number, Number> 
            getSegmentIntersectionIndices(
                const Number &x, const Number &y, 
                const Number &w, const Number &h,
                const Number &x1, const Number &y1, 
                const Number &x2, const Number &y2,
                Number ti1, Number ti2
        )
        {
            Number dx = x2 - x1;
            Number dy = y2 - y1;

            Number nx = 0;
            Number ny = 0;

            Number nx1 = 0;
            Number ny1 = 0;
            Number nx2 = 0;
            Number ny2 = 0;

            Number p = 0;
            Number q = 0;
            Number r = 0;

            for (std::int8_t side = 1; side <= 4; side++)
            {
                switch (side)
                {
                case 1:
                    // Left
                    std::tie(nx, ny, p, q) = std::make_tuple(-1, 0, -dx, x1 - x);
                    break;
                case 2:
                    // Right
                    std::tie(nx, ny, p, q) = std::make_tuple(1, 0, dx, x + w - x1);
                    break;
                case 3:
                    // Top
                    std::tie(nx, ny, p, q) = std::make_tuple(0, -1, -dy, y1 - y);
                    break;
                case 4:
                    // Bottom
                    std::tie(nx, ny, p, q) = std::make_tuple(0, 1, dy, y + h - y1);
                    break;
                default:
                    assert(false);
                    break;
                }

                if (p == 0)
                {
                    if (q <= 0)
                    {
                        throw Exception::ComputationError();
                    }
                }
                else
                {
                    r = q / p;
                    if (p < 0)
                    {
                        if (r > ti2)
                        {
                            throw Exception::ComputationError();
                        }
                        if (r > ti1)
                        {
                            std::tie(ti1, nx1, ny1) = std::make_tuple(r, nx, ny);
                        }
                    }
                    else
                    {
                        if (r < ti1)
                        {
                            throw Exception::ComputationError();
                        }
                        if (r < ti2)
                        {
                            std::tie(ti2, nx2, ny2) = std::make_tuple(r, nx, ny);
                        }
                    }
                }
            }

            return std::make_tuple(ti1, ti2, nx1, ny1, nx2, ny2);
        }

        // Calculates the minkowsky difference between 2 rects, which is another rect
        std::tuple<Number, Number, Number, Number> getDiff(
            const Number &x1, const Number &y1, 
            const Number &w1, const Number &h1,
            const Number &x2, const Number &y2, 
            const Number &w2, const Number &h2
        )
        {
            return std::make_tuple(
                x2 - x1 - w1,
                y2 - y1 - h1,
                w1 + w2,
                h1 + h2
            );
        }

        bool containsPoint(
            const Number &x, const Number &y, 
            const Number &w, const Number &h,
            const Number &px, const Number &py
        )
        {
            return px - x > deltaError && py - y > deltaError &&
                x + w - px > deltaError && y + h - py > deltaError;
        }

        bool isIntersecting(
            const Number &x1, const Number &y1, 
            const Number &w1, const Number &h1,
            const Number &x2, const Number &y2, 
            const Number &w2, const Number &h2
        )
        {
            return x1 < x2 + w2 && x2 < x1 + w1 &&
                y1 < y2 + h2 && y2 < y1 + h1;
        }

        Number getSquareDistance(
            const Number &x1, const Number &y1, 
            const Number &w1, const Number &h1,
            const Number &x2, const Number &y2, 
            const Number &w2, const Number &h2
        )
        {
            const Number dx = x1 - x2 + (w1 - w2) / 2;
            const Number dy = y1 - y2 + (h1 - h2) / 2;
            return dx * dx + dy * dy;
        }

        Collision detectCollision(
            const Number &x1, const Number &y1, 
            const Number &w1, const Number &h1,
            const Number &x2, const Number &y2, 
            const Number &w2, const Number &h2,
            const Number &goalX, const Number &goalY
        )
        {
            const Number dx = goalX - x1;
            const Number dy = goalY - y1;

            Number x, y, w, h;
            std::tie(x, y, w, h) = getDiff(x1, y1, w1, h1, x2, y2, w2, h2);

            bool overlaps = false;
            Number ti = 0;
            Number nx = 0;
            Number ny = 0;

            if (containsPoint(x, y, w, h, 0, 0)) // Item was intersecting other
            {
                Number px, py;
                std::tie(px, py) = getNearestCorner(x, y, w, h, 0, 0);

                // Area of intersection
                Number wi = std::min(w1, std::abs(px));
                Number hi = std::min(h1, std::abs(py));

                // ti is the negative area of intersection
                ti = -wi * hi;
                overlaps = true;
            }
            else
            {
                Number ti1, ti2, nx1, ny1, nx2, ny2;
                std::tie(ti1, ti2, nx1, ny1, nx2, ny2) =
                    getSegmentIntersectionIndices(
                        x, y, w, h, 0, 0, dx, dy,
                        -std::numeric_limits<Number>::max(),
                        std::numeric_limits<Number>::max()
                        );

                if (ti < 1 &&
                    (std::abs(ti1 - ti2) >= deltaError) &&
                    (ti1 + deltaError > 0 || (ti1 == 0 && ti2 > 0)))
                {
                    std::tie(ti, nx, ny) = std::make_tuple(ti1, nx1, ny1);
                    overlaps = false;
                }
            }

            Number tx = 0;
            Number ty = 0;

            if (overlaps)
            {
                if (dx == 0 && dy == 0)
                {
                    Number px = 0;
                    Number py = 0;
                    std::tie(px, py) = getNearestCorner(x, y, w, h, 0, 0);
                    if (std::abs(px) < std::abs(py))
                    {
                        py = 0;
                    }
                    else
                    {
                        px = 0;
                    }
                    std::tie(nx, ny) = std::make_tuple(Aux::sign(px), Aux::sign(py));
                    std::tie(tx, ty) = std::make_tuple(x1 + px, y1 + py);
                }
                else
                {
                    Number ti1, _1, _2, _3;
                    std::tie(ti1, _1, nx, ny, _2, _3) =
                        getSegmentIntersectionIndices(
                            x, y, w, h, 0, 0, dx, dy,
                            -std::numeric_limits<Number>::max(), 1
                            );
                    std::tie(tx, ty) = std::make_tuple(x1 + dx * ti1, y1 + dy * ti1);
                }
            }
            else
            {
                std::tie(tx, ty) = std::make_tuple(x1 + dx * ti, y1 + dy * ti);
            }

            return Collision
            {
                { dx, dy }, { nx, ny }, { tx, ty },
                { x1, y1, w1, h1 },
                { x2, y2, w2, h2 },
                overlaps, ti,
                nullptr, nullptr,
                { 0, 0 }, { 0, 0 }
            };
        }

        Collision detectCollision(
            const Number &x1, const Number &y1, 
            const Number &w1, const Number &h1,
            const Number &x2, const Number &y2, 
            const Number &w2, const Number &h2
        )
        {
            return detectCollision(x1, y1, w1, h1, x2, y2, w2, h2, x1, y1);
        }
    }

    /// ------------------------------------------
    /// -- Grid functions
    /// ------------------------------------------
    namespace Grid
    {
        std::tuple<Number, Number> toWorld(
            const Number &cellSize, 
            const Number &cx, const Number &cy
        )
        {
            return std::make_tuple((cx - 1) * cellSize, (cy - 1) * cellSize);
        }

        std::tuple<Number, Number> toCell(
            const Number &cellSize, 
            const Number &x, const Number &y
        )
        {
            return std::make_tuple(
                static_cast<Number>(x / cellSize) + 1,
                static_cast<Number>(y / cellSize) + 1
            );
        }

        // Grid_traverse* functions are based on "A Fast Voxel Traversal Algorithm for Ray Tracing",
        // by John Amanides and Andrew Woo - http://www.cse.yorku.ca/~amana/research/grid.pdf
        // It has been modified to include both cells when the ray "touches a grid corner",
        // and with a different exit condition
        std::tuple<Number, Number, Number> traverseInitStep(
            const Number &cellSize, const Number &ct,
            const Number &t1, const Number &t2
        )
        {
            const Number v = t2 - t1;
            if (v > 0)
            {
                return std::make_tuple(1, cellSize / v, ((ct + v) * cellSize - t1) / v);
            }
            if (v < 0)
            {
                return std::make_tuple(-1, -cellSize / v, ((ct + v - 1) * cellSize - t1) / v);
            }

            return std::make_tuple(
                0,
                std::numeric_limits<Number>::max(),
                std::numeric_limits<Number>::max()
            );
        }

        void traverse(
            const Number &cellSize,
            const Number &x1, const Number &y1,
            const Number &x2, const Number &y2,
            const std::function<void(const Number &, const Number &)> &f
        )
        {
            Number cx1, cy1;
            std::tie(cx1, cy1) = toCell(cellSize, x1, y1);
            Number cx2, cy2;
            std::tie(cx2, cy2) = toCell(cellSize, x2, y2);

            Number stepX, dx, tx;
            std::tie(stepX, dx, tx) = traverseInitStep(cellSize, cx1, x1, x2);
            Number stepY, dy, ty;
            std::tie(stepY, dy, ty) = traverseInitStep(cellSize, cy1, y1, y2);

            Number cx = cx1;
            Number cy = cy1;

            f(cx, cy);

            // The default implementation had an infinite loop problem when
            // approaching the last cell in some occassions.We finish iterating
            // when we are *next* to the last cell
            while (std::abs(cx - cx2) + std::abs(cy - cy2) > 1)
            {
                if (tx < ty)
                {
                    std::tie(tx, cx) = std::make_tuple(tx + dx, cx + stepX);
                    f(cx, cy);
                }
                else
                {
                    // Addition: include both cells when going through corners
                    if (tx == ty)
                    {
                        f(cx + stepX, cy);
                    }
                    std::tie(ty, cy) = std::make_tuple(ty + dy, cy + stepY);
                    f(cx, cy);
                }
            }

            // If we have not arrived to the last cell, use it
            if (cx != cx2 || cy != cy2)
            {
                f(cx2, cy2);
            }
        }

        std::tuple<Number, Number, Number, Number> toCellRect(
            const Number &cellSize,
            const Number &x, const Number &y, 
            const Number &w, const Number &h
        )
        {
            Number cx, cy;
            std::tie(cx, cy) = toCell(cellSize, x, y);
            Number cr, cb;
            std::tie(cr, cb) = std::make_tuple(
                std::ceil(x + w) / cellSize,
                std::ceil(y + h) / cellSize
            );

            return std::make_tuple(cx, cy, cr - cx + 1, cb - cy + 1);
        }
    }

    /// ------------------------------------------
    /// -- Responses functions
    /// ------------------------------------------
    namespace Responses
    {
        Response touch(
            World &world, Collision &col,
            const Number &x, const Number &y,
            const Number &w, const Number &h,
            const Number &goalX, const Number &goalY,
            const Filter &filter
        )
        {
            return Response{ col.touch.x, col.touch.y, {}, 0 };
        }

        Response cross(
            World &world, Collision &col,
            const Number &x, const Number &y,
            const Number &w, const Number &h,
            const Number &goalX, const Number &goalY,
            const Filter &filter
        )
        {
            std::vector<Collision> cols;
            std::uint32_t len;
            std::tie(cols, len) = world.project(
                col.item, x, y, w, h, goalX, goalY, filter
            );

            return Response{ goalX, goalY, cols, len };
        }

        Response slide(
            World &world, Collision &col,
            const Number &x, const Number &y,
            const Number &w, const Number &h,
            const Number &goalX, const Number &goalY,
            const Filter &filter
        )
        {
            Point touch = col.touch;
            Point move = col.move;

            Number sx = touch.x;
            Number sy = touch.y;

            if (move.x != 0 || move.y != 0)
            {
                if (col.normal.x == 0)
                {
                    sx = goalX;
                }
                else
                {
                    sy = goalY;
                }
            }

            col.slide = { sx, sy };

            Number newX = touch.x;
            Number newY = touch.y;

            Number newGoalX = sx;
            Number newGoalY = sy;

            std::vector<Collision> cols;
            std::uint32_t len;
            std::tie(cols, len) = world.project(
                col.item, newX, newY, w, h, newGoalX, newGoalY, filter
            );

            return Response{ newGoalX, newGoalY, cols, len };
        }

        Response bounce(
            World &world, Collision &col,
            const Number &x, const Number &y,
            const Number &w, const Number &h,
            const Number &goalX, const Number &goalY,
            const Filter &filter
        )
        {
            Point touch = col.touch;
            Point move = col.move;

            Number tx = touch.x;
            Number ty = touch.y;

            Number bx = tx;
            Number by = ty;

            if (move.x != 0 || move.y != 0)
            {
                Number bnx, bny;
                std::tie(bnx, bny) = std::make_tuple(goalX - tx, goalY - ty);
                if (col.normal.x == 0)
                {
                    bny = -bny;
                }
                else
                {
                    bnx = -bnx;
                }

                std::tie(bx, by) = std::make_tuple(tx + bnx, ty + bny);
            }

            col.bounce = { bx, by };
            
            Number newX = touch.x;
            Number newY = touch.y;

            Number newGoalX = bx;
            Number newGoalY = by;

            std::vector<Collision> cols;
            std::uint32_t len;
            std::tie(cols, len) = world.project(
                col.item, newX, newY, w, h, newGoalX, newGoalY, filter
            );

            return Response{ newGoalX, newGoalY, cols, len };
        }
    }

    /// ------------------------------------------
    /// -- Classes
    /// ------------------------------------------
    World::World(const Number &cellSize_) : cellSize(cellSize_)
    {
        addResponse("touch", Responses::touch);
        addResponse("cross", Responses::cross);
        addResponse("slide", Responses::slide);
        addResponse("bounce", Responses::bounce);
    }

    Collisions World::project(
        const Item &item,
        const Number &x, const Number &y,
        const Number &w, const Number &h,
        const Number &goalX, const Number &goalY,
        const Filter &filter
    )
    {
        return { {  }, 0 };
    }

    void World::addResponse(const std::string &name, const ResponseFunction &handler)
    {
        responses[name] = handler;
    }

    void World::addItemToCell(const Item &item, const Number &cx, const Number &cy)
    {
        Index ix = static_cast<Index>(cx);
        Index iy = static_cast<Index>(cy);

        if (rows.size() < iy + 1)
        {
            rows.resize(iy + 1);
        }
        auto &row = rows[iy];
        if (row.size() < ix + 1)
        {
            row.resize(ix + 1);
        }

        auto &cell = row[ix];

        nonEmptyCells[&cell] = true;
    }

    bool World::sortByWeight(const ItemInfo &a, const ItemInfo &b)
    {
        return a.weight < b.weight;
    }

    bool World::sortByTiAndDistance(const Collision &a, const Collision &b)
    {
        if (a.ti == b.ti)
        {
            Rectangle ir, ar, br;
            std::tie(ir, ar, br) = std::make_tuple(a.itemRect, a.otherRect, b.otherRect);

            Number ad = Rect::getSquareDistance(
                ir.x, ir.y, ir.w, ir.h,
                ar.x, ar.y, ar.w, ar.h
            );

            Number bd = Rect::getSquareDistance(
                ir.x, ir.y, ir.w, ir.h,
                br.x, br.y, br.w, br.h
            );

            return ad < bd;
        }
        return a.ti < b.ti;
    }

    inline ResponseFunction World::getResponseByName(const std::string &name)
    {
        auto result = responses.find(name);
        if (result == responses.end())
        {
            throw Exception::NotFoundError();
        }

        return result->second;
    }

    /// ------------------------------------------
    /// -- Public Functions
    /// ------------------------------------------    

    void test()
    {
        World w;
    }
}