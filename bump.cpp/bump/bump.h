#ifndef BUMP_H_INCLUDED_2E6948B0_55AF_4C64_8BC6_FAE4BA07D63F
#define BUMP_H_INCLUDED_2E6948B0_55AF_4C64_8BC6_FAE4BA07D63F
#include <cstdint>
#include <exception>
#include <functional>
#include <map>
#include <tuple>
#include <vector>

namespace Bump
{
    /// ------------------------------------------
    /// -- Types
    /// ------------------------------------------
    using Number = double;
    using Index = std::int32_t;
    using Item = void*;

    /// ------------------------------------------
    /// -- Constants
    /// ------------------------------------------
    static const double deltaError = 1e-10;

    /// ------------------------------------------
    /// -- Exceptions
    /// ------------------------------------------
    namespace Exception
    {
        class ComputationError : public std::exception {};
        class NotFoundError : public std::exception {};
    }

    /// ------------------------------------------
    /// -- Enums
    /// ------------------------------------------
    enum class FilterType
    {
        Touch, Slide, Cross, Bounce,
    };    

    /// ------------------------------------------
    /// -- Structures
    /// ------------------------------------------
    struct Point
    {
        Number x = 0;
        Number y = 0;
    };

    struct Rectangle
    {
        Number x = 0;
        Number y = 0;
        Number w = 0;
        Number h = 0;
    };

    struct Collision
    {
        Point move;
        Point normal;
        Point touch;        
        Rectangle itemRect;
        Rectangle otherRect;
        bool overlaps = false;
        Number ti = 0;

        Item item;
        Item other;

        Point slide;
        Point bounce;
    };

    struct ItemInfo
    {
        Item &item;
        Number ti1 = 0;
        Number ti2 = 0;
        Number weight = 0;
    };

    struct Cell
    {
        Number itemCount = 0;
        Number x = 0;
        Number y = 0;
        std::vector<std::reference_wrapper<Item>> items;
    };

    class World;

    /// ------------------------------------------
    /// -- Aliases
    /// ------------------------------------------
    using Filter = std::function<bool(const Item &, const Item &)>;
    using Response = std::tuple<Number, Number, std::vector<Collision>, std::uint32_t>;
    using ResponseFunction = std::function<Response(
        World &world, Collision &col,
        const Number &x, const Number &y,
        const Number &w, const Number &h,
        const Number &goalX, const Number &goalY,
        const Filter &filter)>;
    using Collisions = std::tuple<std::vector<Collision>, std::uint32_t>;

    /// ------------------------------------------
    /// -- Classes
    /// ------------------------------------------
    class World
    {
    public:
        World(const Number &cellSize = 64);
        ~World() = default;

        World(const World &a) = delete;
        World &operator=(const World &a) = delete;

        World(World &&a) = default;

        Collisions project(
            const Item &item,
            const Number &x, const Number &y,
            const Number &w, const Number &h,
            const Number &goalX, const Number &goalY,
            const Filter &filter
        );

        void addResponse(const std::string &name, const ResponseFunction &handler);

    private:
        void addItemToCell(const Item &item, const Number &cx, const Number &cy);

        static bool sortByWeight(const ItemInfo &a, const ItemInfo &b);
        static bool sortByTiAndDistance(const Collision &a, const Collision &b);  
        inline ResponseFunction getResponseByName(const std::string &name);

    private:
        Number cellSize;
        std::vector<std::vector<Cell>> rows;
        std::vector<Rectangle> rects;

        std::map<Cell*, bool> nonEmptyCells;

        std::map<std::string, ResponseFunction> responses;
    };

    /// ------------------------------------------
    /// -- Functions
    /// ------------------------------------------
    namespace Rect
    {
        std::tuple<Number, Number> getNearestCorner(
            const Number &x, const Number &y,
            const Number &w, const Number &h,
            const Number &px, const Number &py
        );

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
                Number ti1 = 0, Number ti2 = 1
            );

        // Calculates the minkowsky difference between 2 rects, which is another rect
        std::tuple<Number, Number, Number, Number> getDiff(
            const Number &x1, const Number &y1,
            const Number &w1, const Number &h1,
            const Number &x2, const Number &y2,
            const Number &w2, const Number &h2
        );

        bool containsPoint(
            const Number &x, const Number &y,
            const Number &w, const Number &h,
            const Number &px, const Number &py
        );

        bool isIntersecting(
            const Number &x1, const Number &y1,
            const Number &w1, const Number &h1,
            const Number &x2, const Number &y2,
            const Number &w2, const Number &h2
        );

        Number getSquareDistance(
            const Number &x1, const Number &y1,
            const Number &w1, const Number &h1,
            const Number &x2, const Number &y2,
            const Number &w2, const Number &h2
        );

        Collision detectCollision(
            const Number &x1, const Number &y1,
            const Number &w1, const Number &h1,
            const Number &x2, const Number &y2,
            const Number &w2, const Number &h2,
            const Number &goalX, const Number &goalY
        );

        Collision detectCollision(
            const Number &x1, const Number &y1,
            const Number &w1, const Number &h1,
            const Number &x2, const Number &y2,
            const Number &w2, const Number &h2
        );
    }

    namespace Responses
    {
        Response touch(
            World &world, Collision &col,
            const Number &x, const Number &y,
            const Number &w, const Number &h,
            const Number &goalX, const Number &goalY,
            const Filter &filter
        );

        Response cross(
            World &world, Collision &col,
            const Number &x, const Number &y,
            const Number &w, const Number &h,
            const Number &goalX, const Number &goalY,
            const Filter &filter
        );

        Response slide(
            World &world, Collision &col,
            const Number &x, const Number &y,
            const Number &w, const Number &h,
            const Number &goalX, const Number &goalY,
            const Filter &filter
        );

        Response bounce(
            World &world, Collision &col,
            const Number &x, const Number &y,
            const Number &w, const Number &h,
            const Number &goalX, const Number &goalY,
            const Filter &filter
        );
    }    

    void test();
}

#endif