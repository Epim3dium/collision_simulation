#pragma once
#include "SFML/Graphics/PrimitiveType.hpp"
#include "col_utils.hpp"
#include "rigidbody.hpp"
#include "types.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>
#include <array>
#include <list>
#include <set>

//remember that type T should always be pointer 
namespace EPI_NAMESPACE {
template<typename T, typename GetBox, typename Equal = std::equal_to<T*>, typename Float = float>
class QuadTree
{
    static_assert(std::is_convertible_v<std::invoke_result_t<GetBox, T*>, AABB>,
        "GetBox must be a callable of signature Box<Float>(const T&)");
    static_assert(std::is_convertible_v<std::invoke_result_t<Equal, T*, T*>, bool>,
        "Equal must be a callable of signature bool(const T&, const T&)");
    static_assert(std::is_arithmetic_v<Float>);

    struct Node
    {
        std::array<std::unique_ptr<Node>, 4> children;
        std::vector<T*> values;
        AABB box;

        Node(AABB b) : box(b) {}
        ~Node() {
        }
    };
public:

    QuadTree(const AABB& box, const GetBox& getBox = GetBox(),
        const Equal& equal = Equal()) :
        _box(box), _root(std::make_unique<Node>(box)), _getBox(getBox), _equalCheck(equal)
    {

    }

    void add(T* value)
    {
        _locations[value] = add(_root.get(), 0, _box, value);
    }

    void remove(T* value)
    {
        auto itr = _locations.find(value);
        if(itr == _locations.end()) 
            return;
        removeValue(itr->second, value);
        _locations.erase(itr);
    }
        //adds if isnt added yet
    void update(T* value) {
        remove(value);
        add(value);
    }

    std::vector<T*> query(const AABB& box) const
    {
        auto values = std::vector<T*>();
        query(_root.get(), _box, box, values);
        return values;
    }

    std::vector<std::pair<T*, T*>> findAllIntersections() const
    {
        auto intersections = std::vector<std::pair<T*, T*>>();
        findAllIntersections(_root.get(), intersections);
        return intersections;
    }

    AABB getBox() const 
    {
        return _box;
    }
    void updateLeafes() {
        updateLeafes(_root.get());
    }
    void clear() {
        clear(_root.get());

    }
    
private:
    static constexpr auto Threshold = std::size_t(16);
    static constexpr auto MaxDepth = std::size_t(8);


    std::map<T*, Node*> _locations;

    AABB _box;
    std::unique_ptr<Node> _root;

    GetBox _getBox;
    Equal _equalCheck;

    bool isLeaf(const Node* node) const
    {
        return !static_cast<bool>(node->children[0]);
    }

    AABB computeBox(const AABB & box, int i) const
    {
        auto origin = box.min;
        auto childSize = box.size() / static_cast<Float>(2);
        switch (i)
        {
            // North West
            case 0:
                return AABB::CreateMinSize(origin, childSize);
            // Norst East
            case 1:
                return AABB::CreateMinSize(vec2f(origin.x + childSize.x, origin.y), childSize);
            // South West
            case 2:
                return AABB::CreateMinSize(vec2f(origin.x, origin.y + childSize.y), childSize);
            // South East
            case 3:
                return AABB::CreateMinSize(origin + childSize, childSize);
            default:
                assert(false && "Invalid child index");
                return AABB ();
        }
    }

    int getQuadrant(const AABB & nodeBox, const AABB & valueBox) const
    {
        auto center = nodeBox.center();
        // West
        if (valueBox.right() < center.x)
        {
            // North West
            if (valueBox.bot() < center.y)
                return 0;
            // South West
            else if (valueBox.top() >= center.y)
                return 2;
            // Not contained in any quadrant
            else
                return -1;
        }
        // East
        else if (valueBox.left() >= center.x)
        {
            // North East
            if (valueBox.bot() < center.y)
                return 1;
            // South East
            else if (valueBox.top() >= center.y)
                return 3;
            // Not contained in any quadrant
            else
                return -1;
        }
        // Not contained in any quadrant
        else
            return -1;
    }

    Node* add(Node* node, std::size_t depth, const AABB & box, T* value)
    {
        assert(node != nullptr);
        assert(AABBcontainsAABB(box, _getBox(value)));
        if (isLeaf(node))
        {
            // Insert the value in this node if possible
            if (depth >= MaxDepth || node->values.size() < Threshold) {
                node->values.push_back(value);
                return node;
            } else {
                split(node, box);
                return add(node, depth, box, value);
            }
        }
        else
        {
            auto i = getQuadrant(box, _getBox(value));
            // Add the value in a child if the value is entirely contained in it
            if (i != -1) {
                return add(node->children[static_cast<std::size_t>(i)].get(), depth + 1, computeBox(box, i), value);
            } else {
                node->values.push_back(value);
                return node;
            }
        }
    }

    void split(Node* node, const AABB & box)
    {
        assert(node != nullptr);
        assert(isLeaf(node) && "Only leaves can be split");
        // Create children
        int quad = 0;
        for (auto& child : node->children)
            child = std::make_unique<Node>(computeBox(node->box, quad++));
        // Assign values to children
        auto newValues = std::vector<T*>(); // New values for this node
        for (const auto& value : node->values)
        {
            auto i = getQuadrant(box, _getBox(value));
            if (i != -1) {
                _locations[value] = node->children[static_cast<std::size_t>(i)].get();
                node->children[static_cast<std::size_t>(i)]->values.push_back(value);
            } else {
                newValues.push_back(value);
            }
        }
        node->values = std::move(newValues);
    }

    bool removeManually(Node* node, const AABB & box, T* value)
    {
        assert(node != nullptr);
        assert(AABBcontainsAABB(box, _getBox(value)));
        if (isLeaf(node))
        {
            // Remove the value from node
            removeValue(node, value);
            return true;
        }
        else
        {
            // Remove the value in a child if the value is entirely contained in it
            auto i = getQuadrant(box, _getBox(value));
            if (i != -1)
            {
                if(remove(node->children[static_cast<std::size_t>(i)].get(), computeBox(box, i), value)) {
                    tryMerge(node->children[static_cast<std::size_t>(i)].get());
                        std::cerr << "merged";
                    return true;
                }
                return false;
            }
            // Otherwise, we remove the value from the current node
            else
                removeValue(node, value);
            return false;
        }
    }

    void removeValue(Node* node, T* value)
    {
        // Find the value in node->values
        auto it = std::find_if(std::begin(node->values), std::end(node->values),
            [this, &value](const auto& rhs){ return _equalCheck(value, rhs); });
        assert(it != std::end(node->values) && "Trying to remove a value that is not present in the node");
        // Swap with the last element and pop back
        *it = std::move(node->values.back());
        node->values.pop_back();
    }

    bool tryMerge(Node* node)
    {
        assert(node != nullptr);
        assert(!isLeaf(node) && "Only interior nodes can be merged");
        auto nbValues = node->values.size();
        for (const auto& child : node->children)
        {
            if (!isLeaf(child.get()))
                return false;
            nbValues += child->values.size();
        }
        if (nbValues <= Threshold)
        {
            node->values.reserve(nbValues);
            // Merge the values of all the children
            for (const auto& child : node->children) {
                for (const auto& value : child->values) {
                    _locations[value] = node;
                    node->values.push_back(value);
                }
            }
            // Remove the children
            for (auto& child : node->children)
                child.reset();
            return true;
        }
        else
            return false;
    }

    void query(Node* node, const AABB & box, const AABB & queryBox, std::vector<T*>& values) const
    {
        assert(node != nullptr);
        if(!isOverlappingAABBAABB(queryBox, box)) {
            return;
        }
        for (const auto& value : node->values) {
            if (isOverlappingAABBAABB(queryBox, _getBox(value)))
                values.push_back(value);
        }
        if (!isLeaf(node)) {
            for (auto i = std::size_t(0); i < node->children.size(); ++i) {
                auto childBox = computeBox(box, static_cast<int>(i));
                if (isOverlappingAABBAABB(queryBox, childBox))
                    query(node->children[i].get(), childBox, queryBox, values);
            }
        }
    }

    void findAllIntersections(Node* node, std::vector<std::pair<T*, T*>>& intersections) const {
        // Find intersections between values stored in this node
        // Make sure to not report the same intersection twice
        for (auto i = std::size_t(0); i < node->values.size(); ++i) {
            for (auto j = std::size_t(0); j < i; ++j) {
                if (isOverlappingAABBAABB(_getBox(node->values[i]), _getBox(node->values[j])))
                    intersections.emplace_back(node->values[i], node->values[j]);
            }
        }
        if (!isLeaf(node)) {
            // Values in this node can intersect values in descendants
            for (const auto& child : node->children) {
                for (const auto& value : node->values)
                    findIntersectionsInDescendants(child.get(), value, intersections);
            }
            // Find intersections in children
            for (const auto& child : node->children)
                findAllIntersections(child.get(), intersections);
        }
    }

    void findIntersectionsInDescendants(Node* node, T* value, std::vector<std::pair<T*, T*>>& intersections) const {
        // Test against the values stored in this node
        for (const auto& other : node->values) {
            if (isOverlappingAABBAABB(_getBox(value), _getBox(other)) )
                intersections.emplace_back(value, other);
        }
        // Test against values stored into descendants of this node
        if (!isLeaf(node)) {
            for (const auto& child : node->children)
                findIntersectionsInDescendants(child.get(), value, intersections);
        }
    }
};
}
