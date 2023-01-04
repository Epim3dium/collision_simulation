#pragma once
#include "SFML/Graphics/PrimitiveType.hpp"
#include "col_utils.h"
#include "rigidbody.hpp"
#include "types.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>
#include <array>
#include <list>

namespace EPI_NAMESPACE {
constexpr size_t MAX_DEPTH = 8;

template <typename OBJECT_TYPE>
class QuadTree;

template<typename T>
struct QuadTreeItemLocation {
    typename std::list< std::pair <AABB, T>>* container;
    typename std::list< std::pair <AABB, T>>::iterator iterator;
    AABB parent_size;
};

template <class T>
struct QuadTreeItem {
    T item;
    QuadTreeItemLocation<typename std::list<QuadTreeItem<T>>::iterator > pItem;
};
template <typename OBJECT_TYPE>
class QuadTree
{
public:
    static void drawAABB(AABB aabb, Window& rw, Color clr) {
        sf::Vertex v[2];
        v[0].color = clr;
        v[1].color = clr;
        std::pair<vec2f, vec2f> lines[] = {{aabb.bl(), aabb.br()}, {aabb.bl(), aabb.tl()}, {aabb.tl(), aabb.tr()}, {aabb.br(), aabb.tr()}};
        for(auto [a, b] : lines) {
            v[0].position = a;
            v[1].position = b;
            rw.draw(v, 2U, sf::Lines);
        }
    }
	QuadTree(const AABB shape, const size_t nDepth = 0)
	{
		m_depth = nDepth;
		resize(shape);
	}

	// Force area change on Tree, invalidates this and all child layers
	void resize(const AABB& rArea)
	{
		// Erase this layer
		clear();

		// Recalculate area of children
		m_rect = rArea;
		vec2f vChildSize = rArea.size() / 2.0f;

		// Cache child areas local to this layer
		m_rChild =
		{
			AABB(rArea.min, rArea.min + vChildSize),
			AABB(rArea.min + vChildSize, rArea.max),

			AABB(rArea.min + vec2f(0, vChildSize.y), rArea.min + vec2f(0, vChildSize.y) + vChildSize),
			AABB(rArea.min + vec2f(vChildSize.x, 0), rArea.min + vec2f(vChildSize.x, 0) + vChildSize)
		};

	}

	// Clears the contents of this layer, and all child layers
	void clear()
	{
		// Erase any items stored in this layer
		m_pItems.clear();

		// Iterate through children, erase them too
		for (int i = 0; i < 4; i++)
		{
			if (m_pChild[i])
				m_pChild[i]->clear();
			m_pChild[i].reset();
		}
	}

	// Returns a count of how many items are stored in this layer, and all children of this layer
	size_t size() const
	{
		size_t nCount = m_pItems.size();
		for (int i = 0; i < 4; i++)
			if (m_pChild[i]) nCount += m_pChild[i]->size();
		return nCount;
	}

	// Inserts an object into this layer (or appropriate child layer), given the area the item occupies
	QuadTreeItemLocation <OBJECT_TYPE> insert(const OBJECT_TYPE& item, const AABB& itemsize)
	{
		// Check each child
		for (int i = 0; i < 4; i++)
		{
			// If the child can wholly contain the item being inserted
			if (AABBcontainsAABB(m_rChild[i], itemsize))
			{
				// Have we reached depth limit?
				if (m_depth + 1 < MAX_DEPTH)
				{
					// No, so does child exist?
					if (!m_pChild[i])
					{
						// No, so create it
						m_pChild[i] = std::make_shared<QuadTree<OBJECT_TYPE>>(m_rChild[i], m_depth + 1);
					}

					// Yes, so add item to it
					return m_pChild[i]->insert(item, itemsize);
				}
			}
		}

		// It didnt fit, so item must belong to this quad
		m_pItems.push_back({ itemsize, item});
        return { &m_pItems, std::prev(m_pItems.end()), m_rect};
	}

	// Returns a list of objects in the given search area
	std::list<OBJECT_TYPE> search(const AABB& rArea) const
	{
		std::list<OBJECT_TYPE> listItems;
		search(rArea, listItems);
		return listItems;
	}

	// Returns the objects in the given search area, by adding to supplied list
	void search(const AABB& rArea, std::list<OBJECT_TYPE>& listItems) const
	{
		// First, check for items belonging to this area, add them to the list
		// if there is overlap
		for (const auto& p : m_pItems)
		{
			if (AABBvAABB(rArea, p.first))
				listItems.push_back(p.second);
		}

		// Second, recurse through children and see if they can
		// add to the list
		for (int i = 0; i < 4; i++)
		{
			if (m_pChild[i])
			{
				// If child is entirely contained within area, recursively
				// add all of its children, no need to check boundaries
				if (AABBcontainsAABB(rArea, m_rChild[i]))
					m_pChild[i]->items(listItems);

				// If child overlaps with search area then checks need
				// to be made
				else if (AABBvAABB(m_rChild[i], rArea))
					m_pChild[i]->search(rArea, listItems);
			}
		}
	}

	void items(std::list<OBJECT_TYPE>& listItems) const
	{
		// No questions asked, just return child items
		for (const auto& p : m_pItems)
			listItems.push_back(p.second);

		// Now add children of this layer's items
		for (int i = 0; i < 4; i++)
			if (m_pChild[i]) m_pChild[i]->items(listItems);
	}

    bool contains(OBJECT_TYPE obj) {
        auto it = std::find_if(m_pItems.begin(), m_pItems.end(), 
            [&](const std::pair<AABB, OBJECT_TYPE>& a) {
               return a.second == obj;
            });
        if(it != m_pItems.end()) {
            return true;
        } else {
            for(auto& c : m_pChild) {
                if(c->contains(obj))
                    return true;
            }
        }
        return false;
    }
    bool remove(OBJECT_TYPE pItem) {
        auto it = std::find_if(m_pItems.begin(), m_pItems.end(), 
            [&](const std::pair<AABB, OBJECT_TYPE>& a) {
               return a.second == pItem;
            });
        if(it != m_pItems.end()) {
            m_pItems.earse(it);
            return true;
        } else {
            for(auto& c : m_pChild) {
                if(c->remove(pItem))
                    return true;
            }

        }
        return false;

    }

	std::list<OBJECT_TYPE> items() const
	{
		// No questions asked, just return child items
		std::list<OBJECT_TYPE> listItems;
		items(listItems);
		return listItems;
	}

	// Returns area of this layer
	const AABB& area()
	{
		return m_rect;
	}
    void draw(Window& rw, Color clr) {
        drawAABB(m_rect, rw, clr);
        for(auto& i : m_pItems) {
            drawAABB(i.first, rw, clr);
        }
        for(auto& c : m_pChild) {
            if(c.get() != nullptr)
                c->draw(rw, clr);
        }
    }


        AABB m_rect;
protected:
	// Depth of this StaticQuadTree layer
	size_t m_depth = 0;

	// Area of this StaticQuadTree

	// 4 child areas of this StaticQuadTree
	std::array<AABB, 4> m_rChild{};

	// 4 potential children of this StaticQuadTree
	std::array<std::shared_ptr<QuadTree<OBJECT_TYPE>>, 4> m_pChild{};

	// Items which belong to this StaticQuadTree
	std::list<std::pair<AABB, OBJECT_TYPE>> m_pItems;
};


template <typename OBJECT_TYPE>
class QuadTreeContainer
{
	// Using a std::list as we dont want pointers to be invalidated to objects stored in the
	// tree should the contents of the tree change
	using QuadTreeContainerType = std::list<QuadTreeItem<OBJECT_TYPE>>;

protected:
	// The actual container
	QuadTreeContainerType m_allItems;

	// Use our StaticQuadTree to store "pointers" instead of objects - this reduces
	// overheads when moving or copying objects 
	QuadTree<typename QuadTreeContainerType::iterator> root;

public:
	QuadTreeContainer(const AABB& size, const size_t nDepth = 0) : root(size, nDepth) { }

	// Sets the spatial coverage area of the quadtree
	// Invalidates tree
	void resize(const AABB& rArea)
	{
		root.resize(rArea);
	}

	// Returns number of items within tree
	size_t size() const
	{
		return m_allItems.size();
	}

	// Returns true if tree is empty
	bool empty() const
	{
		return m_allItems.empty();
	}

	// Removes all items from tree
	void clear()
	{
		root.clear();
		m_allItems.clear();
	}


	// Convenience functions for ranged for loop
	typename QuadTreeContainerType::iterator begin()
	{
		return m_allItems.begin();
	}

	typename QuadTreeContainerType::iterator end()
	{
		return m_allItems.end();
	}

	typename QuadTreeContainerType::const_iterator cbegin()
	{
		return m_allItems.cbegin();
	}

	typename QuadTreeContainerType::const_iterator cend()
	{
		return m_allItems.cend();
	}


	// Insert item into tree in specified area
	void insert(const OBJECT_TYPE& item, const AABB& itemsize)
	{
        QuadTreeItem<OBJECT_TYPE> newItem = {item};

		// Item is stored in container
		m_allItems.push_back(newItem);
		m_allItems.back().pItem = root.insert(std::prev(m_allItems.end()), itemsize);
	}

	// Returns a std::list of pointers to items within the search area
	std::list<typename QuadTreeContainerType::iterator> search(const AABB& rArea) const
	{
		std::list<typename QuadTreeContainerType::iterator> listItemPointers;
		root.search(rArea, listItemPointers);
		return listItemPointers;
	}
    void remove(typename QuadTreeContainerType::iterator item) {
        item->pItem.container->erase(item->pItem.iterator);

        m_allItems.erase(item);
    }
    void relocate(typename QuadTreeContainerType::iterator& item, AABB new_pos) {
        item->pItem.container->erase(item->pItem.iterator);
        item->pItem = root.insert(item, new_pos);
    }
    void draw(Window& rw, Color clr) {
        root.draw(rw, clr);
    }

};

}
