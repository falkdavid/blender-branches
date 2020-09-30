/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#pragma once

/** \file
 * \ingroup bli
 */

#include <list>

#include "BLI_double2.hh"
#include "BLI_double3.hh"

namespace blender::polyclip {

enum CapType {
  CAP_ROUND = 0,
  CAP_FLAT = 1,
  CAP_MAX,
};

enum ClipType { CT_INTERSECTION, CT_UNION, CT_DIFFERENCE, CT_EXCLUSIVEOR };
enum FillType { FT_EVEN_ODD, FT_NON_ZERO };

struct Vert {
  double2 co;
  double radius;

  Vert() = default;

  Vert(double2 co, double radius = 0) : co(co), radius(radius)
  {
  }

  Vert(double x, double y, double radius = 0) : co(x, y), radius(radius)
  {
  }

  friend std::ostream &operator<<(std::ostream &stream, const Vert &v)
  {
    stream << "(" << v.co.x << ", " << v.co.y << ")";
    return stream;
  }
};

typedef std::list<Vert> VertList;
typedef std::list<double2> PointList;

struct Polyline {
  VertList verts;
  bool is_closed;

  Polyline() = default;

  Polyline(VertList &verts, bool is_closed = false) : verts(verts), is_closed(is_closed)
  {
  }

  friend std::ostream &operator<<(std::ostream &stream, const Polyline &v)
  {
    stream << "{";
    for (auto it : v.verts) {
      stream << it << ", ";
    }
    stream << "}";
    return stream;
  }
};

typedef std::list<Polyline> Polylines;

struct Polygon {
  Polyline contour;
  Polylines holes;
};

template<typename T> struct TLNode {
  T data;
  TLNode<T> *next;
  TLNode<T> *prev;
  TLNode<T> *link;

  TLNode() = default;
  TLNode(const T &data) : data(data), next(nullptr), prev(nullptr), link(nullptr)
  {
  }
};

template<typename T> class TripleLinkedList {
 private:
  uint size_;
  TLNode<T> *head;
  TLNode<T> *tail;

 public:
  TripleLinkedList()
  {
    size_ = 0;
    head = nullptr;
    tail = nullptr;
  }

  TripleLinkedList(const std::list<T> &list);
  ~TripleLinkedList();

  class Iterator {
   private:
    uint idx_;
    TLNode<T> *current_;

   public:
    Iterator(TLNode<T> *current) : current_(current)
    {
    }

    Iterator &operator++()
    {
      current_ = current_->next;
      return *this;
    }

    Iterator operator++(int)
    {
      Iterator iterator = *this;
      ++*this;
      return iterator;
    }

    bool operator!=(const Iterator &iterator) const
    {
      return current_ != iterator.current_;
    }

    TLNode<T> *operator*() const
    {
      return current_;
    }

    T operator->() const
    {
      return current_->data;
    }
  };

  uint size()
  {
    return size_;
  };

  TLNode<T> *front()
  {
    return head;
  };

  TLNode<T> *back()
  {
    return tail;
  };

  Iterator begin() const
  {
    return Iterator(head);
  }

  Iterator end() const
  {
    return Iterator(nullptr);
  }

  friend std::ostream &operator<<(std::ostream &stream, const TripleLinkedList<T> &l)
  {
    for (auto elem : l) {
      stream << elem << ", ";
    }
    stream << "\n";
    return stream;
  }

  TLNode<T> *search(const T &data);
  TLNode<T> *insert_front(const T &data);
  TLNode<T> *insert_back(const T &data);
  TLNode<T> *insert_after(TLNode<T> *insert_node, const T &data);
  TLNode<T> *insert_before(TLNode<T> *insert_node, const T &data);
  void link(TLNode<T> *nodeA, TLNode<T> *nodeB);
  void remove(TLNode<T> *node);
};

// typedef TripleLinkedList<double2> ClipPath;

Polyline polyline_offset(Polyline &pline,
                         const uint subdivisions,
                         const double pline_radius,
                         CapType start_cap_t,
                         CapType end_cap_t);

} /* namespace blender::polyclip */