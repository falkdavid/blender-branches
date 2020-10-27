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
#include <queue>
#include <set>

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

template<typename T> class LinkedChain {
 public:
  LinkedChain()
  {
    size_ = 0;
    head = nullptr;
    tail = nullptr;
  }

  LinkedChain(const LinkedChain<T> &src);
  LinkedChain(const std::list<T> &list);

  ~LinkedChain();

  LinkedChain<T> &operator=(LinkedChain<T> src);

  struct Node {
    T data;
    Node *next;
    Node *prev;
    Node *link;

    Node(const T &data, Node *next = nullptr, Node *prev = nullptr, Node *link = nullptr)
        : data(data), next(next), prev(prev), link(link)
    {
    }
  };

  class Iterator {
   private:
    Node *current_;

   public:
    Iterator(Node *current) : current_(current)
    {
    }

    void switch_link()
    {
      BLI_assert(current_ != nullptr);
      current_ = current_->link;
    }

    static Iterator next(const Iterator &it)
    {
      return Iterator(it.current_->next);
    }

    static Iterator prev(const Iterator &it)
    {
      return Iterator(it.current_->prev);
    }

    Iterator &operator++()
    {
      BLI_assert(current_ != nullptr);
      current_ = current_->next;
      return *this;
    }

    Iterator operator++(int)
    {
      Iterator iterator = *this;
      ++*this;
      return iterator;
    }

    Iterator &operator--()
    {
      BLI_assert(current_ != nullptr);
      current_ = current_->prev;
      return *this;
    }

    Iterator operator--(int)
    {
      Iterator iterator = *this;
      --*this;
      return iterator;
    }

    bool operator!=(const Iterator &iterator) const
    {
      return current_ != iterator.current_;
    }

    bool operator==(const Iterator &iterator) const
    {
      return current_ == iterator.current_;
    }

    Node *operator*() const
    {
      return current_;
    }
  };

  class PairIterator {
   private:
    std::pair<Iterator, Iterator> pair_;

   public:
    PairIterator() : pair_(nullptr, nullptr)
    {
    }

    PairIterator(Node *first, Node *second) : pair_(first, second)
    {
    }

    PairIterator(Iterator first, Iterator second) : pair_(first, second)
    {
    }

    static PairIterator next(const PairIterator &it)
    {
      return PairIterator(Iterator::next(it.pair_.first), Iterator::next(it.pair_.second));
    }

    static PairIterator prev(const PairIterator &it)
    {
      return PairIterator(Iterator::prev(it.pair_.first), Iterator::prev(it.pair_.second));
    }

    PairIterator &operator++()
    {
      pair_.first++;
      pair_.second++;
      return *this;
    }

    bool operator!=(const PairIterator &it) const
    {
      return pair_.first != it.pair_.first && pair_.second != it.pair_.second;
    }

    std::pair<Node *, Node *> operator*() const
    {
      return std::pair(*pair_.first, *pair_.second);
    }

    PairIterator &incr_first()
    {
      pair_.first++;
      return *this;
    }

    PairIterator &incr_second()
    {
      pair_.second++;
      return *this;
    }

    void set_first(Node *node)
    {
      pair_.first = Iterator(node);
    }

    void set_second(Node *node)
    {
      pair_.second = Iterator(node);
    }

    void set_first(Iterator &it)
    {
      pair_.first = Iterator(it);
    }

    void set_second(Iterator &it)
    {
      pair_.second = Iterator(it);
    }
  };

  uint size() const
  {
    return size_;
  }

  bool empty() const
  {
    return size_ == 0;
  }

  Node *front() const
  {
    return head;
  }

  Node *back() const
  {
    return tail;
  }

  Iterator begin() const
  {
    return Iterator(head);
  }

  Iterator end() const
  {
    return Iterator(nullptr);
  }

  PairIterator begin_pair() const
  {
    if (size_ > 1) {
      return PairIterator(head, head->next);
    }
    return PairIterator();
  }

  PairIterator end_pair() const
  {
    if (size_ > 0) {
      return PairIterator(tail, nullptr);
    }
    return PairIterator();
  }

  friend std::ostream &operator<<(std::ostream &stream, const LinkedChain<T> &l)
  {
    stream << "[";
    for (auto elem : l) {
      stream << elem->data << ", ";
    }
    stream << "]";
    return stream;
  }

  Node *search(const T &data);
  Node *insert_front(const T &data);
  Node *insert_back(const T &data);
  Node *insert_after(Node *insert_node, const T &data);
  Node *insert_before(Node *insert_node, const T &data);
  void link(Node *nodeA, Node *nodeB);
  void remove(Node *node);
  void link_ends();
  void unlink_ends();

 private:
  uint size_;

  Node *head;
  Node *tail;
};

typedef LinkedChain<double2> ClipPath;
typedef std::list<ClipPath> ClipPaths;

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
typedef std::pair<double2, double2> Edge;

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

class PolyclipBentleyOttmann {
 public:
  PolyclipBentleyOttmann()
  {
  }

  struct Edge {
    Edge() = default;
    Edge(ClipPath::Node *first,
         ClipPath::Node *second,
         const std::pair<double2, bool> *sweep_pt,
         bool x_dir = true)
        : first(first), second(second), sweep_pt(sweep_pt), x_dir(x_dir)
    {
      sweep_node = x_dir ? first : second;
    }

    Edge(std::pair<ClipPath::Node *, ClipPath::Node *> node_pair,
         const std::pair<double2, bool> *sweep_pt,
         bool x_dir = true)
        : first(node_pair.first), second(node_pair.second), sweep_pt(sweep_pt), x_dir(x_dir)
    {
      sweep_node = x_dir ? first : second;
    }

    ClipPath::Node *first;
    ClipPath::Node *second;
    /* Pointer to node after which the next intersection should be inserted. */
    ClipPath::Node *sweep_node;
    /* Pointer to the sweep pt, shared between Edge instances. */
    const std::pair<double2, bool> *sweep_pt;
    /* True = +X; False = -X*/
    bool x_dir;

    static double y_intercept(const Edge &edge, const double x);
    friend bool operator<(const Edge &e1, const Edge &e2);

    friend bool operator==(const Edge &e1, const Edge &e2)
    {
      return double2::compare_limit(e1.first->data, e2.first->data, FLT_EPSILON) &&
             double2::compare_limit(e1.second->data, e2.second->data, FLT_EPSILON);
    }

    friend std::ostream &operator<<(std::ostream &stream, const Edge &e)
    {
      if (e.x_dir) {
        return stream << e.first->data << "--" << e.second->data;
      }
      return stream << e.second->data << "--" << e.first->data;
    }
  };

  struct Event {
    /* Order is important here since it dictates in what Order Events with the same coordinates
     * get handled. */
    enum Type { EMPTY = 0, START = 1, END = 2, INTERSECTION = 3 };

    Event()
    {
      type = EMPTY;
    }

    Event(const double2 point, const Edge edge, Type type) : pt(point), edge(edge), type(type)
    {
    }

    Event(const double2 isect_point, const Edge edge, const Edge isect_edge)
        : pt(isect_point), edge(edge), isect_edge(isect_edge)
    {
      type = INTERSECTION;
    }

    double2 pt;
    Edge edge;
    std::optional<Edge> isect_edge;
    Type type;

    friend bool operator==(const Event &a, const Event &b)
    {
      return double2::compare_limit(a.pt, b.pt, FLT_EPSILON) && a.type == b.type;
    }

    friend bool operator>(const Event &e1, const Event &e2)
    {
      if (IS_EQ(e1.pt.x, e2.pt.x)) {
        if (IS_EQ(e1.pt.y, e2.pt.y)) {
          return e1.type < e2.type;
        }
        return e1.pt.y > e2.pt.y;
      }
      return e1.pt.x > e2.pt.x;
    };

    friend std::ostream &operator<<(std::ostream &stream, const Event &e)
    {
      const char *type_name[] = {"EMPTY", "START", "END", "INTERSECTION"};
      if (e.type != INTERSECTION) {
        return stream << "Event(" << type_name[e.type] << ") at " << e.pt << ", " << e.edge;
      }
      return stream << "Event(" << type_name[e.type] << ") at " << e.pt << ", " << e.edge << " x "
                    << e.isect_edge.value();
    }
  };

  ClipPath find_intersections(const PointList &list);

 private:
  std::priority_queue<Event, std::deque<Event>, std::greater<Event>> event_queue;
  std::set<Edge> sweep_line_edges;

  std::pair<double2, bool> sweep_pt;

  Event check_edge_edge_isect(const Edge &e1, const Edge &e2);
};

ClipPath point_list_find_intersections_brute_force(const PointList &list);
PointList clip_path_get_outer_boundary(ClipPath &path);

Polyline polyline_offset(Polyline &pline,
                         const uint subdivisions,
                         const double pline_radius,
                         CapType start_cap_t,
                         CapType end_cap_t);

/* For Debugging only */

// void debug_output(std::ostream &stream) {
//   std::cout << stream;
// }

} /* namespace blender::polyclip */