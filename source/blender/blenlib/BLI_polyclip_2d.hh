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

  LinkedChain(LinkedChain<T> &src);
  LinkedChain(const std::list<T> &list);

  ~LinkedChain();

  LinkedChain<T> &operator=(LinkedChain<T> src);

  struct Node {
    T data;
    Node *next;
    Node *prev;
    Node *link;
    bool visited = false;

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

    static Iterator next(const Iterator &it, uint i)
    {
      Iterator new_it = Iterator(it);
      while (i > 0) {
        new_it++;
        --i;
      }
      return new_it;
    }

    static Iterator prev(const Iterator &it, uint i)
    {
      Iterator new_it = Iterator(it);
      while (i > 0) {
        new_it--;
        --i;
      }
      return new_it;
    }

    Iterator &operator++()
    {
      if (current_ != nullptr)
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
      if (current_ != nullptr)
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

    bool operator==(const PairIterator &it) const
    {
      return pair_.first == it.pair_.first && pair_.second == it.pair_.second;
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
  Node *insert_link(Node *insert_node, const T &data);
  Node *insert_front(const T &data);
  Node *insert_back(const T &data);
  Node *insert_after(Node *insert_node, const T &data);
  Node *insert_before(Node *insert_node, const T &data);
  void link(Node *nodeA, Node *nodeB);
  void remove(Node *node);
  void link_ends();
  void unlink_ends();
  void shift_head(Node *nodeA);
  void shift_head(Iterator &it);
  void set_front(Node *node);
  void set_back(Node *node);
  void clear_visited();

 private:
  uint size_;

  Node *head;
  Node *tail;
};

using ClipPath = LinkedChain<double2>;
using ClipPaths = std::list<ClipPath>;

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

using VertList = std::list<Vert>;
using PointList = std::list<double2>;
using Edge = std::pair<double2, double2>;

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

using Polylines = std::list<Polyline>;

struct Polygon {
  Polyline contour;
  Polylines holes;
};

class PolyclipBruteForce {
 public:
  PolyclipBruteForce()
  {
  }

  ClipPath find_intersections(const PointList &list);
};

class PolyclipBentleyOttmann {
 public:
  PolyclipBentleyOttmann()
  {
  }

  struct Edge {
    ClipPath::Node *first;
    ClipPath::Node *second;
    /* Pointer to node after which the next intersection should be inserted. */
    ClipPath::Node *sweep_node;
    /* Pointer to the sweep pt, shared between Edge instances. */
    const std::pair<double2, bool> *sweep_pt;
    /* True = +X; False = -X*/
    bool x_dir;

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
    /* Order is important here since it dictates in what order events with the same coordinates
     * get handled. */
    enum Type { EMPTY = 0, START = 1, END = 2, INTERSECTION = 3 };
    double2 pt;
    Edge edge;
    std::optional<Edge> isect_edge;
    Type type;

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

class PolyclipParkShin {
 public:
  PolyclipParkShin()
  {
  }

  /* Forward declaration*/
  struct MonotoneChain;

  struct Point {
    std::pair<std::shared_ptr<MonotoneChain>, std::shared_ptr<MonotoneChain>> isect_chains;
    double2 pt;

    Point(double2 pt) : pt(pt)
    {
    }

    friend std::ostream &operator<<(std::ostream &stream, const Point &p)
    {
      return stream << p.pt;
    }
  };

  using ClipPath = LinkedChain<Point>;

  struct MonotoneChain {
    ClipPath::Node *begin;
    ClipPath::Node *end;
    ClipPath::Node *front;
    const std::pair<double2, bool> *sweep_pt;
    bool x_dir;

    MonotoneChain(ClipPath::Node *begin,
                  ClipPath::Node *end,
                  const std::pair<double2, bool> *sweep_pt,
                  bool x_dir)
        : begin(begin), end(end), sweep_pt(sweep_pt), x_dir(x_dir)
    {
      front = begin;
    }

    void advance_front()
    {
      if (front != end) {
        front = x_dir ? front->next : front->prev;
      }
    }

    double y_intercept() const
    {
      double x = sweep_pt->first.x, y = sweep_pt->first.y;
      double2 start = x_dir ? front->prev->data.pt : front->next->data.pt;
      double2 end = front->data.pt;

      if (x < start.x) {
        return start.y;
      }
      if (x > end.x) {
        return end.y;
      }

      double x_dist = end.x - start.x;
      if (IS_EQ(x_dist, 0.0)) {
        double sweep_y = y;
        if (end.y < start.y) {
          CLAMP(sweep_y, end.y, start.y);
        }
        else {
          CLAMP(sweep_y, start.y, end.y);
        }
        return sweep_y;
      }

      if (IS_EQ(x, start.x)) {
        return start.y;
      }
      if (IS_EQ(x, end.x)) {
        return end.y;
      }

      double dx0 = x - start.x;
      double dx1 = end.x - x;

      double fac, ifac;
      if (dx0 > dx1) {
        ifac = dx0 / x_dist;
        fac = 1.0 - ifac;
      }
      else {
        fac = dx1 / x_dist;
        ifac = 1.0 - fac;
      }

      return start.y * fac + end.y * ifac;
    }

    /* Greater operator used to sort chains by their front node. */
    friend bool operator>(const MonotoneChain &m1, const MonotoneChain &m2)
    {
      double2 fm1 = m1.front->data.pt;
      double2 fm2 = m2.front->data.pt;
      if (!double2::compare_limit(fm1, fm2, FLT_EPSILON)) {
        return double2::compare_less(fm1, fm2);
      }

      double2 e1_start, e1_end, e2_start, e2_end;
      if (m1.front != m1.end) {
        e1_start = m1.front->data.pt;
        e1_end = m1.x_dir ? m1.front->next->data.pt : m1.front->prev->data.pt;
      }
      else {
        e1_start = m1.x_dir ? m1.front->prev->data.pt : m1.front->next->data.pt;
        e1_end = m1.front->data.pt;
      }

      if (m2.front != m2.end) {
        e2_start = m2.front->data.pt;
        e2_end = m2.x_dir ? m2.front->next->data.pt : m2.front->prev->data.pt;
      }
      else {
        e2_start = m2.x_dir ? m2.front->prev->data.pt : m2.front->next->data.pt;
        e2_end = m2.front->data.pt;
      }

      double e1_slope = double2::slope(e1_start, e1_end);
      double e2_slope = double2::slope(e2_start, e2_end);

      if (!IS_EQ(e1_slope, e2_slope)) {
        if (e1_slope > 0 && e2_slope > 0) {
          return e1_slope > e2_slope;
        }
        else if (e1_slope < 0 && e2_slope < 0) {
          return std::abs(e1_slope) > std::abs(e2_slope);
        }
        return e1_slope > e2_slope;
      }

      return double2::compare_less(e1_end, e2_end);
    }

    /* Less operator used to sort chains by the current front y values and by the slope in case the
     * y coodrinate is the same. */
    friend bool operator<(const MonotoneChain &m1, const MonotoneChain &m2)
    {
      double y_m1 = m1.y_intercept();
      double y_m2 = m2.y_intercept();

      if (!IS_EQ(y_m1, y_m2)) {
        return y_m1 > y_m2;
      }

      bool is_before = m1.sweep_pt->second;

      double2 e1_start = m1.x_dir ? m1.front->prev->data.pt : m1.front->next->data.pt;
      double2 e1_end = m1.front->data.pt;
      double2 e2_start = m2.x_dir ? m2.front->prev->data.pt : m2.front->next->data.pt;
      double2 e2_end = m2.front->data.pt;

      double e1_slope = double2::slope(e1_start, e1_end);
      double e2_slope = double2::slope(e2_start, e2_end);

      if (!IS_EQ(e1_slope, e2_slope)) {
        // std::cout << "e1_slope: " << e1_slope << ", e2_slope: " << e2_slope << std::endl;
        if (is_before) {
          return e1_slope < e2_slope;
        }
        else {
          return e1_slope > e2_slope;
        }
      }

      if (is_before) {
        return double2::compare_less(e1_start, e2_start);
      }
      return double2::compare_less(e1_end, e2_end);
    }

    friend std::ostream &operator<<(std::ostream &stream, const MonotoneChain &m)
    {
      stream << "(" << &m << ") [";
      // ClipPath::Node *it = m.begin;
      // while (it != m.end) {
      //   if (it == m.front) {
      //     stream << "{" << it->data << "} ";
      //   }
      //   else {
      //     stream << it->data << " ";
      //   }

      //   it = m.x_dir ? it->next : it->prev;
      // }
      // if (it == m.front) {
      //   stream << "{" << it->data << "} ";
      // }
      // else {
      //   stream << it->data << " ";
      // }
      if (m.x_dir) {
        if (m.front->prev != nullptr) {
          if (m.front->prev->prev != nullptr) {
            stream << m.front->prev->prev->data.pt << " ";
          }
          else {
            stream << "(nil) ";
          }
          stream << m.front->prev->data.pt << " ";
        }
        else {
          stream << "(nil) (nil) ";
        }
        stream << "{" << m.front->data.pt << "} ";
        if (m.front->next != nullptr) {
          if (m.front->next->next != nullptr) {
            stream << m.front->next->next->data.pt << " ";
          }
          else {
            stream << "(nil) ";
          }
          stream << m.front->next->data.pt << " ";
        }
        else {
          stream << "(nil) (nil) ";
        }
      }
      else {
        if (m.front->next != nullptr) {
          if (m.front->next->next != nullptr) {
            stream << m.front->next->next->data.pt << " ";
          }
          else {
            stream << "(nil) ";
          }
          stream << m.front->next->data.pt << " ";
        }
        else {
          stream << "(nil) (nil) ";
        }
        stream << "{" << m.front->data.pt << "} ";
        if (m.front->prev != nullptr) {
          if (m.front->prev->prev != nullptr) {
            stream << m.front->prev->prev->data.pt << " ";
          }
          else {
            stream << "(nil) ";
          }
          stream << m.front->prev->data.pt << " ";
        }
        else {
          stream << "(nil) (nil) ";
        }
      }

      stream << "]";
      return stream;
    }
  };

  void print_mono_chains()
  {
    if (mono_chains.empty()) {
      std::cout << "[]" << std::endl;
      return;
    }
    for (auto m : mono_chains) {
      std::cout << *m.get() << std::endl;
    }
  }

  void print_active_queue()
  {
    std::cout << "Active chains (" << active_chain_queue.size() << "): " << std::endl;
    if (active_chain_queue.empty()) {
      std::cout << "[]" << std::endl;
      return;
    }
    for (auto m : active_chain_queue) {
      std::cout << *m.get() << std::endl;
    }
  }

  void print_sweep_chains()
  {
    std::cout << "Sweep chains (" << sweep_line_chains.size() << "): " << std::endl;
    if (sweep_line_chains.empty()) {
      std::cout << "[]" << std::endl;
      return;
    }
    for (auto m : sweep_line_chains) {
      std::cout << *m.get() << std::endl;
    }
  }

  void add_monotone_chains_from_point_list(const PointList &list);
  polyclip::ClipPath find_intersections();

 private:
  PolyclipParkShin::ClipPath clip_path_from_point_list(const PointList &list);
  polyclip::ClipPath convert_clip_path();

  PolyclipParkShin::ClipPath::Node *find_intersection_mono_chains(
      const std::shared_ptr<MonotoneChain> &pm1, const std::shared_ptr<MonotoneChain> &pm2);
  bool isect_and_update_chains(std::set<std::shared_ptr<MonotoneChain>>::iterator &it1,
                               std::set<std::shared_ptr<MonotoneChain>>::iterator &it2);
  static bool comp_chain_greater(std::shared_ptr<MonotoneChain> m1,
                                 std::shared_ptr<MonotoneChain> m2)
  {
    bool res = *(m1.get()) > *(m2.get());
    // std::cout << *(m1.get()) << (res ? " > " : " < ") << *(m2.get()) << std::endl;
    return res;
  };

  static bool comp_chain_less(std::shared_ptr<MonotoneChain> m1, std::shared_ptr<MonotoneChain> m2)
  {
    bool res = *(m1.get()) < *(m2.get());
    // std::cout << *(m1.get()) << (res ? " > " : " < ") << *(m2.get()) << std::endl;
    return res;
  };

  PolyclipParkShin::ClipPath clip_path;
  std::pair<double2, bool> sweep_pt;

  std::list<std::shared_ptr<MonotoneChain>> mono_chains;
  std::multiset<std::shared_ptr<MonotoneChain>, decltype(&comp_chain_greater)> active_chain_queue{
      comp_chain_greater};
  std::set<std::shared_ptr<MonotoneChain>, decltype(&comp_chain_less)> sweep_line_chains{
      comp_chain_less};
};

ClipPath point_list_find_intersections_brute_force(const PointList &list);
PointList clip_path_get_outer_boundary(ClipPath &path);

ClipPath find_intersections(const PointList &list, uint method);

Polyline polyline_offset_default(Polyline &pline,
                                 const uint subdivisions,
                                 const double factor,
                                 const double pline_radius,
                                 CapType start_cap_t,
                                 CapType end_cap_t);

Polyline polyline_offset(Polyline &pline,
                         const uint subdivisions,
                         const double factor,
                         const double pline_radius,
                         uint method,
                         CapType start_cap_t,
                         CapType end_cap_t);

/* For Debugging only */

// void debug_output(std::ostream &stream) {
//   std::cout << stream;
// }

} /* namespace blender::polyclip */