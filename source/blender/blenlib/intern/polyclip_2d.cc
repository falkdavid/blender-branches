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

/** \file
 * \ingroup bli
 */

#include <algorithm>
#include <iostream>

#include "BLI_double2.hh"
#include "BLI_map.hh"
#include "BLI_vector.hh"

#include "BLI_polyclip_2d.hh"

#ifdef WITH_CLIPPER
#  include "clipper.hpp"
#endif

extern "C" {
#include "BLI_polyclip_2d.h"
}

#define MIN_MITER_LENGTH 0.0001
#define CUT_OFF_ANGLE 0.0001

namespace blender::polyclip {

/* -------------------------------------------------------------------- */
/** \name LinkedChain implementation
 * \{ */

/**
 * Create a LinkedChain from a std::list.
 */
template<typename T> LinkedChain<T>::LinkedChain(const std::list<T> &list)
{
  this->head = nullptr;
  this->tail = nullptr;
  this->size_ = 0;

  for (auto elem : list) {
    this->insert_back(elem);
  }
}

/**
 * Copy a LinkedChain.
 */
template<typename T> LinkedChain<T>::LinkedChain(LinkedChain<T> &src)
{
  this->head = nullptr;
  this->tail = nullptr;
  this->size_ = 0;

  std::unordered_map<Node *, Node *> visited_map;
  std::queue<Node *> queue;

  Node *front_src = src.front();
  queue.push(front_src);

  Node *front_dst = this->insert_front(front_src->data);
  visited_map[front_src] = front_dst;

  while (!queue.empty()) {
    Node *src_elem = queue.front();
    Node *dst_elem = visited_map[src_elem];

    if (src_elem->prev != nullptr) {
      auto it = visited_map.find(src_elem->prev);
      if (it == visited_map.end()) {
        Node *src_prev = src_elem->prev;
        Node *dst_prev = this->insert_before(dst_elem, src_prev->data);
        visited_map[src_prev] = dst_prev;
        queue.push(src_prev);
      }
      else if (dst_elem->prev == nullptr) {
        dst_elem->prev = it->second;
      }
    }
    if (src_elem->next != nullptr) {
      auto it = visited_map.find(src_elem->next);
      if (it == visited_map.end()) {
        Node *src_next = src_elem->next;
        Node *dst_next = this->insert_after(dst_elem, src_next->data);
        visited_map[src_next] = dst_next;
        queue.push(src_next);
      }
      else if (dst_elem->next == nullptr) {
        dst_elem->next = it->second;
      }
    }
    if (src_elem->link != nullptr) {
      auto it = visited_map.find(src_elem->link);
      if (it == visited_map.end()) {
        Node *src_link = src_elem->link;
        Node *dst_link = this->insert_link(dst_elem, src_link->data);
        visited_map[src_link] = dst_link;
        queue.push(src_link);
      }
      else if (dst_elem->link == nullptr) {
        dst_elem->link = it->second;
      }
    }

    queue.pop();
  }

  this->head = visited_map[src.front()];
  this->tail = visited_map[src.back()];
}

template<typename T> LinkedChain<T>::~LinkedChain()
{
  Node *node, *next_node;
  for (node = this->head; node != nullptr; node = next_node) {
    next_node = node->next;
    remove(node);
  }
}

template<typename T> LinkedChain<T> &LinkedChain<T>::operator=(LinkedChain<T> src)
{
  std::swap(src.head, this->head);
  std::swap(src.tail, this->tail);
  this->size_ = src.size_;
  return *this;
}

/**
 * Linear search for data in the list.
 * \param data: Data to search for.
 * \returns: Pointer to the TLNode or `nullptr` if the data was not found.
 */
template<typename T> typename LinkedChain<T>::Node *LinkedChain<T>::search(const T &data)
{
  Node *node;
  for (auto it : *this) {
    if (it->data == data) {
      node = it;
      break;
    }
  }

  return node;
}

/**
 * Inserts data as a link node to "insert_node".
 * \param insert_node: Pointer to the node that will be the link of the new node.
 * \param data: The data to insert.
 * \returns: Pointer to the inserted node.
 */
template<typename T>
typename LinkedChain<T>::Node *LinkedChain<T>::insert_link(Node *insert_node, const T &data)
{
  BLI_assert(insert_node != nullptr);
  Node *node = new Node(data);
  node->link = insert_node;
  insert_node->link = node;

  this->size_++;
  return node;
}

/**
 * Inserts data after "insert_node".
 * \param insert_node: Pointer to the node to insert the data after.
 * \param data: The data to insert.
 * \returns: Pointer to the inserted node.
 */
template<typename T>
typename LinkedChain<T>::Node *LinkedChain<T>::insert_after(Node *insert_node, const T &data)
{
  BLI_assert(insert_node != nullptr);
  Node *node = new Node(data);
  node->prev = insert_node;
  if (this->tail == insert_node || insert_node->next == nullptr) {
    this->tail = node;
  }
  else {
    node->next = insert_node->next;
    insert_node->next->prev = node;
  }
  insert_node->next = node;

  this->size_++;
  return node;
}

/**
 * Inserts data before "insert_node".
 * \param insert_node: Pointer to the node to insert the data before.
 * \param data: The data to insert.
 * \returns: Pointer to the inserted node.
 */
template<typename T>
typename LinkedChain<T>::Node *LinkedChain<T>::insert_before(Node *insert_node, const T &data)
{
  BLI_assert(insert_node != nullptr);
  Node *node = new Node(data);
  node->next = insert_node;
  if (this->head == insert_node || insert_node->prev == nullptr) {
    this->head = node;
  }
  else {
    node->prev = insert_node->prev;
    insert_node->prev->next = node;
  }
  insert_node->prev = node;

  this->size_++;
  return node;
}

/**
 * Inserts data at the beginning of the list.
 * \param data: The data to insert.
 * \returns: Pointer to the inserted node.
 */
template<typename T> typename LinkedChain<T>::Node *LinkedChain<T>::insert_front(const T &data)
{
  if (this->head == nullptr) {
    Node *node = new Node(data);
    this->head = node;
    this->tail = node;
    this->size_++;
    return node;
  }
  return insert_before(this->head, data);
}

/**
 * Inserts data at the end of the list.
 * \param data: The data to insert.
 * \returns: Pointer to the inserted node.
 */
template<typename T> typename LinkedChain<T>::Node *LinkedChain<T>::insert_back(const T &data)
{
  if (this->tail == nullptr) {
    Node *node = new Node(data);
    this->head = node;
    this->tail = node;
    this->size_++;
    return node;
  }
  return insert_after(this->tail, data);
}

/**
 * Links the two nodes together.
 * \param nodeA: First node.
 * \param nodeB: Second node.
 */
template<typename T> void LinkedChain<T>::link(Node *nodeA, Node *nodeB)
{
  if (nodeA == nullptr || nodeB == nullptr) {
    return;
  }

  nodeA->link = nodeB;
  nodeB->link = nodeA;
}

/**
 * Removes the node from the list and deletes it.
 * \param node: The node to be removed and deleted.
 */
template<typename T> void LinkedChain<T>::remove(Node *node)
{
  if (node == nullptr) {
    return;
  }

  if (this->head == node || node->prev == nullptr) {
    this->head = node->next;
  }
  else {
    node->prev->next = node->next;
  }
  if (this->tail == node || node->next == nullptr) {
    this->tail = node->prev;
  }
  else {
    node->next->prev = node->prev;
  }

  node->prev = nullptr;
  node->next = nullptr;

  delete node;
  this->size_--;
}

/**
 * Connects first and last element of the chain togther to form a loop.
 */
template<typename T> void LinkedChain<T>::link_ends()
{
  if (size_ < 2) {
    return;
  }

  head->prev = tail;
  tail->next = head;
}

/**
 * Deconnects the first and last element of the chain.
 */
template<typename T> void LinkedChain<T>::unlink_ends()
{
  if (size_ < 2) {
    return;
  }

  head->prev = nullptr;
  tail->next = nullptr;
}
/* \} */

/**
 * Sets the given node as the head of the LinkedChain.
 */
template<typename T> void LinkedChain<T>::shift_head(LinkedChain<T>::Node *nodeA)
{
  if (this->head == nodeA) {
    return;
  }
  this->link_ends();
  this->head = nodeA;
  this->tail = nodeA->prev;
  this->unlink_ends();
}

/**
 * Sets the node pointed to by a given iterator as the head of the LinkedChain.
 */
template<typename T> void LinkedChain<T>::shift_head(LinkedChain<T>::Iterator &it)
{
  this->shift_head(*it);
}

template<typename T> void LinkedChain<T>::set_front(LinkedChain<T>::Node *node)
{
  this->head = node;
}

template<typename T> void LinkedChain<T>::set_back(LinkedChain<T>::Node *node)
{
  this->tail = node;
}

/**
 * Clear the visited mark for all nodes.
 *
 */
template<typename T> void LinkedChain<T>::clear_visited()
{
  for (auto elem : *this) {
    elem->visited = false;
  }
}

template class LinkedChain<double2>;

/* -------------------------------------------------------------------- */
/** \name Find outer boundary
 * \{ */

PointList clip_path_get_outer_boundary(ClipPath &path)
{
  BLI_assert(path.size() > 1);
  PointList outer;

  /* Find the left-most point (x minimum). Break ties with the y minimum. */
  auto min_elem = std::min_element(
      path.begin(), path.end(), [](const ClipPath::Node *a, const ClipPath::Node *b) {
        if (a->data.x == b->data.x) {
          return a->data.y < b->data.y;
        }
        return a->data.x < b->data.x;
      });

  /* Connect the ends of the path to form a loop. */
  path.link_ends();

  /* Figure out if the outside is to the left or right of the first edge. */
  double2 current_pt = (*min_elem)->data;
  double2 next_pt = (*min_elem)->next->data;
  double2 prev_pt = (*min_elem)->prev->data;
  int orient = double2::orientation(current_pt, next_pt, prev_pt) > 0 ? -1 : 1;

  /* Keep track of the direction of iteration in the linked list, e.g. forward or backward. Always
   * start iterating clockwise so that the edge-direction of the boundary is also clockwise. */
  bool forward = (orient == 1);

  outer.push_back(current_pt);
  auto it = (forward == true) ? ClipPath::Iterator::next(min_elem) :
                                ClipPath::Iterator::prev(min_elem);
  while (it != min_elem && !(*it)->visited) {
    auto vert = *it;
    outer.push_back(vert->data);
    vert->visited = true;

    /* At intersection point, always iterate towards the outside. */
    if (vert->link != nullptr) {
      current_pt = vert->data;
      next_pt = (forward == true) ? vert->next->data : vert->prev->data;
      double2 link_next_pt = vert->link->next->data;
      forward = (double2::orientation(current_pt, next_pt, link_next_pt) == 1);

      it.switch_link();
    }

    if (forward) {
      ++it;
    }
    else {
      --it;
    }
  }

  /* Avoid infinite loops by disconnecting the ends. */
  path.unlink_ends();

  return outer;
}

/* \} */

/* -------------------------------------------------------------------- */
/** \name Polyline clipping
 * \{ */
/* Brute force algorithm */

ClipPath PolyclipBruteForce::find_intersections(const PointList &list)
{
  ClipPath path = ClipPath(list);

  /* Iterate over all pairs of edges (edgeA, edgeB). */
  auto it_edgeA = path.begin_pair();
  while (it_edgeA != path.end_pair()) {
    auto node_edgeA = *it_edgeA;

    auto it_edgeB = ClipPath::PairIterator::next(it_edgeA);
    while (it_edgeB != path.end_pair()) {
      auto node_edgeB = *it_edgeB;
      Edge A = Edge(node_edgeA.first->data, node_edgeA.second->data);
      Edge B = Edge(node_edgeB.first->data, node_edgeB.second->data);

      auto result = double2::isect_seg_seg(A.first, A.second, B.first, B.second);
      if (result.kind == result.LINE_LINE_CROSS) {
        double2 isect_pt = double2::interpolate(A.first, A.second, result.lambda);

        ClipPath::Node *nodeA = path.insert_after(node_edgeA.first, isect_pt);
        ClipPath::Node *nodeB = path.insert_after(node_edgeB.first, isect_pt);
        path.link(nodeA, nodeB);

        /* Shrink edge A so it is no longer a split edge. */
        it_edgeA.set_second(nodeA);
        node_edgeA = *it_edgeA;

        /* Go to the next edge B by skipping over the insersection points */
        it_edgeB.set_first(node_edgeB.second);
        it_edgeB.incr_second();
      }
      else {
        ++it_edgeB;
      }
    }

    ++it_edgeA;
  }

  return path;
}

/* -------------------------------------------------------------------- */
/* Bentley-Ottmann algorithm */

double PolyclipBentleyOttmann::Edge::y_intercept(const Edge &edge, const double x)
{
  double2 edge_start = edge.x_dir ? edge.first->data : edge.second->data;
  double2 edge_end = edge.x_dir ? edge.second->data : edge.first->data;
  double start_x = edge_start.x;
  double start_y = edge_start.y;
  double end_x = edge_end.x;
  double end_y = edge_end.y;

  if (x < start_x) {
    return start_y;
  }
  if (x > end_x) {
    return end_y;
  }

  double x_dist = end_x - start_x;
  if (IS_EQ(x_dist, 0.0)) {
    double sweep_y = edge.sweep_pt->first.y;
    if (end_y < start_y) {
      SWAP(double, end_y, start_y);
    }
    CLAMP(sweep_y, start_y, end_y);
    return sweep_y;
  }

  if (IS_EQ(x, start_x)) {
    return start_y;
  }
  if (IS_EQ(x, end_x)) {
    return end_y;
  }

  double dx0 = x - start_x;
  double dx1 = end_x - x;

  double fac, ifac;
  if (dx0 > dx1) {
    ifac = dx0 / x_dist;
    fac = 1.0 - ifac;
  }
  else {
    fac = dx1 / x_dist;
    ifac = 1.0 - fac;
  }

  return start_y * fac + end_y * ifac;
}

bool operator<(const PolyclipBentleyOttmann::Edge &e1, const PolyclipBentleyOttmann::Edge &e2)
{
  if (e1 == e2) {
    return false;
  }

  /* TODO: Add bounding box check for faster sorting */

  double2 sweep_pt = e1.sweep_pt->first;
  bool is_before = e1.sweep_pt->second;

  double e1_y_icept = PolyclipBentleyOttmann::Edge::y_intercept(e1, sweep_pt.x);
  double e2_y_icept = PolyclipBentleyOttmann::Edge::y_intercept(e2, sweep_pt.x);

  /* Compare edges at their y intercept. If the intercept of an edge is below another it is
   * considered smaller. */
  if (!IS_EQ(e1_y_icept, e2_y_icept)) {
    if (e1_y_icept < e2_y_icept) {
      return false;
    }
    if (e1_y_icept > e2_y_icept) {
      return true;
    }
  }

  /* If the y intercepts are equal, compare the slopes of the edges. */
  double2 e1_start = e1.x_dir ? e1.first->data : e1.second->data;
  double2 e1_end = e1.x_dir ? e1.second->data : e1.first->data;
  double2 e2_start = e2.x_dir ? e2.first->data : e2.second->data;
  double2 e2_end = e2.x_dir ? e2.second->data : e2.first->data;

  double e1_slope = double2::slope(e1_start, e1_end);
  double e2_slope = double2::slope(e2_start, e2_end);

  if (!IS_EQ(e1_slope, e2_slope)) {
    if (is_before) {
      if (e1_slope < e2_slope) {
        return true;
      }
      if (e1_slope > e2_slope) {
        return false;
      }
    }
    else {
      if (e1_slope < e2_slope) {
        return false;
      }
      if (e1_slope > e2_slope) {
        return true;
      }
    }
  }

  /* If the slopes are also equal, compare start and end points. */
  if (is_before && !double2::compare_limit(e1_start, e2_start, FLT_EPSILON)) {
    return double2::compare_less(e1_start, e2_start);
  }
  return double2::compare_less(e1_end, e2_end);
}

PolyclipBentleyOttmann::Event PolyclipBentleyOttmann::check_edge_edge_isect(
    const PolyclipBentleyOttmann::Edge &e1, const PolyclipBentleyOttmann::Edge &e2)
{
  auto result = double2::isect_seg_seg(
      e1.first->data, e1.second->data, e2.first->data, e2.second->data);
  if (result.kind == double2::isect_result::LINE_LINE_CROSS) {
    double2 isect_pt = double2::interpolate(e1.first->data, e1.second->data, result.lambda);
    if (!double2::compare_limit(isect_pt, e1.first->data, FLT_EPSILON) &&
        !double2::compare_limit(isect_pt, e1.second->data, FLT_EPSILON)) {
      return Event(isect_pt, e1, e2);
    }
  }
  return Event();
}

ClipPath PolyclipBentleyOttmann::find_intersections(const PointList &list)
{
  ClipPath clip_path = ClipPath(list);

  for (auto it = clip_path.begin_pair(); it != clip_path.end_pair(); ++it) {
    auto node_edge = *it;
    Event start_event, end_event;
    /* Make the edge point in +X (or +Y if vertical). */
    if (double2::compare_less(node_edge.first->data, node_edge.second->data) == true) {
      Edge edge = Edge(node_edge, &sweep_pt, true);
      start_event = Event(edge.first->data, edge, Event::START);
      end_event = Event(edge.second->data, edge, Event::END);
    }
    else {
      Edge edge = Edge(node_edge, &sweep_pt, false);
      start_event = Event(edge.second->data, edge, Event::START);
      end_event = Event(edge.first->data, edge, Event::END);
    }

    event_queue.push(start_event);
    event_queue.push(end_event);
  }

  while (!event_queue.empty()) {
    Event event = event_queue.top();
    event_queue.pop();

    // std::cout << event << std::endl;
    sweep_pt.first = event.pt;

    // auto print = [](const Edge &e) {
    //   std::cout << "\n"
    //             << e << " (" << PolyclipBentleyOttmann::Edge::y_intercept(e,
    //             e.sweep_pt->first.x)
    //             << ")";
    // };
    // std::cout << "Before sweep_line_edges: ";
    // std::for_each(sweep_line_edges.begin(), sweep_line_edges.end(), print);
    // std::cout << std::endl;

    if (event.type == Event::INTERSECTION) {
      /* Pop duplicates from the event queue. */
      while (event == event_queue.top()) {
        event_queue.pop();
      }

      BLI_assert(event.isect_edge.has_value());

      Edge event_isect_edge = event.isect_edge.value();

      /* Set before sweep point to true. */
      sweep_pt.second = true;

      auto itA = sweep_line_edges.extract(event.edge);
      auto itB = sweep_line_edges.extract(event_isect_edge);

      /* Fall back to a linear search. */
      if (itA.empty() || itB.empty()) {
        bool found_A = !itA.empty(), found_B = !itB.empty();
        for (auto e = sweep_line_edges.begin(); e != sweep_line_edges.end(); ++e) {
          if (!found_A && *e == event.edge) {
            itA = sweep_line_edges.extract(e);
            found_A = true;
          }
          if (!found_B && *e == event_isect_edge) {
            itB = sweep_line_edges.extract(e);
            found_B = true;
          }
          if (found_A && found_B) {
            break;
          }
        }
      }

      if (itA.empty() || itB.empty()) {
        continue;
        // std::cout << "Error: could not extract intersecting edges!";
      }

      // BLI_assert(!itA.empty() && !itB.empty());

      ClipPath::Node *isect_nodeA, *isect_nodeB;
      if (event.edge.x_dir) {
        isect_nodeA = clip_path.insert_after(itA.value().sweep_node, event.pt);
      }
      else {
        isect_nodeA = clip_path.insert_before(itA.value().sweep_node, event.pt);
      }
      if (event_isect_edge.x_dir) {
        isect_nodeB = clip_path.insert_after(itB.value().sweep_node, event.pt);
      }
      else {
        isect_nodeB = clip_path.insert_before(itB.value().sweep_node, event.pt);
      }
      clip_path.link(isect_nodeA, isect_nodeB);

      itA.value().sweep_node = isect_nodeA;
      itB.value().sweep_node = isect_nodeB;

      /* Set position to after. */
      sweep_pt.second = false;

      auto node_A = sweep_line_edges.insert(std::move(itA)).position;
      auto node_B = sweep_line_edges.insert(std::move(itB)).position;

      BLI_assert(node_A != node_B);

      if (std::next(node_B) == node_A) {
        auto next = std::next(node_A);
        auto prev = std::prev(node_B);

        if (next != sweep_line_edges.end()) {
          Event e = check_edge_edge_isect(*node_A, *next);
          if (e.type != Event::Type::EMPTY && e.pt.x >= sweep_pt.first.x) {
            event_queue.push(e);
            // std::cout << "Added Event: " << e << std::endl;
          }
        }

        if (prev != sweep_line_edges.end()) {
          Event e = check_edge_edge_isect(*node_B, *prev);
          if (e.type != Event::Type::EMPTY && e.pt.x >= sweep_pt.first.x) {
            event_queue.push(e);
            // std::cout << "Added Event: " << e << std::endl;
          }
        }
      }
      else if (std::next(node_A) == node_B) {
        auto next = std::next(node_B);
        auto prev = std::prev(node_A);

        if (next != sweep_line_edges.end()) {
          Event e = check_edge_edge_isect(*node_B, *next);
          if (e.type != Event::Type::EMPTY && e.pt.x >= sweep_pt.first.x) {
            event_queue.push(e);
            // std::cout << "Added Event: " << e << std::endl;
          }
        }

        if (prev != sweep_line_edges.end()) {
          Event e = check_edge_edge_isect(*node_A, *prev);
          if (e.type != Event::Type::EMPTY && e.pt.x >= sweep_pt.first.x) {
            event_queue.push(e);
            // std::cout << "Added Event: " << e << std::endl;
          }
        }
      }
    }
    else if (event.type == Event::START) {
      /* Set position to after. */
      sweep_pt.second = false;
      /* Insert the new edge into the sweep line set. */
      auto it = sweep_line_edges.insert(event.edge);

      if (it.second) {
        auto current = it.first;
        auto next = std::next(current);
        auto prev = std::prev(current);

        if (next != sweep_line_edges.end()) {
          Event e = check_edge_edge_isect(*current, *next);
          if (e.type != Event::Type::EMPTY) {
            event_queue.push(e);
            // std::cout << "Added Event: " << e << std::endl;
          }
        }

        if (current != sweep_line_edges.begin() && current != sweep_line_edges.end()) {
          Event e = check_edge_edge_isect(*current, *prev);
          if (e.type != Event::Type::EMPTY && e.pt.x >= sweep_pt.first.x) {
            event_queue.push(e);
            // std::cout << "Added Event: " << e << std::endl;
          }
        }
      }
      else {
        // std::cout << "Error: Event edge " << event.edge << " was not inserted!" << std::endl;
        BLI_assert(it.second == true);
      }
    }
    else if (event.type == Event::END) {
      /* Set position to before. */
      sweep_pt.second = true;
      /* Find the edge and remove it from the sweep line set. */
      auto it = sweep_line_edges.find(event.edge);

      /* Fall back to linear search if nessesary. */
      if (it == sweep_line_edges.end()) {
        for (auto e = sweep_line_edges.begin(); e != sweep_line_edges.end(); ++e) {
          if (*e == event.edge) {
            it = e;
            break;
          }
        }
      }

      if (it != sweep_line_edges.end()) {
        auto next = std::next(it);
        auto prev = std::prev(it);

        if (next != sweep_line_edges.end() && it != sweep_line_edges.begin()) {
          Event e = check_edge_edge_isect(*prev, *next);
          if (e.type != Event::Type::EMPTY && e.pt.x >= sweep_pt.first.x) {
            event_queue.push(e);
            // std::cout << "Added Event: " << e << std::endl;
          }
        }

        sweep_line_edges.erase(it);
      }
      else if (sweep_line_edges.size() == 1) {
        sweep_line_edges.erase(sweep_line_edges.begin());
      }
      else {
        // std::cout << "Error: Event edge " << event.edge << " could not be deleted!" <<
        // std::endl; BLI_assert(it != sweep_line_edges.end());
        continue;
      }
    }

    // std::cout << "After sweep_line_edges: ";
    // std::for_each(sweep_line_edges.begin(), sweep_line_edges.end(), print);
    // std::cout << std::endl;
  }

  return clip_path;
}

/* -------------------------------------------------------------------- */
/* Park & Shin (2001) algorithm */

PolyclipParkShin::ClipPath PolyclipParkShin::clip_path_from_point_list(const PointList &list)
{
  PolyclipParkShin::ClipPath clip_path;
  for (auto elem : list) {
    clip_path.insert_back(elem);
  }
  return clip_path;
}

polyclip::ClipPath PolyclipParkShin::convert_clip_path()
{
  polyclip::ClipPath out_clip_path;
  clip_path.clear_visited();

  std::unordered_map<PolyclipParkShin::ClipPath::Node *, polyclip::ClipPath::Node *> visited_map;
  std::queue<PolyclipParkShin::ClipPath::Node *> queue;

  PolyclipParkShin::ClipPath::Node *front_src = clip_path.front();
  queue.push(front_src);

  polyclip::ClipPath::Node *front_dst = out_clip_path.insert_front(front_src->data.pt);
  visited_map[front_src] = front_dst;

  while (!queue.empty()) {
    PolyclipParkShin::ClipPath::Node *src_elem = queue.front();
    polyclip::ClipPath::Node *dst_elem = visited_map[src_elem];

    if (src_elem->prev != nullptr) {
      auto it = visited_map.find(src_elem->prev);
      if (it == visited_map.end()) {
        PolyclipParkShin::ClipPath::Node *src_prev = src_elem->prev;
        polyclip::ClipPath::Node *dst_prev = out_clip_path.insert_before(dst_elem,
                                                                         src_prev->data.pt);
        visited_map[src_prev] = dst_prev;
        queue.push(src_prev);
      }
      else if (dst_elem->prev == nullptr) {
        dst_elem->prev = it->second;
      }
    }
    if (src_elem->next != nullptr) {
      auto it = visited_map.find(src_elem->next);
      if (it == visited_map.end()) {
        PolyclipParkShin::ClipPath::Node *src_next = src_elem->next;
        polyclip::ClipPath::Node *dst_next = out_clip_path.insert_after(dst_elem,
                                                                        src_next->data.pt);
        visited_map[src_next] = dst_next;
        queue.push(src_next);
      }
      else if (dst_elem->next == nullptr) {
        dst_elem->next = it->second;
      }
    }
    if (src_elem->link != nullptr) {
      auto it = visited_map.find(src_elem->link);
      if (it == visited_map.end()) {
        PolyclipParkShin::ClipPath::Node *src_link = src_elem->link;
        polyclip::ClipPath::Node *dst_link = out_clip_path.insert_link(dst_elem,
                                                                       src_link->data.pt);
        visited_map[src_link] = dst_link;
        queue.push(src_link);
      }
      else if (dst_elem->link == nullptr) {
        dst_elem->link = it->second;
      }
    }

    queue.pop();
  }

  out_clip_path.set_front(visited_map[clip_path.front()]);
  out_clip_path.set_back(visited_map[clip_path.back()]);

  return out_clip_path;
}

void PolyclipParkShin::add_monotone_chains_from_point_list(const PointList &list)
{
  clip_path = clip_path_from_point_list(list);
  auto min_elem = std::min_element(
      clip_path.begin(), clip_path.end(), [](const ClipPath::Node *a, const ClipPath::Node *b) {
        return (a->data.pt.x == b->data.pt.x) ? a->data.pt.y < b->data.pt.y :
                                                a->data.pt.x < b->data.pt.x;
      });
  clip_path.shift_head(min_elem);

  // std::cout << "Min Elem: " << (*min_elem)->data.pt << std::endl;
  // std::cout << clip_path << std::endl;

  bool x_dir = true;
  ClipPath::Node *start_node = *min_elem;
  ClipPath::Node *end_node = *min_elem;
  for (auto pair_it = clip_path.begin_pair(); pair_it != clip_path.end_pair(); ++pair_it) {
    auto pair = *pair_it;
    double2 current = pair.first->data.pt;
    double2 next = pair.second->data.pt;
    if (double2::compare_less(current, next) == x_dir || IS_EQ(current.x, next.x)) {
      end_node = pair.second;
    }
    else {
      mono_chains.push_back(std::make_shared<MonotoneChain>(MonotoneChain(
          x_dir ? start_node : end_node, x_dir ? end_node : start_node, &sweep_pt, x_dir)));

      x_dir = !x_dir;
      start_node = pair.first;
      end_node = pair.second;
    }
  }
  mono_chains.push_back(std::make_shared<MonotoneChain>(MonotoneChain(
      x_dir ? start_node : end_node, x_dir ? end_node : start_node, &sweep_pt, x_dir)));
}

PolyclipParkShin::ClipPath::Node *PolyclipParkShin::find_intersection_mono_chains(
    const std::shared_ptr<PolyclipParkShin::MonotoneChain> &pm1,
    const std::shared_ptr<PolyclipParkShin::MonotoneChain> &pm2)
{
  MonotoneChain m1 = *pm1.get();
  MonotoneChain m2 = *pm2.get();

  double2 e1_start = m1.x_dir ? m1.front->prev->data.pt : m1.front->next->data.pt;
  double2 e1_end = m1.front->data.pt;
  double2 e2_start = m2.x_dir ? m2.front->prev->data.pt : m2.front->next->data.pt;
  double2 e2_end = m2.front->data.pt;
  if (double2::compare_limit(e1_start, e2_start, FLT_EPSILON) ||
      double2::compare_limit(e1_end, e2_end, FLT_EPSILON) ||
      double2::compare_limit(e1_start, e2_end, FLT_EPSILON) ||
      double2::compare_limit(e1_end, e2_start, FLT_EPSILON)) {
    return nullptr;
  }

  auto result = double2::isect_seg_seg(e1_start, e1_end, e2_start, e2_end);
  if (result.kind == double2::isect_result::LINE_LINE_CROSS) {
    double2 isect_pt = double2::interpolate(e1_start, e1_end, result.lambda);
    if (!double2::compare_limit(isect_pt, e1_start, FLT_EPSILON) &&
        !double2::compare_limit(isect_pt, e1_end, FLT_EPSILON)) {

      auto m1_isect_node = m1.x_dir ? clip_path.insert_before(m1.front, isect_pt) :
                                      clip_path.insert_after(m1.front, isect_pt);
      auto m2_isect_node = m2.x_dir ? clip_path.insert_before(m2.front, isect_pt) :
                                      clip_path.insert_after(m2.front, isect_pt);
      clip_path.link(m1_isect_node, m2_isect_node);

      m1_isect_node->data.isect_chains = {pm1, pm2};
      m2_isect_node->data.isect_chains = {pm2, pm1};

      return m1_isect_node;
    }
  }
  return nullptr;
}

bool PolyclipParkShin::isect_and_update_chains(
    std::set<std::shared_ptr<PolyclipParkShin::MonotoneChain>>::iterator &it1,
    std::set<std::shared_ptr<PolyclipParkShin::MonotoneChain>>::iterator &it2)
{
  if (it1 != sweep_line_chains.end() && it2 != sweep_line_chains.end()) {
    auto isect_node = find_intersection_mono_chains(*it1, *it2);
    if (isect_node != nullptr) {
      auto node_it1 = active_chain_queue.extract(*it1);
      auto node_it2 = active_chain_queue.extract(*it2);

      /* Set front node. */
      it2->get()->front = isect_node->link;
      it1->get()->front = isect_node;

      /* Update chain in active chain queue. */
      active_chain_queue.insert(std::move(node_it1));
      active_chain_queue.insert(std::move(node_it2));
      return true;
    }
  }
  return false;
}

polyclip::ClipPath PolyclipParkShin::find_intersections()
{
  /* Add all monotone chains to the active chain queue. */
  for (auto chain : mono_chains) {
    active_chain_queue.emplace(chain);
  }

  // int i = 0;
  // std::cout << "Step: " << i << std::endl;
  // print_active_queue();
  // print_sweep_chains();

  /* Loop while there are active chains. */
  while (!active_chain_queue.empty()) {
    auto chain = active_chain_queue.extract(active_chain_queue.begin());
    auto current_chain = chain.value();
    ClipPath::Node *front_node = current_chain->front;

    sweep_pt.first = front_node->data.pt;

    // std::cout << "Step: " << ++i << std::endl;
    // std::cout << "Current chain: " << *chain.value().get() << std::endl;

    /* Begin event */
    if (front_node == current_chain->begin) {
      current_chain->advance_front();
      // std::cout << "INSERT" << std::endl;
      /* Set position to after. */
      sweep_pt.second = false;
      /* Insert chain into sweep_line_chains. Check if it intersects with prev or next. */
      auto current_node = sweep_line_chains.insert(current_chain);

      if (current_node.second) {
        auto current = current_node.first;
        auto next = std::next(current);
        auto prev = std::prev(current);

        isect_and_update_chains(current, next);
        isect_and_update_chains(current, prev);
      }
      else {
        // std::cout << *current_chain.get() << " could not be inserted!" << std::endl;
        // BLI_assert(current_node.second);
      }

      active_chain_queue.insert(std::move(chain));

      // print_sweep_chains();
    }
    /* End event */
    else if (front_node == current_chain->end) {
      current_chain->advance_front();
      // std::cout << "REMOVE" << std::endl;
      /* Set position to before. */
      sweep_pt.second = true;
      /* Remove chain from sweep_line_chains. Check if prev or next intersect each other. */
      auto current = sweep_line_chains.find(current_chain);

      /* Fall back to linear search. */
      if (current == sweep_line_chains.end()) {
        // std::cout << *current_chain.get() << " not found! Falling back to linear search!"
        //           << std::endl;
        // BLI_assert(current != sweep_line_chains.end());
        for (auto it = sweep_line_chains.begin(); it != sweep_line_chains.end(); ++it) {
          if (*it == current_chain) {
            current = it;
            break;
          }
        }
      }
      auto next = std::next(current);
      auto prev = std::prev(current);

      sweep_line_chains.erase(current);
      active_chain_queue.erase(current_chain);

      isect_and_update_chains(next, prev);

      // print_sweep_chains();
    }
    /* Intersection event */
    else if (front_node->link != nullptr) {
      // std::cout << "INTERSECT" << std::endl;

      /* Set position to before. */
      sweep_pt.second = true;

      /* Get intersecting chain. Advance its current node. Swap chains in sweep_line_chains. Check
       * if prev or next are intersecting. */
      auto current = sweep_line_chains.find(current_chain);
      /* Fall back to linear search. */
      if (current == sweep_line_chains.end()) {
        // std::cout << "current(" << *current_chain.get()
        //           << ") not found! Falling back to linear search!" << std::endl;
        // BLI_assert(current != sweep_line_chains.end());
        for (auto it = sweep_line_chains.begin(); it != sweep_line_chains.end(); ++it) {
          if (*it == current_chain) {
            current = it;
            break;
          }
        }
      }

      auto isect_chain = front_node->data.isect_chains.second;
      auto isect_it = sweep_line_chains.find(isect_chain);
      /* Fall back to linear search. */
      if (isect_it == sweep_line_chains.end()) {
        // std::cout << "isect_it(" << *isect_chain << ") not found! Falling back to linear
        // search!"
        //           << std::endl;
        // BLI_assert(isect_it != sweep_line_chains.end());
        for (auto it = sweep_line_chains.begin(); it != sweep_line_chains.end(); ++it) {
          if (*it == isect_chain) {
            isect_it = it;
            break;
          }
        }
      }

      /* Extract current node from sweep chains to swap position with isect chain. */
      auto chain_node = sweep_line_chains.extract(current);
      auto isect_chain_node = sweep_line_chains.extract(isect_it);

      // print_sweep_chains();

      /* Set position to after. */
      sweep_pt.second = false;

      current = sweep_line_chains.insert(std::move(chain_node)).position;
      isect_it = sweep_line_chains.insert(std::move(isect_chain_node)).position;

      // print_sweep_chains();

      current_chain->advance_front();

      auto active_isect_chain = active_chain_queue.extract(*isect_it);
      active_isect_chain.value().get()->advance_front();
      active_chain_queue.insert(std::move(active_isect_chain));
      active_chain_queue.insert(std::move(chain));

      if (current == std::prev(isect_it)) {
        auto next = std::next(isect_it);
        auto prev = std::prev(current);

        isect_and_update_chains(isect_it, next);
        isect_and_update_chains(current, prev);
      }
      else if (current == std::next(isect_it)) {
        auto next = std::next(current);
        auto prev = std::prev(isect_it);

        isect_and_update_chains(current, next);
        isect_and_update_chains(isect_it, prev);
      }
      else {
        // std::cout << "Intersecting chains (" << *current_chain.get() << ", " << *isect_chain
        //           << ") are not neigbours!" << std::endl;
        // BLI_assert(false);
      }
    }
    /* Inner event */
    else {
      current_chain->advance_front();
      // std::cout << "INNER" << std::endl;
      /* Set position to after. */
      sweep_pt.second = false;

      auto current = sweep_line_chains.find(current_chain);

      /* Fall back to linear search. */
      if (current == sweep_line_chains.end()) {
        // std::cout << *current_chain.get() << " not found! Falling back to linear search!"
        //           << std::endl;
        // BLI_assert(current != sweep_line_chains.end());
        for (auto it = sweep_line_chains.begin(); it != sweep_line_chains.end(); ++it) {
          if (*it == current_chain) {
            current = it;
          }
        }
      }
      auto next = std::next(current);
      auto prev = std::prev(current);

      isect_and_update_chains(current, next);
      isect_and_update_chains(current, prev);

      active_chain_queue.insert(std::move(chain));
    }

    // std::cout << std::endl;
  }

  // std::cout << "result " << clip_path << std::endl;

  return convert_clip_path();
}

/* -------------------------------------------------------------------- */

/**
 * Calculate and insert intersection points of a polyline.
 * \param list: The input polyline.
 * \param method: The method/algorithm to be used.
 * \returns a clip path with the original polyline and intersection points inserted at the right
 * places.
 */
ClipPath find_intersections(const PointList &list, uint method)
{
  ClipPath clip_path;
  switch (method) {
    case BRUTE_FORCE: {
      PolyclipBruteForce bf;
      clip_path = bf.find_intersections(list);
      break;
    }
    case BRUTE_FORCE_AABB:
      break;
    case BENTLEY_OTTMANN: {
      polyclip::PolyclipBentleyOttmann bo;
      clip_path = bo.find_intersections(list);
      break;
    }
    case PARK_SHIN: {
      polyclip::PolyclipParkShin ps;
      ps.add_monotone_chains_from_point_list(list);
      clip_path = ps.find_intersections();
      break;
    }
    default:
      break;
  }

  return clip_path;
}

/* \} */

/* -------------------------------------------------------------------- */
/** \name Polyline offsetting
 * \{ */

static void generate_arc_from_point_to_point(VertList &list,
                                             VertList::iterator &it_from,
                                             VertList::iterator &it_to,
                                             const double2 center_pt,
                                             const uint subdivisions,
                                             const bool is_left)
{
  double2 from_pt = double2((*it_from).co);
  double2 to_pt = double2((*it_to).co);
  double2 vec_from = from_pt - center_pt;
  double2 vec_to = to_pt - center_pt;
  if (vec_from.is_zero() || vec_to.is_zero()) {
    return;
  }

  /* The dot product is proportional to the sine and the determinant to the cosine of the angle
   * between the vectors. Taking the atan2 of those two will give an angle between -180 and 180
   * degrees. The sign will determine the direction of rotation. */
  double angle = std::atan2(double2::cross(vec_from, vec_to), double2::dot(vec_from, vec_to));
  uint num_points = (uint)std::floor((subdivisions + 2) * (std::abs(angle) / M_PI));

  if (num_points > 0) {
    double angle_incr = angle / (double)num_points;
    for (uint i = 1; i < num_points; i++) {
      double tmp_angle = (double)i * angle_incr;
      double2 vec_p = center_pt + double2::rotate(vec_from, tmp_angle);

      Vert *new_point = new Vert(vec_p);
      list.insert(it_to, *new_point);
    }
  }
}

static void generate_semi_circle_from_point_to_point(VertList &list,
                                                     const double2 from_pt,
                                                     VertList::iterator &it_to,
                                                     const double2 center_pt,
                                                     const uint subdivisions,
                                                     const bool is_left)
{
  uint num_points = subdivisions + 2;
  double2 vec_center = from_pt - center_pt;
  if (vec_center.is_zero()) {
    return;
  }

  double angle_incr = M_PI / (double)(num_points - 1);
  for (int i = 1; i < num_points - 1; i++) {
    double angle = (double)i * angle_incr;
    double2 vec_p = center_pt + double2::rotate(vec_center, angle);

    Vert *new_point = new Vert(vec_p);
    list.insert(it_to, *new_point);
  }
}

static void generate_end_cap(VertList &list,
                             const double2 point,
                             const double2 other_point,
                             const double radius,
                             const uint subdivisions,
                             const CapType cap_type,
                             const bool is_left)
{
  double2 cap_vec = other_point - point;
  cap_vec.normalize();

  double2 cap_nvec;
  if (cap_vec.is_zero()) {
    cap_nvec = double2(0, radius);
  }
  else {
    cap_nvec = double2(-cap_vec.y, cap_vec.x);
    cap_nvec *= radius;
  }

  double2 cap_nvec_inv = double2::invert(cap_nvec);
  double2 vec_offset = point + cap_nvec;
  double2 vec_offset_inv = point + cap_nvec_inv;

  Vert *p_pt = new Vert(vec_offset);
  Vert *p_pt_inv = new Vert(vec_offset_inv);

  list.push_back(*p_pt);
  VertList::iterator it_p_pt_inv = list.insert(list.end(), *p_pt_inv);

  if (cap_type == CAP_ROUND) {
    generate_semi_circle_from_point_to_point(
        list, vec_offset, it_p_pt_inv, point, subdivisions, is_left);
  }
}

/**
 * Calculate the offset (outline) of a polyline as new polyline.
 * \param subdivisions: Number of subdivions for the start and end caps
 * \param pline_radius: polyline radius. Every vertex has an additional radius that is a factor of
 * this radius.
 * \return: offset polyline.
 */
Polyline polyline_offset(Polyline &pline,
                         const uint subdivisions,
                         const double pline_radius,
                         CapType start_cap_t,
                         CapType end_cap_t)
{
  /* sanity check */
  if (pline.verts.size() < 1) {
    return Polyline();
  }

  VertList right_side, left_side;

  Vert first = pline.verts.front();
  Vert last = pline.verts.back();

  double first_radius = pline_radius * first.radius;
  double last_radius = pline_radius * last.radius;

  Vert first_next;
  Vert last_prev;
  if (pline.verts.size() > 1) {
    first_next = *std::next(pline.verts.begin(), 1);
    last_prev = *std::prev(pline.verts.end(), 2);
  }
  else {
    first_next = first;
    last_prev = last;
  }

  double2 first_pt = double2(first.co);
  double2 last_pt = double2(last.co);
  double2 first_next_pt = double2(first_next.co);
  double2 last_prev_pt = double2(last_prev.co);

  /* edgecase if single point */
  if (pline.verts.size() == 1) {
    first_next_pt.x += 1.0;
    last_prev_pt.x -= 1.0;
  }

  /* generate points for start cap */
  generate_end_cap(
      right_side, first_pt, first_next_pt, first_radius, subdivisions, start_cap_t, false);

  /* generate offset points  */
  auto it = std::next(pline.verts.begin());
  for (uint i = 1; i < pline.verts.size() - 1; ++i, ++it) {
    Vert curr = *it;
    Vert prev = *std::prev(it);
    Vert next = *std::next(it);
    double r = pline_radius * curr.radius;

    double2 curr_pt = double2(curr.co);
    double2 next_pt = double2(next.co);
    double2 prev_pt = double2(prev.co);

    double2 vec_prev = curr_pt - prev_pt;
    double2 vec_next = next_pt - curr_pt;
    double prev_length = vec_prev.length();
    double next_length = vec_next.length();

    if (vec_prev.compare_zero()) {
      vec_prev = double2(1.0, 0.0);
    }
    else {
      vec_prev.normalize();
    }

    if (vec_next.compare_zero()) {
      vec_next = double2(1.0, 0.0);
    }
    else {
      vec_next.normalize();
    }

    /* Rotate 90 degrees. */
    double2 nvec_prev = double2(-vec_prev.y, vec_prev.x);
    double2 nvec_next = double2(-vec_next.y, vec_next.x);

    double2 vec_tangent = vec_prev + vec_next;
    if (vec_tangent.compare_zero()) {
      vec_tangent = double2(nvec_prev);
    }
    else {
      vec_tangent.normalize();
    }

    double2 vec_miter_left = double2(-vec_tangent.y, vec_tangent.x);

    /* calculate miter length */
    double an1 = double2::dot(vec_miter_left, nvec_prev);
    if (IS_EQ(an1, 0.0)) {
      an1 = 1.0;
    }

    double miter_length = r / an1;
    CLAMP_MIN(miter_length, MIN_MITER_LENGTH);
    vec_miter_left *= miter_length;
    double2 vec_miter_right = double2::invert(vec_miter_left);
    double angle = double2::dot(vec_next, nvec_prev);

    /* add two points if angle is close to beeing straight */
    if (std::abs(angle) < CUT_OFF_ANGLE) {
      nvec_prev.normalize(r);
      nvec_next.normalize(r);

      double2 nvec_prev_pt = curr_pt + nvec_prev;
      double2 nvec_next_pt = curr_pt + double2::invert(nvec_next);

      Vert *normal_prev = new Vert(nvec_prev_pt);
      Vert *normal_next = new Vert(nvec_next_pt);
      left_side.push_back(*normal_prev);
      right_side.push_back(*normal_next);
    }
    else {
      /* bend to the left */
      if (angle < 0.0) {
        nvec_prev.normalize(r);
        nvec_next.normalize(r);

        double2 nvec_prev_pt = curr_pt + nvec_prev;
        double2 nvec_next_pt = curr_pt + nvec_next;

        Vert *normal_prev = new Vert(nvec_prev_pt);
        Vert *normal_next = new Vert(nvec_next_pt);

        VertList::iterator it_prev = left_side.insert(left_side.end(), *normal_prev);
        VertList::iterator it_next = left_side.insert(left_side.end(), *normal_next);

        generate_arc_from_point_to_point(left_side, it_prev, it_next, curr_pt, subdivisions, true);

        double2 miter_right_pt;
        if (miter_length < prev_length && miter_length < next_length) {
          miter_right_pt = curr_pt + vec_miter_right;
        }
        else {
          miter_right_pt = curr_pt + double2::invert(nvec_next);
        }

        Vert *miter_right = new Vert(miter_right_pt);
        right_side.push_back(*miter_right);
      }
      /* bend to the right */
      else {
        nvec_prev.normalize(-r);
        nvec_next.normalize(-r);

        double2 nvec_prev_pt = curr_pt + nvec_prev;
        double2 nvec_next_pt = curr_pt + nvec_next;

        Vert *normal_prev = new Vert(nvec_prev_pt);
        Vert *normal_next = new Vert(nvec_next_pt);

        VertList::iterator it_prev = right_side.insert(right_side.end(), *normal_prev);
        VertList::iterator it_next = right_side.insert(right_side.end(), *normal_next);

        generate_arc_from_point_to_point(
            right_side, it_prev, it_next, curr_pt, subdivisions, false);

        double2 miter_left_pt;
        if (miter_length < prev_length && miter_length < next_length) {
          miter_left_pt = curr_pt + vec_miter_left;
        }
        else {
          miter_left_pt = curr_pt + double2::invert(nvec_prev);
        }

        Vert *miter_left = new Vert(miter_left_pt);
        left_side.push_back(*miter_left);
      }
    }
  }

  /* generate points for end cap */
  generate_end_cap(right_side, last_pt, last_prev_pt, last_radius, subdivisions, end_cap_t, false);

  /* merge both sides to one list */
  right_side.reverse();
  left_side.splice(left_side.end(), right_side);
  VertList offset_vert_list = left_side;

  /* close by creating a point close to the first (make a small gap) */
  Vert close_first = offset_vert_list.front();
  Vert close_last = offset_vert_list.back();
  double2 close_pt = double2::interpolate(close_first.co, close_last.co, 0.99f);

  if (double2::compare_limit(close_pt, close_first.co, DBL_EPSILON) == false) {
    Vert *close_p_pt = new Vert(close_pt);
    offset_vert_list.push_back(*close_p_pt);
  }

  return Polyline(offset_vert_list);
}

/* \} */

} /* namespace blender::polyclip */

#ifdef WITH_CLIPPER
// void test()
// {
//   ClipperLib::Clipper c;
//   ClipperLib::Paths paths;
// }
#endif

/* Wrapper for C. */
extern "C" {

using namespace blender;

static polyclip::PointList point_list_from_flat_array(const double *verts, uint num_verts)
{
  polyclip::PointList pline;
  /* Fill pline with data from verts. */
  for (uint i = 0; i < num_verts; i++) {
    double2 co = double2(verts[i * 2], verts[i * 2 + 1]);
    pline.push_back(co);
  }
  return pline;
}

static void flat_array_from_point_list(polyclip::PointList &list,
                                       double **r_array,
                                       uint *r_size_array)
{
  uint num_points = list.size();
  if (num_points == 0) {
    *r_array = NULL;
    *r_size_array = 0;
    return;
  }

  double *point_array = (double *)MEM_mallocN(sizeof(double) * num_points * 2, __func__);
  polyclip::PointList::iterator it = list.begin();
  for (uint i = 0; i < num_points; i++, it++) {
    double2 vert = *it;
    copy_v2_v2_db(&point_array[i * 2], vert);
  }
  *r_array = point_array;
  *r_size_array = num_points;
}

static void flat_array_from_clip_path(polyclip::ClipPath &list,
                                      double **r_array,
                                      uint *r_size_array)
{
  uint num_points = list.size();
  if (num_points == 0) {
    *r_array = NULL;
    *r_size_array = 0;
    return;
  }

  double *point_array = (double *)MEM_mallocN(sizeof(double) * num_points * 2, __func__);
  auto it = list.begin();
  for (uint i = 0; i < num_points; i++, it++) {
    double2 vert = (*it)->data;
    copy_v2_v2_db(&point_array[i * 2], vert);
  }
  *r_array = point_array;
  *r_size_array = num_points;
}

void BLI_polyline_outer_boundary(const double *verts,
                                 uint num_verts,
                                 CLIP_METHOD method,
                                 double **r_boundary_verts,
                                 uint *r_num_boundary_verts)
{
  /* Convert flat array to point list. */
  polyclip::PointList pline = point_list_from_flat_array(verts, num_verts);

  /* Connect ends so we find intersections at the last edge. */
  pline.push_back(*pline.begin());
  polyclip::ClipPath clip_path = polyclip::find_intersections(pline, method);
  clip_path.remove(*clip_path.end());

  /* Find the outer boundary and convert the point list back to a flat array. */
  polyclip::PointList outer_boundary = polyclip::clip_path_get_outer_boundary(clip_path);
  flat_array_from_point_list(outer_boundary, r_boundary_verts, r_num_boundary_verts);
}

void BLI_polyline_intersections(const double *verts,
                                uint num_verts,
                                CLIP_METHOD method,
                                double **r_isect_verts,
                                uint *r_num_isect_verts)
{
  /* Convert flat array to point list. */
  polyclip::PointList pline = point_list_from_flat_array(verts, num_verts);

  /* Connect ends so we find intersections at the last edge. */
  pline.push_back(*pline.begin());
  polyclip::ClipPath clip_path = polyclip::find_intersections(pline, method);
  clip_path.remove(*clip_path.end());

  flat_array_from_clip_path(clip_path, r_isect_verts, r_num_isect_verts);
}

void BLI_polyline_offset(const double *verts,
                         uint num_verts,
                         const double radius,
                         const uint subdivisions,
                         uint start_cap_t,
                         uint end_cap_t,
                         double **r_offset_verts,
                         uint *r_num_offset_verts)
{
  polyclip::Polyline pline;

  /* Fill pline with data from verts. */
  for (uint i = 0; i < num_verts; i++) {
    double2 co = double2(verts[i * 3], verts[i * 3 + 1]);
    double radius = verts[i * 3 + 2];
    pline.verts.push_back(polyclip::Vert(co, radius));
  }

  polyclip::Polyline offset_pline = polyline_offset(pline,
                                                    subdivisions,
                                                    radius,
                                                    static_cast<polyclip::CapType>(start_cap_t),
                                                    static_cast<polyclip::CapType>(end_cap_t));

  uint num_offset_verts = offset_pline.verts.size();
  if (num_offset_verts == 0) {
    *r_offset_verts = NULL;
    *r_num_offset_verts = 0;
    return;
  }

  /* Allocate and populate returning flat array of offset vertices. */
  double *offset_verts = (double *)MEM_callocN(sizeof(double) * num_offset_verts * 3, __func__);
  polyclip::VertList::iterator it = offset_pline.verts.begin();
  for (uint i = 0; i < num_offset_verts; i++, it++) {
    polyclip::Vert vert = *it;
    copy_v2_v2_db(&offset_verts[i * 2], double2(vert.co));
  }

  *r_offset_verts = offset_verts;
  *r_num_offset_verts = num_offset_verts;
}

} /* extern "C" */