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

template<typename T> LinkedChain<T>::LinkedChain(const LinkedChain<T> &src)
{
  this->head = nullptr;
  this->tail = nullptr;
  this->size_ = 0;

  for (auto it = src.begin(); it != src.end(); ++it) {
    auto elem = *it;
    this->insert_back(elem->data);
  }
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
  for (node = this->head; node != nullptr; node = node->next) {
    if (node->data == data) {
      break;
    }
  }

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
  if (insert_node->next == nullptr) {
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
  if (insert_node->prev == nullptr) {
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

  if (node->prev == nullptr) {
    this->head = node->next;
  }
  else {
    node->prev->next = node->next;
  }
  if (node->next == nullptr) {
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

template class LinkedChain<double2>;

/* -------------------------------------------------------------------- */
/** \name Polyline clipping
 * \{ */

ClipPath point_list_find_intersections_brute_force(const PointList &list)
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

PointList clip_path_get_outer_boundary(ClipPath &path)
{
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
  while (it != min_elem) {
    auto vert = *it;
    outer.push_back(vert->data);

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

double PolyclipBentleyOttmann::Edge::y_intercept(const Edge &edge, const double x)
{
  double2 edge_start = edge.x_dir ? edge.first->data : edge.second->data;
  double2 edge_end = edge.x_dir ? edge.second->data : edge.first->data;
  double start_x = edge_start.x;
  double start_y = edge_start.y;
  double end_x = edge_end.x;
  double end_y = edge_end.y;
  if (x <= start_x) {
    return start_y;
  }
  if (x >= end_x) {
    return end_y;
  }

  double x_dist = end_x - start_x;
  if (IS_EQ(x_dist, 0.0)) {
    /* edge is vertical, return smallest y */
    return start_y < end_y ? start_y : end_y;
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

  double2 e1_start = e1.x_dir ? e1.first->data : e1.second->data;
  double2 e1_end = e1.x_dir ? e1.second->data : e1.first->data;
  double2 e2_start = e2.x_dir ? e2.first->data : e2.second->data;
  double2 e2_end = e2.x_dir ? e2.second->data : e2.first->data;

  /* If the edge is vertical pick the lower point as the sweep point. */
  double2 sweep_pt_e1 = (e1.first->data.x == e1.second->data.x) ? e1_start : e1.sweep_pt;
  double2 sweep_pt_e2 = (e2.first->data.x == e2.second->data.x) ? e2_start : e2.sweep_pt;
  if (!(sweep_pt_e1 == sweep_pt_e2)) {
    /* Calculate the y intercept of the second edge. */
    double y_icept = PolyclipBentleyOttmann::Edge::y_intercept(e2, sweep_pt_e1.x);
    if (sweep_pt_e1.y < y_icept) {
      return false;
    }
    if (sweep_pt_e1.y > y_icept) {
      return true;
    }
  }

  /* Edges intersect at start point. */
  if (e1_start == sweep_pt_e1 && e2_start == sweep_pt_e1 && e2_start == sweep_pt_e2) {
    if (e2_end == sweep_pt_e1) {
      return double2::compare_less(e1_end, e2_start);
    }
    int cmp = double2::orientation(e1_start, e1_end, e2_end);
    if (cmp != 0) {
      return cmp < 0;
    }
    return double2::compare_less(e1_end, e2_end);
  }
  /* Edges intersect at end point. */
  if (e1_end == sweep_pt_e1 && e2_end == sweep_pt_e1 && e2_end == sweep_pt_e2) {
    if (e2_start == sweep_pt_e1) {
      return double2::compare_less(e1_start, e2_end);
    }
    int cmp = double2::orientation(e1_start, e1_end, e2_start);
    if (cmp != 0) {
      return cmp < 0;
    }
    return double2::compare_less(e1_start, e2_start);
  }
  /* Edges intersect at sweep points. */
  if (sweep_pt_e2 == sweep_pt_e1) {
    if (sweep_pt_e2 == e2_end) {
      int cmp = double2::orientation(e1_start, e1_end, e2_start);
      if (cmp != 0) {
        return cmp < 0;
      }
      return double2::compare_less(e1_start, e2_start);
    }
    int cmp = double2::orientation(e1_start, e1_end, e2_end);
    if (cmp != 0) {
      return cmp < 0;
    }
    return double2::compare_less(e1_end, e2_end);
  }

  if (e2_start == sweep_pt_e1) {
    int cmp = double2::orientation(e1_start, e1_end, e2_end);
    if (cmp != 0) {
      return cmp < 0;
    }
    return double2::compare_less(e1_end, e2_end);
  }
  /* Edgecase for vertical edges. This happens when there is no y intercept for e2 and the y of the
   * start point is returned. In this case the edges do not intersect at their start point. We need
   * to figure out on what side e2.start of e1 lies.*/
  else if (e2_start.x > sweep_pt_e1.x) {
    int cmp = double2::orientation(e1_start, e1_end, e2_start);
    if (cmp != 0) {
      return cmp < 0;
    }
    return double2::compare_less(e1_end, e2_start);
  }

  if (e2_end == sweep_pt_e1) {
    int cmp = double2::orientation(e1_start, e1_end, e2_start);
    if (cmp != 0) {
      return cmp < 0;
    }
    return double2::compare_less(e1_start, e2_start);
  }

  return false;
}

PolyclipBentleyOttmann::Event PolyclipBentleyOttmann::check_edge_edge_isect(
    const PolyclipBentleyOttmann::Edge &e1, const PolyclipBentleyOttmann::Edge &e2)
{
  auto result = double2::isect_seg_seg(
      e1.first->data, e1.second->data, e2.first->data, e2.second->data);
  if (result.kind == double2::isect_result::LINE_LINE_CROSS) {
    double2 isect_pt = double2::interpolate(e1.first->data, e1.second->data, result.lambda);
    return Event(isect_pt, e1, e2);
  }
  else {
    return Event();
  }
}

ClipPath PolyclipBentleyOttmann::find_intersections(const PointList &list)
{
  ClipPath clip_path = ClipPath(list);

  for (auto it = clip_path.begin_pair(); it != clip_path.end_pair(); ++it) {
    auto node_edge = *it;
    Event start_event, end_event;
    /* Make the edge point in +X (or +Y if vertical). */
    if (double2::compare_less(node_edge.first->data, node_edge.second->data) == true) {
      Edge edge = Edge(node_edge, node_edge.first->data, true);
      start_event = Event(edge.first->data, edge, Event::START);
      end_event = Event(edge.second->data, edge, Event::END);
    }
    else {
      Edge edge = Edge(node_edge, node_edge.second->data, false);
      start_event = Event(edge.second->data, edge, Event::START);
      end_event = Event(edge.first->data, edge, Event::END);
    }

    event_queue.push(start_event);
    event_queue.push(end_event);
  }

  while (!event_queue.empty()) {
    Event event = event_queue.top();
    event_queue.pop();

    std::cout << event << std::endl;

    auto print = [](const Edge &e) { std::cout << "\n" << e; };
    std::cout << "Before sweep_line_edges: ";
    std::for_each(sweep_line_edges.begin(), sweep_line_edges.end(), print);
    std::cout << std::endl;

    if (event.type == Event::INTERSECTION) {
      /* Pop duplicates from the event queue. */
      while (event == event_queue.top()) {
        event_queue.pop();
      }

      auto itA = sweep_line_edges.extract(event.edge);
      auto itB = sweep_line_edges.extract(event.isect_edge);

      if (itA.empty() || itB.empty()) {
        std::cout << "Error: could not extract intersecting edges!";
      }

      BLI_assert(!itA.empty() && !itB.empty());

      ClipPath::Node *isect_nodeA, *isect_nodeB;
      // std::cout << clip_path << std::endl;
      if (event.edge.x_dir) {
        isect_nodeA = clip_path.insert_after(itA.value().sweep_node, event.pt);
      }
      else {
        isect_nodeA = clip_path.insert_before(itA.value().sweep_node, event.pt);
      }

      // std::cout << clip_path << " sweep_pt: " << event.edge.sweep_pt << std::endl;

      if (event.isect_edge.x_dir) {
        isect_nodeB = clip_path.insert_after(itB.value().sweep_node, event.pt);
      }
      else {
        isect_nodeB = clip_path.insert_before(itB.value().sweep_node, event.pt);
      }

      // std::cout << clip_path << " sweep_pt: " << event.isect_edge.sweep_pt << std::endl;

      clip_path.link(isect_nodeA, isect_nodeB);

      itA.value().sweep_pt = event.pt;
      itB.value().sweep_pt = event.pt;

      itA.value().sweep_node = isect_nodeA;
      itB.value().sweep_node = isect_nodeB;

      auto node_A = sweep_line_edges.insert(std::move(itA)).position;
      auto node_B = sweep_line_edges.insert(std::move(itB)).position;

      BLI_assert(node_A != node_B);

      if (std::next(node_B) == node_A) {
        auto next = std::next(node_A);
        auto prev = std::prev(node_B);

        if (next != sweep_line_edges.end()) {
          Event e = check_edge_edge_isect(*node_A, *next);
          if (e.type != Event::Type::EMPTY) {
            event_queue.push(e);
          }
        }

        if (prev != sweep_line_edges.end()) {
          Event e = check_edge_edge_isect(*node_B, *prev);
          if (e.type != Event::Type::EMPTY) {
            event_queue.push(e);
          }
        }
      }
      else if (std::next(node_A) == node_B) {
        auto next = std::next(node_B);
        auto prev = std::prev(node_A);

        if (next != sweep_line_edges.end()) {
          Event e = check_edge_edge_isect(*node_B, *next);
          if (e.type != Event::Type::EMPTY) {
            event_queue.push(e);
          }
        }

        if (prev != sweep_line_edges.end()) {
          Event e = check_edge_edge_isect(*node_A, *prev);
          if (e.type != Event::Type::EMPTY) {
            event_queue.push(e);
          }
        }
      }
    }
    else if (event.type == Event::START) {
      event.edge.sweep_pt = event.pt;
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
          }
        }

        if (prev != sweep_line_edges.end() && prev != current) {
          Event e = check_edge_edge_isect(*current, *prev);
          if (e.type != Event::Type::EMPTY) {
            event_queue.push(e);
          }
        }
      }
      else {
        std::cout << "Error: Event edge " << event.edge << " was not inserted!" << std::endl;
      }
    }
    else if (event.type == Event::END) {
      event.edge.sweep_pt = event.pt;
      /* Find the edge and remove it from the sweep line set. */
      auto it = sweep_line_edges.find(event.edge);

      if (it != sweep_line_edges.end()) {
        auto next = std::next(it);
        auto prev = std::prev(it);

        sweep_line_edges.erase(it);

        if (next != sweep_line_edges.end() && prev != sweep_line_edges.end()) {
          Event e = check_edge_edge_isect(*prev, *next);
          if (e.type != Event::Type::EMPTY) {
            event_queue.push(e);
          }
        }
      }
      else {
        std::cout << "Error: Event edge " << event.edge << " could not be deleted!" << std::endl;
      }
    }

    std::cout << "After sweep_line_edges: ";
    std::for_each(sweep_line_edges.begin(), sweep_line_edges.end(), print);
    std::cout << std::endl;
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

void BLI_polyline_outer_boundary(const double *verts,
                                 uint num_verts,
                                 CLIP_METHOD method,
                                 double **r_boundary_verts,
                                 uint *r_num_boundary_verts)
{
  polyclip::PointList pline;

  /* Fill pline with data from verts. */
  for (uint i = 0; i < num_verts; i++) {
    double2 co = double2(verts[i * 2], verts[i * 2 + 1]);
    pline.push_back(co);
  }

  polyclip::ClipPath clip_path;
  switch (method) {
    case BRUTE_FORCE: {
      clip_path = polyclip::point_list_find_intersections_brute_force(pline);
      break;
    }
    case BRUTE_FORCE_AABB:
      break;
    case BENTLEY_OTTMANN: {
      polyclip::PolyclipBentleyOttmann bo;
      clip_path = bo.find_intersections(pline);
      break;
    }
    default:
      break;
  }

  polyclip::PointList outer_boundary = polyclip::clip_path_get_outer_boundary(clip_path);
  uint num_boundary_vert = outer_boundary.size();
  if (num_boundary_vert == 0) {
    *r_boundary_verts = NULL;
    *r_num_boundary_verts = 0;
    return;
  }

  /* Allocate and populate returning flat array of boundary vertices. */
  double *boundary_verts = (double *)MEM_mallocN(sizeof(double) * num_boundary_vert * 2, __func__);
  polyclip::PointList::iterator it = outer_boundary.begin();
  for (uint i = 0; i < num_boundary_vert; i++, it++) {
    double2 vert = *it;
    copy_v2_v2_db(&boundary_verts[i * 2], vert);
  }

  *r_boundary_verts = boundary_verts;
  *r_num_boundary_verts = num_boundary_vert;
}

void BLI_polyline_isect_self(const double *verts,
                             uint num_verts,
                             CLIP_METHOD method,
                             double **r_isect_verts,
                             uint *r_num_isect_verts)
{
  polyclip::PointList pline;

  /* Fill pline with data from verts. */
  for (uint i = 0; i < num_verts; i++) {
    double2 co = double2(verts[i * 2], verts[i * 2 + 1]);
    pline.push_back(co);
  }

  polyclip::ClipPath clip_path;
  switch (method) {
    case BRUTE_FORCE: {
      clip_path = polyclip::point_list_find_intersections_brute_force(pline);
      break;
    }
    case BRUTE_FORCE_AABB:
      break;
    case BENTLEY_OTTMANN: {
      polyclip::PolyclipBentleyOttmann bo;
      clip_path = bo.find_intersections(pline);
      break;
    }
    default:
      break;
  }

  uint num_isect_vert = clip_path.size();
  if (num_isect_vert == 0) {
    *r_isect_verts = NULL;
    *r_num_isect_verts = 0;
    return;
  }

  /* Allocate and populate returning flat array of isect vertices. */
  double *isect_verts = (double *)MEM_mallocN(sizeof(double) * num_isect_vert * 2, __func__);
  auto it = clip_path.begin();
  for (uint i = 0; i < num_isect_vert; i++, it++) {
    double2 vert = (*it)->data;
    copy_v2_v2_db(&isect_verts[i * 2], vert);
  }

  *r_isect_verts = isect_verts;
  *r_num_isect_verts = num_isect_vert;
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