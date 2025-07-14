#ifndef ROUTING_KIT_CHLR_H
#define ROUTING_KIT_CHLR_H

#include <routingkit/id_queue.h>
#include <routingkit/label.h>
#include <routingkit/timestamp_flag.h>

#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <set>
#include <vector>

using namespace RoutingKit;


class CHLRArc {
   public:
    unsigned other_node;
    unsigned mid_node;  // Used for shortcuts
    unsigned weight;
    Label label;

    bool is_shortcut() { return mid_node != invalid_id; }
};

class CHLRNode {
   public:
    CHLRNode()
        : neighbour_level(0),
          in_arcs(std::vector<CHLRArc>()),
          out_arcs(std::vector<CHLRArc>()) {}

    unsigned neighbour_level;
    unsigned rank;
    float lat;
    float lon;

    std::vector<CHLRArc> in_arcs;
    std::vector<CHLRArc> out_arcs;

    void sort_arcs_for_weight() {
        std::sort(in_arcs.begin(), in_arcs.end(),
                  [](const CHLRArc& a, const CHLRArc& b) {
                      return a.weight < b.weight;
                  });
        std::sort(out_arcs.begin(), out_arcs.end(),
                  [](const CHLRArc& a, const CHLRArc& b) {
                      return a.weight < b.weight;
                  });
    }
};

enum CHDirection { FORWARD, BACKWARD };

class CHLRGraph {
   public:
    CHLRGraph() : nodes(std::vector<CHLRNode>()) {};
    CHLRGraph(std::vector<CHLRNode> nodes) : nodes(nodes) {}
    CHLRGraph(unsigned node_count, const std::vector<unsigned>& tail,
              const std::vector<unsigned>& head,
              const std::vector<unsigned>& weight,
              const std::vector<float>& latitude,
              const std::vector<float>& longitude, std::vector<Label>& label);

    std::vector<CHLRNode> nodes;
};

class CHLR {
   public:
    CHLRGraph& graph;
    std::vector<unsigned> order;

    CHLR(CHLRGraph& graph) : graph(graph) { order.resize(graph.nodes.size()); }

    void build();
    void extract_forward_and_backward_graphs();
};

unsigned estimate_node_importance(const CHLRGraph& graph, unsigned node_id);

class CHLRQuery {
   public:
    CHLRGraph forward_graph;
    CHLRGraph backward_graph;
    MinIDQueue forward_queue;
    MinIDQueue backward_queue;
    std::vector<unsigned> forward_distance;
    std::vector<unsigned> backward_distance;
    TimestampFlags was_forward_pushed;
    TimestampFlags was_backward_pushed;
    std::vector<unsigned> forward_predecessor_node;
    std::vector<unsigned> backward_predecessor_node;
    std::vector<unsigned> forward_predecessor_arc;
    std::vector<unsigned> backward_predecessor_arc;

    unsigned meeting_node;

    Label restriction;
    unsigned start_node;
    unsigned end_node;

    std::vector<CHLRNode> get_node_path();
    std::vector<CHLRArc> get_arc_path();
    void reset();
};

class DijkstraLR {
   public:
    CHLRGraph& graph;
    MinIDQueue queue;
    unsigned max_pop_count = 500;
    unsigned start_node;
    std::vector<unsigned> tentative_distance;
    std::set<unsigned> visited_nodes;

    DijkstraLR(CHLRGraph& graph, unsigned start_node)
        : graph(graph), start_node(start_node) {
        queue = MinIDQueue(graph.nodes.size());
        tentative_distance.resize(graph.nodes.size(),
                                  std::numeric_limits<unsigned>::max());
    }

    unsigned witness_search(unsigned end_node, Label restriction,
                            const std::function<bool(unsigned)>& is_valid_node);
};

#endif  // ROUTING_KIT_CHLR_H
