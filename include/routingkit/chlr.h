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
#include <unordered_set>
#include <map>

using namespace RoutingKit;


class CHLRArc {
   public:
    unsigned other_node;
    unsigned mid_node;  // Used for shortcuts
    unsigned weight;
    Label label;

    bool is_shortcut() const { return mid_node != invalid_id; }
    bool dominates(const CHLRArc& other) const {
        return label.is_subset_of(other.label) && weight <= other.weight && (label != other.label || weight != other.weight);
    }
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
                      return a.weight != b.weight ? a.weight < b.weight : !a.label.is_superset_of(b.label);
                  });
        std::sort(out_arcs.begin(), out_arcs.end(),
                  [](const CHLRArc& a, const CHLRArc& b) {
                      return a.weight != b.weight ? a.weight < b.weight : !a.label.is_superset_of(b.label);
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

    void remove_incident_arcs(unsigned node_id);
    void add_arc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label);
    bool add_or_reduce_arc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label);
    CHLRArc& get_reverse_arc(CHLRArc &arc, unsigned start_node);
};

class CHLR {
   public:
    CHLRGraph& graph;
    std::vector<unsigned> order;

    CHLR(CHLRGraph& graph) : graph(graph) { order.resize(graph.nodes.size()); }

    void build(bool print_progress = false);
};

unsigned estimate_node_importance(const CHLRGraph& graph, unsigned node_id);

class CHLRQuery {
private:
    CHLRGraph graph;

    Label restriction;
    unsigned start_node;
    unsigned end_node;

    std::vector<unsigned> _forward_predecessor_node;
    std::vector<unsigned> _backward_predecessor_node;
    std::vector<unsigned> _forward_predecessor_arc;
    std::vector<unsigned> _backward_predecessor_arc;

    void extract_directional_graphs();
public:
    CHLRGraph forward;
    CHLRGraph backward;
    unsigned meeting_node;

    CHLRQuery(CHLRGraph& graph) : graph(graph) {
        extract_directional_graphs();
    }

    void set(unsigned start_node, unsigned end_node, Label restriction) {
        this->start_node = start_node;
        this->end_node = end_node;
        this->restriction = restriction;
    }
    
    void settle(MinIDQueue& queue, std::vector<unsigned>& distance,
                std::vector<unsigned>& predecessor_node,
                std::vector<unsigned>& predecessor_arc,
                TimestampFlags& was_pushed,
                MinIDQueue& other_queue, std::vector<unsigned>& other_distance,
                std::vector<unsigned>& other_predecessor_node,
                std::vector<unsigned>& other_predecessor_arc,
                TimestampFlags& other_was_pushed, unsigned& meeting_node,
                bool& search_forward, CHLRGraph& graph, Label restriction, unsigned &best_distance);
    void run();

    std::vector<CHLRNode> get_node_path();
    std::vector<CHLRArc> get_arc_path();
    void reset();

    std::vector<CHLRArc> full_forward_search();
};

class DijkstraLRStorage {
public:
    MinIDQueue queue;
    std::vector<unsigned> tentative_distance;
    std::set<unsigned> visited_nodes;
};

class DijkstraLR {
public:
    CHLRGraph& graph;
    unsigned max_pop_count = 500;
    unsigned start_node;
private:
    Label previous_restriction;
    std::map<Label, DijkstraLRStorage> storage;
public:

    DijkstraLR(CHLRGraph& graph, unsigned start_node, DijkstraLRStorage storage = DijkstraLRStorage())
        : graph(graph), start_node(start_node), previous_restriction(Label::fully_restricted()), storage(std::map<Label, DijkstraLRStorage>()) {};

    unsigned witness_search(unsigned end_node, Label restriction,
                            const std::function<bool(unsigned)>& is_valid_node);
};

#endif  // ROUTING_KIT_CHLR_H
