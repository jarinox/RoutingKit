#ifndef ROUTING_KIT_CHM_H
#define ROUTING_KIT_CHM_H

#include <routingkit/chlr.h>
#include <routingkit/label.h>
#include <routingkit/dijkstra.h>

#include <vector>
#include <unordered_map>
#include <functional>
#include <set>
#include <memory>
#include <queue>


class CHMGraph;
class MinRankQueue;

class CHMArcPos {
public:
    unsigned node_index;
    unsigned arc_index;
};

class CHMArc {
public:
    unsigned from;
    unsigned mid_node;
    unsigned to;

    unsigned arc_index;

    unsigned weight;
    Label label;
    unsigned cnt;

    CHMArc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label)
        : from(from), mid_node(mid_node), to(to), arc_index(invalid_id), weight(weight), label(label), cnt(0) {}

    bool is_shortcut() const { return mid_node != invalid_id; }
    CHMArcPos get_pos() const { return CHMArcPos{from, arc_index}; }
    CHMArc& ref(CHMGraph& graph);
    unsigned rank(CHMGraph& graph);

    bool operator<(const CHMArc& other) const {
        return from < other.from ||
               (from == other.from && (mid_node < other.mid_node ||
               (mid_node == other.mid_node && (to < other.to ||
               (to == other.to && (weight < other.weight || 
               (weight == other.weight && label < other.label)))))));
    }

    bool operator==(const CHMArc& other) const {
        return from == other.from && mid_node == other.mid_node && to == other.to &&
               weight == other.weight && label == other.label;
    }
};

class CHMNode {
public:
    unsigned node_index;
    unsigned rank;

    float lat;
    float lon;

    std::vector<CHMArc> arcs;
};

class CHMGraph {
public:
    std::vector<CHMNode> nodes;
    std::map<CHMArc, std::vector<CHMArc>> dominant_shortcuts;

    CHMGraph() = default;
    CHMGraph(CHLRGraph& graph);
    CHLRGraph to_chlr();
    std::pair<unsigned, std::vector<unsigned>> dijkstra(unsigned from, unsigned to, Label profile);

    void add_arc(CHMArc arc, bool avoid_duplicated = false);
    void add_arc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label);

    CHMArc& get(CHMArcPos pos) {
        return nodes[pos.node_index].arcs[pos.arc_index];
    }

    std::pair<unsigned, unsigned> calculate_weight(CHMArc arc);
    void keep_shortcut_dominance(CHMArc arc, bool increment, MinRankQueue& queue);
    void maintenance(CHMArcPos e_o_pos, unsigned w_n, Label l_n);
    void maintenance_optimized(CHMArcPos e_o, unsigned w_n, Label l_n);

    void defragment();

    std::vector<std::pair<CHMArc, CHMArc>> Nm(CHMArc child);
    std::vector<CHMArc> SCSp(CHMArc e);
    std::vector<CHMArc> Ne(CHMArc arc, bool ignore_ranks = false);
    std::pair<CHMArc, bool> Np(CHMArc e1, CHMArc e2);
    std::pair<CHMArc, bool> child(CHMArc e1, CHMArc e2);

    void add_dominant_shortcut(CHMArc arc, CHMArc shortcut);
    bool has_dominant_shortcut(CHMArc arc, CHMArc shortcut);
    std::vector<CHMArc> get_dominant_shortcuts(CHMArc arc);
    void remove_dominant_shortcut(CHMArc arc, CHMArc shortcut);
    unsigned w(unsigned from, unsigned to, Label label);

    void DCHp_scsWDec(CHMArcPos e_o, unsigned w_n);

    void add_or_reduce_arc(CHMArc arc);
    void maintenance_alt(CHMArcPos e_o, unsigned w_n, Label l_n);

    unsigned witness_search(unsigned from, unsigned to, unsigned weight, Label label);

    std::vector<unsigned> N(unsigned v);
};

inline CHMArc& CHMArc::ref(CHMGraph& graph) {
    if(arc_index == invalid_id) {
        return *this;
    }

    return graph.get(get_pos());
}

class MinRankQueue {
    MinIDQueue queue;
    std::vector<std::queue<std::pair<bool, CHMArc>>> unsigned_to_arc;
    std::set<std::pair<CHMArc, bool>> is_in_queue;
public:
    MinRankQueue(unsigned size) : queue(size), unsigned_to_arc(size) {}

    bool empty() const {
        return queue.empty();
    }

    void push(CHMArc arc, bool increment, CHMGraph& graph);
    std::pair<bool, CHMArc> pop();

    bool contains(const CHMArc& arc, bool increment) const {
        return is_in_queue.find({arc, increment}) != is_in_queue.end();
    }
};



#endif // ROUTING_KIT_CHM_H
