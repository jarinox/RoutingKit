#ifndef ROUTING_KIT_CHLRM_H
#define ROUTING_KIT_CHLRM_H

#include <routingkit/chlr.h>
#include <routingkit/label.h>

#include <vector>
#include <unordered_map>
#include <functional>

class CHLRMArcPos {
public:
    CHLRMArcPos() : arc_index(0), node_index(0) {}; // Default constructor
    CHLRMArcPos(unsigned arc_index, unsigned node_index) 
        : arc_index(arc_index), node_index(node_index) {};
    
    unsigned arc_index;
    unsigned node_index;

    bool operator==(const CHLRMArcPos& other) const {
        return arc_index == other.arc_index && node_index == other.node_index;
    }
};

// Hash specializations must be defined before use
namespace std {
    template <>
    struct hash<CHLRMArcPos> {
        std::size_t operator()(const CHLRMArcPos& pos) const noexcept {
            return std::hash<unsigned>()(pos.node_index) ^ (std::hash<unsigned>()(pos.arc_index) << 1);
        }
    };

    template <>
    struct hash<std::pair<CHLRMArcPos, CHLRMArcPos>> {
        std::size_t operator()(const std::pair<CHLRMArcPos, CHLRMArcPos>& p) const noexcept {
            std::size_t h1 = std::hash<CHLRMArcPos>{}(p.first);
            std::size_t h2 = std::hash<CHLRMArcPos>{}(p.second);
            return h1 ^ (h2 << 1);
        }
    };
}

class CHLRMArc {
public:
    unsigned from;
    unsigned mid_node;
    unsigned to;

    unsigned cnt;

    unsigned weight;
    Label label;

    CHLRMArc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label) :
        from(from), mid_node(mid_node), to(to), cnt(0), weight(weight), label(label) {};

    bool is_shortcut() { return mid_node != invalid_id; }
    unsigned rank(CHLRMGraph& graph) {
        return std::min(graph.nodes[from].rank, graph.nodes[to].rank);
    }
};

class CHLRMNode {
public:
    unsigned index;
    unsigned rank;

    std::vector<CHLRMArc> arcs;
};

class CHLRMGraph {
public:
    std::vector<CHLRMNode> nodes;

    // N+ maps the parent shortcuts e1 and e2 to their child shortcut e3
    std::unordered_map<std::pair<CHLRMArcPos, CHLRMArcPos>, CHLRMArcPos> N_plus;

    // N- maps the child shortcut e3 to its parent shortcuts e1 and e2
    std::unordered_map<CHLRMArcPos, std::vector<std::pair<CHLRMArcPos, CHLRMArcPos>>> N_minus;

    // N= maps a parent shortcut e1 to its partner shortcut e2
    std::unordered_map<CHLRMArcPos, std::vector<CHLRMArcPos>> N_equals;

    CHLRMGraph(CHLRGraph& graph);

    CHLRMArc& get_arc(CHLRMArcPos arc_pos);
    void delete_arc(CHLRMArcPos arc_pos);

    unsigned calculate_weight(CHLRMArcPos arc_pos);
    void keep_shortcut_dominance(CHLRMArcPos arc_pos, MinIDQueue& queue, std::vector<std::pair<CHLRMArcPos, bool>>& unsigned_to_arc_pos, bool increment);
    void maintenance(CHLRMArcPos arc_pos, unsigned new_weight, Label new_label);

private:
    void build_neighbour_index();
};

#endif // ROUTING_KIT_CHLRM_H
