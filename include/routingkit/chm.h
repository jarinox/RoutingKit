#ifndef ROUTING_KIT_CHM_H
#define ROUTING_KIT_CHM_H

#include <routingkit/chlr.h>
#include <routingkit/label.h>

#include <vector>
#include <unordered_map>
#include <functional>
#include <set>
#include <memory>
#include <queue>


class CHMGraph;

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

    CHMArc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label)
        : from(from), mid_node(mid_node), to(to), weight(weight), label(label), arc_index(invalid_id) {}

    bool is_shortcut() const { return mid_node != invalid_id; }
    CHMArcPos get_pos() const { return CHMArcPos{from, arc_index}; }
    CHMArc& ref(CHMGraph& graph);
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
    std::vector<CHMArc> unbound_arcs;

    void add_arc(CHMArc arc) {
        nodes[arc.from].arcs.push_back(arc);
        nodes[arc.from].arcs.back().arc_index = nodes[arc.from].arcs.size() - 1;
    }

    CHMArc& get(CHMArcPos pos) {
        if (pos.node_index == invalid_id)
            return unbound_arcs[pos.arc_index];
        return nodes[pos.node_index].arcs[pos.arc_index];
    }

    unsigned calculate_weight(CHMArc arc);

    std::vector<std::pair<CHMArc, CHMArc>> Nm(CHMArc child);
    std::vector<CHMArc> Ne(CHMArc arc);
    CHMArc Np(CHMArc e1, CHMArc e2);

};

inline CHMArc& CHMArc::ref(CHMGraph& graph) {
    if(arc_index == invalid_id) {
        return *this;
    }

    return graph.get(get_pos());
}

#endif // ROUTING_KIT_CHM_H
