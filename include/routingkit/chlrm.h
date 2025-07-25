#ifndef ROUTING_KIT_CHLRM_H
#define ROUTING_KIT_CHLRM_H

#include <routingkit/chlr.h>
#include <routingkit/label.h>

#include <vector>
#include <unordered_map>

class CHLRMArcPos {
public:
    CHLRMArcPos(unsigned arc_index, unsigned node_index) 
        : arc_index(arc_index), node_index(node_index) {};
    
    unsigned arc_index;
    unsigned node_index;
};

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

    CHLRMGraph(CHLR& chlr);

    CHLRMArc& get_arc(CHLRMArcPos arc_pos);
    void delete_arc(CHLRMArcPos arc_pos);

    unsigned calculate_weight(CHLRMArcPos arc_pos);

private:
    void build_neighbour_index();
};

#endif // ROUTING_KIT_CHLRM_H
