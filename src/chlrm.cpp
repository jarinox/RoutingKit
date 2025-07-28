#include <routingkit/chlrm.h>

CHLRMGraph::CHLRMGraph(CHLRGraph& graph) {
    N_plus = std::unordered_map<std::pair<CHLRMArcPos, CHLRMArcPos>, CHLRMArcPos>();
    N_minus = std::unordered_map<CHLRMArcPos, std::vector<std::pair<CHLRMArcPos, CHLRMArcPos>>>();
    N_equals = std::unordered_map<CHLRMArcPos, std::vector<CHLRMArcPos>>();

    nodes.resize(graph.nodes.size());

    for (unsigned i = 0; i < graph.nodes.size(); ++i) {
        nodes[i].index = i;
        nodes[i].rank = 0;
        nodes[i].lat = graph.nodes[i].lat;
        nodes[i].lon = graph.nodes[i].lon;

        for (const auto& arc : graph.nodes[i].out_arcs) {
            auto new_arc = CHLRMArc{i, arc.mid_node, arc.other_node, arc.weight, arc.label};
            nodes[i].arcs.emplace_back(new_arc);
        }
    }

    build_neighbour_index();
}

// Builds index structures for N+, N= and N-
void CHLRMGraph::build_neighbour_index() {
    for (unsigned i = 0; i < nodes.size(); ++i) {
        auto& node = nodes[i];
        for (unsigned j = 0; j < node.arcs.size(); ++j) {
            auto& arc = node.arcs[j];
            if(!arc.is_shortcut()) continue;

            for (unsigned k = 0; k < node.arcs.size(); ++k) {
                auto& e1 = node.arcs[k];
                if (!arc.is_shortcut() || e1.to == arc.to) continue;
                if (nodes[e1.to].rank > node.rank) continue;
                if (nodes[e1.to].rank > nodes[arc.to].rank) continue;

                for (unsigned l = 0; l < nodes[arc.to].arcs.size(); ++l) {
                    auto& e2 = nodes[arc.to].arcs[l];
                    if (!e2.is_shortcut() || e2.to != arc.to) continue;
                    if (e1.label.unite(e2.label) != arc.label) continue;

                    CHLRMArcPos e1_pos{i, j};
                    CHLRMArcPos e2_pos{i, k};
                    CHLRMArcPos e3_pos{arc.to, l};

                    N_plus[{e1_pos, e2_pos}] = e3_pos;
                    N_minus[e3_pos].emplace_back(e1_pos, e2_pos);
                    N_equals[e2_pos].emplace_back(e3_pos);
                    N_equals[e3_pos].emplace_back(e2_pos);
                }
            }
        }
    }
}

// See Algorithm 1: CalWeight in paper
unsigned CHLRMGraph::calculate_weight(CHLRMArcPos arc_pos) {
    auto& arc = get_arc(arc_pos);

    if(!arc.is_shortcut()){
        if(arc.weight != inf_weight)
            arc.cnt = 1;
        return arc.weight;
    }

    unsigned k = inf_weight;

    for (auto& parents : N_minus[arc_pos]) {
        auto& p1 = get_arc(parents.first);
        auto& p2 = get_arc(parents.second);

        unsigned comb_weight = p1.weight + p2.weight;
        if (comb_weight < k) {
            k = comb_weight;
            arc.cnt = 1;
        } else if (comb_weight == k)
        {
            arc.cnt++;
        }
    }

    return k;
}

CHLRMArc& CHLRMGraph::get_arc(CHLRMArcPos arc_pos) {
    return nodes[arc_pos.node_index].arcs[arc_pos.arc_index];
}


void CHLRMGraph::keep_shortcut_dominance(CHLRMArcPos arc_pos, MinIDQueue& queue, std::vector<std::pair<CHLRMArcPos, bool>>& unsigned_to_arc_pos, bool increment) {
    auto& arc = get_arc(arc_pos);
}

void CHLRMGraph::maintenance(CHLRMArcPos arc_pos, unsigned new_weight, Label new_label) {
    auto& arc = get_arc(arc_pos);
    assert(!arc.is_shortcut()); // This condition is not explicitly stated in the paper, but only modifying shortcuts might corrupt the graph structure.
    
    MinIDQueue queue(nodes.size());
    std::vector<std::pair<CHLRMArcPos, bool>> unsigned_to_arc_pos;
    unsigned_to_arc_pos.reserve(nodes.size());

    unsigned original_weight = arc.weight;

    if(arc.label != new_label) {
        // TODO: There is a mistake regarding the inital weight in the paper because w(new_arc) is undefined, it is not trivial how to set it.
        auto new_arc = CHLRMArc{arc.from, arc.mid_node, arc.to, original_weight, new_label};
        nodes[arc.from].arcs.push_back(new_arc);
        auto new_arc_pos = CHLRMArcPos{static_cast<unsigned>(nodes[arc.from].arcs.size() - 1), arc.from};

        arc.weight = inf_weight;
        arc.weight = calculate_weight(arc_pos);

        if (arc.weight > original_weight) {
            queue.push({arc.rank(*this), static_cast<unsigned>(unsigned_to_arc_pos.size())});
            unsigned_to_arc_pos.push_back({arc_pos, true});
            keep_shortcut_dominance(arc_pos, queue, unsigned_to_arc_pos, true);
        }

        if (new_arc.weight > new_weight) {
            new_arc.weight = new_weight;
            new_arc.label = new_label;
            queue.push({new_arc.rank(*this), static_cast<unsigned>(unsigned_to_arc_pos.size())});
            unsigned_to_arc_pos.push_back({new_arc_pos, false});
            keep_shortcut_dominance(new_arc_pos, queue, unsigned_to_arc_pos, false);
        }
    } else {
        arc.weight = new_weight;
        arc.weight = calculate_weight(arc_pos);
        queue.push({arc.rank(*this), static_cast<unsigned>(unsigned_to_arc_pos.size())});
        unsigned_to_arc_pos.push_back({arc_pos, arc.weight > original_weight});
    }

    while(!queue.empty()) {
        auto [key, id] = queue.pop();
        auto& arc_pos = unsigned_to_arc_pos[id].first;
        bool increment = unsigned_to_arc_pos[id].second;

        for(auto& partner : N_equals[arc_pos]) {
            auto child_pos = N_plus[{arc_pos, partner}];
            auto& child_arc = get_arc(child_pos);
            if (child_arc.weight == inf_weight) continue;

            if (increment) {
                unsigned k = calculate_weight(child_pos);
                // TODO: and not contained in queue. Is this implicitly true?
                if (child_arc.weight < k) {
                    child_arc.weight = k;
                    queue.push({child_arc.rank(*this), static_cast<unsigned>(unsigned_to_arc_pos.size())});
                    unsigned_to_arc_pos.push_back({child_pos, true});
                    keep_shortcut_dominance(child_pos, queue, unsigned_to_arc_pos, true);
                }
            } else {
                if (child_arc.weight > get_arc(partner).weight + get_arc(arc_pos).weight) {
                    child_arc.weight = get_arc(partner).weight + get_arc(arc_pos).weight;
                    queue.push({child_arc.rank(*this), static_cast<unsigned>(unsigned_to_arc_pos.size())});
                    unsigned_to_arc_pos.push_back({child_pos, false});
                    keep_shortcut_dominance(child_pos, queue, unsigned_to_arc_pos, false);
                }   
                
            }
        }
    }
}

CHLRGraph CHLRMGraph::to_chlr() {
    CHLRGraph chlr_graph;
    chlr_graph.nodes.resize(nodes.size());

    for (unsigned i = 0; i < nodes.size(); ++i) {
        chlr_graph.nodes[i] = nodes[i].to_chlr();
    }

    return chlr_graph;
}

unsigned CHLRMArc::rank(CHLRMGraph& graph) {
    return std::min(graph.nodes[from].rank, graph.nodes[to].rank);
}
