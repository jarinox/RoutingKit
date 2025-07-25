#include <routingkit/chlrm.h>
#include <functional>

// Provide a hash specialization for CHLRMArcPos
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

CHLRMGraph::CHLRMGraph(CHLR& chlr) {
    N_plus = std::unordered_map<std::pair<CHLRMArcPos, CHLRMArcPos>, CHLRMArcPos>();
    N_minus = std::unordered_map<CHLRMArcPos, std::vector<std::pair<CHLRMArcPos, CHLRMArcPos>>>();
    N_equals = std::unordered_map<CHLRMArcPos, std::vector<CHLRMArcPos>>();

    auto& g = chlr.graph;
    nodes.resize(g.nodes.size());

    for (unsigned i = 0; i < g.nodes.size(); ++i) {
        nodes[i].index = i;
        nodes[i].rank = 0;
        for (const auto& arc : g.nodes[i].out_arcs) {
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
                for (unsigned l = 0; l < nodes[arc.to].arcs.size(); ++l) {
                    auto& e2 = nodes[arc.to].arcs[l];
                    if (!e2.is_shortcut() || e2.to != arc.to) continue;

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
