// Implementation of CHLRMArc::get_pos moved from header
#include <routingkit/chlrm.h>

CHLRMArcPos CHLRMArc::get_pos(CHLRMGraph& graph) {
    unsigned arc_index = 0;
    for (const auto& arc : graph.nodes[from].arcs) {
        if (arc.from == from && arc.mid_node == mid_node && arc.to == to && arc.weight == weight && arc.label == label) {
            return CHLRMArcPos(arc_index, from);
        }
        ++arc_index;
    }
    return CHLRMArcPos(inf_weight, inf_weight);
}


CHLRMGraph::CHLRMGraph(CHLRGraph& graph) {
    N_plus = std::unordered_map<std::pair<CHLRMArcPos, CHLRMArcPos>, CHLRMArcPos>();
    N_minus = std::unordered_map<CHLRMArcPos, std::vector<std::pair<CHLRMArcPos, CHLRMArcPos>>>();
    N_equals = std::unordered_map<CHLRMArcPos, std::vector<CHLRMArcPos>>();

    nodes.resize(graph.nodes.size());

    for (unsigned i = 0; i < graph.nodes.size(); ++i) {
        nodes[i].index = i;
        nodes[i].rank = graph.nodes[i].rank;
        nodes[i].lat = graph.nodes[i].lat;
        nodes[i].lon = graph.nodes[i].lon;

        for (unsigned j = 0; j < graph.nodes[i].out_arcs.size(); ++j) {
            const auto& arc = graph.nodes[i].out_arcs[j];
            auto new_arc = CHLRMArc{i, arc.mid_node, arc.other_node, arc.weight, arc.label};
            nodes[i].arcs.emplace_back(new_arc);
            weight[CHLRMArcPos{j, i}] = new_arc.weight;
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
    if(arc_pos.arc_index == inf_weight) {
        return inf_weight; // Arc not found
    }

    unsigned k = inf_weight;
    auto& arc = get_arc(arc_pos);
    assert(arc.is_shortcut());
    arc.cnt = 0;

    for (auto& e : nodes[arc.from].arcs) { 
        if(e.is_shortcut()) continue;
        if(e.to != arc.to) continue;
        if(e.weight == inf_weight || !e.label.is_subset_of(arc.label)) continue;

        k = e.weight;
        arc.cnt = 1;
    }

    for (auto& parents : Nm(arc)) {
        auto& p1 = parents.first;
        auto& p2 = parents.second;

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


void CHLRMGraph::keep_shortcut_dominance(CHLRMArc& arc, MinRankQueue& queue, bool increment) {
    for(auto& e_ : nodes[arc.from].arcs) {
        if(e_.to == arc.to && e_.label.is_subset_of(arc.label) && e_.weight <= arc.weight){
            arc.weight = inf_weight;
            arc.cnt = 0;

            if(std::find(e_.dominant_shortcut_set.begin(), e_.dominant_shortcut_set.end(), &arc) == e_.dominant_shortcut_set.end()) {
                e_.dominant_shortcut_set.push_back(&arc);
            }

            //return; // TODO: can we return here? the paper uses "if e with ... exists then do with it ..." 
        }
    }
    
    if(increment) {
        for (auto* e_ : arc.dominant_shortcut_set) {
            unsigned k = calculate_weight(e_->get_pos(*this));
            if (k < arc.weight) {
                if(!queue.contains(arc, false)) {
                    queue.push(arc, false, *this);
                }

                e_->weight = k;
                arc.dominant_shortcut_set.erase(std::remove(arc.dominant_shortcut_set.begin(), arc.dominant_shortcut_set.end(), e_), arc.dominant_shortcut_set.end());
                nodes[e_->from].arcs.push_back(*e_);
            }
        }
    } else {
        for (auto* e_ : arc.dominant_shortcut_set) {
            if (e_->from != arc.from || e_->to != arc.to || !arc.label.is_subset_of(e_->label)) {
                continue;
            }

            if (e_->weight >= arc.weight) {
                if (!queue.contains(*e_, true)) {
                    queue.push(*e_, true, *this);
                }

                e_->weight = inf_weight;
                e_->cnt = 0;
                arc.dominant_shortcut_set.push_back(e_);
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

void CHLRMGraph::maintenance(CHLRMArc e_o, unsigned w_n, Label l_n) {
    if (e_o.label == l_n && e_o.weight == w_n) return;

    CHLRMArcPos original_arc_pos = e_o.get_pos(*this);

    unsigned w_o = e_o.weight;
    Label l_o = e_o.label;

    MinRankQueue queue(arc_count());

    if (l_n != l_o) {
        e_o.weight = inf_weight;
        auto e_n = CHLRMArc{e_o.from, e_o.mid_node, e_o.to, w_n, l_n};
        e_o.weight = calculate_weight(original_arc_pos);

        if (e_o.weight > w_o) {
            queue.push(e_o, true, *this);
            keep_shortcut_dominance(e_o, queue, true);
        }

        if (e_n.weight > w_n) {
            e_n.weight = w_n;
            queue.push(e_n, false, *this);
            keep_shortcut_dominance(e_n, queue, false);
        }
    } else {
        //e_o.weight = w_n;
        e_o.weight = calculate_weight(original_arc_pos);
        queue.push(e_o, e_o.weight > w_o, *this);
        keep_shortcut_dominance(e_o, queue, e_o.weight > w_o);
    }

    while (!queue.empty()) {
        auto [increment, e] = queue.pop();
        auto pos = e->get_pos(*this);
        if (pos.arc_index == inf_weight) continue; // Arc not found

        for (auto& e_pos : N_equals[pos]) {
            auto& e__pos = N_plus[{pos, e_pos}];
            auto& e__ = get_arc(e__pos);

            if (increment && e__.weight < inf_weight) {
                unsigned k = calculate_weight(e__pos);

                if (e__.weight < k && !queue.contains(e__, true)) {
                    e__.weight = k;
                    queue.push(e__, true, *this);
                    keep_shortcut_dominance(e__, queue, true);
                }
            }

            if (!increment) {
                auto& e_ = get_arc(e_pos);
                if (e__.weight > e->weight + e_.weight) {
                    e__.weight = e->weight + e_.weight;
                    if (!queue.contains(e__, false)) {
                        queue.push(e__, false, *this);
                        keep_shortcut_dominance(e__, queue, false);
                    }
                }
            }
        }
    }
}


void CHLRMGraph::maintenance_optimized(CHLRMArcPos original_arc_pos, unsigned w_n, Label l_n) {
    auto& e_o = get_arc(original_arc_pos);
    assert(!e_o.is_shortcut()); // This condition is not explicitly stated in the paper, but only modifying shortcuts might corrupt the graph structure.

    if (e_o.label == l_n && e_o.weight == w_n) return;

    unsigned w_o = e_o.weight;
    Label l_o = e_o.label;

    auto e_n = CHLRMArc{e_o.from, e_o.mid_node, e_o.to, w_n, l_n};
    MinRankQueue queue(arc_count());

    if (l_n != l_o) {
        if (e_o.weight == w_o && e_o.cnt  < 2) {
            e_o.cnt = 0;
            queue.push(e_o, true, *this);
        }

        if(e_n.weight == w_n) {
            e_n.cnt++;
        }

        if(e_n.weight > w_n) {
            e_n.weight = w_n;
            e_n.cnt = 1;
            queue.push(e_n, false, *this);
        }
    } else {
        if (e_o.weight == w_o) {
            e_o.cnt--;
        }

        if (e_o.weight == w_n) {
            e_o.cnt++;
        }

        if (e_o.weight > w_n) {
            e_o.weight = w_n;
            e_o.cnt = 1;
            queue.push(e_o, false, *this);
        }

        if (e_o.weight < w_n && e_o.cnt < 1) {
            e_o.cnt = 1;
            queue.push(e_o, true, *this);
        }
    }

    while (!queue.empty()) {
        auto [increment, arc] = queue.pop();
    }   
}
