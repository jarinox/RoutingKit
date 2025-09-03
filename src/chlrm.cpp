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
        }
    }
}

// See Algorithm 1: CalWeight in paper
unsigned CHLRMGraph::calculate_weight(CHLRMArc arc) {
    unsigned k = inf_weight;
    arc.cnt = 0;

    if(!arc.is_shortcut() && arc.weight < inf_weight) {
        arc.cnt = 1;
        k = arc.weight;
    }
    
    for (auto& parents : Nm(arc)) {
        auto& p1 = parents.first;
        auto& p2 = parents.second;

        unsigned comb_weight = p1.weight + p2.weight;
        if (comb_weight < k) {
            k = comb_weight;
            arc.cnt = 1;
        } else if (comb_weight == k) {
            arc.cnt++;
        }
    }

    return k;
}

CHLRMArc& CHLRMGraph::get_arc(CHLRMArcPos arc_pos) {
    return nodes[arc_pos.node_index].arcs[arc_pos.arc_index];
}

unsigned CHLRMGraph::w(unsigned from, unsigned to, Label label) {
    unsigned k = inf_weight;

    for (const auto& arc : nodes[from].arcs) {
        if (arc.to != to) continue;
        if (!arc.label.is_subset_of(label)) continue;
        if (k > arc.weight) continue;
        k = arc.weight;
    }

    return k;
}


void CHLRMGraph::keep_shortcut_dominance(CHLRMArc& arc, MinRankQueue& queue, bool increment) {
    bool exists = false;
    for(auto& e_ : nodes[arc.from].arcs) {
        if(e_ == arc) continue;
        if(e_.to == arc.to && e_.label.is_subset_of(arc.label) && e_.weight <= arc.weight){
            arc.weight = inf_weight;
            arc.cnt = 0;

            if(has_dominant_shortcut(e_, arc)) {
                add_dominant_shortcut(e_, arc);
            }

            exists = true;

            //return; // TODO: can we return here? the paper uses "if e with ... exists then do with it ..." 
        }
    }

    if (exists) return;
    
    if(increment) {
        for (auto e_ : get_dominant_shortcuts(arc)) {
            unsigned k = calculate_weight(e_);
            if (k < arc.weight) {
                if(!queue.contains(e_, false)) {
                    queue.push(e_, false, *this);
                }

                auto& e_alt = find_arc(e_);
                if(e_alt.from == invalid_id){
                    e_alt = e_;
                }
                e_.weight = k;
                remove_dominant_shortcut(e_, arc);
                add_arc(e_, true);
            }
        }
    } else {
        unsigned i = 0;
        for (CHLRMArc& e_ : nodes[arc.from].arcs) {
            if (e_.to != arc.to || !arc.label.is_subset_of(e_.label)) {
                continue;
            }

            if (e_.weight >= arc.weight) {
                if (!queue.contains(e_, true)) {
                    queue.push(e_, true, *this);
                }

                e_.weight = inf_weight;
                e_.cnt = 0;
                add_dominant_shortcut(arc, e_);
            }
            i++;
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

void CHLRMGraph::maintenance(CHLRMArcPos e_o_pos, unsigned w_n, Label l_n) {
    auto& e_o_ref = get_arc(e_o_pos);
    if (e_o_ref.label == l_n && e_o_ref.weight == w_n) return;
    unsigned w_o = e_o_ref.weight;
    Label l_o = e_o_ref.label;

    dominant_shortcut_map.clear();
    MinRankQueue queue(nodes.size()*100+64);

    if (l_n != l_o) {
        unsigned from = e_o_ref.from;
        unsigned mid_node = e_o_ref.mid_node;
        unsigned to = e_o_ref.to;

        e_o_ref.weight = inf_weight;

        auto e_n_tmp = CHLRMArc{from, mid_node, to, w(e_o_ref.from, e_o_ref.to, l_n), l_n};
        nodes[from].arcs.push_back(e_n_tmp);
        auto& e_n = nodes[from].arcs.back();

        auto& e_o = get_arc(e_o_pos);
        unsigned k = calculate_weight(e_o);
        e_o.weight = k;

        if (e_o.weight > w_o) {
            queue.push(e_o, true, *this);
            keep_shortcut_dominance(e_o, queue, true);
        }

        if (w(e_n.from, e_n.to, e_n.label) >= w_n) {
            e_n.weight = w_n;
            queue.push(e_n, false, *this);
            keep_shortcut_dominance(e_n, queue, false);
        }
    } else {
        auto& e_o = get_arc(e_o_pos);
        e_o.weight = w_n;
        unsigned k = calculate_weight(e_o);
        e_o.weight = k;

        queue.push(e_o, e_o.weight > w_o, *this);
        keep_shortcut_dominance(e_o, queue, e_o.weight > w_o);
    }

    while (!queue.empty()) {
        auto [increment, e] = queue.pop();
        std::cout << "Processing arc: " << e.from << " -> " << e.to << " w: " << e.weight << " inc: " << increment << std::endl;
        auto partners = Ne(e);

        for (auto e_ : partners) {
            auto e__ = Np(e_, e);
            if (increment && e__.weight < inf_weight) {
                unsigned k = calculate_weight(e__);

                if (e__.weight < k && !queue.contains(e__, true)) {
                    auto& e_alt = find_arc(e__);
                    if(e_alt.from == invalid_id){
                        e_alt = e__;
                    }
                    e_alt.weight = k;
                    queue.push(e__, true, *this);
                    keep_shortcut_dominance(e__, queue, true);
                }
            }

            if (!increment) {
                if (e__.weight > e.weight + e_.weight) {
                    auto& e_alt = find_arc(e__);
                    if(e_alt.from == invalid_id){
                        e_alt = e__;
                    }
                    e_alt.weight = e.weight + e_.weight;
                    if (!queue.contains(e__, false)) {
                        queue.push(e__, false, *this);
                        keep_shortcut_dominance(e__, queue, false);
                    }
                }
            }
        }
    }
}


void MinRankQueue::push(CHLRMArc arc, bool increment, CHLRMGraph& graph) {
    unsigned arc_priority = arc.rank(graph)*100+__builtin_popcount(arc.label.get_label());

    if(queue.contains_id(arc_priority)) {
        if(is_in_queue.find({arc, increment}) != is_in_queue.end()) {
            return;
        }
        unsigned_to_arc[queue.get_key(arc_priority)].push({increment, arc});
    } else {
        queue.push({arc_priority, static_cast<unsigned>(unsigned_to_arc.size())});
        unsigned_to_arc.push_back(std::queue<std::pair<bool, CHLRMArc>>{});
        unsigned_to_arc.back().push({increment, arc});
    }

    is_in_queue.insert({arc, increment});
}

std::pair<bool, CHLRMArc> MinRankQueue::pop() {
    auto vec = queue.peek();

    auto pair = unsigned_to_arc[vec.key].front();
    unsigned_to_arc[vec.key].pop();

    is_in_queue.erase({pair.second, pair.first});

    if(unsigned_to_arc[vec.key].empty()) {
        queue.pop();
    }

    return pair;
}
