#include <routingkit/dch.h>

CHLRGraph DCHGraph::to_chlr() {
    CHLRGraph chlr_graph;
    chlr_graph.nodes.resize(nodes.size());

    for (unsigned i = 0; i < nodes.size(); ++i) {
        chlr_graph.nodes[i].rank = nodes[i].rank;
        chlr_graph.nodes[i].lat = nodes[i].lat;
        chlr_graph.nodes[i].lon = nodes[i].lon;

        for (const auto& arc : nodes[i].arcs) {
            chlr_graph.add_arc(arc.from, arc.mid_node, arc.to, arc.weight, arc.label);
        }

        chlr_graph.nodes[i].sort_arcs_for_weight();
    }

    return chlr_graph;
}

DCHGraph::DCHGraph(CHLRGraph& graph) {
    nodes.resize(graph.nodes.size());

    for (unsigned i = 0; i < graph.nodes.size(); ++i) {
        nodes[i].node_index = i;
        nodes[i].rank = graph.nodes[i].rank;
        nodes[i].lat = graph.nodes[i].lat;
        nodes[i].lon = graph.nodes[i].lon;

        for (unsigned j = 0; j < graph.nodes[i].out_arcs.size(); ++j) {
            const auto& arc = graph.nodes[i].out_arcs[j];
            add_arc(i, arc.mid_node, arc.other_node, arc.weight, arc.label);
        }
    }
}

void DCHGraph::add_arc(CHMArc arc) {
    arc.arc_index = nodes[arc.from].arcs.size();
    arc.in_graph = true;
    nodes[arc.from].arcs.push_back(arc);
}

void DCHGraph::add_arc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label) {
    add_arc(CHMArc{from, mid_node, to, weight, label});
}

CHMArc& DCHGraph::get(CHMArcPos pos) {
    return nodes[pos.node_index].arcs[pos.arc_index];
}

CHMArc& DCHGraph::ref(CHMArc arc) {
    assert(arc.in_graph);
    return get(arc.get_pos());
}

void DCHQueue::push(CHMArc arc, DCHGraph& graph) {
    unsigned arc_priority = std::min(graph.nodes[arc.from].rank, graph.nodes[arc.to].rank);

    if(queue.contains_id(arc_priority)) {
        if(is_in_queue.find(arc) != is_in_queue.end()) {
            return;
        }
        unsigned_to_arc[queue.get_key(arc_priority)].push(arc);
    } else {
        queue.push({arc_priority, static_cast<unsigned>(unsigned_to_arc.size())});
        unsigned_to_arc.push_back(std::queue<CHMArc>{});
        unsigned_to_arc.back().push(arc);
    }

    is_in_queue.insert(arc);
}

CHMArc DCHQueue::pop() {
    auto vec = queue.peek();

    auto arc = unsigned_to_arc[vec.key].front();
    unsigned_to_arc[vec.key].pop();

    is_in_queue.erase(arc);

    if(unsigned_to_arc[vec.key].empty()) {
        queue.pop();
    }

    return arc;
}

std::pair<unsigned, std::vector<unsigned>> DCHGraph::dijkstra(unsigned from, unsigned to, Label profile) {
    std::vector<unsigned> first_out;
    std::vector<unsigned> head;
    std::vector<unsigned> tail;
    std::vector<Label> labels;
    std::vector<unsigned> weights;

    first_out.resize(nodes.size() + 1);
    first_out[0] = 0;
    unsigned arc_count = 0;
    for (unsigned i = 0; i < nodes.size(); ++i) {
        auto arcs = nodes[i].arcs;

        std::sort(arcs.begin(), arcs.end(), [](const CHMArc& a, const CHMArc& b) {
            return a.weight < b.weight;
        });

        for (const auto& arc : arcs) {
            if(arc.is_shortcut()) continue;
            if(arc.weight >= inf_weight) continue;
            head.push_back(arc.to);
            tail.push_back(arc.from);
            labels.push_back(arc.label);
            assert(arc.weight > 0);
            weights.push_back(arc.weight);
            arc_count++;
        }
        first_out[i + 1] = arc_count;
    }

    Dijkstra dij = Dijkstra(first_out, tail, head);

    dij.add_source(from).set_labels(labels).set_profile(profile);

    while(!dij.is_finished()){
        auto settle_result = dij.settle([&](unsigned arc, unsigned distance){
            return weights[arc];
        });
        if(settle_result.node == to){
            break;
        }
    }

    if(dij.get_distance_to(to) == 0u){
        return {inf_weight, {}};
    }

    auto path = dij.get_node_path_to(to);
    if(path.empty()){
        return {inf_weight, {}};
    }

    return {dij.get_distance_to(to), path};
}

/// Computes the upward shortcut pairs <p2, child> of p1 as defined by Zhang and Yu (2022)
std::vector<std::pair<CHMArc, CHMArc>> DCHGraph::SCPPlus(CHMArc p1) {
    std::vector<std::pair<CHMArc, CHMArc>> pairs;

    std::vector<CHMArc> ne;
    
    if (nodes[p1.from].rank > nodes[p1.to].rank) {
        // p1 is a downward arc
        for (CHMArc p2 : nodes[p1.to].arcs) {
            if (nodes[p2.to].rank < nodes[p1.to].rank) continue;
            ne.push_back(p2);
        }
    } else {
        // p1 is an upward arc
        for (CHMNode& node : nodes) {
            for (CHMArc p2 : node.arcs) {
                if (p2.to != p1.from) continue;
                if (p2.from == p1.to) continue; // skip loops
                if (nodes[p2.from].rank < nodes[p1.from].rank) continue;
                ne.push_back(p2);
            }
        }
    }

    for (CHMArc p2 : ne) {
        CHMArc _p1 = p1;
        CHMArc _p2 = p2;

        if(_p1.from == _p2.to) {
            // make sure that Phi(w) > Phi(u)
            if (nodes[p1.from].rank > nodes[p2.from].rank) continue;

            std::swap(_p1, _p2);
        } else {
            // make sure that Phi(w) > Phi(u)
            if (nodes[p2.to].rank < nodes[p1.to].rank) continue;
        }

        assert(_p1.to == _p2.from);


        // Find child of p1 and p2
        for (CHMArc child : nodes[_p1.from].arcs) {
            if (child.to != _p2.to) continue;         // correct direction
            if (child.mid_node != _p1.to) continue;   // correct mid node
            if (child.weight >= inf_weight) continue; // handle infinite weight

            pairs.push_back({p2, child});
        }
    }

    return pairs;
}

/// Computes the downward shortcut pairs <p1, p2> of child as defined by Zhang and Yu (2022)
std::vector<std::pair<CHMArc, CHMArc>> DCHGraph::SCPMinus(CHMArc child) {
    std::vector<std::pair<CHMArc, CHMArc>> pairs;

    for (CHMArc p1 : nodes[child.from].arcs) {
        if (nodes[child.from].rank < nodes[p1.to].rank) continue; // ensure p1 is downward
        if (child.mid_node != p1.to) continue; // ensure p1 is a part of the child

        for (CHMArc p2 : nodes[p1.to].arcs) {
            if (p2.to != child.to) continue;
            if (nodes[p2.to].rank < nodes[p1.to].rank) continue; // ensure p2 is upward

            pairs.push_back({p1, p2});
        }
    }

    return pairs;
}

/// Decrease the weight of an original arc in the Contraction Hierarchy. Maintains a valid CH index.
void DCHGraph::DCHMinus(CHMArcPos e_o_pos, unsigned w_n) {
    CHMArc& e_o = get(e_o_pos);
    unsigned w_o = e_o.weight;

    assert(w_o > w_n && "DCHMinus can only be used to decrease weights");

    DCHQueue queue(nodes.size());

    e_o.weight = w_n;
    queue.push(e_o, *this);

    while(!queue.empty()) {
        CHMArc p1 = queue.pop();

        for(std::pair<CHMArc, CHMArc> scp : SCPPlus(p1)) {
            CHMArc p2 = scp.first;
            CHMArc& child = ref(scp.second);

            if (p1.weight + p2.weight < child.weight) {
                child.weight = p1.weight + p2.weight;
                queue.push(child, *this);
            }
        }
    }
}


/// Increase the weight of an original arc in the Contraction Hierarchy. Maintains a valid CH index.
void DCHGraph::DCHPlus(CHMArcPos e_o_pos, unsigned w_n) {
    CHMArc& e_o = get(e_o_pos);
    unsigned w_o = e_o.weight;

    assert(w_o < w_n && "DCHPlus can only be used to increase weights");

    DCHQueue queue(nodes.size());

    queue.push(e_o, *this);
    e_o.weight = w_n;

    while(!queue.empty()) {
        CHMArc arc = queue.pop();

        for(std::pair<CHMArc, CHMArc> scp : SCPPlus(arc)) {
            CHMArc p2 = scp.first;
            CHMArc child = scp.second;

            if (arc.weight + p2.weight == child.weight) {
                queue.push(child, *this);
            }
        }

        ref(arc).weight = compute_weight(arc);
    }
}


unsigned DCHGraph::compute_weight(CHMArc arc) {
    unsigned k = inf_weight;

    if(arc.mid_node == invalid_id) {
        return ref(arc).weight;
    }

    for(std::pair<CHMArc, CHMArc> scp : SCPMinus(arc)) {
        CHMArc p1 = scp.first;
        CHMArc p2 = scp.second;

        if (p1.weight + p2.weight < k) {
            k = p1.weight + p2.weight;
        }
    }

    return k;
}
