#include <routingkit/chlr.h>

CHLRGraph::CHLRGraph(unsigned node_count, const std::vector<unsigned> &tail,
                     const std::vector<unsigned> &head,
                     const std::vector<unsigned> &weight,
                     const std::vector<float> &latitude,
                     const std::vector<float> &longitude,
                     std::vector<Label> &label) {
    nodes.resize(node_count);

    for(unsigned i = 0; i < node_count; ++i){
        nodes[i].lat = latitude[i];
        nodes[i].lon = longitude[i];
    }

    for (unsigned i = 0; i < tail.size(); ++i) {
        add_arc(tail[i], invalid_id, head[i], weight[i], label[i]);
    }
}

void CHLR::build() {
    CHLRGraph contraction_graph = this->graph;
    order.resize(graph.nodes.size());

    MinIDQueue queue(graph.nodes.size());
    unsigned contracted_node_count = 0;

    for (unsigned i = 0; i < graph.nodes.size(); ++i) {
        queue.push({i, estimate_node_importance(graph, i)});
    }

    while (!queue.empty()) {
        auto [node_id, importance] = queue.pop();
        order[contracted_node_count++] = node_id;
        graph.nodes[node_id].rank = contracted_node_count;
        contraction_graph.nodes[node_id].rank = contracted_node_count;

        CHLRNode &node = graph.nodes[node_id];

        // Raise neighbour levels for all arcs connected to this node
        // This is used for estimating node importance
        std::set<unsigned> neighbors;
        for (const auto &arc : node.in_arcs) {
            neighbors.insert(arc.other_node);
            graph.nodes[arc.other_node].neighbour_level++;
        }
        for (const auto &arc : node.out_arcs) {
            if (neighbors.find(arc.other_node) == neighbors.end()) {
                graph.nodes[arc.other_node].neighbour_level++;
                neighbors.insert(arc.other_node);
            }
        }
        for (const auto &neighbor : neighbors) {
            if (neighbor == node_id) continue;  // Skip self-loops
            if (!queue.contains_id(neighbor)) continue;

            unsigned new_importance = estimate_node_importance(graph, neighbor);
            unsigned current_importance = queue.get_key(neighbor);

            if (new_importance < current_importance) {
                queue.decrease_key({neighbor, new_importance});
            } else if (new_importance > current_importance) {
                queue.increase_key({neighbor, new_importance});
            }
        }

        assert(!queue.contains_id(node_id));

        std::cout << "Contracting node " << node_id
             << " with arc combinations " << node.in_arcs.size() * node.out_arcs.size()
             << ", contracted nodes: " << contracted_node_count
             << ", queue size: " << queue.size() << std::endl;

        // Contract the node
        node.sort_arcs_for_weight();
        for (const auto &in_arc : graph.nodes[node_id].in_arcs) {
            if (in_arc.other_node == node_id) continue;  // Skip self-loops
            if (!queue.contains_id(in_arc.other_node)) // TODO: slow, may be optimized
                continue;  // Ensure rank(other_node) > rank(node_id)

            DijkstraLR dijkstra(graph, in_arc.other_node);

            for (const auto &out_arc : graph.nodes[node_id].out_arcs) {
                if (in_arc.other_node == out_arc.other_node)
                    continue;  // Skip self-loops
                if (!queue.contains_id(out_arc.other_node)) // TODO: slow, may be optimized
                    continue;  // Ensure rank(other_node) > rank(node_id)

                Label newLabel = in_arc.label.unite(out_arc.label);
                Label R = newLabel;
                R.invert();

                unsigned shortcut_weight = in_arc.weight + out_arc.weight;

                unsigned witness_weight = dijkstra.witness_search(
                    out_arc.other_node, R, [&](unsigned bypass_node) {
                        return node_id != bypass_node;
                    });

                if (shortcut_weight < witness_weight) {
                    // Create a shortcuts in graph
                    graph.add_arc(in_arc.other_node, node_id, out_arc.other_node, shortcut_weight, newLabel);
                    contraction_graph.add_arc(in_arc.other_node, node_id, out_arc.other_node, shortcut_weight, newLabel);
                }
            }
        }

        graph.remove_incident_arcs(node_id);
    }

    graph = std::move(contraction_graph);
}

unsigned estimate_node_importance(const CHLRGraph &graph, unsigned node_id) {
    const CHLRNode &node = graph.nodes[node_id];

    unsigned in_deg = node.in_arcs.size();
    unsigned out_deg = node.out_arcs.size();

    unsigned estimated_shortcuts = in_deg * out_deg;
    unsigned arcs_removed = in_deg + out_deg;
    unsigned edge_difference = estimated_shortcuts > arcs_removed ? estimated_shortcuts - arcs_removed : 0;

    unsigned neighbor_level_sum = 0;
    std::set<unsigned> neighbors;
    for (const auto &arc : node.in_arcs) {
        neighbors.insert(arc.other_node);
        neighbor_level_sum += graph.nodes[arc.other_node].neighbour_level;
    }
    for (const auto &arc : node.out_arcs) {
        if (neighbors.find(arc.other_node) == neighbors.end()) {
            neighbor_level_sum += graph.nodes[arc.other_node].neighbour_level;
        }
    }

    // Combine factors (weights can be tuned)
    return 1000 * edge_difference + 100 * neighbor_level_sum + in_deg + out_deg;
}

unsigned DijkstraLR::witness_search(
    unsigned end_node, Label restriction,
    const std::function<bool(unsigned)> &is_valid_node) {
    if (visited_nodes.empty()) {
        visited_nodes.clear();
        queue.clear();
        queue.push({start_node, 0});
        tentative_distance[start_node] = 0;
    } else {
        if (visited_nodes.find(end_node) != visited_nodes.end()) {
            return tentative_distance[end_node];
        }
    }

    unsigned pop_count = 0;
    while (!queue.empty() && pop_count < max_pop_count) {
        auto p = queue.pop();
        unsigned current_node = p.id;
        unsigned current_distance = p.key;

        if (current_node == end_node) {
            return current_distance;
        }

        visited_nodes.insert(current_node);

        for (const auto &arc : graph.nodes[current_node].out_arcs) {
            if (!arc.label.is_allowed(restriction)) continue;
            if (!is_valid_node(arc.other_node)) continue;

            unsigned new_distance = current_distance + arc.weight;
            if (new_distance >= tentative_distance[arc.other_node]) continue;

            tentative_distance[arc.other_node] = new_distance;
            if (!queue.contains_id(arc.other_node)) {
                queue.push({arc.other_node, new_distance});
            } else {
                queue.decrease_key({arc.other_node, new_distance});
            }
        }
    }

    return std::numeric_limits<unsigned>::max();
}


void CHLRGraph::remove_incident_arcs(unsigned node_id) {
    auto &node = nodes[node_id];
    for (const auto &in_arc : node.in_arcs) {
        auto &other_node = nodes[in_arc.other_node];
        other_node.out_arcs.erase(
            std::remove_if(other_node.out_arcs.begin(), other_node.out_arcs.end(),
                           [&](const CHLRArc &arc) { return arc.other_node == node_id; }),
            other_node.out_arcs.end());
    }
    for (const auto &out_arc : node.out_arcs) {
        auto &other_node = nodes[out_arc.other_node];
        other_node.in_arcs.erase(
            std::remove_if(other_node.in_arcs.begin(), other_node.in_arcs.end(),
                           [&](const CHLRArc &arc) { return arc.other_node == node_id; }),
            other_node.in_arcs.end());
    }

    node.in_arcs.clear();
    node.out_arcs.clear();
    node.in_arcs.shrink_to_fit();
    node.out_arcs.shrink_to_fit();
}


void CHLRGraph::add_arc(unsigned from, unsigned mid_node, unsigned to, unsigned weight, Label label) {
    nodes[from].out_arcs.push_back({
        .other_node = to,
        .mid_node = mid_node,
        .weight = weight,
        .label = label,
    });

    nodes[to].in_arcs.push_back({
        .other_node = from,
        .mid_node = mid_node,
        .weight = weight,
        .label = label,
    });
}


void CHLRQuery::extract_directional_graphs() {
    forward.nodes.resize(graph.nodes.size());
    backward.nodes.resize(graph.nodes.size());

    for (unsigned i = 0; i < graph.nodes.size(); ++i) {
        // Copy node metadata to forward and backward graphs
        forward.nodes[i].lat = graph.nodes[i].lat;
        forward.nodes[i].lon = graph.nodes[i].lon;
        forward.nodes[i].rank = graph.nodes[i].rank;
        backward.nodes[i].lat = graph.nodes[i].lat;
        backward.nodes[i].lon = graph.nodes[i].lon;
        backward.nodes[i].rank = graph.nodes[i].rank;
        
        // Clean up existing arcs
        forward.nodes[i].out_arcs.clear();
        backward.nodes[i].out_arcs.clear();
    }

    for (unsigned i = 0; i < graph.nodes.size(); ++i) {
        // Add arcs to forward/backward graph
        for (auto arc : graph.nodes[i].out_arcs) {
            unsigned from_rank = graph.nodes[i].rank;
            unsigned to_rank = graph.nodes[arc.other_node].rank;

            if (from_rank < to_rank) {
                forward.nodes[i].out_arcs.push_back(arc);
            } else {
                // reverse backward arcs
                backward.nodes[arc.other_node].out_arcs.push_back({
                    .other_node = i,
                    .mid_node = arc.mid_node,
                    .weight = arc.weight,
                    .label = arc.label,
                });
            }
        }
    }
}

void CHLRQuery::run() {
    MinIDQueue forward_queue = MinIDQueue(graph.nodes.size());
    MinIDQueue backward_queue = MinIDQueue(graph.nodes.size());
    std::vector<unsigned> forward_distance;
    std::vector<unsigned> backward_distance;
    TimestampFlags was_forward_pushed(graph.nodes.size());
    TimestampFlags was_backward_pushed(graph.nodes.size());
    std::vector<unsigned> forward_predecessor_node;
    std::vector<unsigned> backward_predecessor_node;
    std::vector<unsigned> forward_predecessor_arc;
    std::vector<unsigned> backward_predecessor_arc;

    meeting_node = invalid_id;

    forward_distance.resize(graph.nodes.size(), std::numeric_limits<unsigned>::max());
    backward_distance.resize(graph.nodes.size(), std::numeric_limits<unsigned>::max());
    forward_predecessor_node.resize(graph.nodes.size(), invalid_id);
    backward_predecessor_node.resize(graph.nodes.size(), invalid_id);
    forward_predecessor_arc.resize(graph.nodes.size(), invalid_id);
    backward_predecessor_arc.resize(graph.nodes.size(), invalid_id);

    forward_distance[start_node] = 0;
    backward_distance[end_node] = 0;

    forward_queue.push({start_node, 0});
    backward_queue.push({end_node, 0});

    was_forward_pushed.set(start_node);
    was_backward_pushed.set(end_node);

    bool forward_finished = false;
    bool backward_finished = false;

    bool search_forward = true;

    while((!forward_finished || !backward_finished) && meeting_node == invalid_id) {
        if(search_forward) {
            settle(
                forward_queue, forward_distance, forward_predecessor_node,
                forward_predecessor_arc, was_forward_pushed, forward_finished,
                backward_queue, backward_distance, backward_predecessor_node,
                backward_predecessor_arc, was_backward_pushed, meeting_node,
                search_forward, forward, restriction
            );
        } else {
            settle(
                backward_queue, backward_distance, backward_predecessor_node,
                backward_predecessor_arc, was_backward_pushed, backward_finished,
                forward_queue, forward_distance, forward_predecessor_node,
                forward_predecessor_arc, was_forward_pushed, meeting_node,
                search_forward, backward, restriction
            );
        }
        

        if(forward_queue.empty()) {
            forward_finished = true;
            search_forward = false;
        }

        if(backward_queue.empty()) {
            backward_finished = true;
            search_forward = true;
        }
    }

    _forward_predecessor_arc = std::move(forward_predecessor_arc);
    _backward_predecessor_arc = std::move(backward_predecessor_arc);
    _forward_predecessor_node = std::move(forward_predecessor_node);
    _backward_predecessor_node = std::move(backward_predecessor_node);
}


void CHLRQuery::settle(
    MinIDQueue &queue, std::vector<unsigned> &distance,
    std::vector<unsigned> &predecessor_node,
    std::vector<unsigned> &predecessor_arc,
    TimestampFlags &was_pushed, bool &finished,
    MinIDQueue &other_queue, std::vector<unsigned> &other_distance,
    std::vector<unsigned> &other_predecessor_node,
    std::vector<unsigned> &other_predecessor_arc,
    TimestampFlags &other_was_pushed, unsigned &meeting_node,
    bool &search_forward, CHLRGraph &graph, Label restriction) {

    assert(!finished);
    assert(!queue.empty());

    auto p = queue.pop();
    unsigned current_node = p.id;
    unsigned current_distance = p.key;

    if (other_was_pushed.is_set(current_node)) {
        meeting_node = current_node;
        return; // Already processed by other search, meeting node found
    }

    was_pushed.set(current_node);

    unsigned j = 0;
    for (const auto &arc : graph.nodes[current_node].out_arcs) {
        if (!arc.label.is_allowed(restriction)) continue;

        unsigned next_node = arc.other_node;
        unsigned new_distance = current_distance + arc.weight;

        if (new_distance < distance[next_node]) {
            distance[next_node] = new_distance;
            predecessor_node[next_node] = current_node;
            predecessor_arc[next_node] = j;

            if (!queue.contains_id(next_node)) {
                queue.push({next_node, new_distance});
            } else {
                queue.decrease_key({next_node, new_distance});
            }
        }

        if (other_was_pushed.is_set(next_node)) {
            meeting_node = next_node;
            return;
        }

        j++;
    }
}


std::vector<CHLRArc> CHLRQuery::get_arc_path() {
    std::vector<CHLRArc> path;
    if (meeting_node == invalid_id) return path;

    unsigned current_node = meeting_node;
    
    while (current_node != start_node) {
        unsigned predecessor = _forward_predecessor_node[current_node];
        unsigned arc_index = _forward_predecessor_arc[current_node];

        if (predecessor == invalid_id || arc_index == invalid_id) break;
        
        CHLRArc arc = forward.nodes[predecessor].out_arcs[arc_index];
        if(arc.is_shortcut()) {
            path.push_back({
                .other_node = arc.mid_node,
                .mid_node = invalid_id,
                .weight = arc.weight,
                .label = arc.label
            });
            path.push_back({
                .other_node = arc.other_node,
                .mid_node = invalid_id,
                .weight = arc.weight,
                .label = arc.label
            });
        } else {
            path.push_back(arc);
        }

        current_node = predecessor;
    }

    std::reverse(path.begin(), path.end());

    current_node = meeting_node;

    while (current_node != end_node) {
        unsigned predecessor = _backward_predecessor_node[current_node];
        unsigned arc_index = _backward_predecessor_arc[current_node];

        if (predecessor == invalid_id || arc_index == invalid_id) break;

        CHLRArc arc = backward.nodes[predecessor].out_arcs[arc_index];
        
        if(arc.is_shortcut()) {
            path.push_back({
                .other_node = arc.mid_node,
                .mid_node = invalid_id,
                .weight = arc.weight,
                .label = arc.label
            });
            path.push_back({
                .other_node = predecessor,
                .mid_node = invalid_id,
                .weight = arc.weight,
                .label = arc.label
            });
        } else {
            path.push_back({
                .other_node = predecessor,
                .mid_node = invalid_id,
                .weight = arc.weight,
                .label = arc.label
            });
        }

        current_node = predecessor;
    }

    return path;
}
