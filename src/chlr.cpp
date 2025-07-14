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
        nodes[tail[i]].out_arcs.push_back({
            .other_node = head[i],
            .mid_node = invalid_id,  // no shortcut
            .weight = weight[i],
            .label = label[i],
        });
        nodes[head[i]].in_arcs.push_back({
            .other_node = tail[i],
            .mid_node = invalid_id,
            .weight = weight[i],
            .label = label[i],
        });
    }
}

void CHLR::build() {
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

        CHLRNode &node = graph.nodes[node_id];
        node.rank = contracted_node_count;


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
                    // Create a shortcut in graph
                    graph.nodes[in_arc.other_node].out_arcs.push_back({
                        .other_node = out_arc.other_node,
                        .mid_node = node_id,
                        .weight = shortcut_weight,
                        .label = newLabel,
                    });

                    graph.nodes[out_arc.other_node].in_arcs.push_back({
                        .other_node = in_arc.other_node,
                        .mid_node = node_id,
                        .weight = shortcut_weight,
                        .label = newLabel,
                    });
                }
            }
        }

        // Remove only non-shortcut edges from search graph
        auto& in_arcs = graph.nodes[node_id].in_arcs;
        in_arcs.erase(
            std::remove_if(in_arcs.begin(), in_arcs.end(),
                [](const CHLRArc& arc) { return arc.mid_node == invalid_id; }),
            in_arcs.end()
        );
        auto& out_arcs = graph.nodes[node_id].out_arcs;
        out_arcs.erase(
            std::remove_if(out_arcs.begin(), out_arcs.end(),
                [](const CHLRArc& arc) { return arc.mid_node == invalid_id; }),
            out_arcs.end()
        );
    }
}

unsigned estimate_node_importance(const CHLRGraph &graph, unsigned node_id) {
    CHLRNode node = graph.nodes[node_id];

    unsigned deleted_neighbor_count = 1;
    unsigned added_arc_count = 1;
    unsigned removed_arc_count = 1;

    std::set<unsigned> neighbors;
    for (const auto &arc : node.in_arcs) {
        neighbors.insert(arc.other_node);
        deleted_neighbor_count += graph.nodes[arc.other_node].neighbour_level;
    }

    for (const auto &arc : node.out_arcs) {
        if (neighbors.find(arc.other_node) == neighbors.end()) {
            deleted_neighbor_count +=
                graph.nodes[arc.other_node].neighbour_level;
        }
    }

    return 1 + 1000 * deleted_neighbor_count +
           (1000 * added_arc_count) / removed_arc_count; // TODO: improve this heuristic, however irrelevant for correctness
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
