# Steiner-Tree
A well-known method to approximate classical Steiner Tree problem is MST Heuristic. It is a polynomial time algorithm. In this algorithm, we first find shortest path from each terminal node to every other terminal node. Then we construct a metric closure with only terminal nodes and shortest paths between them as edges. Then we find an MST on the computed metric closure. Finally the MST is transformed back to a Steiner tree by replacing each edge with the shortest path and some straightforward postprocessing to remove redundant edges.

### Contributors
[Atul Anand](https://github.com/atul2938) and [Tushar Kadian](https://github.com/tusharkadian)