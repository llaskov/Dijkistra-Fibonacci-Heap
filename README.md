# Dijkistra-Fibonacci-Heap
Single source shortest path using Dijkstra's algorithm using Fibonacci heap priority queue.

The efficient implementation of the Dijkstra's algorithm adopts the Fibonacci heap data structure, that ensures computational complexity of O(n log n + m). The computational complexity is proved based on the amortized analysis (see T. H. Cormen, C. E. Leiserson, R. L. Rivest, and C. Stein, Introduction to algorithms, 3rd ed. Cambridge, Massachusetts: The MIT Press, 2009. doi: 10.5555/1614191).

fibonacci-heap.jl   --> implementation of the Fibonacci heap data structure.
dijkstra.jl         --> implementation of the classical Dijkstra's algorithm using Fibonacci Heap data structure.
mtrx14.txt          --> an example weighed graph with 14 vertices. 
