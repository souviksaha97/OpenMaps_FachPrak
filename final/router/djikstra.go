package router

import (
	"container/heap"
	"final/generator"
	"final/types"
	"math"
	"time"
)

func Djikstra(nodes [][2]float64, edges [][4]int, edgeweights [][4]int, src int, dst int) ([]int, []int) {
	// Initialize GraphData
	// init_time := time.Now()
	// total_push_time := time.Duration(0)
	total_loop2_time := time.Duration(0)
	// no_of_pushes := 0
	data := types.NewGraphData(len(nodes), src)
	// fmt.Println("Init time: ", time.Since(init_time))

	// loop_time := time.Now()
	for data.PQ.Len() > 0 {
		current := heap.Pop(data.PQ).(*types.QueueItem)
		currentNode := current.Node

		// If the node is already visited, skip it
		if data.Visited[currentNode] {
			continue
		}
		data.Visited[currentNode] = true

		// If we reached the destination, exit the loop
		if currentNode == dst {
			break
		}

		// Iterate over neighbors
		loop2_time := time.Now()
		for i, neighbor := range edges[currentNode] {
			if neighbor < 0 || data.Visited[neighbor] {
				continue
			}
			newDist := data.Dist[currentNode] + edgeweights[currentNode][i]
			if newDist < data.Dist[neighbor] {
				data.Dist[neighbor] = newDist
				data.Prev[neighbor] = currentNode
				// push_time := time.Now()
				heap.Push(data.PQ, &types.QueueItem{Node: neighbor, Priority: newDist})
				// total_push_time += time.Since(push_time)
				// no_of_pushes++
			}
		}
		total_loop2_time += time.Since(loop2_time)
	}
	// fmt.Println("loop2 time: ", total_loop2_time)
	// fmt.Println("Dest dist: ", data.Dist[dst])

	// Reconstruct the path
	path := []int{}
	if dst != -1 {
		if data.Prev[dst] != -1 || src == dst {
			for at := dst; at != -1; at = data.Prev[at] {
				path = append([]int{at}, path...)
			}
		}
	}
	return path, data.Dist
}

func AlgoDijkstra(Start types.Point, End types.Point, graphNodes [][2]float64, graphEdges [][4]int, distancesEdges [][4]int, grid [][][]int) ([]types.Point, int) {
	nearestnodeStart := [2]float64{Start.Lat, Start.Lng}
	nearestnodeEnd := [2]float64{End.Lat, End.Lng}
	nearestpointStartIndex := -1
	nearpointEndIndex := -1
	distpointStart := math.MaxFloat64
	distpointEnd := math.MaxFloat64

	// Find the nearest start and end nodes
	for k, node := range graphNodes {
		distStart := generator.Haversine(node[0], node[1], nearestnodeStart[0], nearestnodeStart[1])
		distEnd := generator.Haversine(node[0], node[1], nearestnodeEnd[0], nearestnodeEnd[1])

		if distStart < distpointStart {
			nearestpointStartIndex = k
			distpointStart = distStart
		}
		if distEnd < distpointEnd {
			nearpointEndIndex = k
			distpointEnd = distEnd
		}
	}

	path, dist := Djikstra(graphNodes, graphEdges, distancesEdges, nearestpointStartIndex, nearpointEndIndex)

	// Convert the path to the required format
	shortestPath := make([]types.Point, len(path))
	for i, nodeIndex := range path {
		shortestPath[i] = types.Point{Lat: graphNodes[nodeIndex][0], Lng: graphNodes[nodeIndex][1]}
	}
	return shortestPath, dist[nearpointEndIndex]
}
