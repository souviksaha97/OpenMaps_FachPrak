package router

import (
	"container/heap"
	"final/generator"
	"final/types"
	"math"
)

func AStar(nodes [][2]float64, edges [][4]int, edgeweights [][4]int, src int, dst int) (int, []int) {
	dist := make(map[int]int)
	prev := make(map[int]int)
	found := false
	for node := range nodes {
		dist[node] = math.MaxInt32
	}
	dist[src] = 0

	pq := &types.PriorityQueue{}
	heap.Init(pq)
	heap.Push(pq, &types.QueueItem{Node: src, Priority: 0})

	for pq.Len() > 0 {
		current := heap.Pop(pq).(*types.QueueItem)
		currentNode := current.Node

		if currentNode == dst {
			found = true
			break
		}

		for i, neighbor := range edges[currentNode] {
			if neighbor < 0 {
				break
			}
			newDist := dist[currentNode] + edgeweights[currentNode][i] + int(generator.Haversine(nodes[edges[currentNode][i]][0], nodes[edges[currentNode][i]][1], nodes[dst][0], nodes[dst][1]))
			if newDist < dist[neighbor] {
				dist[neighbor] = newDist
				prev[neighbor] = currentNode
				priority := newDist
				heap.Push(pq, &types.QueueItem{Node: neighbor, Priority: priority})
			}

		}

	}

	path := []int{}
	if found {
		for at := dst; at != src; at = prev[at] {
			path = append([]int{at}, path...)
		}
		path = append([]int{src}, path...)
	} else {
		path = append([]int{dst}, path...)
		path = append([]int{src}, path...)
		return -1, path
	}
	return dist[dst], path
}

func AlgoAStar(Start types.Point, End types.Point, graphNodes [][2]float64, graphEdges [][4]int, distancesEdges [][4]int, grid [][][]int) []types.Point {
	//read graphNodes graphEdges distancesEdges grid from json files
	// var start = time.Now()

	// fmt.Println(distancesEdges)

	nearestnodeStart := [2]float64{Start.Lat, Start.Lng}
	distpointStart := 100000000.0
	distpointEnd := 100000000.0
	nearestnodeEnd := [2]float64{End.Lat, End.Lng}
	nearestpointStartIndex := -1
	nearpointEndIndex := -1
	for k := range graphNodes {
		distpointStartNew := generator.Haversine(graphNodes[k][0], graphNodes[k][1], nearestnodeStart[0], nearestnodeStart[1])
		distpointENdNew := generator.Haversine(graphNodes[k][0], graphNodes[k][1], nearestnodeEnd[0], nearestnodeEnd[1])

		if distpointStartNew < float64(distpointStart) {
			nearestpointStartIndex = k
			distpointStart = distpointStartNew
		}
		if distpointENdNew < distpointEnd {
			nearpointEndIndex = k
			distpointEnd = distpointENdNew
		}
	}

	_, path := AStar(graphNodes[:], graphEdges[:], distancesEdges[:], nearestpointStartIndex, nearpointEndIndex)

	returndykstrapath := [][2]float64{}
	for k := range path {
		returndykstrapath = append(returndykstrapath, graphNodes[path[k]])
	}

	// Convert returndykstrapath to []types.Point
	shortestPath := make([]types.Point, len(returndykstrapath))
	for i, point := range returndykstrapath {
		shortestPath[i] = types.Point{Lat: point[0], Lng: point[1]}
	}

	return shortestPath
}
