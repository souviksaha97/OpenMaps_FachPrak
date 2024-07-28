package router

import (
	"container/heap"
	"final/generator"
	"final/types"
	"fmt"
	"math"
)

func Djikstra(nodes [][2]float64, edges [][4]int, edgeweights [][4]int, src int, dst int) (int, []int) {
	dist := make(map[int]int)
	prev := make(map[int]int)
	found := false
	for node := range nodes {
		dist[node] = math.MaxInt32
	}
	dist[src] = 0
	//fmt.Println("dist", dist)
	pq := &types.PriorityQueue{}
	heap.Init(pq)
	heap.Push(pq, &types.QueueItem{Node: src, Priority: 0})

	for pq.Len() > 0 {
		current := heap.Pop(pq).(*types.QueueItem)
		currentNode := current.Node

		if currentNode == dst {
			// fmt.Println("reached from", currentNode)
			found = true
			break
		}
		//fmt.Println("cuurentnode", currentNode)

		for k := range edges[currentNode] {
			neighbor := edges[currentNode][k]
			if neighbor < 0 {
				break
			}
			newDist := dist[currentNode] + edgeweights[currentNode][k]
			if edgeweights[currentNode][k] > 10000 {
				fmt.Println("not allowed")
				break
			}
			//fmt.Println(neighbor)
			//fmt.Println("dist", newDist, dist[neighbor])
			if newDist < 0 {
				fmt.Println("neine")
			}
			if newDist < dist[neighbor] {
				dist[neighbor] = newDist
				prev[neighbor] = currentNode
				heap.Push(pq, &types.QueueItem{Node: neighbor, Priority: newDist})
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

func AlgoDijkstra(Start types.Point, End types.Point, graphNodes [][2]float64, graphEdges [][4]int, distancesEdges [][4]int, grid [][][]int) []types.Point {
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

	_, path := Djikstra(graphNodes[:], graphEdges[:], distancesEdges[:], nearestpointStartIndex, nearpointEndIndex)

	returndykstrapath := [][2]float64{}
	for k := range path {
		returndykstrapath = append(returndykstrapath, graphNodes[path[k]])
	}

	shortestPath := make([]types.Point, len(returndykstrapath))
	for i, point := range returndykstrapath {
		shortestPath[i] = types.Point{Lat: point[0], Lng: point[1]}
	}

	return shortestPath
}
