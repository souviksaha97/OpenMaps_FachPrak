package router

import (
	"container/heap"
	"final/generator"
	"final/types"
	"math"
)

const k = 0.001

func AStar(nodes [][2]float64, edges [][2]int, edgeweights []int, startindicesmap []int, src int, dst int) ([]int, int) {
	data := types.NewGraphData(len(nodes), src)
	// heuristic := 0
	for data.PQ.Len() > 0 {
		current := heap.Pop(data.PQ).(*types.QueueItem)
		currentNode := current.Node

		if data.Visited[currentNode] {
			continue
		}
		data.Visited[currentNode] = true

		if currentNode == dst {
			// fmt.Println("Current Node: ", currentNode, "Destination: ", dst)
			// fmt.Println("Found")
			break
		}

		startindex := startindicesmap[currentNode]
		endindex := startindicesmap[currentNode+1]
		for i := startindex; i < endindex; i++ {
			neighbor := edges[i][1]
			if neighbor < 0 {
				continue
			}
			// fmt.Println("Current Node: ", currentNode, "Destination: ", dst)
			// fmt.Println("Current Node: ", nodes[currentNode], "Destination: ", nodes[dst])
			// fmt.Println("Distance Current Node: ", data.Dist[currentNode])
			heuristic := int(math.Round(k * 1000.0 * generator.Haversine(nodes[currentNode][0], nodes[currentNode][1], nodes[dst][0], nodes[dst][1])))
			newDist := data.Dist[currentNode] + edgeweights[i] + heuristic
			// fmt.Println("Haversine Distance: ", int(math.Round(1000.0*generator.Haversine(nodes[currentNode][0], nodes[currentNode][1], nodes[dst][0], nodes[dst][1]))))

			if newDist < data.Dist[neighbor] {
				// fmt.Println("Heuristic Distance: ", heuristic, "Current Distance: ", data.Dist[neighbor])
				data.Dist[neighbor] = newDist
				data.Prev[neighbor] = currentNode
				heap.Push(data.PQ, &types.QueueItem{Node: neighbor, Priority: newDist})
			}
		}
	}

	path := []int{}
	if dst != -1 && (data.Prev[dst] != -1 || src == dst) {
		for at := dst; at != -1; at = data.Prev[at] {
			path = append([]int{at}, path...)
		}
	}

	return path, data.Dist[dst]
}

func AlgoAStar(Start types.Point, End types.Point, graphNodes [][2]float64, graphEdges [][2]int, distancesEdges []int, startIndices []int) ([]types.Point, int) {
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

	path, dist := AStar(graphNodes, graphEdges, distancesEdges, startIndices, nearestpointStartIndex, nearpointEndIndex)

	// Convert the path to the required format
	shortestPath := make([]types.Point, len(path))
	for i, nodeIndex := range path {
		shortestPath[i] = types.Point{Lat: graphNodes[nodeIndex][0], Lng: graphNodes[nodeIndex][1]}
	}
	return shortestPath, dist
}
