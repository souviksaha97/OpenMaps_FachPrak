// Implements the A* algorithm for finding the shortest path between two points

package router

import (
	"container/heap"
	"final/generator"
	"final/types"
	"math"
)

// AStar is the A* algorithm for finding the shortest path between two points
func AStar(nodes [][2]float64, edges [][2]int, edgeweights []int, startindicesmap []int, src int, dst int) ([]int, []int, int) {
	data := types.NewGraphData(len(nodes), src)
	popCounter := 0
	for data.PQ.Len() > 0 {
		current := heap.Pop(data.PQ).(*types.QueueItem)
		popCounter += 1
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

			newDist := data.Dist[currentNode] + edgeweights[i]

			if newDist < data.Dist[neighbor] {
				data.Dist[neighbor] = newDist
				data.Prev[neighbor] = currentNode
				heuristic := generator.Haversine(nodes[neighbor][0], nodes[neighbor][1], nodes[dst][0], nodes[dst][1])
				newPriority := newDist + heuristic
				heap.Push(data.PQ, &types.QueueItem{Node: neighbor, Priority: newPriority})
			}
		}
	}

	path := []int{}
	if dst != -1 && (data.Prev[dst] != -1 || src == dst) {
		// Build the path in reverse order
		for at := dst; at != -1; at = data.Prev[at] {
			path = append(path, at)
		}
	}


	return path, data.Dist, popCounter
}

// Searches for the nearest node to the start and end points and then runs the A* algorithm
func AlgoAStar(Start types.Point, End types.Point, graphNodes [][2]float64, gridNodes [][][][3]float64, graphEdges [][2]int, distancesEdges []int, startIndices []int) ([]types.Point, int, int) {
	nearestnodeStart := [2]float64{Start.Lat, Start.Lng}
	nearestnodeEnd := [2]float64{End.Lat, End.Lng}
	nearestpointStartIndex := -1
	nearpointEndIndex := -1
	distpointStart := math.MaxInt64
	distpointEnd := math.MaxInt64

	// Find the nearest start and end nodes
	a, b := generator.FindRowAndColumnInGrid(180, 360, Start.Lat, Start.Lng)
	possiblestartandendpoints := gridNodes[a][b]
	c, d := generator.FindRowAndColumnInGrid(180, 360, End.Lat, End.Lng)
	possiblestartandendpoints = append(possiblestartandendpoints, gridNodes[c][d]...)

	for _, node := range possiblestartandendpoints {
		distStart := generator.Haversine(node[0], node[1], nearestnodeStart[0], nearestnodeStart[1])
		distEnd := generator.Haversine(node[0], node[1], nearestnodeEnd[0], nearestnodeEnd[1])

		if distStart < distpointStart {
			nearestpointStartIndex = int(node[2])
			distpointStart = distStart
		}	
		if distEnd < distpointEnd {
			nearpointEndIndex = int(node[2])
			distpointEnd = distEnd
		}
	}

	path, dist, popCounter := AStar(graphNodes, graphEdges, distancesEdges, startIndices, nearestpointStartIndex, nearpointEndIndex)

	// Convert the path to the required format
	shortestPath := make([]types.Point, len(path))
	for i, nodeIndex := range path {
		shortestPath[i] = types.Point{Lat: graphNodes[nodeIndex][0], Lng: graphNodes[nodeIndex][1]}
	}
	return shortestPath, dist[nearpointEndIndex], popCounter
}
