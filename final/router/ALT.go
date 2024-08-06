package router

import (
	"container/heap"
	"final/generator"
	"final/types"
	"fmt"
	"math"
	"math/rand"

	"github.com/adhocore/chin"
	"github.com/gookit/slog"
)

const numLandmarksConst = 40

// Nodes, Edges, Distances, Landmarks, LandmarkDistances, src, dst
// Single landmark heuristic
func ALT(nodes [][2]float64, edges [][2]int, edgeweights []int,
	startindicesmap []int, landmarks []int,
	landmarkDistances map[int][]int, sortedLandmarks map[int][]int, src int, dst int) ([]int, int) {

	// Initialize GraphData
	data := types.NewGraphData(len(nodes), src)
	// fmt.Println("Distance array", landmarkDistances[landmarks[0]][src])
	// fmt.Println("Closest landmark:", closestLandmarkIndex)
	farthestLandmarkFromDst := sortedLandmarks[dst][1]

	for data.PQ.Len() > 0 {
		current := heap.Pop(data.PQ).(*types.QueueItem)
		currentNode := current.Node

		if data.Visited[currentNode] {
			continue
		}
		data.Visited[currentNode] = true

		if currentNode == dst {
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
			// fmt.Println("Neighbor: ", nodes[neighbor])
			farthestLandmarkFromSrc := sortedLandmarks[currentNode][1]
			heuristic1 := generator.Abs(landmarkDistances[farthestLandmarkFromDst][currentNode] - landmarkDistances[farthestLandmarkFromDst][dst])
			heuristic2 := generator.Abs(landmarkDistances[farthestLandmarkFromSrc][currentNode] - landmarkDistances[farthestLandmarkFromSrc][dst])
			newDist := data.Dist[currentNode] + edgeweights[i] + generator.Max([]int{heuristic1, heuristic2})
			if newDist < data.Dist[neighbor] {
				data.Dist[neighbor] = newDist
				data.Prev[neighbor] = currentNode
				heap.Push(data.PQ, &types.QueueItem{Node: neighbor, Priority: newDist})
			}
		}
	}

	// fmt.Println("Time taken for ALT: ", time.Since(timeStart))

	path := []int{}
	if dst != -1 && (data.Prev[dst] != -1 || src == dst) {
		for at := dst; at != -1; at = data.Prev[at] {
			path = append([]int{at}, path...)
		}
	}

	return path, data.Dist[dst]
}

// Nodes, Edges, Distances, Landmarks, LandmarkDistances, src, dst
func ALTv2(nodes [][2]float64, edges [][2]int, edgeweights []int,
	startindicesmap []int, landmarks []int,
	landmarkDistances map[int][]int, sortedLandmarks map[int][]int, src int, dst int) ([]int, int) {
	// Initialize Graph
	data := types.NewGraphData(len(nodes), src)
	// fmt.Println("Distance array", landmarkDistances[landmarks[0]][src])

	heuristic := 0
	chosenLandmarkIndex := landmarks[0]
	dstDistance := landmarkDistances[chosenLandmarkIndex][dst]
	srcDistance := landmarkDistances[chosenLandmarkIndex][src]
	for _, landmark := range landmarks {
		tempSrc := landmarkDistances[landmark][src]
		tempDst := landmarkDistances[landmark][dst]

		if tempDst > dstDistance && tempSrc > srcDistance {
			chosenLandmarkIndex = landmark
			dstDistance = tempDst
			srcDistance = tempSrc
		}
	}
	closestLandmarkArray := landmarkDistances[chosenLandmarkIndex]
	fmt.Println("Chosen landmark:", chosenLandmarkIndex)

	for data.PQ.Len() > 0 {
		current := heap.Pop(data.PQ).(*types.QueueItem)
		currentNode := current.Node

		if data.Visited[currentNode] {
			continue
		}
		data.Visited[currentNode] = true

		if currentNode == dst {
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
			// fmt.Println("Neighbor: ", nodes[neighbor])
			heuristic = generator.Abs(closestLandmarkArray[currentNode] - closestLandmarkArray[dst])
			newDist := data.Dist[currentNode] + edgeweights[i] + heuristic
			if newDist < data.Dist[neighbor] {
				data.Dist[neighbor] = newDist
				data.Prev[neighbor] = currentNode
				heap.Push(data.PQ, &types.QueueItem{Node: neighbor, Priority: newDist})
			}
		}
	}

	// fmt.Println("Time taken for ALT: ", time.Since(timeStart))

	path := []int{}
	if dst != -1 && (data.Prev[dst] != -1 || src == dst) {
		for at := dst; at != -1; at = data.Prev[at] {
			path = append([]int{at}, path...)
		}
	}

	return path, data.Dist[dst]
}

func AlgoALT(Start types.Point, End types.Point, graphNodes [][2]float64, graphEdges [][2]int, distancesEdges []int, startIndices []int,
	landmarks []int, landmarkDistances map[int][]int, sortedLandmarks map[int][]int) ([]types.Point, int) {
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

	path, dist := ALT(graphNodes, graphEdges, distancesEdges, startIndices, landmarks, landmarkDistances, sortedLandmarks, nearestpointStartIndex, nearpointEndIndex)
	// path, dist := ALTv2(graphNodes, graphEdges, distancesEdges, landmarks, landmarkDistances, nearestpointStartIndex, nearpointEndIndex)
	// Convert the path to the required format
	shortestPath := make([]types.Point, len(path))
	for i, nodeIndex := range path {
		shortestPath[i] = types.Point{Lat: graphNodes[nodeIndex][0], Lng: graphNodes[nodeIndex][1]}
	}

	return shortestPath, dist
}

func LandmarksDistanceMaximiser(numLandmarks int) {
	// nodes, edges, distances, _, _, _, _ := FileReader()
	nodes, _, _, _, _, _, _, _, _, _, _ := FileReader()
	longSearch := false

	maxDistance := 0.0
	if longSearch {
		for i := 0; i < len(nodes); i++ {
			for j := i + 1; j < len(nodes); j++ {
				distance := generator.Haversine(nodes[i][0], nodes[i][1], nodes[j][0], nodes[j][1])
				if distance > maxDistance {
					maxDistance = distance
				}
			}
		}
	} else {
		// circumference of earth in km
		maxDistance = 6350.0
	}

	slog.Debug("Max distance: ", maxDistance)
	if numLandmarks == 0 {
		numLandmarks = numLandmarksConst
	}

	landmarks := make([]int, numLandmarks)
	for {
		res := chooseLandmarks(nodes, numLandmarks, int(maxDistance))
		// res := chooseLandmarksV2(nodes, edges, distances, numLandmarks, int(maxDistance))
		if res == nil {
			maxDistance = maxDistance * 0.9
			slog.Debug("Trying with smaller distance: ", maxDistance)
		} else {
			landmarks = res
			break
		}
	}

	landmarksNodes := make([][2]float64, len(landmarks))

	generator.WriteToJSONFile("landmarkNodes.json", landmarks)

	for i, landmark := range landmarks {
		landmarksNodes[i][0] = nodes[landmark][0]
		landmarksNodes[i][1] = nodes[landmark][1]
	}
	generator.WriteToJSONFile("landmarks.json", landmarksNodes)

	fidgetor := chin.New()
	go fidgetor.Start()
	landMarksDistanceFinder()
	fidgetor.Stop()
}

func chooseLandmarks(nodes [][2]float64, numLandmarks int, minDistance int) []int {
	landmarks := make([]int, numLandmarks)
	landmarkCounter := 0
	validityCounter := 0
	for landmarkCounter < numLandmarks && validityCounter < 1000 {
		randomPoint := rand.Intn(len(nodes))
		suitablePoint := true
		if !generator.Contains(landmarks, randomPoint) {
			for _, landmark := range landmarks {
				if generator.Haversine(nodes[randomPoint][0], nodes[randomPoint][1], nodes[landmark][0], nodes[landmark][1]) < float64(minDistance) {
					suitablePoint = false
					validityCounter++
					break
				}
			}
			if suitablePoint {
				landmarks[landmarkCounter] = randomPoint
				landmarkCounter++
				validityCounter = 0
				slog.Debug("Landmark", landmarkCounter, ":", randomPoint)
			}

		}
	}

	if validityCounter >= 1000 {
		slog.Debug("Could not find suitable landmarks")
		return nil
	}
	return landmarks
}

// func chooseLandmarksV2(nodes [][2]float64, edges [][4]int, distances [][4]int, numLandmarks int, minDistance int) []int {
// 	landmarks := make([]int, numLandmarks)
// 	landmarkCounter := 0
// 	validityCounter := 0
// 	for landmarkCounter < numLandmarks && validityCounter < 1000 {
// 		randomPoint := rand.Intn(len(nodes))
// 		suitablePoint := true
// 		if !generator.Contains(landmarks, randomPoint) {
// 			for _, landmark := range landmarks {
// 				// if generator.Haversine(nodes[randomPoint][0], nodes[randomPoint][1], nodes[landmark][0], nodes[landmark][1]) < float64(minDistance) {
// 				_, dist := Djikstra(nodes, edges, distances, randomPoint, landmark)
// 				distLandmark := dist[landmark]
// 				if distLandmark < minDistance {
// 					suitablePoint = false
// 					validityCounter++
// 					break
// 				}
// 			}
// 			if suitablePoint {
// 				landmarks[landmarkCounter] = randomPoint
// 				landmarkCounter++
// 				validityCounter = 0
// 				slog.Debug("Landmark", landmarkCounter, ":", randomPoint)
// 			}

// 		}
// 	}

// 	if validityCounter >= 1000 {
// 		slog.Debug("Could not find suitable landmarks")
// 		return nil
// 	}
// 	return landmarks
// }

func landMarksDistanceFinder() {
	// graphNodes, graphEdges, distancesEdges, _, _, landmarkNodes, _ := FileReader()
	graphNodes, _, _, _, sorted_edges, sorted_distances, start_indices, _, landmarkNodes, _, _ := FileReader()
	completeLandmarksMap := make(map[int][]int)
	maxMinLandmarksDistanceMap := make(map[int][]int, len(graphNodes))
	for i, landmark := range landmarkNodes {

		slog.Info("Landmark:", i, landmark)
		// _, dist := Djikstra(graphNodes, graphEdges, distancesEdges, landmark, -1)
		_, dist := Djikstra(graphNodes, sorted_edges, sorted_distances, start_indices, landmark, -1)
		completeLandmarksMap[landmark] = dist
		// fmt.Println("Distance from landmark", landmark, ":", dist)
	}

	for i := 0; i < len(graphNodes); i++ {
		minDistance := math.MaxInt64
		maxDistance := 0
		minIndex := 0
		maxIndex := 0
		for landmark, dist := range completeLandmarksMap {
			if dist[i] < minDistance {
				minDistance = dist[i]
				minIndex = landmark
			}
			if dist[i] > maxDistance {
				maxDistance = dist[i]
				maxIndex = landmark
			}
		}
		maxMinLandmarksDistanceMap[i] = []int{minIndex, maxIndex}
	}

	generator.WriteToJSONFile("landmarkDistances.json", completeLandmarksMap)
	generator.WriteToJSONFile("sortedLandmarks.json", maxMinLandmarksDistanceMap)
}
