package router

import (
	"container/heap"
	"final/generator"
	"final/types"
	"math"

	"math/rand"
	// "time"
	"github.com/adhocore/chin"
	"github.com/gookit/slog"
	// "runtime"
)

// Nodes, Edges, Distances, Landmarks, LandmarkDistances, src, dst
// Single landmark heuristic
func ALT(nodes [][2]float64, edges [][2]int, edgeweights []int,
	startindicesmap []int, landmarks []int,
	landmarkDistances map[int][]int, sortedLandmarks map[int][]int, landmarkPairDistances [][]int, src int, dst int) ([]int, int) {

	// Initialize GraphData
	data := types.NewGraphData(len(nodes), src)

	// Calculate farthest landmark
	farthestLandmark := 0
	maxDistance := 0
	closestLandmark := 0
	minDistance := math.MaxInt64
	for _, landmark := range landmarks {
		distance := landmarkDistances[landmark][dst]
		if distance > maxDistance {
			maxDistance = distance
			farthestLandmark = landmark
		}
		if distance < minDistance {
			minDistance = distance
			closestLandmark = landmark
		}
	}

	// fmt.Println("Distance array", landmarkDistances[landmarks[0]][src])

	for data.PQ.Len() > 0 {
		current := heap.Pop(data.PQ).(*types.QueueItem)
		currentNode := current.Node

		if data.Visited[currentNode] {
			continue
		}
		data.Visited[currentNode] = true

		if currentNode == dst {
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
				closestHeuristic := generator.Abs(landmarkDistances[closestLandmark][neighbor] - landmarkDistances[closestLandmark][dst])
				farthestHeuristic := generator.Abs(landmarkDistances[farthestLandmark][neighbor] - landmarkDistances[farthestLandmark][dst])
				heuristic := generator.Max([]int{closestHeuristic, farthestHeuristic})

				newPriority := newDist + heuristic
				data.Dist[neighbor] = newDist
				data.Prev[neighbor] = currentNode
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

	return path, data.Dist[dst]
}

func AlgoALT(Start types.Point, End types.Point, graphNodes [][2]float64, graphEdges [][2]int, distancesEdges []int, startIndices []int,
	landmarks []int, landmarkDistances map[int][]int, sortedLandmarks map[int][]int, landmarkPairDistances [][]int) ([]types.Point, int) {
	nearestnodeStart := [2]float64{Start.Lat, Start.Lng}
	nearestnodeEnd := [2]float64{End.Lat, End.Lng}
	nearestpointStartIndex := -1
	nearpointEndIndex := -1
	distpointStart := math.MaxInt64
	distpointEnd := math.MaxInt64

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

	path, dist := ALT(graphNodes, graphEdges, distancesEdges, startIndices, landmarks, landmarkDistances, sortedLandmarks, landmarkPairDistances, nearestpointStartIndex, nearpointEndIndex)
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
	nodes, _, _, _, _, _, _, _, _, _, _, _ := FileReader()
	longSearch := false

	maxDistance := 0
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
		maxDistance = 6371000.0
	}

	slog.Debug("Max distance: ", maxDistance)
	if numLandmarks == 0 {
		numLandmarks = 10
	}

	landmarks := make([]int, numLandmarks)
	for {
		res := chooseLandmarks(nodes, numLandmarks, int(maxDistance))
		// res := chooseLandmarksV2(nodes, edges, distances, numLandmarks, int(maxDistance))
		if res == nil {
			maxDistance = int(float64(maxDistance) * 0.90)
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
	landmarkPairDistances := make([][]int, len(landmarks))
	for i := range landmarkPairDistances {
		landmarkPairDistances[i] = make([]int, len(landmarks))
	}

	for i, landmark := range landmarks {
		landmarksNodes[i][0] = nodes[landmark][0]
		landmarksNodes[i][1] = nodes[landmark][1]
		// slog.Info("Landmark", i, ":", landmarksNodes[i])
	}
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
				if generator.Haversine(nodes[randomPoint][0], nodes[randomPoint][1], nodes[landmark][0], nodes[landmark][1]) < minDistance {
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

func landMarksDistanceFinder() {
	// graphNodes, graphEdges, distancesEdges, _, _, landmarkNodes, _ := FileReader()
	graphNodes, _, _, _, sorted_edges, sorted_distances, start_indices, _, landmarkNodes, _ , _, _:= FileReader()
	completeLandmarksMap := make(map[int][]int)
	for i, landmark := range landmarkNodes {
		slog.Info("Landmark:", i, landmark)
		// _, dist := Djikstra(graphNodes, graphEdges, distancesEdges, landmark, -1)
		_, dist := Djikstra(graphNodes, sorted_edges, sorted_distances, start_indices, landmark, -1)
		completeLandmarksMap[landmark] = dist
		// fmt.Println("Distance from landmark", landmark, ":", dist)
	}
	generator.WriteToJSONFile("landmarkDistances.json", completeLandmarksMap)
}

