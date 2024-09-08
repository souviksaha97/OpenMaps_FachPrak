package router

import (
	"container/heap"
	"final/generator"
	"final/types"
	"fmt"
	"math"
	"math/rand"
	"time"

	"github.com/adhocore/chin"
	"github.com/gookit/slog"
	// "runtime"
)

// Nodes, Edges, Distances, Landmarks, LandmarkDistances, src, dst
// Single landmark heuristic
func ALT(nodes [][2]float64, edges [][2]int, edgeweights []int,
	startindicesmap []int, landmarks []int,
	landmarkDistances map[int][]int, src int, dst int) ([]int, int, int) {

	// Initialize GraphData
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
				maxHeuristic := 0
				heuristic := 0
				for _, landmark := range landmarks {
					heuristic = generator.Abs(landmarkDistances[landmark][neighbor] - landmarkDistances[landmark][dst])
					if heuristic > maxHeuristic {
						maxHeuristic = heuristic
					}
				}
				newPriority := newDist + maxHeuristic
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

	return path, data.Dist[dst], popCounter
}

func AlgoALT(Start types.Point, End types.Point, graphNodes [][2]float64, graphEdges [][2]int, distancesEdges []int, startIndices []int,
	landmarks []int, landmarkDistances map[int][]int) ([]types.Point, int, int) {
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

	path, dist, popCounter := ALT(graphNodes, graphEdges, distancesEdges, startIndices, landmarks, landmarkDistances, nearestpointStartIndex, nearpointEndIndex)
	// path, dist := ALTv2(graphNodes, graphEdges, distancesEdges, landmarks, landmarkDistances, nearestpointStartIndex, nearpointEndIndex)
	// Convert the path to the required format
	shortestPath := make([]types.Point, len(path))
	for i, nodeIndex := range path {
		shortestPath[i] = types.Point{Lat: graphNodes[nodeIndex][0], Lng: graphNodes[nodeIndex][1]}
	}

	return shortestPath, dist, popCounter
}

func LandmarksDistanceMaximiser(numLandmarks int) {
	// nodes, edges, distances, _, _, _, _ := FileReader()
	nodes, _, _, _, _, _, _, _, _, _ := FileReader()
	// longSearch := false

	maxDistance := 0
	// if longSearch {
	// 	for i := 0; i < len(nodes); i++ {
	// 		for j := i + 1; j < len(nodes); j++ {
	// 			distance := generator.Haversine(nodes[i][0], nodes[i][1], nodes[j][0], nodes[j][1])
	// 			if distance > maxDistance {
	// 				maxDistance = distance
	// 			}
	// 		}
	// 	}
	// } else {
	// 	// circumference of earth in km
	maxDistance = 6371000.0
	// }

	// slog.Debug("Max distance: ", maxDistance)
	// if numLandmarks == 0 {
	// 	numLandmarks = 10
	// }

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
	// landmarks = chooseLandmarksV2_knownPoints()
	// landmarks = chooseLandmarksV3_knownCoords(nodes)
	// landmarks = LandmarksPruning(numLandmarks, 10, 1)

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
	// landmarkPairDistances := make([][]int, len(landmarks))
	// for i := range landmarkPairDistances {
	// 	landmarkPairDistances[i] = make([]int, len(landmarks))
	// }

	// for i, landmark := range landmarks {
	// 	landmarksNodes[i][0] = nodes[landmark][0]
	// 	landmarksNodes[i][1] = nodes[landmark][1]
	// 	// slog.Info("Landmark", i, ":", landmarksNodes[i])
	// }
	fidgetor.Stop()
}

func LandmarksPruning(landmarks int, iterations int, epochs int) {
	// initLandmarks := 256
	landmarkCounter := make(map[int]int, landmarks)
	LandmarksDistanceMaximiser(landmarks)

	graphNodes, _, _, _, sortedEdges, sortedDistances, startIndices, _, landmarkNodes, landmarkDistances := FileReader()

	slog.Info("generated landmarks")
	for _, landmark := range landmarkNodes {
		landmarkCounter[landmark] = 0
	}
	for i := 0; i < epochs; i++ {
		slog.Info("============Epoch", i+1, "=================")
		srcDstPairs := [][2]int{}
		for j := 0; j < iterations; j++ {
			src := rand.Intn(len(graphNodes))
			dst := rand.Intn(len(graphNodes))
			srcDstPairs = append(srcDstPairs, [2]int{src, dst})
		}
		slog.Info("Pairs generated")
		for _, pair := range srcDstPairs {
			tempMap := PruneALT(graphNodes, sortedEdges, sortedDistances, startIndices, landmarkNodes, landmarkDistances, pair[0], pair[1])
			// fmt.Println(tempMap)
			for landmark := range landmarkCounter {
				landmarkCounter[landmark] += tempMap[landmark]
				// fmt.Println(tempMap[landmark])
			}
		}
		slog.Info("ALT completed on ", iterations, " pairs")
		slog.Info("=========================================")
	}

	pruned := generator.GetTopNKeys(landmarkCounter, 16)
	fmt.Println("Pruned :", pruned)
	generator.WriteToJSONFile("landmarkNodes.json", pruned)
	slog.Info("pruned nodes writted")
	landmarksNodes := make([][2]float64, len(pruned))
	for i, landmark := range pruned {
		landmarksNodes[i][0] = graphNodes[landmark][0]
		landmarksNodes[i][1] = graphNodes[landmark][1]
	}
	generator.WriteToJSONFile("landmarks.json", landmarksNodes)

	landMarksDistanceFinder()
}

func chooseLandmarks(nodes [][2]float64, numLandmarks int, minDistance int) []int {
	landmarks := make([]int, numLandmarks)
	landmarkCounter := 0
	validityCounter := 0
	rand.Seed(time.Now().UnixNano())
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

func chooseLandmarksV2_knownPoints() []int {
	return []int{
		3804942,
		2808130,
		1055298,
		2947496,
		1326119,
		450081,
		1605771,
		968020,
		1845136,
		3727989,
		3234498,
		2602491,
		584979,
		2093229,
		1870122,
		3677333,
	}
}

func chooseLandmarksV3_knownCoords(nodes [][2]float64) []int {
	coords := [][2]float64{
		{-37.10, 23.04},    //Africa tip
		{-53.26, -70.67},   //South America tip
		{47.58, -31.71},    //North Atlantic
		{-19.62, -20.08},   //South Atlantic
		{7.411, 77.69},     //India
		{-36.04, 116.87},   //Australia
		{13.659, 114.43},   //South China Sea
		{65.1099, -168.89}, //Bering Sea
		{59.11, -45.12},    //Greenland
		{74.55, -104.19},   //NW Passage
		{77.27, 68.5},      //Arctic
		{-68.94, 161.26},   //Antarctica
		{-73.72, -150.13},
		{26.739, -135.97}, //North Pacific
		{-39.087, -104.367},
		{76.65, 140.02},
	}

	landmarks := make([]int, len(coords))

	for i, coord := range coords {
		distMax := math.MaxInt64
		idx := 0
		for k, node := range nodes {
			distTemp := generator.Haversine(node[0], node[1], coord[0], coord[1])

			if distTemp < distMax {
				distMax = distTemp
				idx = k
			}
		}
		landmarks[i] = idx
	}
	return landmarks
}

func landMarksDistanceFinder() {
	// graphNodes, graphEdges, distancesEdges, _, _, landmarkNodes, _ := FileReader()
	graphNodes, _, _, _, sorted_edges, sorted_distances, start_indices, _, landmarkNodes, _ := FileReader()
	completeLandmarksMap := make(map[int][]int)
	for i, landmark := range landmarkNodes {
		slog.Info("Landmark:", i, landmark)
		// _, dist := Djikstra(graphNodes, graphEdges, distancesEdges, landmark, -1)
		_, dist, _ := Djikstra(graphNodes, sorted_edges, sorted_distances, start_indices, landmark, -1)
		completeLandmarksMap[landmark] = dist
		// fmt.Println("Distance from landmark", landmark, ":", dist)
	}
	generator.WriteToJSONFile("landmarkDistances.json", completeLandmarksMap)
}

func PruneALT(nodes [][2]float64, edges [][2]int, edgeweights []int,
	startindicesmap []int, landmarks []int,
	landmarkDistances map[int][]int, src int, dst int) map[int]int {

	// Initialize GraphData
	data := types.NewGraphData(len(nodes), src)
	landmarkCounter := make(map[int]int, len(landmarks))

	// // Calculate farthest landmark
	// farthestLandmarkDst := 0
	// maxDistance := 0
	// closestLandmarkDst := 0
	// minDistance := math.MaxInt64
	// for _, landmark := range landmarks {
	// 	distance := landmarkDistances[landmark][dst]
	// 	if distance > maxDistance {
	// 		maxDistance = distance
	// 		farthestLandmarkDst = landmark
	// 	}
	// 	if distance < minDistance {
	// 		minDistance = distance
	// 		closestLandmarkDst = landmark
	// 	}
	// }

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
				maxHeuristic := 0
				heuristic := 0
				chosenLandmark := 0
				for _, landmark := range landmarks {
					heuristic = generator.Abs(landmarkDistances[landmark][neighbor] - landmarkDistances[landmark][dst])
					if heuristic > maxHeuristic {
						maxHeuristic = heuristic
						chosenLandmark = landmark
					}
				}
				landmarkCounter[chosenLandmark] += 1
				newPriority := newDist + maxHeuristic
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

	return landmarkCounter
}
