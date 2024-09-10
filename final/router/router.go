package router

import (
	"encoding/json"
	"final/generator"
	"fmt"
	"math/rand"
	"os"
	"runtime"
	"time"

	"github.com/adhocore/chin"
	"github.com/gookit/slog"
)

// Nodes, Edges, Distances, Grid, Landmarks, LandmarkNodes, LandmarkDistances
func FileReader() (nodes [][2]float64, edges [][4]int, distances [][4]int, grid [][][][3]float64,
	sorted_edges [][2]int, sorted_distances []int, start_indices []int,
	landmarkCoords [][2]float64, landmarks []int, landmarkMap map[int][]int) {

	slog.Info("Reading the files")
	graphNodesJSON, err := os.ReadFile("objects/graphNodes.json")
	if err != nil {
		slog.Info("Error reading graphNodes from file:", err)
	}
	var graphNodes [][2]float64
	err = json.Unmarshal(graphNodesJSON, &graphNodes)
	if err != nil {
		slog.Info("Error unmarshalling graphNodes:", err)
	}

	slog.Debug("Graph Nodes:", len(graphNodes))

	graphEdgesJSON, err := os.ReadFile("objects/graphEdges.json")
	if err != nil {
		slog.Info("Error reading graphEdges from file:", err)
	}
	var graphEdges [][4]int
	err = json.Unmarshal(graphEdgesJSON, &graphEdges)
	if err != nil {
		slog.Info("Error unmarshalling graphEdges:", err)
	}

	slog.Debug("Graph Edges:", len(graphEdges))

	distancesEdgesJSON, err := os.ReadFile("objects/distancesEdges.json")
	if err != nil {
		slog.Info("Error reading distancesEdges from file:", err)
	}
	var distancesEdges [][4]int
	err = json.Unmarshal(distancesEdgesJSON, &distancesEdges)
	if err != nil {
		slog.Info("Error unmarshalling distancesEdges:", err)
	}

	slog.Debug("Distances Edges:", len(distancesEdges))

	gridJSON, err := os.ReadFile("objects/grid.json")
	if err != nil {
		slog.Info("Error reading grid from file:", err)
	}
	var gridNodes [][][][3]float64
	err = json.Unmarshal(gridJSON, &gridNodes)
	if err != nil {
		slog.Info("Error unmarshalling grid:", err)
	}

	slog.Debug("Grid:", len(gridNodes))

	var landmarksCoords [][2]float64
	landmarksJSON, err := os.ReadFile("objects/landmarks.json")
	if err != nil {
		slog.Info("Error reading landmarks from file:", err)
	}
	err = json.Unmarshal(landmarksJSON, &landmarksCoords)
	if err != nil {
		slog.Info("Error unmarshalling landmarks:", err)
	}

	slog.Debug("Landmarks:", len(landmarksCoords))

	var landmarkNodes []int
	landmarkNodesJSON, err := os.ReadFile("objects/landmarkNodes.json")
	if err != nil {
		slog.Info("Error reading landmarkNodes from file:", err)
	}
	err = json.Unmarshal(landmarkNodesJSON, &landmarkNodes)
	if err != nil {
		slog.Info("Error unmarshalling landmarkNodes:", err)
	}

	landmarkDistances := make(map[int][]int)
	landmarkDistancesJSON, err := os.ReadFile("objects/landmarkDistances.json")
	if err != nil {
		slog.Info("Error reading landmarkDistances from file:", err)
	}
	err = json.Unmarshal(landmarkDistancesJSON, &landmarkDistances)
	if err != nil {
		slog.Info("Error unmarshalling landmarkDistances:", err)
	}

	slog.Debug("Landmark Nodes:", len(landmarkNodes))
	slog.Debug("Landmark Distances:", len(landmarkDistances))

	var sEdges [][2]int
	sortedEdgesJSON, err := os.ReadFile("objects/sortedEdges.json")
	if err != nil {
		slog.Info("Error reading sortedEdges from file:", err)
	}
	err = json.Unmarshal(sortedEdgesJSON, &sEdges)
	if err != nil {
		slog.Info("Error unmarshalling sortedEdges:", err)
	}

	slog.Debug("Sorted Edges:", len(sEdges))

	var sDistances []int
	sortedDistancesJSON, err := os.ReadFile("objects/sortedDistances.json")
	if err != nil {
		slog.Info("Error reading sortedDistances from file:", err)
	}
	err = json.Unmarshal(sortedDistancesJSON, &sDistances)
	if err != nil {
		slog.Info("Error unmarshalling sortedDistances:", err)
	}

	slog.Debug("Sorted Distances:", len(sDistances))

	var startIndices []int
	startIndicesJSON, err := os.ReadFile("objects/startIndices.json")
	if err != nil {
		slog.Info("Error reading startIndices from file:", err)
	}
	err = json.Unmarshal(startIndicesJSON, &startIndices)
	if err != nil {
		slog.Info("Error unmarshalling startIndices:", err)
	}

	slog.Debug("Start Indices:", len(startIndices))

	return graphNodes, graphEdges, distancesEdges, gridNodes, sEdges, sDistances,
		startIndices, landmarksCoords, landmarkNodes, landmarkDistances
}

func Debugging() {
	graphNodes, _, _, _, sortedEdges, sortedDistances, startIndices, _, _, _ := FileReader()

	start := 763767
	end := 543231
	pathD, distD, _ := Djikstra(graphNodes, sortedEdges, sortedDistances, startIndices, start, end)
	pathA, distA, _ := AStar(graphNodes, sortedEdges, sortedDistances, startIndices, start, end)

	slog.Info("Coordinates: ", graphNodes[start], graphNodes[end])

	fmt.Println("Djikstra Distance: ", distD[end])
	fmt.Println("A* Distance: ", distA[end])
	fmt.Println()
	for i := 0; i < len(pathD); i++ {
		slog.Info("pathD: ", pathD[i])
	}

	for i := 0; i < len(pathA); i++ {
		slog.Info("pathA: ", pathA[i])
	}

	for i := 0; i < len(pathA)-1; i++ {
		slog.Info("------------------------------Point ", i, "------------------------------")
		slog.Info("Point ", i, ": ", pathA[i])
		for j := startIndices[pathA[i]]; j < startIndices[pathA[i]+1]; j++ {
			slog.Info("Neighbours of ", pathA[i], ": ", sortedEdges[j][1], " with distance ", sortedDistances[j], " and heuristic ", generator.Haversine(graphNodes[sortedEdges[j][1]][0], graphNodes[sortedEdges[j][1]][1], graphNodes[end][0], graphNodes[end][1]))
			slog.Info("                                                       Priority: ", sortedDistances[j]+generator.Haversine(graphNodes[sortedEdges[j][1]][0], graphNodes[sortedEdges[j][1]][1], graphNodes[end][0], graphNodes[end][1]))
		}
		slog.Info("------------Chosen Node: ", pathA[i+1], " with distance ", distA[pathA[i+1]], "------------")
		slog.Info()
	}
}

func MultiRouter(iterations int) {

	fidgeter := chin.New()
	go fidgeter.Start()
	runtime.GOMAXPROCS(runtime.NumCPU())

	// graphNodes, _, _, _, sortedEdges, sortedDistances, startIndices, _, landmarkNodes, landmarkDistances := FileReader()
	graphNodes, _, _, _, sortedEdges, sortedDistances, startIndices, _, landmarkNodes, landmarkDistances := FileReader()
	// graphNodes, _, _, _, sortedEdges, sortedDistances, startIndices, _, _, _, _, _ := FileReader()
	slog.Info("Multi Router started")
	rand.Seed(time.Now().UnixNano())
	var randomIndices = make([][2]int, iterations)
	for i := 0; i < iterations; i++ {
		randomIndices[i][0] = rand.Intn(len(graphNodes))
		randomIndices[i][1] = rand.Intn(len(graphNodes))
	}

	runtime.GC()
	var startDijkstra = time.Now()
	avgDijkPops := 0.0
	for i := 0; i < iterations; i++ {
		path, dist, dijkstra_pops := Djikstra(graphNodes, sortedEdges, sortedDistances, startIndices, randomIndices[i][0], randomIndices[i][1])
		if (dist[randomIndices[i][1]] <= 0 || len(path) == 0) && (randomIndices[i][0] != randomIndices[i][1]) {
			// panic("Djikstra failed")
			slog.Info("Dijkstra No route found", randomIndices[i][0], randomIndices[i][1])
			slog.Info("Coordinates: ", graphNodes[randomIndices[i][0]], graphNodes[randomIndices[i][1]])
			continue
		}
		avgDijkPops += float64(dijkstra_pops)
	}

	avgDijkPops /= float64(iterations)
	avgDijkstra := time.Since(startDijkstra) / time.Duration(iterations)

	runtime.GC()
	var startAStar = time.Now()
	avgAstarPops := 0.0
	for i := 0; i < iterations; i++ {
		path, dist, astar_pops := AStar(graphNodes, sortedEdges, sortedDistances, startIndices, randomIndices[i][0], randomIndices[i][1])
		if dist[randomIndices[i][1]] <= 0 || len(path) == 0 {
			// panic("A* failed")
			slog.Info("A* No route found", randomIndices[i][0], randomIndices[i][1])
			slog.Info("Coordinates: ", graphNodes[randomIndices[i][0]], graphNodes[randomIndices[i][1]])
			continue
		}
		avgAstarPops += float64(astar_pops)
	}

	avgAstarPops /= float64(iterations)
	avgAstar := time.Since(startAStar) / time.Duration(iterations)

	runtime.GC()
	var startALTv1 = time.Now()
	avgALTPops := 0.0
	for i := 0; i < iterations; i++ {
		path, dist, alt_pops, _ := ALT(graphNodes, sortedEdges, sortedDistances, startIndices, landmarkNodes, landmarkDistances,
			randomIndices[i][0], randomIndices[i][1])
		if dist <= 0 || len(path) == 0 {
			slog.Info("ALT No route found", randomIndices[i][0], randomIndices[i][1])
			slog.Info("Coordinates: ", graphNodes[randomIndices[i][0]], graphNodes[randomIndices[i][1]])
		}
		avgALTPops += float64(alt_pops)
	}
	avgALTPops /= float64(iterations)
	avgALTv1 := time.Since(startALTv1) / time.Duration(iterations)

	slog.Info("==================================== Results ====================================")
	slog.Info(fmt.Sprintf("Average Dijkstra time:       %s", avgDijkstra.String()))
	slog.Info(fmt.Sprintf("Average Dijkstra Pops:       %d", int(avgDijkPops)))
	slog.Info(fmt.Sprintf("Average AStar time:          %s", avgAstar.String()))
	slog.Info(fmt.Sprintf("Average AStar Pops:          %d", int(avgAstarPops)))
	slog.Info(fmt.Sprintf("Average ALT time:            %s", avgALTv1.String()))
	slog.Info(fmt.Sprintf("Average ALT Pops:            %d", int(avgALTPops)))
	slog.Info("------------------------------------------------------------------------------")
	slog.Info(fmt.Sprintf("A* speedup vs Dijkstra:      %.2f%%", (float64(avgDijkstra-avgAstar)/float64(avgDijkstra))*100))
	slog.Info(fmt.Sprintf("ALT speedup vs Dijkstra:     %.2f%%", (float64(avgDijkstra-avgALTv1)/float64(avgDijkstra))*100))
	slog.Info(fmt.Sprintf("ALT speedup vs A*:           %.2f%%", (float64(avgAstar-avgALTv1)/float64(avgAstar))*100))
	slog.Info("------------------------------------------------------------------------------")
	slog.Info(fmt.Sprintf("A* Pops vs Dijkstra:         %.2f%%", (float64(avgDijkPops-avgAstarPops)/float64(avgDijkPops))*100))
	slog.Info(fmt.Sprintf("ALT Pops vs Dijkstra:        %.2f%%", (float64(avgDijkPops-avgALTPops)/float64(avgDijkPops))*100))
	slog.Info(fmt.Sprintf("ALT Pops vs A*:              %.2f%%", (float64(avgAstarPops-avgALTPops)/float64(avgAstarPops))*100))
	slog.Info("==================================== End ======================================")


	fidgeter.Stop()

}

func SingleRouter(router string, iterations int) {
	slog.Info("Single Router started")
	runtime.GOMAXPROCS(runtime.NumCPU())
	fidgeter := chin.New()
	go fidgeter.Start()

	graphNodes, _, _, _, sortedEdges, sortedDistances, startIndices, _, landmarkNodes, landmarkDistances := FileReader()
	rand.Seed(time.Now().UnixNano())
	var randomIndices = make([][2]int, iterations)
	for i := 0; i < iterations; i++ {
		randomIndices[i][0] = rand.Intn(len(graphNodes))
		randomIndices[i][1] = rand.Intn(len(graphNodes))
	}

	switch router {
	case "djikstra":
		slog.Info("Running Djikstra")
		var startDijkstra = time.Now()
		for i := 0; i < iterations; i++ {
			path, dist, _ := Djikstra(graphNodes, sortedEdges, sortedDistances, startIndices, randomIndices[i][0], randomIndices[i][1])
			if dist[randomIndices[i][1]] <= 0 || len(path) == 0 {
				slog.Info("Djikstra failed")
			}
		}

		fmt.Println("Average Dijsktra time: ", time.Since(startDijkstra)/time.Duration(iterations))

	case "astar":
		var startAStar = time.Now()
		for i := 0; i < iterations; i++ {
			path, dist, _ := AStar(graphNodes, sortedEdges, sortedDistances, startIndices, randomIndices[i][0], randomIndices[i][1])
			if dist[randomIndices[i][1]] <= 0 || len(path) == 0 {
				slog.Info("A* failed")
			}
		}

		fmt.Println("Average AStar time: ", time.Since(startAStar)/time.Duration(iterations))

	case "alt":
		var startALT = time.Now()
		for i := 0; i < iterations; i++ {
			path, dist, _, _ := ALT(graphNodes, sortedEdges, sortedDistances, startIndices, landmarkNodes, landmarkDistances, randomIndices[i][0], randomIndices[i][1])
			if dist <= 0 || len(path) == 0 {
				slog.Info("ALT failed")
			}
		}

		fmt.Println("Average ALT time: ", time.Since(startALT)/time.Duration(iterations))
	}

	fidgeter.Stop()
}
