// 21398

package main

import (
	"container/heap"
	"context"
	"encoding/json"
	"fmt"
	"io/ioutil"
	"math"
	"math/rand"
	"os"
	"runtime"
	"sort"
	"strconv"
	"sync"
	"time"

	"github.com/gookit/slog"
	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"

	"net/http"

	"github.com/gin-contrib/cors"
	"github.com/gin-gonic/gin"

	"github.com/adhocore/chin"
)

/*
	type GraphNode struct {
		Lat float64
		Lon float64

		nw int
		ne int
		sw int
		se int
	}
*/
type RandomNode struct {
	lat float64
	lng float64
}
type Node struct {
	ID          int64
	Lat         float64
	Lon         float64
	isCoastline bool
}
type Edge struct {
	Startpoint [2]float64
	Endpoint   [2]float64
	WayId      osm.NodeID
}
type WayComplete struct {
	WayMapKey  osm.NodeID
	isFinished bool
}
type QueueItem struct {
	node     int
	priority int
}
type PriorityQueue []*QueueItem

type EdgeWithDistance struct {
	Edge     [2]int
	Distance float64
}

// Point represents a geographical point with latitude and longitude
type Point struct {
	Lat float64 `json:"lat"`
	Lng float64 `json:"lng"`
}

func (pq PriorityQueue) Len() int { return len(pq) }
func (pq PriorityQueue) Less(i, j int) bool {
	return pq[i].priority < pq[j].priority
}

func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
}

func (pq *PriorityQueue) Push(x interface{}) {
	*pq = append(*pq, x.(*QueueItem))
}

func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	*pq = old[0 : n-1]
	return item
}

/*
	type PolygonBoundingBox struct {
		LatMin float64
		LatMax float64
		LonMin float64
		LonMax float64
	}
*/
const PBF_FILE_PATH = "/home/sahask/osm_data/planet-coastlines.osm.pbf"

const pointcount = 4000000
const longBincount = 3600

func graphGenerator() {
	var start = time.Now()
	f, err := os.Open(PBF_FILE_PATH)
	if err != nil {
		slog.Error("Error opening PBF File!")
		panic(err)
	}
	defer f.Close()

	scanner := osmpbf.New(context.Background(), f, runtime.NumCPU())
	defer scanner.Close()

	// featureCollection := geojson.NewFeatureCollection()

	nodes_map := make(map[osm.NodeID]Node)
	edges_map := make([][]Edge, longBincount)

	ways_map := make(map[osm.NodeID][]osm.NodeID)
	nodeRelations := make(map[osm.NodeID]WayComplete)
	wayMapIdForNodeId := make(map[osm.NodeID]osm.NodeID)
	wayBoundingBox := make(map[osm.NodeID][4]float64)
	waysMidPoint := make(map[osm.NodeID][2]float64)
	rows := 180
	colums := 360
	randomPointCount := pointcount
	grid := make([][][][3]float64, rows)
	graphNodes := [pointcount][2]float64{}
	graphEdges := [pointcount][4]int{}
	for i := range graphEdges {
		for j := range graphEdges[i] {
			graphEdges[i][j] = -1
		}
	}
	distancesEdges := [pointcount][4]int{}
	for i := range grid {
		grid[i] = make([][][3]float64, colums)
		for j := range grid[i] {
			grid[i][j] = [][3]float64{}
		}
	}

	//a, b := findRowAndColumnInGrid(rows, colums, -90.0, -180.0)
	//fmt.Println("row colum", a, b)

	scanner.SkipRelations = true

	slog.Info("Starting scan...")

	for scanner.Scan() {
		o := scanner.Object()
		// bar.Set(int(scanner.FullyScannedBytes()))
		// fmt.Println(scanner.FullyScannedBytes() / (1024 * 1024))
		switch v := o.(type) {
		case *osm.Node:
			nodes_map[v.ID] = Node{ID: int64(v.ID), Lat: float64(v.Lat), Lon: float64(v.Lon), isCoastline: false}
		case *osm.Way:
			if v.Tags.Find("natural") == "coastline" {
				ways_map[v.Nodes.NodeIDs()[0]] = v.Nodes.NodeIDs()
				for _, node := range v.Nodes.NodeIDs() {
					temp_node := nodes_map[node]
					temp_node.isCoastline = true
					nodes_map[node] = temp_node
				}
			} else if v.Tags.Find("place") == "ocean" {
				fmt.Println("found ocean")
			}
		}
	}
	runtime.GC()

	slog.Info("Nodes cleaned...")
	for key, way := range ways_map {
		nodeRelations[way[0]] = WayComplete{WayMapKey: key, isFinished: false}
	}

	for i := range nodeRelations {
		if !nodeRelations[i].isFinished {
			copy := nodeRelations[i]
			copy.isFinished = true
			nodeRelations[i] = copy

			way := ways_map[nodeRelations[i].WayMapKey]
			if way[0] != way[len(way)-1] {
				RecursivWayROunding(ways_map, nodeRelations, i, way)
			}

		}
	}
	slog.Info("Ways merged...Julian")
	fmt.Println(len(ways_map))
	for k := range ways_map { //delete duplicated ways
		if ways_map[k][0] != ways_map[k][len(ways_map[k])-1] {
			delete(ways_map, k)
		}
	}
	slog.Info("duplicate ways deleted...Julian")
	for k := range ways_map {
		for l := range ways_map[k] {
			if l+1 < len(ways_map[k]) {
				AddEdge(&edges_map, Edge{[2]float64{nodes_map[ways_map[k][l]].Lat, nodes_map[ways_map[k][l]].Lon}, [2]float64{nodes_map[ways_map[k][l+1]].Lat, nodes_map[ways_map[k][l+1]].Lon}, k})
			}

		}
	}
	mostedges := -1
	leastedges := 10000000000
	for k := range edges_map {
		if len(edges_map[k]) > mostedges {
			mostedges = len(edges_map[k])
		}
		if len(edges_map[k]) < leastedges {
			leastedges = len(edges_map[k])
		}
	}
	fmt.Println("edges cound", mostedges, leastedges)
	interedges := FindEdgesForPointpoint(&edges_map, [2]float64{-70.0, 80.0})
	returedges := [][4]float64{}
	for k := range interedges {
		startpoint := interedges[k].Startpoint
		endpoint := interedges[k].Endpoint
		returedges = append(returedges, [4]float64{startpoint[0], startpoint[1], endpoint[0], endpoint[1]})
	}
	fmt.Println(returedges)

	//random points
	slog.Info("starttime 1 mio: " + time.Since(start).String())
	for s := 0; s < randomPointCount; s++ {
		if s%10000 == 0 {
			slog.Info("10k: " + time.Since(start).String())
		}
		randLong := rand.Float64()*360.0 - 180.0
		randLat := float64((math.Asin(rand.Float64()*2.0-1.0) * 180.0 / math.Pi))
		for {
			randLong = rand.Float64()*360.0 - 180.0
			randLat = float64((math.Asin(rand.Float64()*2.0-1.0) * 180.0 / math.Pi))

			edges := FindEdgesForPointpoint(&edges_map, [2]float64{randLat, randLong}) //polygon test
			if len(edges)%2 == 0 && randLong != 0.0 && randLat != 0.0 {
				break
			}

		}
		if randLat >= 89.0 {
			grid[0][0] = append(grid[0][0], [3]float64{randLat, randLong, float64(s)})
		} else if randLat <= -89.0 {
			grid[0][0] = append(grid[0][0], [3]float64{randLat, randLong, float64(s)})
		} else {

			a, b := findRowAndColumnInGrid(rows, colums, float64(randLat), randLong)
			//fmt.Println(a, b, randLat, randLong)
			grid[a][b] = append(grid[a][b], [3]float64{randLat, randLong, float64(s)})
		}
		graphNodes[s] = [2]float64{randLat, randLong}
	}
	highestam := -1
	v := -1
	u := -1
	for k := 0; k < len(grid); k++ {
		for l := 0; l < len(grid[k]); l++ {
			if len(grid[k][l]) > highestam {
				highestam = len(grid[k][l])
				u = k
				v = l
			}
		}
	}
	fmt.Println("highestamountingrid", highestam, u, v)
	fmt.Println("00 grid length", len(grid[0][0]))
	slog.Info("endtime 1 mio: " + time.Since(start).String())
	//fmt.Println(graphNodes)
	//fmt.Println(grid)

	//neu
	//sorttest
	edges := [][2]int{
		{0, 3},
		{2, 5},
		{1, 3},
		{0, 3},
		{2, 5},
		{1, 4},
		{5, 4},
	}
	distances := []float64{
		1.2,
		2.3,
		1.5,
		1.2,
		2.3,
		2.5,
		2.1,
	}

	// Sort and remove duplicates
	sortedEdges, sortedDistances, startIndices := sortAndRemoveDuplicates(edges, distances, 5)
	fmt.Println(sortedEdges, sortedDistances, startIndices)
	var mu sync.Mutex
	var wg sync.WaitGroup

	for s, k := range grid {
		for t, points := range k {

			if len(points) == 0 {
				break
			}
			p := getAllNeighbourCellss(grid, s, t, points[0][0], 1)

			for _, u := range points {

				nearestHelper(u, p, graphNodes, &edges, &distances, &mu, 1)
				nearestHelper(u, p, graphNodes, &edges, &distances, &mu, 2)
				nearestHelper(u, p, graphNodes, &edges, &distances, &mu, 3)
				nearestHelper(u, p, graphNodes, &edges, &distances, &mu, 4)
			}
		}

	}

	slog.Info("neighbours finsished good: " + time.Since(start).String())

	fmt.Println("finished neighbours")

	for k := range ways_map { //fill boundingBox of ways map (used for faster point in polygon test) makes polygones around +-180longitude max but i dont care
		latmin := math.MaxFloat64
		latmax := -math.MaxFloat64
		lonmin := math.MaxFloat64
		lonmax := -math.MaxFloat64
		for m := range ways_map[k] {
			if nodes_map[ways_map[k][m]].Lat < latmin {
				latmin = nodes_map[ways_map[k][m]].Lat
			}
			if nodes_map[ways_map[k][m]].Lon < lonmin {
				lonmin = nodes_map[ways_map[k][m]].Lon
			}
			if nodes_map[ways_map[k][m]].Lat > latmax {
				latmax = nodes_map[ways_map[k][m]].Lat
			}
			if nodes_map[ways_map[k][m]].Lon > lonmax {
				lonmax = nodes_map[ways_map[k][m]].Lon
			}
		}
		if latmin-0.01 > -90.0 {
			latmin = latmin - 0.01
		} else {
			latmin = -90.0
		}
		if latmax+0.01 < 90.0 {
			latmax = latmax + 0.01
		} else {
			latmax = 90.0
		}
		if lonmin-0.01 > -180.0 {
			lonmin = lonmin - 0.01
		} else {
			lonmin = -180.0
		}
		if lonmax+0.01 < 180.0 {
			lonmax = lonmax + 0.01
		} else {
			lonmax = 180.0
		}
		wayBoundingBox[k] = [4]float64{latmin, latmax, lonmin, lonmax}

	}
	slog.Info("bounding box finished...Julian")
	//fmt.Println(wayBoundingBox)
	for key, value := range ways_map { //used to find way associated with node id
		for s := range value {
			wayMapIdForNodeId[value[s]] = key
		}
	}
	slog.Info("waymapcreated...Julian")

	slog.Info("centroid...Julian")
	//fmt.Println(waysMidPoint)
	testPoint := [2]float64{-48.8766666, -123.3933333} //watwre
	//testPoint := [2]float64{-62.2385073, -59.10429} //land way/way/1270573186
	//fmt.Println(wayMapIdForNodeId)
	//testWhichPolyCrosses180(ways_map, nodes_map)
	fmt.Println(ways_map[wayMapIdForNodeId[11799186160]])
	fmt.Println(testPolygon(testPoint, wayMapIdForNodeId[11799186160], ways_map[wayMapIdForNodeId[11799186160]], nodes_map, waysMidPoint))
	//inttest := testAllPolygonesIfMidpointsisIn(ways_map, nodes_map, waysMidPoint, wayBoundingBox)
	//fmt.Println("must be -1", inttest)
	//inttest := testAllPolygonesWithBounding(testPoint, ways_map, nodes_map, waysMidPoint, wayBoundingBox)
	//fmt.Println(inttest)
	fmt.Println("Coastlines count:	")
	fmt.Println(len(ways_map))
	const numPoints = 200
	const batchSize = 10

	points := generateRandomPoints(numPoints)
	results := make([]int, numPoints)

	//var wg sync.WaitGroup
	numBatches := (numPoints + batchSize - 1) / batchSize

	for batch := 0; batch < numBatches; batch++ {
		start := batch * batchSize
		end := (batch + 1) * batchSize
		if end > numPoints {
			end = numPoints
		}
		wg.Add(1)
		go processPoints(points, ways_map, nodes_map, wayBoundingBox, results, start, end, &wg)
	}

	wg.Wait()

	slog.Info("neighbours finsished good: " + time.Since(start).String())

	// write graphNodes graphEdges distancesEdges grid to different json files
	graphNodesJSON, _ := json.Marshal(graphNodes)
	err = os.WriteFile("graphNodes.json", graphNodesJSON, 0644)
	if err != nil {
		fmt.Println("Error writing graphNodes to file:", err)
	}

	graphEdgesJSON, _ := json.Marshal(graphEdges)
	err = os.WriteFile("graphEdges.json", graphEdgesJSON, 0644)
	if err != nil {
		fmt.Println("Error writing graphEdges to file:", err)
	}

	distancesEdgesJSON, _ := json.Marshal(distancesEdges)
	err = os.WriteFile("distancesEdges.json", distancesEdgesJSON, 0644)
	if err != nil {
		fmt.Println("Error writing distancesEdges to file:", err)
	}

	gridJSON, _ := json.Marshal(grid)
	err = os.WriteFile("grid.json", gridJSON, 0644)
	if err != nil {
		fmt.Println("Error writing grid to file:", err)
	}

	slog.Info("Finished scan...")
	slog.Info("Total time: " + time.Since(start).String())

}

func server(graphNodes [][2]float64, graphEdges [][4]int, distancesEdges [][4]int, landmarks [][2]float64, grid [][][]int) {

	router := gin.Default()

	// Set up CORS middleware
	config := cors.DefaultConfig()
	config.AllowAllOrigins = true // This allows all origins, you can specify the allowed origins if needed
	config.AllowMethods = []string{"GET", "POST", "PUT", "PATCH", "DELETE", "OPTIONS"}
	config.AllowHeaders = []string{"Origin", "Content-Type", "Accept"}

	router.Use(cors.New(config))

	dist := make(map[int]int)

	for node := range graphNodes {
		dist[node] = math.MaxInt32
	}

	router.POST("/submit_points", func(c *gin.Context) {
		var requestData map[string]Point
		if err := c.BindJSON(&requestData); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"message": "Invalid points"})
			return
		}

		start, startOk := requestData["start"]
		end, endOk := requestData["end"]
		if !startOk || !endOk {
			c.JSON(http.StatusBadRequest, gin.H{"message": "Invalid points"})
			return
		}

		// fmt.Printf("Received start point: %+v\n", start)
		// fmt.Printf("Received end point: %+v\n", end)

		// Call the Algo function
		// copier.CopyWithOption(&tempDist, &dist, copier.Option{DeepCopy: true})
		// copier.Copy(&tempDist, &dist)
		// copier.CopyWithOption(&tempDist, &dist, copier.Option{DeepCopy: true})
		// copier.Copy(&tempDist, &dist)
		var startTimeDijkstra = time.Now()
		shortestPathDjikstra := AlgoDijkstra(start, end, graphNodes, graphEdges, distancesEdges, grid)
		var timeTakenDijkstra = time.Since(startTimeDijkstra).Milliseconds()

		// copier.Copy(&tempDist, &dist)

		fmt.Println("Dijkstra Time:", timeTakenDijkstra)
		// fmt.Println("Shortest Path:", shortestPath)

		c.JSON(http.StatusOK, gin.H{
			"dijkstra_time":      timeTakenDijkstra,
			"shortest_path_djik": shortestPathDjikstra,
		})
	})

	router.Run(":5000")
}

func main() {
	fmt.Println(os.Args)

	graphNodesJSON, err := os.ReadFile("graphNodes.json")
	if err != nil {
		fmt.Println("Error reading graphNodes from file:", err)
	}
	var graphNodes [][2]float64
	err = json.Unmarshal(graphNodesJSON, &graphNodes)
	if err != nil {
		fmt.Println("Error unmarshalling graphNodes:", err)
	}

	fmt.Println("Graph Nodes:", len(graphNodes))

	graphEdgesJSON, err := os.ReadFile("graphEdges.json")
	if err != nil {
		fmt.Println("Error reading graphEdges from file:", err)
	}
	var graphEdges [][4]int
	err = json.Unmarshal(graphEdgesJSON, &graphEdges)
	if err != nil {
		fmt.Println("Error unmarshalling graphEdges:", err)
	}

	fmt.Println("Graph Edges:", len(graphEdges))

	distancesEdgesJSON, err := os.ReadFile("distancesEdges.json")
	if err != nil {
		fmt.Println("Error reading distancesEdges from file:", err)
	}
	var distancesEdges [][4]int
	err = json.Unmarshal(distancesEdgesJSON, &distancesEdges)
	if err != nil {
		fmt.Println("Error unmarshalling distancesEdges:", err)
	}

	fmt.Println("Distances Edges:", len(distancesEdges))

	gridJSON, err := os.ReadFile("grid.json")
	if err != nil {
		fmt.Println("Error reading grid from file:", err)
	}
	var grid [][][]int
	err = json.Unmarshal(gridJSON, &grid)
	if err != nil {
		fmt.Println("Error unmarshalling grid:", err)
	}

	fmt.Println("Grid:", len(grid))

	var landmarks [][2]float64
	landmarksJSON, err := os.ReadFile("landmarks.json")
	if err != nil {
		fmt.Println("Error reading landmarks from file:", err)
	}
	err = json.Unmarshal(landmarksJSON, &landmarks)
	if err != nil {
		fmt.Println("Error unmarshalling landmarks:", err)
	}

	if os.Args[1] == "server" {
		fmt.Println("Server")
		server(graphNodes, graphEdges, distancesEdges, landmarks, grid)
	} else if os.Args[1] == "graph" {
		fmt.Println("Graph Generator")
		graphGenerator()
	} else if os.Args[1] == "multiple" {
		fmt.Println("Multiple Analysis")
		iterations, err := strconv.Atoi(os.Args[2])
		if err != nil {
			fmt.Println("Error parsing iterations:", err)
			return
		}
		fidgeter := chin.New()
		go fidgeter.Start()

		var randomIndices = make([][2]int, iterations)
		for i := 0; i < iterations; i++ {
			randomIndices[i][0] = rand.Intn(len(graphNodes))
			randomIndices[i][1] = rand.Intn(len(graphNodes))
		}

		fmt.Println(iterations, "Random points generated")

		// tempDist := make(map[int]int)
		var startDijkstra = time.Now()
		for i := 0; i < iterations; i++ {
			// copier.CopyWithOption(&tempDist, &dist, copier.Option{DeepCopy: true})
			// copier.Copy(&tempDist, &dist)
			Dijkstra(graphNodes, graphEdges, distancesEdges, randomIndices[i][0], randomIndices[i][1])
		}
		fmt.Println("Average Dijsktra time: ", time.Since(startDijkstra)/time.Duration(iterations))
		fidgeter.Stop()
	} else {
		fmt.Println("Invalid arguments")
	}
}

func AlgoDijkstra(Start Point, End Point, graphNodes [][2]float64, graphEdges [][4]int, distancesEdges [][4]int, grid [][][]int) []Point {
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
		distpointStartNew := haversine(graphNodes[k][0], graphNodes[k][1], nearestnodeStart[0], nearestnodeStart[1])
		distpointENdNew := haversine(graphNodes[k][0], graphNodes[k][1], nearestnodeEnd[0], nearestnodeEnd[1])

		if distpointStartNew < float64(distpointStart) {
			nearestpointStartIndex = k
			distpointStart = distpointStartNew
		}
		if distpointENdNew < distpointEnd {
			nearpointEndIndex = k
			distpointEnd = distpointENdNew
		}
	}

	// fmt.Println("dijkstra go", nearestpointStartIndex, graphNodes[nearestpointStartIndex], nearpointEndIndex, graphNodes[nearpointEndIndex])
	// slog.Info("dykstra start: " + time.Since(start).String())
	// var startDijkstra = time.Now()
	// _, path := Dijkstra(graphNodes[:], graphEdges[:], distancesEdges[:], nearestpointStartIndex, nearpointEndIndex)
	_, path := Dijkstra(graphNodes[:], graphEdges[:], distancesEdges[:], nearestpointStartIndex, nearpointEndIndex)
	// fmt.Println(dist)
	// slog.Info("dykstra end: " + time.Since(startDijkstra).String())
	returndykstrapath := [][2]float64{}
	for k := range path {
		returndykstrapath = append(returndykstrapath, graphNodes[path[k]])
	}

	//fmt.Println(viewEdges)
	//fmt.Println(graphEdges)
	//fmt.Println(returnEdges2)

	// highest := -1
	// counter := 0
	// for k := range grid {
	// 	for l := range grid[k] {
	// 		if len(grid[k][l]) > highest {
	// 			highest = len(grid[k][l])
	// 		}
	// 		if len(grid[k][l]) > 0 && len(grid[k][l]) < 5 {
	// 			counter++
	// 		}

	// 	}

	// }
	// fmt.Println("highest", highest, "count greater 0", counter)

	//randompoints end
	// fmt.Println(returndykstrapath)

	// Convert returndykstrapath to []Point
	shortestPath := make([]Point, len(returndykstrapath))
	for i, point := range returndykstrapath {
		shortestPath[i] = Point{Lat: point[0], Lng: point[1]}
	}

	return shortestPath
}

func AddEdge(longranges *[][]Edge, edge Edge) {
	// Assuming longitude ranges from -180 to 180
	rangeWidth := 360.0 / float64(longBincount)
	startRange := int((edge.Startpoint[1] + 180) / rangeWidth)
	endRange := int((edge.Endpoint[1] + 180) / rangeWidth)

	if startRange >= longBincount {
		startRange = longBincount - 1
	}
	if endRange >= longBincount {
		endRange = longBincount - 1
	}
	if startRange <= endRange && startRange-endRange < 90 {
		for i := startRange; i <= endRange; i++ {
			(*longranges)[i] = append((*longranges)[i], edge)
		}
	} else if endRange-startRange < 90 {
		for i := endRange; i <= startRange; i++ {
			(*longranges)[i] = append((*longranges)[i], edge)
		}
	}

}
func RecursivWayROunding(ways_map map[osm.NodeID][]osm.NodeID, nodeRelations map[osm.NodeID]WayComplete, i osm.NodeID, way []osm.NodeID) {
	copy := nodeRelations[i]
	copy.isFinished = true
	nodeRelations[i] = copy
	if way[0] != way[len(way)-1] {
		index := way[len(way)-1]
		if node, exists := nodeRelations[index]; exists { //search for fitting lastnode in node reltions
			if ways_map[node.WayMapKey][0] == ways_map[node.WayMapKey][len(ways_map[node.WayMapKey])-1] {
				fmt.Println("found a fucking roundway instead of next slice")
				//fmt.Println(len(ways_map[found.WayMapKey]))
			} else {
				if !nodeRelations[index].isFinished {
					way = append(way, ways_map[nodeRelations[index].WayMapKey][1:]...)
					node.isFinished = true
					nodeRelations[index] = node
					if way[0] == way[len(way)-1] {
						//fmt.Println("perfekt")
						ways_map[way[0]] = way
					} else {

						RecursivWayROunding(ways_map, nodeRelations, index, way)

					}

				}
			}
		} else {
			fmt.Println("error")
		}
	}

}

// convert spherical coordinates(long lat) to cartesian (or other way round)
func sphereToCart(latitude float64, longitude float64) (float64, float64, float64) {
	latitudeinRadian := latitude * (math.Pi / 180)
	logitudeinRadian := longitude * (math.Pi / 180)
	x := math.Cos(latitudeinRadian) * math.Cos(logitudeinRadian)
	y := math.Cos(latitudeinRadian) * math.Sin(logitudeinRadian)
	z := math.Sin(latitudeinRadian)
	return x, y, z
}
func cartToSphere(x float64, y float64, z float64) (float64, float64) {
	lat := math.Atan2(z, math.Sqrt(x*x+y*y))
	long := math.Atan2(y, x)
	return lat * (180 / math.Pi), long * (180 / math.Pi)
}
func crossProduct(u, v [3]float64) [3]float64 {
	return [3]float64{
		u[1]*v[2] - u[2]*v[1],
		u[2]*v[0] - u[0]*v[2],
		u[0]*v[1] - u[1]*v[0],
	}
}
func dotProduct(u, v [3]float64) float64 {
	return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]
}
func normalize(v [3]float64) [3]float64 {
	mag := math.Sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
	if mag == 0 {
		return v
	}
	return [3]float64{v[0] / mag, v[1] / mag, v[2] / mag}
}
func FindEdgesForPointpoint(longranges *[][]Edge, testpoint [2]float64) []Edge {
	rangeWidth := 360.0 / float64(longBincount)
	bin := int((testpoint[1] + 180) / rangeWidth)
	if bin >= longBincount {
		bin = longBincount - 1
	}

	var intersectingEdges []Edge
	for _, edge := range (*longranges)[bin] {
		found := intersection2Lines(edge.Startpoint, edge.Endpoint, testpoint)
		if found {
			intersectingEdges = append(intersectingEdges, edge)
		}
	}
	//fmt.Println("intersecting edgs", len(intersectingEdges))
	return intersectingEdges
}
func findIntersection(a1, a2, testpoint [2]float64) ([2]float64, bool) {
	R := 6371.0
	northpole := [2]float64{90.0, 90.0}
	x1, y1, z1 := sphereToCart(a1[0], a1[1])
	x2, y2, z2 := sphereToCart(a2[0], a2[1])
	x3, y3, z3 := sphereToCart(northpole[0], northpole[1])
	x4, y4, z4 := sphereToCart(testpoint[0], testpoint[1])
	//fmt.Println("x1", x1, y1, z1)
	//fmt.Println("x1", x2, y2, z2)
	//fmt.Println("x1", x1, y1, z1)
	//normal vectors
	n1 := crossProduct([3]float64{x1, y1, z1}, [3]float64{x2, y2, z2})
	n2 := crossProduct([3]float64{x3, y3, z3}, [3]float64{x4, y4, z4})
	//fmt.Println("n1", n1, n2)
	// intersection line
	line := crossProduct(n1, n2)
	//fmt.Println(line)
	line = normalize(line)
	//fmt.Println(line)
	// Intersection points
	p11, p12 := cartToSphere(line[0]*R, line[1]*R, line[2]*R)
	p21, p22 := cartToSphere(-line[0]*R, -line[1]*R, -line[2]*R)
	p1 := [2]float64{p11, p12}
	p2 := [2]float64{p21, p22}
	fmt.Println(p11, p12, p21, p22)
	//find correct
	if isPointOnGreatCircleArc(p1, a1, a2) && isPointOnGreatCircleArc(p1, northpole, testpoint) {
		fmt.Println("worked")
		return p1, true
	}
	if isPointOnGreatCircleArc(p2, a1, a2) && isPointOnGreatCircleArc(p2, northpole, testpoint) {
		fmt.Println("worked")
		return p2, true
	}

	return [2]float64{}, false
}
func pInBounds(p, a1, a2 [2]float64) bool {
	minLon, maxLon := math.Min(a1[1], a2[1]), math.Max(a1[1], a2[1])
	minLat, maxLat := math.Min(a1[0], a2[0]), math.Max(a1[0], a2[0])

	return minLon <= p[1] && p[1] <= maxLon && minLat <= p[0] && p[0] <= maxLat
}
func isPointOnGreatCircleArc(p, a1, a2 [2]float64) bool {
	R := 6371.0
	// Convert points to Cartesian coordinates
	px, py, pz := sphereToCart(p[0], p[1])
	x1, y1, z1 := sphereToCart(a1[0], a1[1])
	x2, y2, z2 := sphereToCart(a2[0], a2[1])

	// Compute vectors
	v1 := [3]float64{x1, y1, z1}
	v2 := [3]float64{x2, y2, z2}
	vp := [3]float64{px, py, pz}

	// Check if the point is on the great circle
	c1 := crossProduct(v1, vp)
	c2 := crossProduct(vp, v2)

	// Ensure cross products are in the same direction
	dot := dotProduct(c1, c2)
	if dot < 0 {
		return false
	}

	// Check if the point lies between a1 and a2
	angle1 := math.Acos(dotProduct(v1, vp) / (R * R))
	angle2 := math.Acos(dotProduct(vp, v2) / (R * R))
	totalAngle := math.Acos(dotProduct(v1, v2) / (R * R))

	return math.Abs((angle1+angle2)-totalAngle) < 1e-9
}
func initialBearingTo(startpoint, endpoint [2]float64) float64 {

	angle1 := (startpoint[0] + 90.0) * (math.Pi / 180.0)
	angle2 := (endpoint[0] + 90.0) * (math.Pi / 180.0)
	lambda := ((startpoint[1] + 180.0) - (endpoint[1] + 180.0)) * (math.Pi / 180.0)

	x := math.Cos(angle1)*math.Sin(angle2) - math.Sin(angle1)*math.Cos(angle2)*math.Cos(lambda)
	y := math.Sin(lambda) * math.Cos(angle2)
	θ := math.Atan2(y, x)
	fmt.Println("bearingresluts", angle1, angle2, lambda, x, y, θ)
	bearing := math.Abs(θ) * (180.0 / math.Pi)
	fmt.Println(bearing)
	if bearing > 360.0 {
		bearing = bearing - 360.0
	}
	if bearing < 0 {
		bearing = bearing + 360.0
	}
	return bearing - 180.0
}
func intersection2Lines(path1start [2]float64, path1end [2]float64, path2start [2]float64) bool {
	path2end := [2]float64{90, 0}
	if path1start == path2start {
		//return path1start[0], path2start[1]
		return true
	} // coincident points

	p00, p01, p02 := sphereToCart(path1start[0], path1start[1])
	p10, p11, p12 := sphereToCart(path1end[0], path1end[1])
	p20, p21, p22 := sphereToCart(path2start[0], path2start[1])
	p30, p31, p32 := sphereToCart(path2end[0], path2end[1])
	v1 := [3]float64{p00, p01, p02}
	v2 := [3]float64{p10, p11, p12}
	v3 := [3]float64{p20, p21, p22}
	v4 := [3]float64{p30, p31, p32}

	c1 := crossProduct(v1, v2)
	c2 := crossProduct(v3, v4)

	i1 := crossProduct(c1, c2)
	i2 := crossProduct(c2, c1)

	//mid = p1.plus(p2).plus(v2).plus(v4);
	//intersection = mid.dot(i1) > 0 ? i1 : i2;
	lat1, lon1 := cartToSphere(i1[0], i1[1], i1[2])
	lat2, lon2 := cartToSphere(i2[0], i2[1], i2[2])
	if lat1 <= math.Max(path2start[0], path2end[0]) && lat1 >= math.Min(path2start[0], path2end[0]) && lon1 <= math.Max(path1start[1], path1end[1]) && lon1 >= math.Min(path1start[1], path1end[1]) {
		//return lat1, lon1
		return true
	}
	if lat2 <= math.Max(path2start[0], path2end[0]) && lat2 >= math.Min(path2start[0], path2end[0]) && lon2 <= math.Max(path1start[1], path1end[1]) && lon2 >= math.Min(path1start[1], path1end[1]) {
		//return lat2, lon2
		return true
	}
	//fmt.Println("intersectcandidates:", lat1, lon1, lat2, lon2)

	return false
}
func intersectionBearing(p1 [2]float64, brng1 float64, p2 [2]float64, brng2 float64) [2]float64 {

	φ1 := p1[0] * (math.Pi / 180.0)
	λ1 := p1[1] * (math.Pi / 180.0)
	φ2 := p2[0] * (math.Pi / 180.0)
	λ2 := p2[1] * (math.Pi / 180.0)
	θ13 := brng1 * (math.Pi / 180.0)
	θ23 := brng2 * (math.Pi / 180.0)
	Δφ := φ2 - φ1
	Δλ := λ2 - λ1

	// angular distance p1-p2
	δ12 := 2 * math.Asin(math.Sqrt(math.Sin(Δφ/2)*math.Sin(Δφ/2)+math.Cos(φ1)*math.Cos(φ2)*math.Sin(Δλ/2)*math.Sin(Δλ/2)))
	if math.Abs(δ12) < 0.0000000001 {
		return [2]float64{p1[0], p1[1]}
	} // coincident points

	// initial/final bearings between points
	Cosθa := (math.Sin(φ2) - math.Sin(φ1)*math.Cos(δ12)) / (math.Sin(δ12) * math.Cos(φ1))
	Cosθb := (math.Sin(φ1) - math.Sin(φ2)*math.Cos(δ12)) / (math.Sin(δ12) * math.Cos(φ2))
	θa := math.Acos(math.Min(math.Max(Cosθa, -1), 1)) // protect against rounding errors
	θb := math.Acos(math.Min(math.Max(Cosθb, -1), 1)) // protect against rounding errors

	θ12 := θa
	if math.Sin(λ2-λ1) > 0 {
		θ12 = 2*math.Pi - θa
	}

	θ21 := 2*math.Pi - θb
	if math.Sin(λ2-λ1) > 0 {
		θ21 = θb
	}

	α1 := θ13 - θ12 // angle 2-1-3
	α2 := θ21 - θ23 // angle 1-2-3

	if math.Sin(α1) == 0 && math.Sin(α2) == 0 {
		return [2]float64{}
	} // infinite intersections
	if math.Sin(α1)*math.Sin(α2) < 0 {
		return [2]float64{}
	} // ambiguous intersection (antipodal/360°)

	Cosα3 := -math.Cos(α1)*math.Cos(α2) + math.Sin(α1)*math.Sin(α2)*math.Cos(δ12)

	δ13 := math.Atan2(math.Sin(δ12)*math.Sin(α1)*math.Sin(α2), math.Cos(α2)+math.Cos(α1)*Cosα3)

	φ3 := math.Asin(math.Min(math.Max(math.Sin(φ1)*math.Cos(δ13)+math.Cos(φ1)*math.Sin(δ13)*math.Cos(θ13), -1), 1))

	Δλ13 := math.Atan2(math.Sin(θ13)*math.Sin(δ13)*math.Cos(φ1), math.Cos(δ13)-math.Sin(φ1)*math.Sin(φ3))
	λ3 := λ1 + Δλ13

	lat := φ3 * (180.0 / math.Pi)
	lon := λ3 * (180.0 / math.Pi)

	return [2]float64{lat, lon}
}

// help functions for transform into koordinate system where a point acts as north pole
func rotationMatrix(axis [3]float64, theta float64) [3][3]float64 { //chatgpt better faster and robuster than normal rotation matrix
	a := math.Cos(theta / 2.0)
	b := -axis[0] * math.Sin(theta/2.0)
	c := -axis[1] * math.Sin(theta/2.0)
	d := -axis[2] * math.Sin(theta/2.0)

	aa, bb, cc, dd := a*a, b*b, c*c, d*d
	bc, ad, ac, ab, bd, cd := b*c, a*d, a*c, a*b, b*d, c*d

	return [3][3]float64{
		{aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)},
		{2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)},
		{2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc},
	}
}
func rotatePointWithMatrix(x float64, y float64, z float64, Matrix [3][3]float64) (float64, float64, float64) {
	newX := Matrix[0][0]*x + Matrix[0][1]*y + Matrix[0][2]*z
	newY := Matrix[1][0]*x + Matrix[1][1]*y + Matrix[1][2]*z
	newZ := Matrix[2][0]*x + Matrix[2][1]*y + Matrix[2][2]*z
	return newX, newY, newZ
}
func calculateMatrixWithNewPolePoint(newPoleLat float64, newPoleLong float64) [3][3]float64 {
	newPoleX, newPoleY, newPoleZ := sphereToCart(newPoleLat, newPoleLong) //rotation better in this coordinate system
	// theta= roation angle, axis is roattion axis
	theta := math.Acos(newPoleZ)
	axis := [3]float64{-newPoleY, newPoleX, 0}
	// Normalize the axis
	norm := math.Sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2])
	if norm > 0 {
		axis[0] /= norm
		axis[1] /= norm
		axis[2] /= norm
	}
	Matrix := rotationMatrix(axis, theta)
	return Matrix
}
func calculateNewPointPositionWithMatrix(pLat float64, pLong float64, matrix [3][3]float64) (float64, float64) {
	if pLat <= -90.1 || pLat >= 90.1 {
		fmt.Println("impossible")
	}
	x, y, z := sphereToCart(pLat, pLong)
	newX, newY, newZ := rotatePointWithMatrix(x, y, z, matrix)
	newLat, newLong := cartToSphere(newX, newY, newZ)
	return newLat, newLong
}

// compute centroid of polygon to have a point X which is contained by it
func calculateCentroid(longlatOfNodes [][2]float64) (float64, float64) {
	var sumX float64
	var sumY float64
	var sumZ float64
	n := float64(len(longlatOfNodes))
	for _, node := range longlatOfNodes {
		vx, vy, vz := sphereToCart(node[0], node[1])
		sumX = vx + sumX
		sumY = vy + sumY
		sumZ = vz + sumZ
	}
	x := sumX / n
	y := sumY / n
	z := sumZ / n
	norm := math.Sqrt(x*x + y*y + z*z)
	normedx := x / norm
	normedy := y / norm
	normedz := z / norm
	lat, long := cartToSphere(normedx, normedy, normedz)
	return lat, long
}
func leftAngleSmaller(left float64, right float64) bool {
	left = left + 180.0
	right = right + 180.0
	if left-right <= 180.0 && left-right >= 0.0 { //everything coorect
		return false //right is bigger
	} else if right-left <= 180.0 && right-left >= 0.0 { //correct
		return true //left is biiger

	} else if left-right >= 0.0 { //example 351-20
		return true //in this case the higher value 351 should be the lower boundary value
	} else if right-left >= 0.0 {
		return false
	} else {
		fmt.Println("impossible")
		return false
	}
}
func calculateAnglesABPwithOutPole(a [2]float64, b [2]float64, p [2]float64) int { //1==true
	angleA := a[1]
	angleB := b[1]
	angleP := p[1]
	if angleA <= angleB {
		if angleA <= angleP && angleP < angleB {
			return 1
		} else {
			return -1
		}
	} else {
		if angleP < angleA && angleB <= angleP {
			return 1
		} else {
			return -1
		}
	}
}
func calculateAnglesABPwithNewPoleEasy(a [2]float64, b [2]float64, p [2]float64, matrix [3][3]float64) int { //1==true
	_, angleA := calculateNewPointPositionWithMatrix(a[0], a[1], matrix)
	_, angleB := calculateNewPointPositionWithMatrix(b[0], b[1], matrix)
	_, angleP := calculateNewPointPositionWithMatrix(p[0], p[1], matrix)
	if angleA <= angleB {
		if angleA <= angleP && angleP < angleB {
			return 1
		} else {
			return -1
		}
	} else {
		if angleP < angleA && angleB <= angleP {
			return 1
		} else {
			return -1
		}
	}
}
func calculateAnglesABPwithNewPoleBetter(a [2]float64, b [2]float64, p [2]float64, matrix [3][3]float64) int { //1==true
	_, angleA := calculateNewPointPositionWithMatrix(a[0], a[1], matrix)
	_, angleB := calculateNewPointPositionWithMatrix(b[0], b[1], matrix)
	_, angleP := calculateNewPointPositionWithMatrix(p[0], p[1], matrix)
	if leftAngleSmaller(angleA, angleB) {
		if leftAngleSmaller(angleA, angleP) && leftAngleSmaller(angleP, angleB) {
			if angleP == angleB {
				return -1
			} else {
				return 1 //p is between point A and B
			}
		} else {
			return -1
		}
	} else { //right angle should be lower boundary (could stil have higher value)
		if leftAngleSmaller(angleP, angleA) && leftAngleSmaller(angleB, angleP) {
			if angleP == angleA {
				return -1
			} else {
				return 1 //p is between point A and B (based on longitude)
			}
		} else {
			return -1
		}
	}
}
func calculateAnglesABPwithNewPole(a [2]float64, b [2]float64, p [2]float64, matrix [3][3]float64) int {
	_, angleA := calculateNewPointPositionWithMatrix(a[0], a[1], matrix)
	_, angleB := calculateNewPointPositionWithMatrix(b[0], b[1], matrix)
	_, angleP := calculateNewPointPositionWithMatrix(p[0], p[1], matrix)
	angleA = angleA + 180.0
	angleB = angleB + 180.0
	angleP = angleP + 180.0
	if (angleA-angleB < 180.0 && angleA-angleB >= 0.0) || (angleB-angleA < 180.0 && angleB-angleA >= 0.0) {
		if angleA < angleB {
			if angleP >= angleA && angleP < angleB {
				return 1
			} else {
				return 0
			}
		} else {

			if angleP < angleA && angleP >= angleB {
				return 1
			} else {
				return 0
			}

		}

	} else {
		if angleB-angleA == 0.0 {
			fmt.Println("exactly 180 error", angleA, ",", angleB)

			return -1
		} else { //360 overshoot if 5 < 359 we still need angleP to be greater 359 smaller 365
			if angleA < angleB {
				if angleP <= angleA+360.0 && angleP > angleB {
					return 1
				} else {
					return 0
				}
			} else {

				if angleP > angleA && angleP <= angleB+360.0 {
					return 1
				} else {
					return 0
				}

			}
		}
	}
}
func calculateAnglesABPwithNewPole2ndTest(a [2]float64, b [2]float64, p [2]float64, matrix [3][3]float64) int {
	_, angleB := calculateNewPointPositionWithMatrix(a[0], a[1], matrix)
	_, angleX := calculateNewPointPositionWithMatrix(b[0], b[1], matrix)
	_, angleP := calculateNewPointPositionWithMatrix(p[0], p[1], matrix)
	//fmt.Println(angleX, angleP, "B:", angleB)
	angleX = angleX + 180.0
	angleB = angleB + 180.0
	angleP = angleP + 180.0
	if angleX == angleP {
		return 0
	}
	if angleB <= 180.0 {
		if angleX >= angleB && angleP >= angleB && angleX < angleB+180.0 && angleP <= angleB+180.0 { //X and P lie both in same +180 range

			return 1 //not crossing border
		} else if (angleX <= angleB || angleX >= angleB+180.0) && (angleP <= angleB || angleP >= angleB+180.0) { //both in not +180
			return 1
		} else {
			return -1
		}

	} else {
		if angleX <= angleB && angleP <= angleB && angleX > angleB-180.0 && angleP >= angleB-180.0 { //X and P lie both in same +180 range
			return 1 //not crossing border
		} else if (angleX >= angleB || angleX <= angleB-180.0) && (angleP >= angleB || angleP <= angleB-180.0) { //both in not +180
			return 1
		} else {
			return -1
		}
	}

}
func sign(f float64) int {
	if f > 0 {
		return 1
	} else {
		if f < 0 {
			return -1
		} else {
			return 0
		}
	}
}
func testPolygon(pointtoTest [2]float64, key1 osm.NodeID, way1 []osm.NodeID, nodes_map map[osm.NodeID]Node, waysMidPoint map[osm.NodeID][2]float64) int {
	var array [][2]float64
	for i := 0; i < len(way1)-1; i++ { // Loop stops before the last element
		nodeId := way1[i]
		node := nodes_map[nodeId]
		array = append(array, [2]float64{node.Lat, node.Lon})
	}
	pointxlat := waysMidPoint[key1][0]
	pointxlong := waysMidPoint[key1][1]
	//fmt.Println("midpoint", pointxlat, pointxlong)

	matrix := calculateMatrixWithNewPolePoint(pointxlat, pointxlong)
	wasWrong := false
	counthits := 0
	countsuperhits := 0
	for i := 0; i < len(array)-1; i++ {
		if calculateAnglesABPwithNewPoleEasy(array[i], array[i+1], pointtoTest, matrix) > 0 {
			counthits = counthits + 1
			//fmt.Println("hit point", array[i], array[i+1])
			matrix2 := calculateMatrixWithNewPolePoint(array[i][0], array[i][1]) //point A as north pole
			test2 := calculateAnglesABPwithNewPole2ndTest(array[i+1], [2]float64{pointxlat, pointxlong}, pointtoTest, matrix2)
			if test2 == 0 { //point B and X
				return 0

			} else {
				if test2 > 0 {
					countsuperhits = countsuperhits + 1
				}
			}
		} else {
			if calculateAnglesABPwithNewPoleEasy(array[i], array[i+1], pointtoTest, matrix) == 0 {
				wasWrong = true
				fmt.Println(pointtoTest, pointxlat, pointxlong)
			}
		}
	}
	if wasWrong {
		countsuperhits := 0
		pointxlat := waysMidPoint[key1][0] + 0.0000001
		pointxlong := waysMidPoint[key1][1] + 0.0000001
		matrix := calculateMatrixWithNewPolePoint(pointxlat, pointxlong)
		for i := 0; i < len(array)-1; i++ {
			if calculateAnglesABPwithNewPole(array[i], array[i+1], pointtoTest, matrix) > 0 {
				//counthits = counthits + 1
				matrix2 := calculateMatrixWithNewPolePoint(array[i][0], array[i][1]) //point A as north pole
				test2 := calculateAnglesABPwithNewPole2ndTest(array[i+1], [2]float64{pointxlat, pointxlong}, pointtoTest, matrix2)
				if test2 == 0 { //point B and X
					return 0

				} else {
					if test2 > 0 {
						countsuperhits = countsuperhits + 1
					}
				}
			} else {
				if calculateAnglesABPwithNewPole(array[i], array[i+1], pointtoTest, matrix) == -1 {
					fmt.Println("not fixed")
				}
			}
		}
	}
	//fmt.Println("counthits")
	//fmt.Println(counthits)
	//fmt.Println(countsuperhits)
	if countsuperhits%2 == 0 {
		return -1 // not in polygon
	} else {
		//fmt.Println("super,normhits", countsuperhits, counthits)
		return 1 //in
	}

}
func testPolygonWithoutNorth(pointtoTest [2]float64, key1 osm.NodeID, way1 []osm.NodeID, nodes_map map[osm.NodeID]Node) int {
	var array [][2]float64
	for i := 0; i < len(way1)-1; i++ { // Loop stops before the last element
		nodeId := way1[i]
		node := nodes_map[nodeId]
		array = append(array, [2]float64{node.Lat, node.Lon})
	}

	//fmt.Println("midpoint", pointxlat, pointxlong)

	counthits := 0
	countsuperhits := 0
	for i := 0; i < len(array)-1; i++ {
		if calculateAnglesABPwithOutPole(array[i], array[i+1], pointtoTest) > 0 {
			counthits = counthits + 1
			//fmt.Println("hit point", array[i], array[i+1])
			matrix2 := calculateMatrixWithNewPolePoint(array[i][0], array[i][1]) //point A as north pole
			test2 := calculateAnglesABPwithNewPole2ndTest(array[i+1], [2]float64{-90.0, 0.0}, pointtoTest, matrix2)
			if test2 == 0 { //point B and X
				return 0

			} else {
				if test2 < 0 {
					countsuperhits = countsuperhits + 1
				}
			}
		}
	}

	//fmt.Println("counthits")
	//fmt.Println(counthits)
	//fmt.Println(countsuperhits)
	if countsuperhits%2 == 1 {
		return -1 // not in polygon
	} else {
		//fmt.Println("super,normhits", countsuperhits, counthits)
		return 1 //in
	}

}
func testAllPolygones(pointtoTest [2]float64, ways_map map[osm.NodeID][]osm.NodeID, nodes_map map[osm.NodeID]Node, waysMidPoint map[osm.NodeID][2]float64, wayBoundingBox map[osm.NodeID][4]float64) int {

	inAnyPolygon := -1
	for key1, way1 := range ways_map {
		box := wayBoundingBox[key1]
		if box[0] < pointtoTest[0] && box[1] > pointtoTest[0] && box[2] < pointtoTest[1] && box[3] > pointtoTest[1] {
			test := testPolygon(pointtoTest, key1, way1, nodes_map, waysMidPoint)
			if test >= 0 {
				if test == 0 {
					inAnyPolygon = 0
				} else {
					return 1
				}
			}
		}

	}
	return inAnyPolygon
}
func testAllPolygonesWithBounding(pointtoTest [2]float64, ways_map map[osm.NodeID][]osm.NodeID, nodes_map map[osm.NodeID]Node, wayBoundingBox map[osm.NodeID][4]float64) int {
	lat := pointtoTest[0]
	lon := pointtoTest[1]
	inAnyPolygon := -1
	happened := 0
	for key1, box := range wayBoundingBox {

		if box[0] < lat && box[1] > lat && box[2] < lon && box[3] > lon {
			test := testPolygonWithoutNorth(pointtoTest, key1, ways_map[key1], nodes_map)
			if test >= 0 {
				if test == 0 {
					inAnyPolygon = 0
				} else {
					return 1
				}
			}
		} /*

		 */
	}
	if happened > 1 {
		fmt.Println("happenedtimes:")
		fmt.Println(happened)
	}
	return inAnyPolygon
}
func testAllPolygonesIfMidpointsisIn(ways_map map[osm.NodeID][]osm.NodeID, nodes_map map[osm.NodeID]Node, waysMidPoint map[osm.NodeID][2]float64, wayBoundingBox map[osm.NodeID][4]float64) int {

	inAnyPolygon := -1
	happened := 0
	for key1, way1 := range ways_map {
		pointxlat := waysMidPoint[key1][0]
		pointxlong := waysMidPoint[key1][1]
		if pointxlong >= -1.0 { //creates 180 degree longitude shifted point to test polygon
			pointxlong = pointxlong - 181.0
		} else if pointxlong <= 1.0 {
			pointxlong = pointxlong + 181.0
		}

		test := testPolygon([2]float64{pointxlat, pointxlong}, key1, way1, nodes_map, waysMidPoint)
		if test >= 0 {
			if test == 0 {
				inAnyPolygon = 0
			} else {
				inAnyPolygon = 1
				happened = happened + 1
				//fmt.Println(wayBoundingBox[key1])
			}
		}
	}
	if happened > 1 {
		fmt.Println("happenedtimes:")
		fmt.Println(happened)
	}
	return inAnyPolygon
}
func testWhichPolyCrosses180(ways_map map[osm.NodeID][]osm.NodeID, nodes_map map[osm.NodeID]Node) {
	lenways := 0
	doit := false
	for key, way1 := range ways_map {
		if len(way1) > lenways {
			fmt.Println(len(way1), key)
			lenways = len(way1)
			doit = true
		} else {
			doit = false
		}
		var array [][2]float64
		var otherarray []osm.NodeID
		for i := 0; i < len(way1)-1; i++ { // Loop stops before the last element
			nodeId := way1[i]
			node := nodes_map[nodeId]
			array = append(array, [2]float64{node.Lat, node.Lon})
			otherarray = append(otherarray, nodeId)
		}
		counter := 0
		lowestlong := 190.0
		highestlong := -190.0
		ilow := 0
		ihigh := 0
		for i := 0; i < len(array)-1; i++ {
			//fmt.Println(math.Abs(array[i][1] - array[i+1][1]))
			if array[i][1] > highestlong {
				highestlong = array[i][1]
				ihigh = i
			}
			if array[i][1] < lowestlong {
				lowestlong = array[i][1]
				ilow = i
			}
			if math.Abs(array[i][1]-array[i+1][1]) > 180.0 {
				counter = counter + 1
			}

		}
		if doit {
			fmt.Println(lowestlong, otherarray[ilow], highestlong, otherarray[ihigh])
		}
		if counter > 0 {
			fmt.Println("counter", counter)
		}

	}

}
func findRowAndColumnInGrid(rows int, colums int, lat float64, long float64) (int, int) {
	if lat <= -89.0 || lat >= 89.0 {
		return 0, 0
	}
	newLat := int((lat + 90.0) / (180.0 / float64(rows)))
	if newLat == rows {
		newLat = rows - 1
	}
	newLong := int((long + 180.0) / (360.0 / float64(colums)))
	if newLong == colums {
		newLong = colums - 1
	}
	return newLat, newLong
}
func findLatLongInGrid(rows int, colums int, row int, column int) (float64, float64) {
	cellHeight := 180.0 / float64(rows)
	cellWidth := 360.0 / float64(colums)
	lat := float64(row)*cellHeight - 90.0 + cellHeight/2
	long := float64(column)*cellWidth - 180.0 + cellWidth/2

	return lat, long
}
func generateRandomPoints(numPoints int) [][2]float64 {
	points := make([][2]float64, numPoints)
	for i := 0; i < numPoints; i++ {
		lat := float64((math.Asin(rand.Float64()*2.0-1.0) * 180.0 / math.Pi))
		lon := rand.Float64()*360.0 - 180.0
		points[i] = [2]float64{lat, lon}
	}
	return points
}

func processPoints(points [][2]float64, ways_map map[osm.NodeID][]osm.NodeID, nodes_map map[osm.NodeID]Node, wayBoundingBox map[osm.NodeID][4]float64, results []int, start, end int, wg *sync.WaitGroup) {
	defer wg.Done()
	for i := start; i < end; i++ {
		results[i] = testAllPolygonesWithBounding(points[i], ways_map, nodes_map, wayBoundingBox)

	}
}
func degreesToRadian(deg float64) float64 {
	return deg * (math.Pi / 180)
}
func haversine(lat1, lon1, lat2, lon2 float64) float64 { //chatgpt
	const R = 6371.0 // Earth radius in kilometers
	lat1Rad := degreesToRadian(lat1)
	lon1Rad := degreesToRadian(lon1)
	lat2Rad := degreesToRadian(lat2)
	lon2Rad := degreesToRadian(lon2)

	dlat := lat2Rad - lat1Rad
	dlon := lon2Rad - lon1Rad

	a := math.Sin(dlat/2)*math.Sin(dlat/2) + math.Cos(lat1Rad)*math.Cos(lat2Rad)*math.Sin(dlon/2)*math.Sin(dlon/2)
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))

	// Distance in kilometers
	distance := R * c
	//fmt.Println(distance)
	return math.Abs(distance)
}
func findNearest(comparePoint [3]float64, pointIndexes [][3]float64, fall int) (bool, [3]float64, float64) {

	closestDist := 3000.0
	u := comparePoint
	smallestIndex := comparePoint
	searchpointLat := comparePoint[0]
	searchPointLon := comparePoint[1]
	for _, point := range pointIndexes {
		switch fall {
		case 1:
			if !(point[0] > u[0] && point[1] < u[1]) {
				break
			}
		case 2:
			if !(point[0] > u[0] && point[1] > u[1]) {
				break
			}
		case 3:
			if !(point[0] < u[0] && point[1] < u[1]) {
				break
			}
		case 4:
			if !(point[0] < u[0] && point[1] > u[1]) {
				break
			}
		}

		dist := haversine(searchpointLat, searchPointLon, point[0], point[1])
		if dist < closestDist {
			closestDist = dist
			smallestIndex = point
		}

	}
	if smallestIndex != comparePoint {

		return true, smallestIndex, closestDist
	} else {
		return false, smallestIndex, -1.0
	}
	return false, smallestIndex, -1.0
}
func findTopLeft(u int, graphpoints [pointcount][2]float64, graphEdges *[pointcount][4]int, distancesEdges *[pointcount][4]int, pointIndexes []int) bool {
	comparePoint := graphpoints[u]

	closestDist := 3000.0
	smallestIndex := -1
	searchpointLat := comparePoint[0]
	searchPointLon := comparePoint[1]
	for x := range pointIndexes {
		point := graphpoints[pointIndexes[x]]

		if (*graphEdges)[pointIndexes[x]][3] == -1 {
			if searchpointLat > point[0] && searchPointLon < point[1] {
				dist := haversine(searchpointLat, searchPointLon, point[0], point[1])
				if dist < closestDist {
					closestDist = dist
					smallestIndex = x
				}
			}

		}
	}
	if smallestIndex > -1 {

		(*graphEdges)[u][0] = pointIndexes[smallestIndex]
		(*graphEdges)[pointIndexes[smallestIndex]][3] = u
		(*distancesEdges)[u][0] = int(closestDist)
		(*distancesEdges)[pointIndexes[smallestIndex]][3] = int(closestDist)
		return true
	} else {
		return false
		(*graphEdges)[u][0] = -2 //no neighbour found
	}
	return false
}
func findTopRight(u int, graphpoints [pointcount][2]float64, graphEdges *[pointcount][4]int, distancesEdges *[pointcount][4]int, pointIndexes []int) bool {
	comparePoint := graphpoints[u]

	closestDist := 3000.0
	smallestIndex := -1
	searchpointLat := comparePoint[0]
	searchPointLon := comparePoint[1]
	for x := range pointIndexes {
		point := graphpoints[pointIndexes[x]]

		if (*graphEdges)[pointIndexes[x]][2] == -1 {
			if searchpointLat > point[0] && searchPointLon > point[1] {
				dist := haversine(searchpointLat, searchPointLon, point[0], point[1])
				if dist < closestDist {
					closestDist = dist
					smallestIndex = x
				}
			}

		}
	}
	if smallestIndex > -1 {
		(*graphEdges)[u][1] = pointIndexes[smallestIndex]
		(*graphEdges)[pointIndexes[smallestIndex]][2] = u
		(*distancesEdges)[u][1] = int(closestDist)
		(*distancesEdges)[pointIndexes[smallestIndex]][2] = int(closestDist)
		return true

	} else {
		return false
		(*graphEdges)[u][1] = -2 //no neighbour found
	}
	return false

}
func findBottomLeft(u int, graphpoints [pointcount][2]float64, graphEdges *[pointcount][4]int, distancesEdges *[pointcount][4]int, pointIndexes []int) bool {
	comparePoint := graphpoints[u]

	closestDist := 3000.0
	smallestIndex := -1
	searchpointLat := comparePoint[0]
	searchPointLon := comparePoint[1]
	for x := range pointIndexes {
		point := graphpoints[pointIndexes[x]]

		if (*graphEdges)[pointIndexes[x]][1] == -1 {
			if searchpointLat < point[0] && searchPointLon < point[1] {
				dist := haversine(searchpointLat, searchPointLon, point[0], point[1])
				if dist < closestDist {
					closestDist = dist
					smallestIndex = x
				}
			}

		}
	}
	if smallestIndex > -1 {
		(*graphEdges)[u][2] = pointIndexes[smallestIndex]
		(*graphEdges)[pointIndexes[smallestIndex]][1] = u
		(*distancesEdges)[u][2] = int(closestDist)
		(*distancesEdges)[pointIndexes[smallestIndex]][1] = int(closestDist)
		return true

	} else {
		return false
		(*graphEdges)[u][2] = -2 //no neighbour found
	}
	return false

}
func findBottomRight(u int, graphpoints [pointcount][2]float64, graphEdges *[pointcount][4]int, distancesEdges *[pointcount][4]int, pointIndexes []int) bool {
	comparePoint := graphpoints[u]

	closestDist := 3000.0
	smallestIndex := -1
	searchpointLat := comparePoint[0]
	searchPointLon := comparePoint[1]
	for x := range pointIndexes {
		point := graphpoints[pointIndexes[x]]

		if (*graphEdges)[pointIndexes[x]][0] == -1 {
			if searchpointLat < point[0] && searchPointLon > point[1] {
				dist := haversine(searchpointLat, searchPointLon, point[0], point[1])
				if dist < closestDist {
					closestDist = dist
					smallestIndex = x
				}
			}

		}
	}
	if smallestIndex > -1 {

		(*graphEdges)[u][3] = pointIndexes[smallestIndex]
		(*graphEdges)[pointIndexes[smallestIndex]][0] = u
		(*distancesEdges)[u][3] = int(closestDist)
		(*distancesEdges)[pointIndexes[smallestIndex]][0] = int(closestDist)
		return true
	} else {
		return false
		(*graphEdges)[u][3] = -2 //no neighbour found
	}
	return false

}
func Dijkstra(nodes [][2]float64, edges [][4]int, edgeweights [][4]int, src, dst int) (int, []int) {
	dist := make(map[int]int)
	prev := make(map[int]int)
	found := false
	for node := range nodes {
		dist[node] = math.MaxInt64
	}
	dist[src] = 0
	//fmt.Println("dist", dist)
	pq := &PriorityQueue{}
	heap.Init(pq)
	heap.Push(pq, &QueueItem{node: src, priority: 0})

	for pq.Len() > 0 {
		current := heap.Pop(pq).(*QueueItem)
		currentNode := current.node

		if currentNode == dst {
			fmt.Println("reached from", currentNode)
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
				heap.Push(pq, &QueueItem{node: neighbor, priority: newDist})
			}
		}
	}
	fmt.Println("finisheddykstra , calculating path")

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
func ReadCoordinates(filename string) ([][]float64, error) {
	// Define the structure to hold the data
	var coordinates [][]float64

	// Open the JSON file
	jsonFile, err := os.Open(filename)
	if err != nil {
		return nil, err
	}
	defer jsonFile.Close()

	// Read the JSON file
	byteValue, err := ioutil.ReadAll(jsonFile)
	if err != nil {
		return nil, err
	}

	// Unmarshal the JSON data into the coordinates variable
	err = json.Unmarshal(byteValue, &coordinates)
	if err != nil {
		return nil, err
	}

	return coordinates, nil
}
func getAllNeighbourCellss(grid [][][][3]float64, x int, y int, latitude float64, radius int) [][3]float64 {
	var neighbors [][3]float64

	/*startlat, startlon := findLatLongInGrid(len(grid), len(grid[0]), x, y)
	k := 0
	l := 0
	if x+1 < len(grid) {
		k = x + 1
	} else {
		k = x - 1
	}
	if y+1 < len(grid[0]) {
		l = y + 1
	} else {
		l = y - 1
	}*/
	//cellendlat, cellendlong := findLatLongInGrid(len(grid), len(grid[0]), k, l)
	//cellwidth := haversine(startlat, startlon, cellendlat, cellendlong)
	//fmt.Println("cellwidth", cellwidth)
	if latitude >= 89.0 {
		neighbors = append(neighbors, grid[0][0]...)

	} else if latitude <= -89.0 {
		neighbors = append(neighbors, grid[0][0]...)

	}
	counter := 0
	for dx := -radius; dx <= radius; dx++ {
		for dy := -radius; dy <= radius; dy++ {
			//if dx == radius || dy == radius || dx == -radius || dy == -radius {

			aplus := dx + x
			bplus := dy + y
			if aplus >= len(grid) {
				break
			}
			if aplus <= -1 {
				break
			}
			if bplus >= len(grid[x]) {
				break
			}
			if bplus <= -1 {
				break
			}
			//add points to return
			counter++
			neighbors = append(neighbors, grid[aplus][bplus]...)

			//}
		}
	}
	//fmt.Println("nachbarlänge", counter)
	return neighbors
}
func sortAndRemoveDuplicates(edges [][2]int, distances []float64, maxNode int) ([][2]int, []float64, map[int]int) {
	if len(edges) != len(distances) {
		panic("edges and distances must have the same length")
	}

	// Combine edges and distances into a slice of EdgeWithDistance
	combined := make([]EdgeWithDistance, len(edges))
	for i := range edges {
		combined[i] = EdgeWithDistance{Edge: edges[i], Distance: distances[i]}
	}

	// Sort combined slice by the Edge
	sort.Slice(combined, func(i, j int) bool {
		if combined[i].Edge[0] == combined[j].Edge[0] {
			if combined[i].Edge[1] == combined[j].Edge[1] {
				return combined[i].Distance < combined[j].Distance
			}
			return combined[i].Edge[1] < combined[j].Edge[1]
		}
		return combined[i].Edge[0] < combined[j].Edge[0]
	})

	// Remove duplicates and maintain unique edges and distances
	uniqueEdges := make([][2]int, 0, len(edges))
	uniqueDistances := make([]float64, 0, len(distances))
	for i, item := range combined {
		if i == 0 || item.Edge != combined[i-1].Edge {
			uniqueEdges = append(uniqueEdges, item.Edge)
			uniqueDistances = append(uniqueDistances, item.Distance)
		}
	}

	// Create start indices for each node, including nodes without edges
	startIndices := make(map[int]int)
	edgeIndex := 0
	for node := 0; node <= maxNode; node++ {
		if edgeIndex < len(uniqueEdges) && uniqueEdges[edgeIndex][0] == node {
			startIndices[node] = edgeIndex
			for edgeIndex < len(uniqueEdges) && uniqueEdges[edgeIndex][0] == node {
				edgeIndex++
			}
		} else {
			startIndices[node] = edgeIndex
		}
	}

	// Add dummy entry for the node maxNode + 1
	startIndices[maxNode+1] = edgeIndex

	return uniqueEdges, uniqueDistances, startIndices
}

func nearestHelper(k [3]float64, points [][3]float64, graphNodes [pointcount][2]float64, edges *[][2]int, distances *[]float64, mu *sync.Mutex, fall int) {
	found, index, dist := findNearest(k, points, fall)
	if found {
		mu.Lock()
		*edges = append(*edges, [2]int{int(k[2]), int(index[2])})
		*distances = append(*distances, dist)
		mu.Unlock()
	}
}
