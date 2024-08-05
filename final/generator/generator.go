package generator

import (
	"context"
	"fmt"
	"math"
	"math/rand"
	"os"
	"runtime"
	"sync"
	"sync/atomic"
	"time"

	"github.com/gookit/slog"
	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"

	"final/types"
)

const PBF_FILE_PATH = "/home/sahask/osm_data/planet-coastlines.osm.pbf"

const pointcount = 4000000
const longBincount = 200000

func Generator() {
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

	nodes_map := make(map[osm.NodeID]types.Node)
	edges_map := make([][]types.Edge, longBincount)

	ways_map := make(map[osm.NodeID][]osm.NodeID)
	nodeRelations := make(map[osm.NodeID]types.WayComplete)
	//wayMapIdForNodeId := make(map[osm.NodeID]osm.NodeID)
	//wayBoundingBox := make(map[osm.NodeID][4]float64)
	//waysMidPoint := make(map[osm.NodeID][2]float64)
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
			nodes_map[v.ID] = types.Node{ID: int64(v.ID), Lat: float64(v.Lat), Lon: float64(v.Lon), IsCoastline: false}
		case *osm.Way:
			if v.Tags.Find("natural") == "coastline" {
				ways_map[v.Nodes.NodeIDs()[0]] = v.Nodes.NodeIDs()
				for _, node := range v.Nodes.NodeIDs() {
					temp_node := nodes_map[node]
					temp_node.IsCoastline = true
					nodes_map[node] = temp_node
				}
			}
		}
	}
	runtime.GC()

	slog.Info("Nodes cleaned...")
	for key, way := range ways_map {
		nodeRelations[way[0]] = types.WayComplete{WayMapKey: key, IsFinished: false}
	}

	for i := range nodeRelations {
		if !nodeRelations[i].IsFinished {
			copy := nodeRelations[i]
			copy.IsFinished = true
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
				AddEdge(&edges_map, types.Edge{
					Startpoint: [2]float64{nodes_map[ways_map[k][l]].Lat, nodes_map[ways_map[k][l]].Lon},
					Endpoint:   [2]float64{nodes_map[ways_map[k][l+1]].Lat, nodes_map[ways_map[k][l+1]].Lon},
					WayId:      k})
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
	slog.Info("starttime 4 mio: " + time.Since(start).String())
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
	slog.Info("endtime 4 mio: " + time.Since(start).String())

	start = time.Now()
	var mu sync.Mutex
	var counterNeighbour int64
	var edgeandDistindex int64

	// Preallocate edges and distances arrays
	edges := make([][2]int, len(graphNodes)*8)
	distances := make([]int, len(graphNodes)*8)

	const numWorkers = 1000           // Number of worker goroutines
	jobs := make(chan types.Job, 100) // Buffered channel for jobs

	var wg sync.WaitGroup

	// Start worker goroutines
	for w := 0; w < numWorkers; w++ {
		wg.Add(1)
		go Worker(w, jobs, &edges, &distances, &edgeandDistindex, &mu, &wg)
	}

	for s, k := range grid {
		for t, points := range k {
			if len(points) == 0 {
				continue
			}
			p, wraparound := getAllNeighbourCellss(grid, s, t, points[0][0], 1)

			atomic.AddInt64(&counterNeighbour, int64(len(points))) // Increment the counter for each k
			if atomic.LoadInt64(&counterNeighbour)%10000 == 0 {
				fmt.Println("Processed 10k points", atomic.LoadInt64(&counterNeighbour))
			}

			jobs <- types.Job{Points: points, Neighbours: p, WrapAround: wraparound} // Send job to workers
		}
	}

	close(jobs) // Close the jobs channel to signal workers to stop
	wg.Wait()   // Wait for all workers to finish

	// Trim the slices to the actual size
	edges = edges[:edgeandDistindex]
	//fmt.Println(edges)
	distances = distances[:edgeandDistindex]

	sortedEdges1, sortedDistances1, startIndices1 := SortAndRemoveDuplicates(edges, distances, pointcount)
	fmt.Println(sortedEdges1[0], sortedDistances1[0], startIndices1[0])
	fmt.Println("Total edges:", edgeandDistindex)
	fmt.Println(len(sortedEdges1))

	fmt.Println("Neighbours finished in: " + time.Since(start).String())

	//alt
	fmt.Println("finished neighbours")

	// write graphNodes graphEdges distancesEdges grid to different json files
	//

	WriteToJSONFile("graphNodes.json", graphNodes)
	WriteToJSONFile("graphEdges.json", graphEdges)
	WriteToJSONFile("distancesEdges.json", distancesEdges)
	WriteToJSONFile("grid.json", grid)
	WriteToJSONFile("sortedEdges.json", sortedEdges1)
	WriteToJSONFile("sortedDistances.json", sortedDistances1)
	WriteToJSONFile("startIndices.json", startIndices1)

	slog.Info("Finished scan...")
	slog.Info("Total time: " + time.Since(start).String())

}
