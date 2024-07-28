package generator

import (
	"context"
	"math"
	"math/rand"
	"os"
	"runtime"
	"strconv"
	"time"

	"github.com/adhocore/chin"
	"github.com/divan/num2words"
	"github.com/gookit/slog"
	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"

	"final/types"
)

// const PBF_FILE_PATH = "/home/sahask/osm_data/planet-coastlines.osm.pbf"
const PBF_FILE_PATH = "/home/fsociety/Documents/Uni-Stuttgart/SS2024/OpenMaps_FachPrak/osm_data/planet-coastlines.osm.pbf"

const pointcount = 1000
const longBincount = 3600

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

	nodes_map := make(map[osm.NodeID]types.Node)
	edges_map := make([][]types.Edge, longBincount)

	ways_map := make(map[osm.NodeID][]osm.NodeID)
	nodeRelations := make(map[osm.NodeID]types.WayComplete)

	rows := 180
	colums := 360
	randomPointCount := pointcount
	grid := make([][][]int, rows)
	graphNodes := [pointcount][2]float64{}
	graphEdges := [pointcount][4]int{}

	for i := range graphEdges {
		for j := range graphEdges[i] {
			graphEdges[i][j] = -1
		}
	}

	distancesEdges := [pointcount][4]int{}
	for i := range grid {
		grid[i] = make([][]int, colums)
		for j := range grid[i] {
			grid[i][j] = []int{}
		}
	}

	scanner.SkipRelations = true

	slog.Info("Starting scan...")

	for scanner.Scan() {
		o := scanner.Object()

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

	slog.Info("Start cleaning nodes")
	slog.Info("Number of nodes : " + strconv.Itoa(len(nodes_map)))

	for key, node := range nodes_map {
		if !node.IsCoastline {
			delete(nodes_map, key)
		}
	}

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

	for k := range ways_map { //delete duplicated ways
		if ways_map[k][0] != ways_map[k][len(ways_map[k])-1] {
			delete(ways_map, k)
		}
	}

	slog.Info("Duplicate ways deleted")

	for k := range ways_map {
		for l := range ways_map[k] {
			if l+1 < len(ways_map[k]) {
				tempEdge := types.Edge{}
				tempEdge.Startpoint = [2]float64{nodes_map[ways_map[k][l]].Lat, nodes_map[ways_map[k][l]].Lon}
				tempEdge.Endpoint = [2]float64{nodes_map[ways_map[k][l+1]].Lat, nodes_map[ways_map[k][l+1]].Lon}
				tempEdge.WayId = k
				AddEdge(&edges_map, tempEdge)
			}

		}
	}
	// mostedges := -1
	// leastedges := 10000000000
	// for k := range edges_map {
	// 	if len(edges_map[k]) > mostedges {
	// 		mostedges = len(edges_map[k])
	// 	}
	// 	if len(edges_map[k]) < leastedges {
	// 		leastedges = len(edges_map[k])
	// 	}
	// }
	// fmt.Println("edges count", mostedges, leastedges)
	// interedges := FindEdgesForPointpoint(&edges_map, [2]float64{-70.0, 80.0})
	// returedges := [][4]float64{}
	// for k := range interedges {
	// 	startpoint := interedges[k].Startpoint
	// 	endpoint := interedges[k].Endpoint
	// 	returedges = append(returedges, [4]float64{startpoint[0], startpoint[1], endpoint[0], endpoint[1]})
	// }
	// fmt.Println(returedges)

	//random points
	s := chin.New()
	go s.Start()
	slog.Info(num2words.Convert(pointcount) + " : " + time.Since(start).String())
	for s := 0; s < randomPointCount; s++ {
		var randLong float64
		var randLat float64
		for {
			randLong = rand.Float64()*360.0 - 180.0
			randLat = float64((math.Asin(rand.Float64()*2.0-1.0) * 180.0 / math.Pi))

			edges := FindEdgesForPointpoint(&edges_map, [2]float64{randLat, randLong})
			if len(edges)%2 == 0 {
				break
			}
			if randLong == 0.0 || randLat == 0.0 {
				break
			}
		}
		if randLat >= 89.0 {
			grid[0][0] = append(grid[0][0], s)
		} else if randLat <= -89.0 {
			grid[0][0] = append(grid[0][0], s)
		} else {

			a, b := findRowAndColumnInGrid(rows, colums, float64(randLat), randLong)

			grid[a][b] = append(grid[a][b], s)
		}
		graphNodes[s] = [2]float64{randLat, randLong}
	}

	slog.Info(num2words.Convert(pointcount) + " : " + time.Since(start).String())

	for k := range graphNodes {
		if k%10000 == 0 {
			slog.Info(strconv.Itoa(k) + " : " + time.Since(start).String())
		}

		u := graphNodes[k]
		edge := graphEdges[k]
		a, b := findRowAndColumnInGrid(rows, colums, u[0], u[1])
		points := getAllNeighbourCellss(grid, a, b, u[0], 2)
		if edge[0] == -1 {

			if !findTopLeft(k, graphNodes, &graphEdges, &distancesEdges, grid[a][b]) {
				findTopLeft(k, graphNodes, &graphEdges, &distancesEdges, points)
			}
		}
		if edge[1] == -1 {
			if !findTopRight(k, graphNodes, &graphEdges, &distancesEdges, grid[a][b]) {
				findTopRight(k, graphNodes, &graphEdges, &distancesEdges, points)

			}

		}
		if edge[2] == -1 {
			if !findBottomLeft(k, graphNodes, &graphEdges, &distancesEdges, grid[a][b]) {
				findBottomLeft(k, graphNodes, &graphEdges, &distancesEdges, points)

			}

		}
		if edge[3] == -1 {
			if !findBottomRight(k, graphNodes, &graphEdges, &distancesEdges, grid[a][b]) {
				findBottomRight(k, graphNodes, &graphEdges, &distancesEdges, points)

			}

		}
	}
	s.Stop()
	slog.Info("Finished neighbours")

	WriteToJSONFile("graphNodes.json", graphNodes)
	WriteToJSONFile("graphEdges.json", graphEdges)
	WriteToJSONFile("distancesEdges.json", distancesEdges)
	WriteToJSONFile("grid.json", grid)

	slog.Info("Finished scan...")
	slog.Info("Total time: " + time.Since(start).String())
}
