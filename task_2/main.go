package main

import (
	"context"
	"fmt"
	"log/slog"
	"os"
	"runtime"
	"time"

	// "github.com/gosuri/uiprogress"
	// "github.com/gookit/slog"
	"github.com/adhocore/chin"

	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"
)

type Node struct {
	ID          int64
	Lat         float32
	Lon         float32
	isCoastline bool
}
type WayComplete struct {
	WayMapKey  osm.NodeID
	isFinished bool
}

const PBF_FILE_PATH = "../osm_data/planet-coastlines.osm.pbf"

const GJSON_FILE_PATH = "../osm_data/output.geojson"

// main is the entry point of the program.
// It opens a PBF file, scans its contents, extracts coastline data,
// and creates a GeoJSON file containing the coastline features.
// The function also logs various information such as file size, number of nodes read,
// and the total time taken for the operation.
func main() {
	s := chin.New()
	go s.Start()

	f, err := os.Open(PBF_FILE_PATH)
	if err != nil {
		slog.Error("Error opening PBF File!")
		panic(err)
	}
	defer f.Close()

	if err != nil {
		slog.Error("Error retrieving PBF File stats!")
		panic(err)
	}
	fmt.Println("good")
	scanner := osmpbf.New(context.Background(), f, runtime.NumCPU())
	defer scanner.Close()

	// featureCollection := geojson.NewFeatureCollection()

	nodes_map := make(map[osm.NodeID]Node)
	ways_map := make(map[osm.NodeID][]osm.NodeID)
	// closed_loops := make(map[osm.NodeID][]osm.NodeID)
	nodeRelations := make(map[osm.NodeID]WayComplete)

	scanner.SkipRelations = true

	slog.Info("Starting scan...")

	for scanner.Scan() {
		o := scanner.Object()
		// bar.Set(int(scanner.FullyScannedBytes()))
		// fmt.Println(scanner.FullyScannedBytes() / (1024 * 1024))
		switch v := o.(type) {
		case *osm.Node:
			nodes_map[v.ID] = Node{ID: int64(v.ID), Lat: float32(v.Lat), Lon: float32(v.Lon), isCoastline: false}
		case *osm.Way:
			if v.Tags.Find("natural") == "coastline" {
				ways_map[v.Nodes.NodeIDs()[0]] = v.Nodes.NodeIDs()
				for _, node := range v.Nodes.NodeIDs() {
					temp_node := nodes_map[node]
					temp_node.isCoastline = true
					nodes_map[node] = temp_node
				}
			}
		}
	}
	var start_Julian = time.Now()
	for key, way := range ways_map {
		nodeRelations[way[0]] = WayComplete{WayMapKey: key, isFinished: false}
	}

	for i := range nodeRelations {
		if nodeRelations[i].isFinished == false {
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
	for k := range ways_map {
		if ways_map[k][0] != ways_map[k][len(ways_map[k])-1] {
			delete(ways_map, k)
		}
	}
	fmt.Println("Coastlines count:	")
	fmt.Println(len(ways_map))

	scanErr := scanner.Err()
	if scanErr != nil {
		panic(scanErr)
	}
	slog.Info("Julian took time: " + time.Since(start_Julian).String())
	// slog.Info("Souvik starts now")
	// scanner = nil
	// f = nil
	// runtime.GC()
	// var start_Souvik = time.Now()
	// // slog.Info("Start cleaning nodes")
	// // slog.Info("Nodes : " + strconv.Itoa(len(nodes_map)))

	// for key, node := range nodes_map {
	// 	if !node.isCoastline {
	// 		// fmt.Println("Deleting node: ", key)
	// 		delete(nodes_map, key)
	// 	}
	// }
	// runtime.GC()

	// slog.Info("Nodes cleaned...")
	// slog.Info("Nodes : " + strconv.Itoa(len(nodes_map)))

	// slog.Info("PBF Scan complete!")

	// slog.Info("Start merging ways...")

	// uiprogress.Start()
	// bar := uiprogress.AddBar(len(ways_map)) // Add a new bar
	// bar.AppendCompleted()
	// bar.PrependElapsed()

	// for outer_key, outer_way := range ways_map {
	// 	inner_way := outer_way
	// 	for {
	// 		// fmt.Println("Outer Key: ", outer_key)

	// 		// Check if the way is already closed
	// 		if inner_way[0] == inner_way[len(inner_way)-1] {
	// 			closed_loops[outer_key] = inner_way
	// 			// fmt.Println(outer_key)
	// 			delete(ways_map, outer_key)
	// 			runtime.GC()
	// 			// bar.Incr()
	// 			// slog.Info("Closed loop found: " + strconv.Itoa(int(outer_key)))
	// 			break
	// 			// fmt.Println("Closed Ways", outer_way[0], outer_way[len(outer_way)-1])

	// 			// fmt.Println("Found closed loop: ", outer_key)
	// 		}
	// 		// fmt.Println("Looking for closing node for way: ", outer_key)

	// 		// Find a way that continues the loop
	// 		new_way, exists := ways_map[inner_way[len(inner_way)-1]]
	// 		// fmt.Println(outer_way[len(outer_way)-1])
	// 		if exists {
	// 			// slog.Info("Found a closing way!")
	// 			// slog.Debug("Merging way: " + strconv.Itoa(int(outer_way[0])) + " with way: " + strconv.Itoa(int(outer_way[len(outer_way)-1])))
	// 			// fmt.Println("Merging way: ", outer_key, " with way: ", outer_way[len(outer_way)-1])
	// 			// fmt.Println("Old way: ", outer_way)

	// 			// fmt.Println(nodes_map[outer_way[len(outer_way)-1]].Lat, nodes_map[outer_way[len(outer_way)-1]].Lon)
	// 			inner_way = append(inner_way, new_way[1:]...)
	// 			// slog.Info("Appended way: " + strconv.Itoa(int(outer_way[len(outer_way)-1])))
	// 			// fmt.Println("New way: ", outer_way)
	// 			// _, ok := ways_map[inner_way[len(inner_way)-1]]
	// 			// if !ok {
	// 			// delete(ways_map, inner_way[len(inner_way)-1])
	// 			// 	slog.Info("Deleted way")
	// 			// }
	// 			// delete(ways_map, outer_way[len(outer_way)-1])

	// 		} else {
	// 			// fmt.Println()
	// 			// slog.Error("No closing way: " + strconv.Itoa(int(inner_way[len(inner_way)-1])))
	// 			// slog.Info("Deleting way: " + strconv.Itoa(int(outer_key)))
	// 			// delete(ways_map, outer_key)
	// 			break
	// 		}

	// 	}
	// }
	//s.Stop()

	// slog.Info("Ways merged...")
	// slog.Info("Clean up...")

	// ways_map = nil
	// runtime.GC()

	// // slog.Info("Merging complete!")

	// slog.Info("Souvik took: " + time.Since(start_Souvik).String())

	// slog.Info("No of closed ways: " + strconv.Itoa(len(closed_loops)))
	// // slog.Info("Average nodes per closed way: " + strconv.FormatFloat(float64(len(nodes_map))/float64(len(closed_loops)), 'f', 6, 64))
	// // b, _ := json.MarshalIndent(closed_loops, "", " ")

	// // // write output of closed loops to file
	// // os.WriteFile("closed_loops.json", b, 0644)

	// // featureCollection := geojson.NewFeatureCollection()

	// // for _, nodes := range closed_loops {
	// // 	geometry := geojson.NewLineStringGeometry(make([][]float64, len(nodes)))
	// // 	c := 0
	// // 	for _, node := range nodes {
	// // 		geometry.LineString[c] = append(geometry.LineString[c], []float64{float64(nodes_map[node].Lon), float64(nodes_map[node].Lat)}...)
	// // 		c += 1
	// // 	}
	// // 	feature := geojson.NewFeature(geometry)
	// // 	feature.SetProperty("id", nodes[0])
	// // 	featureCollection.AddFeature(feature)
	// // }

	// // geojsonData, err := featureCollection.MarshalJSON()
	// // if err != nil {
	// // 	log.Fatalf("Error marshaling GeoJSON: %v", err)
	// // }

	// // err = os.WriteFile(GJSON_FILE_PATH, geojsonData, 0644)
	// // if err != nil {
	// // 	log.Fatalf("Error writing GeoJSON file: %v", err)
	// // }

	// // fmt.Println("GeoJSON file created successfully.")

	// // // find the longest closed loop
	// // longest := 0
	// // index := 0
	// // for key, nodes := range closed_loops {
	// // 	if len(nodes) > longest {
	// // 		longest = len(nodes)
	// // 		index = int(key)
	// // 	}
	// // }
	// // fmt.Println("Longest closed loop: ", longest, index)

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
				if nodeRelations[index].isFinished != true {
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
