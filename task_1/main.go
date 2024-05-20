package main

import (
	"context"
	"fmt"
	"log"
	"os"

	"github.com/paulmach/osm/osmpbf"
)

const PBF_FILE_PATH = "../osm_data/antarctica-latest.osm.pbf"
const GJSON_FILE_PATH = "../osm_data/antarctica-latest.geojson"

func main() {
	f, err := os.Open(PBF_FILE_PATH)
	if err != nil {
		log.Fatal(err)
	}
	defer f.Close()
	scanner := osmpbf.New(context.Background(), f, 3)
	defer scanner.Close()

	var nc, wc, rc uint64
	for scanner.Scan() {
		o := scanner.Object()
		// do something
		if o.ObjectID().Type() == "node" {
			nc++
		} else if o.ObjectID().Type() == "way" {
			wc++
		} else if o.ObjectID().Type() == "relation" {
			rc++
		}
	}

	scanErr := scanner.Err()
	if scanErr != nil {
		panic(scanErr)
	}

	fmt.Printf("Nodes: %d, Ways: %d, Relations: %d\n", nc, wc, rc)

}
