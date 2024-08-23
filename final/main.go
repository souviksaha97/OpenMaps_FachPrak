package main

// TODO: use gob format for saving and reading files
// TODO: use goroutines for reading files
// TODO: remove uneccessary functions

import (
	"bufio"
	"final/generator"
	"final/router"
	"final/server" // Add this line to import the tester package
	"final/tester"
	"fmt"
	"os"
	"runtime"
	"strconv"
	"strings"

	"time"

	"github.com/gookit/slog"
)

func main() {
	slog.Info("Starting the program")
	runtime.GOMAXPROCS(runtime.NumCPU())
	if !init_main() {
		generateTime := time.Now()
		reader := bufio.NewReader(os.Stdin)
		fmt.Print("Do you want to run the generator? (y/N): ")
		input, _ := reader.ReadString('\n')
		input = strings.TrimSpace(input) // Trim any leading/trailing whitespace

		if strings.EqualFold(input, "y") {
			slog.Info("Running the generator")
			slog.Info("Good night! 💤")
			generator.Generator()
			// router.LandmarksDistanceMaximiser(0)
		} else {
			fmt.Println("Exiting program.")
			os.Exit(0)
		}
		slog.Info("Generator finished")
		slog.Info("Time taken: ", time.Since(generateTime))
		slog.Info("Good morning! ☀️")
		os.Exit(0)
	} else {
		slog.Debug("All files exist")
	}

	if len(os.Args) < 2 {
		slog.Info("No arguments provided")
		return
	}

	switch os.Args[1] {
	case "graph":
		slog.Info("Running the generator")
		generator.Generator()

	case "server":
		slog.Info("Running the server")
		server.Server()

	case "multi":
		slog.Info("Running the multi")
		if len(os.Args) < 3 {
			slog.Info("Iterations argument missing")
			return
		}
		iterations, err := strconv.Atoi(os.Args[2])
		if err != nil {
			slog.Info("Invalid iterations argument")
			return
		}
		router.MultiRouter(iterations)

	case "single":
		slog.Info("Running the single")
		if len(os.Args) < 4 {
			slog.Info("Arguments missing")
		}

		run := os.Args[2]

		iterations, err2 := strconv.Atoi(os.Args[3])
		if err2 != nil {
			slog.Info("Invalid iterations argument")
			return
		}

		router.SingleRouter(run, iterations)

	case "alt-pre":
		slog.Info("Running the alt-pre")
		if len(os.Args) < 3 {
			slog.Info("Iterations argument missing")
			return
		}
		iterations, err := strconv.Atoi(os.Args[2])
		if err != nil {
			slog.Info("Invalid iterations argument")
			return
		}
		router.LandmarksDistanceMaximiser(iterations)

	case "tester":
		slog.Info("Running the tester")
		if len(os.Args) < 4 {
			slog.Info("Iterations argument missing")
			return
		}
		iterations, err := strconv.Atoi(os.Args[2])
		points, err := strconv.Atoi(os.Args[3])
		if err != nil {
			slog.Info("Invalid iterations argument")
			return
		}
		tester.Tester(iterations, points)

	case "debug":
		slog.Info("Running the debug")
		router.Debugging()

	default:
		slog.Info("Invalid argument")
	}

}

func init_main() bool {
	slog.Configure(func(logger *slog.SugaredLogger) {
		logger.Level = slog.InfoLevel
		f := logger.Formatter.(*slog.TextFormatter)
		f.EnableColor = true
	})
	flag := true
	slog.Info("Initializing the program")
	basePath := "objects/"
	filesList := []string{"graphEdges.json", "graphNodes.json", "grid.json",
		"distancesEdges.json", "sortedEdges.json", "sortedDistances.json", "startIndices.json",
		"landmarks.json", "landmarkNodes.json", "landmarkDistances.json"}
	for _, file := range filesList {
		if _, err := os.Stat(basePath + file); os.IsNotExist(err) {
			slog.Info("File: " + file + " does not exist")
			touch(basePath + file)
			flag = false
		} else {
			slog.Debug("File: " + file + " exists")
		}
	}
	return flag
}

func touch(filename string) error {
	// Open the file in append mode, create if it doesn't exist, for writing
	f, err := os.OpenFile(filename, os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
	if err != nil {
		return err
	}
	defer f.Close()

	// Update the file timestamps to the current time
	currentTime := time.Now().Local()
	if err := os.Chtimes(filename, currentTime, currentTime); err != nil {
		return err
	}

	return nil
}
