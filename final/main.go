package main

import (
	"bufio"
	"final/generator"
	"final/router"
	"final/server" // Add this line to import the tester package
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
	const landmarksCount = 200
	if !init_main() {
		generateTime := time.Now()
		reader := bufio.NewReader(os.Stdin)
		fmt.Print("Do you want to run the generator? (y/N): ")
		input, _ := reader.ReadString('\n')
		input = strings.TrimSpace(input) // Trim any leading/trailing whitespace

		if strings.EqualFold(input, "y") {
			slog.Info("Running the generator")
			generator.Generator()
			router.LandmarksDistanceMaximiser(landmarksCount)
		} else {
			fmt.Println("Exiting program.")
			os.Exit(0)
		}
		slog.Info("Generator finished")
		slog.Info("Time taken: ", time.Since(generateTime))
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
		router.LandmarksDistanceMaximiser(landmarksCount)

	default:
		printHelp()
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
	if err := os.Mkdir("objects", os.ModePerm); err != nil {
		slog.Info(err)
	}

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

func printHelp() {
	fmt.Println("Help Section:")
	fmt.Println("-------------")
	fmt.Println("Usage: <program_name> <command> [options]")
	fmt.Println()
	fmt.Println("Available Commands:")
	fmt.Println()
	fmt.Println("1. graph")
	fmt.Println("   Description: Generates a graph using the generator.")
	fmt.Println("   Usage: <program_name> graph")
	fmt.Println()
	fmt.Println("2. server")
	fmt.Println("   Description: Starts the server to handle requests.")
	fmt.Println("   Usage: <program_name> server")
	fmt.Println()
	fmt.Println("3. multi")
	fmt.Println("   Description: Runs the multi-router with the specified number of iterations.")
	fmt.Println("   Usage: <program_name> multi <iterations>")
	fmt.Println("   Example: <program_name> multi 10")
	fmt.Println()
	fmt.Println("4. single")
	fmt.Println("   Description: Runs the single-router for a specific run and iteration count.")
	fmt.Println("   Usage: <program_name> single <run_name> <iterations>")
	fmt.Println("   Example: <program_name> single testRun 5")
	fmt.Println()
	fmt.Println("5. alt-pre")
	fmt.Println("   Description: Pre-processes landmarks for the ALT algorithm.")
	fmt.Println("   Usage: <program_name> alt-pre")
	fmt.Println()
	fmt.Println("6. tester")
	fmt.Println("   Description: Runs the tester with a specified number of iterations.")
	fmt.Println("   Usage: <program_name> tester <iterations>")
	fmt.Println("   Example: <program_name> tester 20")
	fmt.Println()
	fmt.Println("7. debug")
	fmt.Println("   Description: Runs the router in debug mode to assist in troubleshooting.")
	fmt.Println("   Usage: <program_name> debug")
}
