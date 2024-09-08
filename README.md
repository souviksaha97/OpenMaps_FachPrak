# Open Street Maps Fachpraktikum

## Table of Contents

- [Open Street Maps Fachpraktikum](#open-street-maps-fachpraktikum)
  - [Table of Contents](#table-of-contents)
  - [About](#about)
  - [Getting Started](#getting-started)
    - [Go](#go)
  - [Instructions](#instructions)

## About<a name = "about"></a>

Submission for OSM Fachpraktikum submission for SS2024.

## Getting Started<a name = "getting_started"></a>

If you do not have a working system of Go up already, please follow the section below.

### Go
Use the instructions mentioned in the [official Go instructions](https://go.dev/doc/install) to install the latest version of Go for your respective operating system.
Note - The code has been tested on Go version 1.22.6

Once Go is up and running, please run 
```bash
go mod tidy
```
to install the required libraries and packages

## Instructions<a name = "instructions"></a>

1. Graph generation
   The code will prompt you to generate graphs on first run, or if the files are missing from the objects folder.
   To generate manually, use

   ```bash
   go run . graph
   ```

2. Server
    To run the server, use

    ```bash
    go run . server
    ```

    ![Server](final/docs/pic3.png)

    Once the server runs, open the  plotters/renderer.html. Select any two points on the map. If a path between them exists then the server will return the path. Each algorithm will have a different path, indicated by the box text colour.

3. Headless comparison
   To run all the different algorithms with the same set of points, one after the other, use

   ```bash
   go run . multi <number of point pairs>
   ```

   For example,
   ```bash
   go run . multi 100
   ```

   Runs the Djikstra, A* and ALT algorithms one after the other, and prints a comparison of the algorithms.

   ![Comparative result](final/docs/pic1.png)

4. Headless single
   To run one of the algorithms, use

   ```bash
    go run . single <algo> <number of point pairs>
    ```

    Djikstra - djikstra
    A* - astar
    ALT - alt

    For example,
    ```bash
    go run . single alt 100
    ```

    ![Single run](final/docs/pic2.png)