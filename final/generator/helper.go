package generator

import (
	"encoding/json"
	"fmt"
	"math"
	"os"

	"github.com/paulmach/osm"

	"final/types"
)

func getAllNeighbourCellss(grid [][][]int, x int, y int, latitude float64, radius int) []int {
	var neighbors []int

	if latitude >= 89.0 {
		neighbors = append(neighbors, grid[0][0]...)

	} else if latitude <= -89.0 {
		neighbors = append(neighbors, grid[0][0]...)

	}
	counter := 0
	for dx := -radius; dx <= radius; dx++ {
		for dy := -radius; dy <= radius; dy++ {
			if dx != 0 || dy != 0 {

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
			}
		}
	}
	//fmt.Println("nachbarlänge", counter)
	return neighbors
}

func AddEdge(longranges *[][]types.Edge, edge types.Edge) {
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
func RecursivWayROunding(ways_map map[osm.NodeID][]osm.NodeID, nodeRelations map[osm.NodeID]types.WayComplete, i osm.NodeID, way []osm.NodeID) {
	copy := nodeRelations[i]
	copy.IsFinished = true
	nodeRelations[i] = copy
	if way[0] != way[len(way)-1] {
		index := way[len(way)-1]
		if node, exists := nodeRelations[index]; exists { //search for fitting lastnode in node reltions
			if ways_map[node.WayMapKey][0] == ways_map[node.WayMapKey][len(ways_map[node.WayMapKey])-1] {
				fmt.Println("found a fucking roundway instead of next slice")
				//fmt.Println(len(ways_map[found.WayMapKey]))
			} else {
				if !nodeRelations[index].IsFinished {
					way = append(way, ways_map[nodeRelations[index].WayMapKey][1:]...)
					node.IsFinished = true
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

func DotProduct(u, v [3]float64) float64 {
	return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]
}

func Normalize(v [3]float64) [3]float64 {
	mag := math.Sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
	if mag == 0 {
		return v
	}
	return [3]float64{v[0] / mag, v[1] / mag, v[2] / mag}
}

func FindEdgesForPointpoint(longranges *[][]types.Edge, testpoint [2]float64) []types.Edge {
	rangeWidth := 360.0 / float64(longBincount)
	bin := int((testpoint[1] + 180) / rangeWidth)
	if bin >= longBincount {
		bin = longBincount - 1
	}

	var intersectingEdges []types.Edge
	for _, edge := range (*longranges)[bin] {
		found := intersection2Lines(edge.Startpoint, edge.Endpoint, testpoint)
		if found {
			intersectingEdges = append(intersectingEdges, edge)
		}
	}
	//fmt.Println("intersecting edgs", len(intersectingEdges))
	return intersectingEdges
}

func FindIntersection(a1, a2, testpoint [2]float64) ([2]float64, bool) {
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
	line = Normalize(line)
	//fmt.Println(line)
	// Intersection points
	p11, p12 := cartToSphere(line[0]*R, line[1]*R, line[2]*R)
	p21, p22 := cartToSphere(-line[0]*R, -line[1]*R, -line[2]*R)
	p1 := [2]float64{p11, p12}
	p2 := [2]float64{p21, p22}
	fmt.Println(p11, p12, p21, p22)
	//find correct
	if IsPointOnGreatCircleArc(p1, a1, a2) && IsPointOnGreatCircleArc(p1, northpole, testpoint) {
		fmt.Println("worked")
		return p1, true
	}
	if IsPointOnGreatCircleArc(p2, a1, a2) && IsPointOnGreatCircleArc(p2, northpole, testpoint) {
		fmt.Println("worked")
		return p2, true
	}

	return [2]float64{}, false
}

func IsPointOnGreatCircleArc(p, a1, a2 [2]float64) bool {
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
	dot := DotProduct(c1, c2)
	if dot < 0 {
		return false
	}

	// Check if the point lies between a1 and a2
	angle1 := math.Acos(DotProduct(v1, vp) / (R * R))
	angle2 := math.Acos(DotProduct(vp, v2) / (R * R))
	totalAngle := math.Acos(DotProduct(v1, v2) / (R * R))

	return math.Abs((angle1+angle2)-totalAngle) < 1e-9
}
func InitialBearingTo(startpoint, endpoint [2]float64) float64 {

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
func IntersectionBearing(p1 [2]float64, brng1 float64, p2 [2]float64, brng2 float64) [2]float64 {

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
func RotationMatrix(axis [3]float64, theta float64) [3][3]float64 { //chatgpt better faster and robuster than normal rotation matrix
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
func RotatePointWithMatrix(x float64, y float64, z float64, Matrix [3][3]float64) (float64, float64, float64) {
	newX := Matrix[0][0]*x + Matrix[0][1]*y + Matrix[0][2]*z
	newY := Matrix[1][0]*x + Matrix[1][1]*y + Matrix[1][2]*z
	newZ := Matrix[2][0]*x + Matrix[2][1]*y + Matrix[2][2]*z
	return newX, newY, newZ
}
func CalculateMatrixWithNewPolePoint(newPoleLat float64, newPoleLong float64) [3][3]float64 {
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
	Matrix := RotationMatrix(axis, theta)
	return Matrix
}
func CalculateNewPointPositionWithMatrix(pLat float64, pLong float64, matrix [3][3]float64) (float64, float64) {
	if pLat <= -90.1 || pLat >= 90.1 {
		fmt.Println("impossible")
	}
	x, y, z := sphereToCart(pLat, pLong)
	newX, newY, newZ := RotatePointWithMatrix(x, y, z, matrix)
	newLat, newLong := cartToSphere(newX, newY, newZ)
	return newLat, newLong
}

// compute centroid of polygon to have a point X which is contained by it
func CalculateCentroid(longlatOfNodes [][2]float64) (float64, float64) {
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
func LeftAngleSmaller(left float64, right float64) bool {
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
func CalculateAnglesABPwithOutPole(a [2]float64, b [2]float64, p [2]float64) int { //1==true
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
func CalculateAnglesABPwithNewPoleEasy(a [2]float64, b [2]float64, p [2]float64, matrix [3][3]float64) int { //1==true
	_, angleA := CalculateNewPointPositionWithMatrix(a[0], a[1], matrix)
	_, angleB := CalculateNewPointPositionWithMatrix(b[0], b[1], matrix)
	_, angleP := CalculateNewPointPositionWithMatrix(p[0], p[1], matrix)
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
func CalculateAnglesABPwithNewPoleBetter(a [2]float64, b [2]float64, p [2]float64, matrix [3][3]float64) int { //1==true
	_, angleA := CalculateNewPointPositionWithMatrix(a[0], a[1], matrix)
	_, angleB := CalculateNewPointPositionWithMatrix(b[0], b[1], matrix)
	_, angleP := CalculateNewPointPositionWithMatrix(p[0], p[1], matrix)
	if LeftAngleSmaller(angleA, angleB) {
		if LeftAngleSmaller(angleA, angleP) && LeftAngleSmaller(angleP, angleB) {
			if angleP == angleB {
				return -1
			} else {
				return 1 //p is between point A and B
			}
		} else {
			return -1
		}
	} else { //right angle should be lower boundary (could stil have higher value)
		if LeftAngleSmaller(angleP, angleA) && LeftAngleSmaller(angleB, angleP) {
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
func CalculateAnglesABPwithNewPole(a [2]float64, b [2]float64, p [2]float64, matrix [3][3]float64) int {
	_, angleA := CalculateNewPointPositionWithMatrix(a[0], a[1], matrix)
	_, angleB := CalculateNewPointPositionWithMatrix(b[0], b[1], matrix)
	_, angleP := CalculateNewPointPositionWithMatrix(p[0], p[1], matrix)
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
func CalculateAnglesABPwithNewPole2ndTest(a [2]float64, b [2]float64, p [2]float64, matrix [3][3]float64) int {
	_, angleB := CalculateNewPointPositionWithMatrix(a[0], a[1], matrix)
	_, angleX := CalculateNewPointPositionWithMatrix(b[0], b[1], matrix)
	_, angleP := CalculateNewPointPositionWithMatrix(p[0], p[1], matrix)
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

func TestPolygon(pointtoTest [2]float64, key1 osm.NodeID, way1 []osm.NodeID, nodes_map map[osm.NodeID]types.Node, waysMidPoint map[osm.NodeID][2]float64) int {
	var array [][2]float64
	for i := 0; i < len(way1)-1; i++ { // Loop stops before the last element
		nodeId := way1[i]
		node := nodes_map[nodeId]
		array = append(array, [2]float64{node.Lat, node.Lon})
	}
	pointxlat := waysMidPoint[key1][0]
	pointxlong := waysMidPoint[key1][1]
	//fmt.Println("midpoint", pointxlat, pointxlong)

	matrix := CalculateMatrixWithNewPolePoint(pointxlat, pointxlong)
	wasWrong := false
	counthits := 0
	countsuperhits := 0
	for i := 0; i < len(array)-1; i++ {
		if CalculateAnglesABPwithNewPoleEasy(array[i], array[i+1], pointtoTest, matrix) > 0 {
			counthits = counthits + 1
			//fmt.Println("hit point", array[i], array[i+1])
			matrix2 := CalculateMatrixWithNewPolePoint(array[i][0], array[i][1]) //point A as north pole
			test2 := CalculateAnglesABPwithNewPole2ndTest(array[i+1], [2]float64{pointxlat, pointxlong}, pointtoTest, matrix2)
			if test2 == 0 { //point B and X
				return 0

			} else {
				if test2 > 0 {
					countsuperhits = countsuperhits + 1
				}
			}
		} else {
			if CalculateAnglesABPwithNewPoleEasy(array[i], array[i+1], pointtoTest, matrix) == 0 {
				wasWrong = true
				fmt.Println(pointtoTest, pointxlat, pointxlong)
			}
		}
	}
	if wasWrong {
		countsuperhits := 0
		pointxlat := waysMidPoint[key1][0] + 0.0000001
		pointxlong := waysMidPoint[key1][1] + 0.0000001
		matrix := CalculateMatrixWithNewPolePoint(pointxlat, pointxlong)
		for i := 0; i < len(array)-1; i++ {
			if CalculateAnglesABPwithNewPole(array[i], array[i+1], pointtoTest, matrix) > 0 {
				//counthits = counthits + 1
				matrix2 := CalculateMatrixWithNewPolePoint(array[i][0], array[i][1]) //point A as north pole
				test2 := CalculateAnglesABPwithNewPole2ndTest(array[i+1], [2]float64{pointxlat, pointxlong}, pointtoTest, matrix2)
				if test2 == 0 { //point B and X
					return 0

				} else {
					if test2 > 0 {
						countsuperhits = countsuperhits + 1
					}
				}
			} else {
				if CalculateAnglesABPwithNewPole(array[i], array[i+1], pointtoTest, matrix) == -1 {
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
func TestPolygonWithoutNorth(pointtoTest [2]float64, key1 osm.NodeID, way1 []osm.NodeID, nodes_map map[osm.NodeID]types.Node) int {
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
		if CalculateAnglesABPwithOutPole(array[i], array[i+1], pointtoTest) > 0 {
			counthits = counthits + 1
			//fmt.Println("hit point", array[i], array[i+1])
			matrix2 := CalculateMatrixWithNewPolePoint(array[i][0], array[i][1]) //point A as north pole
			test2 := CalculateAnglesABPwithNewPole2ndTest(array[i+1], [2]float64{-90.0, 0.0}, pointtoTest, matrix2)
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

func TestAllPolygonesWithBounding(pointtoTest [2]float64, ways_map map[osm.NodeID][]osm.NodeID, nodes_map map[osm.NodeID]types.Node, wayBoundingBox map[osm.NodeID][4]float64) int {
	lat := pointtoTest[0]
	lon := pointtoTest[1]
	inAnyPolygon := -1
	happened := 0
	for key1, box := range wayBoundingBox {

		if box[0] < lat && box[1] > lat && box[2] < lon && box[3] > lon {
			test := TestPolygonWithoutNorth(pointtoTest, key1, ways_map[key1], nodes_map)
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
func TestAllPolygonesIfMidpointsisIn(ways_map map[osm.NodeID][]osm.NodeID, nodes_map map[osm.NodeID]types.Node, waysMidPoint map[osm.NodeID][2]float64, wayBoundingBox map[osm.NodeID][4]float64) int {

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

		test := TestPolygon([2]float64{pointxlat, pointxlong}, key1, way1, nodes_map, waysMidPoint)
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

func degreesToRadian(deg float64) float64 {
	return deg * (math.Pi / 180)
}

func Haversine(lat1, lon1, lat2, lon2 float64) float64 {
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
				dist := Haversine(searchpointLat, searchPointLon, point[0], point[1])
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
				dist := Haversine(searchpointLat, searchPointLon, point[0], point[1])
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
				dist := Haversine(searchpointLat, searchPointLon, point[0], point[1])
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
				dist := Haversine(searchpointLat, searchPointLon, point[0], point[1])
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
	}
	return false

}

func Contains(slice []int, key int) bool {
	for _, element := range slice {
		if element == key {
			return true
		}
	}
	return false
}

func WriteToJSONFile(filename string, data interface{}) error {
	// Marshal the data to JSON
	jsonData, err := json.Marshal(data) // Pretty print the JSON
	if err != nil {
		return fmt.Errorf("failed to marshal data to JSON: %w", err)
	}

	// Write the JSON data to the file
	err = os.WriteFile(filename, jsonData, 0644)
	if err != nil {
		return fmt.Errorf("failed to write JSON to file: %w", err)
	}

	return nil
}

func Abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

func Max(x []int) int {
	max := x[0]
	for _, value := range x {
		if value > max {
			max = value
		}
	}
	return max
}
