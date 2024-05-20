package main

import (
	"container/heap"
	"fmt"
	"math"
)

type Point struct {
	X, Y int
}

type CellType int

const (
	IMPASSABLE CellType = iota
	DIFFICULT
	EASY
	NORMAL
)

var Difficulties = map[CellType]float64{
	IMPASSABLE: math.Inf(1),
	DIFFICULT:  2.0,
	EASY:       0.5,
	NORMAL:     1.0,
}

type Map struct {
	Grid   [][]CellType
	Width  int
	Height int
}

func main() {
	grid := [][]CellType{
		{NORMAL, IMPASSABLE, NORMAL, NORMAL, NORMAL},
		{NORMAL, IMPASSABLE, NORMAL, DIFFICULT, NORMAL},
		{NORMAL, IMPASSABLE, NORMAL, DIFFICULT, NORMAL},
		{NORMAL, IMPASSABLE, EASY, NORMAL, NORMAL},
		{NORMAL, DIFFICULT, NORMAL, NORMAL, NORMAL},
	}

	m := Map{Grid: grid, Width: 5, Height: 5}
	start := Point{0, 0}
	goal := Point{4, 4}

	path, cost := AStar(m, start, goal)
	if cost == math.Inf(1) {
		fmt.Println("No path found")
	} else {
		fmt.Println("Path:", path)
		fmt.Println("Cost:", cost)
		visualizePath(m, path)
	}

}

type PriorityQueue []*Item

type Item struct {
	point    Point
	priority float64
	index    int
}

func (pq PriorityQueue) Len() int { return len(pq) }

func (pq PriorityQueue) Less(i, j int) bool {
	return pq[i].priority < pq[j].priority
}

func (pq PriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}

func (pq *PriorityQueue) Push(x interface{}) {
	n := len(*pq)
	item := x.(*Item)
	item.index = n
	*pq = append(*pq, item)
}

func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	item.index = -1
	*pq = old[0 : n-1]
	return item
}

func heuristic(a, b Point) float64 {
	return math.Abs(float64(a.X-b.X)) + math.Abs(float64(a.Y-b.Y))
}

func (m *Map) DifficultyAt(point Point) float64 {
	if point.X < 0 || point.X >= m.Width || point.Y < 0 || point.Y >= m.Height {
		return math.Inf(1)
	}
	return Difficulties[m.Grid[point.Y][point.X]]
}

func neighbors(p Point, width, height int) []Point {
	var points []Point
	directions := []Point{
		{X: -1, Y: 0}, {X: 1, Y: 0}, {X: 0, Y: -1}, {X: 0, Y: 1},
	}
	for _, d := range directions {
		np := Point{X: p.X + d.X, Y: p.Y + d.Y}
		if np.X >= 0 && np.X < width && np.Y >= 0 && np.Y < height {
			points = append(points, np)
		}
	}
	return points
}

func AStar(m Map, start, goal Point) ([]Point, float64) {
	openSet := &PriorityQueue{}
	heap.Init(openSet)
	heap.Push(openSet, &Item{point: start, priority: 0})

	cameFrom := make(map[Point]Point)
	costSoFar := make(map[Point]float64)
	costSoFar[start] = 0

	for openSet.Len() > 0 {
		current := heap.Pop(openSet).(*Item).point

		if current == goal {
			return reconstructPath(cameFrom, current), costSoFar[current]
		}

		processNeighbors := func(next Point) {
			newCost := costSoFar[current] + m.DifficultyAt(next)
			if oldCost, ok := costSoFar[next]; !ok || newCost < oldCost {
				costSoFar[next] = newCost
				priority := newCost + heuristic(next, goal)
				heap.Push(openSet, &Item{point: next, priority: priority})
				cameFrom[next] = current
			}
		}

		for _, next := range neighbors(current, m.Width, m.Height) {
			processNeighbors(next)
		}
	}

	return nil, math.Inf(1)
}

func reconstructPath(cameFrom map[Point]Point, current Point) []Point {
	var path []Point
	for {
		path = append([]Point{current}, path...)
		var ok bool
		if current, ok = cameFrom[current]; !ok {
			break
		}
	}
	return path
}

func visualizePath(m Map, path []Point) {
	pathSet := make(map[Point]struct{})
	for _, p := range path {
		pathSet[p] = struct{}{}
	}

	cellToChar := func(cell CellType) string {
		switch cell {
		case IMPASSABLE:
			return "I "
		case DIFFICULT:
			return "D "
		case EASY:
			return "E "
		case NORMAL:
			return ". "
		default:
			return "? "
		}
	}

	for y := 0; y < m.Height; y++ {
		for x := 0; x < m.Width; x++ {
			point := Point{X: x, Y: y}
			if _, isPath := pathSet[point]; isPath {
				fmt.Print("P ")
			} else {
				fmt.Print(cellToChar(m.Grid[y][x]))
			}
		}
		fmt.Println()
	}
}
