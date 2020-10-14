/** Class for running the A* pathfinding algorithm */
class AStar {
  constructor() {}

  /**
   * Run the algorithm with the provided values.
   * @param {Object[][]} grid - A 2d array representing the grid the algorithm runs on
   * @param {Object} start - Row and column of the source node
   * @param {Object} end - Row and column of the target node
   */
  async run(grid, start, end) {
    // reset the grid so previous runs don't affect this one
    resetGrid(grid);
    // initialize the list containing the unvisited nodes and the
    // starting point
    var openSet = [];
    var startingPoint = grid[start.y][start.x];
    startingPoint.dist = 0;
    startingPoint.fScore = this.h(startingPoint, end);
    openSet.push(startingPoint);

    // run as long as there are unexplored nodes or until target is found
    while (!openSet.length == 0) {
      var curr = this.findLowestFScore(openSet, end);
      curr.visited = true;

      await colorBlock(
        "#node-" + curr.row + "-" + curr.col,
        "#FF8B84",
        500,
        30,
        "fill"
      );

      // the current node is the target node
      // -> highlight path and break
      if (curr.row == end.y && curr.col == end.x) {
        this.makePath(grid, end);
        break;
      }

      var nbs = getNeighbours(grid, curr);

      //examine every neighbour of the current node
      for (const nb in nbs) {
        var neighbour = nbs[nb];

        // conditional operator evaluating to 'true' if neighbour is already in
        // openSet, or 'false' if it isn't
        var inOpenSet =
          openSet.findIndex(
            (elem) => elem.x == neighbour.x && elem.y == neighbour.y
          ) == -1
            ? false
            : true;

        // skip all following steps if neighbour has been visited
        if (neighbour.visited) {
          continue;
        }

        var tentative_gScore =
          curr.dist + (neighbour.type == "weight" ? 10 : 1);

        // neighbour does not need update -> skip remaining steps
        if (inOpenSet && tentative_gScore >= nb.dist) {
          continue;
        }

        // update neighbour and push it to openSet
        neighbour.dist = tentative_gScore;
        neighbour.predecessor = { col: curr.col, row: curr.row };
        neighbour.fScore = neighbour.dist + this.h(neighbour, end);

        if (!inOpenSet) {
          openSet.push(neighbour);
        }
      }
    }
  }

  /**
   * Heuristic function for the A* algorithm. Calculates Manhattan distance.
   * @param {Object} node - The current node
   * @param {Object} end  - The target node
   * @returns {number} - The Manhattan distance between node and end
   */
  h(node, end) {
    // -> Manhattan distance = |x0-x1| + |y0-y1|
    var manhDist = Math.abs(node.col - end.x) + Math.abs(node.row - end.y);
    return manhDist;
  }

  /**
   * Finds the node with the lowest fScore, removes it from the list
   * and returns it.
   * Takes into consideration raw heuristic distance should there be
   * two nodes with the same fScore.
   * @param {Object[]} q - List of all unvisited nodes
   * @param {Object} end - The target node, needed for calculating heuristic distance
   * @returns {Object} - The node with the lowest fScore
   */
  findLowestFScore(q, end) {
    var closest = { fScore: Infinity };
    var ind = 0;

    for (let i = 0; i < q.length; i++) {
      if (closest.fScore > q[i].fScore) {
        // fScore of the current node is smaller than the
        // fScore of the closest node so far
        // -> set it as closest and update index
        closest = q[i];
        ind = i;
      } else if (this.h(closest, end) > this.h(q[i], end)) {
        // fScores are equal
        // -> compare raw heuristic distances
        closest = q[i];
        ind = i;
      }
    }
    // remove closest node from the openSet
    q.splice(ind, 1);
    return closest;
  }

  /**
   * Reads predecessors starting from the target node and colors the path.
   * @param {Object[][]} grid - 2d array of nodes representing the grid
   * @param {Object} end - Row and column of the target node
   */
  async makePath(grid, end) {
    await timeout(500);
    var list = [];
    var v = grid[end.y][end.x];
    list.unshift(v);
    // step through the predecessors until we hit the start node.
    // color every node on the way and save the path in a list.
    while (v.predecessor != undefined) {
      await colorBlock("#node-" + v.row + "-" + v.col, "#cc1616", 250, 15);
      v = grid[v.predecessor.row][v.predecessor.col];
      list.unshift(v);
    }

    list.unshift(v);
    // animate the stick figure
    makeHimRun(list);
  }
}
