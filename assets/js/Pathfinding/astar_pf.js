class AStar {
  constructor() {}

  /**
   * Runs A* algorithm with animated visualization
   * @param {Object[][]} grid - The grid to run the algorithm on
   * @param {Object} start - Starting position {x, y}
   * @param {Object} end - Target position {x, y}
   */
  async run(grid, start, end) {
    resetGrid(grid);

    const openSet = [];
    const startNode = grid[start.y][start.x];
    startNode.dist = 0;
    startNode.fScore = this.calculateHeuristic(startNode, end);
    openSet.push(startNode);

    while (openSet.length > 0) {
      const current = this.findBestNode(openSet);
      current.visited = true;

      // Animate visiting current node
      await this.animateVisit(current);

      if (current.row === end.y && current.col === end.x) {
        await this.animatePath(grid, end);
        return true;
      }

      const neighbors = this.getNeighbors(grid, current);
      await this.processNeighbors(neighbors, current, openSet, end);
    }

    return false;
  }

  async animateVisit(node) {
    await colorBlock(
      `#node-${node.row}-${node.col}`,
      "#64B5F6",
      150,
      20,
      "fill"
    );
  }

  async animatePath(grid, end) {
    let current = grid[end.y][end.x];
    const path = [];

    while (current.predecessor) {
      path.unshift(current);
      current = grid[current.predecessor.row][current.predecessor.col];
    }
    path.unshift(current);

    for (const node of path) {
      await colorBlock(
        `#node-${node.row}-${node.col}`,
        "#FDD835",
        300,
        30,
        "fill"
      );
    }
  }

  async processNeighbors(neighbors, current, openSet, end) {
    for (const neighbor of neighbors) {
      if (neighbor.visited) continue;

      const tentativeScore = current.dist + (neighbor.type === "weight" ? 10 : 1);

      if (!this.isInOpenSet(openSet, neighbor)) {
        openSet.push(neighbor);
        await colorBlock(
          `#node-${neighbor.row}-${neighbor.col}`,
          "#90CAF9",
          100,
          10,
          "fill"
        );
      }

      if (tentativeScore < neighbor.dist) {
        neighbor.predecessor = { row: current.row, col: current.col };
        neighbor.dist = tentativeScore;
        neighbor.fScore = tentativeScore + this.calculateHeuristic(neighbor, end);
        await this.animateHeuristic(neighbor, end);
      }
    }
  }

  async animateHeuristic(node, end) {
    const heuristic = this.calculateHeuristic(node, end);
    const element = document.querySelector(`#heuristic-${node.row}-${node.col}`);
    if (element) element.textContent = heuristic;
  }

  findBestNode(openSet) {
    let bestNodeIndex = 0;

    for (let i = 1; i < openSet.length; i++) {
      if (
        openSet[i].fScore < openSet[bestNodeIndex].fScore ||
        (openSet[i].fScore === openSet[bestNodeIndex].fScore &&
          openSet[i].dist < openSet[bestNodeIndex].dist)
      ) {
        bestNodeIndex = i;
      }
    }

    return openSet.splice(bestNodeIndex, 1)[0];
  }

  getNeighbors(grid, node) {
    const neighbors = [];
    const directions = [
      [-1, 0], // Up
      [1, 0], // Down
      [0, -1], // Left
      [0, 1], // Right
    ];

    for (const [dx, dy] of directions) {
      const newRow = node.row + dx;
      const newCol = node.col + dy;

      if (this.isValidPosition(grid, newRow, newCol)) {
        neighbors.push(grid[newRow][newCol]);
      }
    }

    return neighbors;
  }

  isValidPosition(grid, row, col) {
    return (
      row >= 0 &&
      row < grid.length &&
      col >= 0 &&
      col < grid[0].length &&
      grid[row][col].type !== "wall"
    );
  }

  isInOpenSet(openSet, node) {
    return openSet.some((n) => n.row === node.row && n.col === node.col);
  }

  calculateHeuristic(node, end) {
    return Math.abs(node.col - end.x) + Math.abs(node.row - end.y);
  }
}

