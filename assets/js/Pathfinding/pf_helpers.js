/** @module PathfindingHelperFunctions */

/**
 * Gets all nodes bordering the current node.
 * @param {Object[][]} grid - 2d array of nodes
 * @param {Object} v - The current node
 * @returns {Object[]} - A list of nodes adjacent to v
 */
function getNeighbours(grid, v) {
  var nbs = [];

  if (v.col - 1 >= 0 && grid[v.row][v.col - 1]) {
    if (grid[v.row][v.col - 1].type != "wall")
      // push left neighbour
      nbs.push(grid[v.row][v.col - 1]);
  }
  if (v.col + 1 < grid[0].length && grid[v.row][v.col + 1]) {
    if (grid[v.row][v.col + 1].type != "wall")
      // push right neighbour
      nbs.push(grid[v.row][v.col + 1]);
  }
  if (v.row - 1 >= 0 && grid[v.row - 1][v.col]) {
    if (grid[v.row - 1][v.col].type != "wall")
      // push upper neighbour
      nbs.push(grid[v.row - 1][v.col]);
  }
  if (v.row + 1 < grid.length && grid[v.row + 1][v.col]) {
    if (grid[v.row + 1][v.col].type != "wall")
      // push lower neighbour
      nbs.push(grid[v.row + 1][v.col]);
  }

  return nbs;
}

/**
 * Resets the grid and every node to their initial state.
 * @param {Object[][]} grid - 2d array of objects representing the grid
 */
function resetGrid(grid) {
  for (let y = 0; y < grid.length; y++) {
    for (let x = 0; x < grid[y].length; x++) {
      // reset distances, fScores, predecessors and visited status of every node
      grid[y][x].dist = Infinity;
      grid[y][x].fScore = Infinity;
      grid[y][x].predecessor = undefined;
      grid[y][x].visited = false;

      // color cell white if it's a floor tile or grey if it's a weight
      if (grid[y][x].type != "wall" && grid[y][x].type != "weight") {
        d3.select("#node-" + y + "-" + x).attr("fill", "#FFF");
      } else if (grid[y][x].type == "weight") {
        d3.select("#node-" + y + "-" + x).attr("fill", "#B0B0B0");
      }
    }
  }
}

/**
 * Animates the stick figure to move from start to target.
 * @param {Object[]} list - List of nodes in the path
 */
async function makeHimRun(list) {
  // loop over the path list
  // and adjust the position of the stick figure
  for (let i in list) {
    d3.select("#start")
      .transition()
      .duration(50)
      .attr("x", list[i].x)
      .attr("y", list[i].y);
    await timeout(50);
  }
  await timeout(200);
  // teleport the stick figure back to the start
  var x = gridData[startPos.y][startPos.x].x;
  var y = gridData[startPos.y][startPos.x].y;
  d3.select("#start").attr("x", x).attr("y", y);
}
