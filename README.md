<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0"/>
</head>
<body>
  <div class="container">
    <h1>Drone Delivery Pathfinding</h1>
    <p><strong>Language:</strong> Prolog</p>
    <p><strong>Search Type:</strong> Uninformed Search (DFS)</p>
    <h2>About</h2>
    <p>This project simulates a delivery drone navigating a city grid to deliver packages. The drone must visit as many delivery points (<code>P</code>) as possible, starting from the top-left corner (<code>D</code>), while avoiding obstacles (<code>O</code>).</p>
    <p>The program uses uninformed search algorithms — either <strong>Depth-First Search (DFS)</strong> or <strong>Breadth-First Search (BFS)</strong> — to explore the grid and determine the optimal path that reaches the most delivery points without hitting any obstacles.</p>
    <h2>Features</h2>
    <ul>
      <li>City grid is represented as an M x N matrix.</li>
      <li>Drone starts from the top-left corner (0, 0).</li>
      <li>Obstacles are avoided automatically by the algorithm.</li>
      <li>Returns the path that visits the maximum number of delivery points.</li>
    </ul>
    <h2>How to Run</h2>
    <ol>
      <li>Install a Prolog interpreter such as <a href="https://www.swi-prolog.org/" target="_blank">SWI-Prolog</a>.</li>
      <li>Load the Prolog file in the terminal:
        <pre><code>?- [drone_delivery].</code></pre>
      </li>
      <li>Run the search:
        <pre><code>?- bfs(Path, Visited).</code></pre>
        or
        <pre><code>?- dfs(Path, Visited).</code></pre>
      </li>
    </ol>
  </div>
</body>
</html>
