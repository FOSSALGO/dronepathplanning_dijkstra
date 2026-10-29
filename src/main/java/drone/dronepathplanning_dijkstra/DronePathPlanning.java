package drone.dronepathplanning_dijkstra;

import java.util.*;

/** Dijkstra's shortest-path algorithm; single drone, static occupancy grid. */
public final class DronePathPlanning {
    private final Point start, finish;
    private final boolean success;
    private final double pathCost;
    private final int expandedNodes;
    private final long elapsedNanos;
    private final int steps;
    private final String terminationReason;
    private static final int[][] N8 = {{-1,0},{-1,1},{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1}};

    public DronePathPlanning(int[][] environment, Drone drone, Point finish) {
        if (environment == null || environment.length == 0 || environment[0] == null || environment[0].length == 0 || drone == null || finish == null)
            throw new IllegalArgumentException("Grid, drone, and finish must be provided");
        this.start = new Point(drone.initialRowPosition, drone.initialColPosition);
        this.finish = finish;
        int rows = environment.length, cols = environment[0].length;
        if (!free(environment, start.row(), start.col()) || !free(environment, finish.row(), finish.col()))
            throw new IllegalArgumentException("Start or finish is outside grid or blocked");
        long begin = System.nanoTime();

        // =====================================================================
        // BEGIN REPLACEABLE ALGORITHM: DIJKSTRA'S SHORTEST PATH
        // Movement: 8-connected; orthogonal cost = 1; diagonal cost = sqrt(2).
        // No diagonal corner cutting: both adjacent orthogonal cells must be free.
        // Dijkstra uses accumulated travel cost g(n), WITHOUT any heuristic.
        // =====================================================================
        double[][] distance = new double[rows][cols];
        for (double[] row : distance) Arrays.fill(row, Double.POSITIVE_INFINITY);
        boolean[][] settled = new boolean[rows][cols];
        Point[][] parent = new Point[rows][cols];
        PriorityQueue<Node> frontier = new PriorityQueue<>(Comparator
                .comparingDouble(Node::cost)
                .thenComparingInt(Node::row)
                .thenComparingInt(Node::col));
        distance[start.row()][start.col()] = 0.0;
        frontier.add(new Node(start.row(), start.col(), 0.0));

        int expanded = 0;
        boolean reached = false;
        while (!frontier.isEmpty()) {
            Node current = frontier.poll();
            int r = current.row(), c = current.col();
            if (settled[r][c]) continue; // Ignore outdated queue entries.
            settled[r][c] = true;
            expanded++;

            if (r == finish.row() && c == finish.col()) {
                reached = true;
                break;
            }

            for (int[] delta : N8) {
                int nr = r + delta[0], nc = c + delta[1];
                if (!free(environment, nr, nc) || settled[nr][nc]) continue;
                boolean diagonal = delta[0] != 0 && delta[1] != 0;
                if (diagonal && (!free(environment, r, nc) || !free(environment, nr, c))) continue;
                double candidateCost = distance[r][c] + (diagonal ? Math.sqrt(2.0) : 1.0);
                if (candidateCost < distance[nr][nc]) {
                    distance[nr][nc] = candidateCost;
                    parent[nr][nc] = new Point(r, c);
                    frontier.add(new Node(nr, nc, candidateCost));
                }
            }
        }

        ArrayList<Point> resultPath = new ArrayList<>();
        double cost = Double.NaN;
        if (reached) {
            LinkedList<Point> chain = new LinkedList<>();
            Point current = finish;
            while (current != null) {
                chain.addFirst(current);
                if (current.equals(start)) break;
                current = parent[current.row()][current.col()];
            }
            resultPath.addAll(chain);
            cost = distance[finish.row()][finish.col()];
        }
        // =====================================================================
        // END REPLACEABLE ALGORITHM: DIJKSTRA'S SHORTEST PATH
        // =====================================================================

        elapsedNanos = System.nanoTime() - begin;
        success = reached;
        expandedNodes = expanded;
        pathCost = cost;
        terminationReason = reached ? "GOAL_REACHED" : "NO_PATH";
        drone.path = reached ? resultPath : new ArrayList<>(List.of(start));
        drone.frameIndex = 0;
        steps = drone.path.size() - 1;
    }

    private static boolean free(int[][] env, int r, int c) {
        return r >= 0 && r < env.length && env[r] != null && c >= 0 && c < env[r].length && env[r][c] != -1;
    }

    private record Node(int row, int col, double cost) {}
    public Point getStart() { return start; }
    public Point getFinish() { return finish; }
    public boolean isSuccess() { return success; }
    public double getPathCost() { return pathCost; }
    public int getExpandedNodes() { return expandedNodes; }
    public long getElapsedNanos() { return elapsedNanos; }
    public int getSteps() { return steps; }
    public String getTerminationReason() { return terminationReason; }
}