//imports
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.Scanner;

public class AStarAlgorithmPathFinder {
    // grid size and percent of blocked off spaces
    private static final int ROWS = 15;
    private static final int COLS = 15;
    private static final int PERCENT_SPACES_BLOCKED_OFF = 10;

    public static void main(String[] args) {
        // creates board and blocked spaces
        int[][] board = createBoard();
        printBoard(board);

        try (Scanner scanner = new Scanner(System.in)) {
            while (true) {
                // asks for initial coordinates on the board
                System.out.println("Enter coordinates of starting node (row X and column Y) (0,0 to 14,14):");
                int startRow = scanner.nextInt();
                int startCol = scanner.nextInt();

                // asks where the end should be
                System.out.println("Enter coordinates of end node (row X and column Y) (0,0 to 14,14):");
                int endRow = scanner.nextInt();
                int endColumn = scanner.nextInt();

                // makes sure coordinates are within the 15x15 board
                if (!isNodeInBounds(startRow, startCol) || !isNodeInBounds(endRow, endColumn)) {
                    System.out.println("This node isn't on the board or isn't available. Try again.");
                    continue;
                }

                // uses A* algorithm to find a path
                List<Node> path = findPath(board, startRow, startCol, endRow, endColumn);

                // if a path is found, each step of the path is printed 
                if (path != null) {
                    System.out.println("Path found:");
                    for (Node node : path) {
                        System.out.println("[Row: " + node.getRow() + ", Col: " + node.getCol() + "]");
                    }
                    System.out.println();
                //if a path cannot be found, a message is displayed
                } else {
                    System.out.println("No path could be found.\n");
                }
            }
        }
    }

    // randomly creates a grid based on the number of rows and columns specified
    // also randomly creates the 1's based on the PERCENT_SPACES_BLOCKED_OFF variable to cover that % of the board
    private static int[][] createBoard() {
        int[][] board = new int[ROWS][COLS];
        Random random = new Random();

        for (int row = 0; row < ROWS; row++) {
            for (int col = 0; col < COLS; col++) {
                if (random.nextInt(100) < PERCENT_SPACES_BLOCKED_OFF) {
                    board[row][col] = 1; // 1 represents a blocked node
                } else {
                    board[row][col] = 0; // 0 represents a pathable node
                }
            }
        }

        return board;
    }

    // prints the randomly genereated board made by createBoard
    private static void printBoard(int[][] board) {
        System.out.println("Created Board:");
        for (int row = 0; row < ROWS; row++) {
            for (int col = 0; col < COLS; col++) {
                System.out.print(board[row][col] + " ");
            }
            System.out.println();
        }
        System.out.println();
    }

    /*  makes sure the number of rows and columns are greater than or equal to
        the x and y coordinates the user specifies for the start and end nodes.
        also accounts for 0*/ 
    private static boolean isNodeInBounds(int row, int col) {
        return row >= 0 && row < ROWS && col >= 0 && col < COLS;
    }

    // A* algorithm is used to find the path from start to end coordinates
    private static List<Node> findPath(int[][] board, int startRow, int startCol, int endRow, int endColumn) {
        // uses the node class to create objects for starting and ending nodes based on their column & row values
        Node startNode = new Node(startRow, startCol, board[startRow][startCol]);
        Node endNode = new Node(endRow, endColumn, board[endRow][endColumn]);

        // open list initialized for A* algorithm
        List<Node> openList = new ArrayList<>();
        // closed list initialized for A* algorithm
        List<Node> closedList = new ArrayList<>();

        // start node is first added to the newly created open list
        openList.add(startNode);

        // A* loop
        while (!openList.isEmpty()) {
            // gets node with lowest f off the open list
            Node currentNode = getNodeWithLowestF(openList);
            // node with lowest f is the current node and then it is moved from the open to closed list
            openList.remove(currentNode);
            closedList.add(currentNode);

            // once the current node is in the closed list, if that is equal to the end, the path is returned
            if (currentNode.equals(endNode)) {
                return reconstructPath(currentNode);
            }

            // gets neighbors of current node
            List<Node> neighbors = getNeighbors(currentNode, board);
            for (Node neighbor : neighbors) {
                // if the neighbor is on the closed list already, this node is skipped
                if (closedList.contains(neighbor)) {
                    continue;
                }

                // gets G or the cost of the path from the start node to the current node
                int provisionalGValue = currentNode.getG() + 1;

                /*  if the neighbor is not in the open list or the new g score is lower,
                    update its values and add it to the open list */
                if (!openList.contains(neighbor) || provisionalGValue < neighbor.getG()) {
                    neighbor.setParent(currentNode);
                    neighbor.setG(provisionalGValue);
                    neighbor.setH(manhattanDistanceHeuristic(neighbor, endNode));
                    neighbor.setF();
                    if (!openList.contains(neighbor)) {
                        openList.add(neighbor);
                    }
                }
            }
        }
        // if no path is possible, returns null
        return null;
    }

    /*  finds node with lowest F of the total estimated cost of 
        the cheapest path from the start node to the goal node 
        that goes through the current node in a list */
    private static Node getNodeWithLowestF(List<Node> nodes) {
        return Collections.min(nodes, (n1, n2) -> n1.getF() - n2.getF());
    }

    // finds neighbors of a
    private static List<Node> getNeighbors(Node node, int[][] board) {
        List<Node> neighbors = new ArrayList<>();
        int row = node.getRow();
        int col = node.getCol();

        // up, down, left, right movements in a coordinate array
        int[][] directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

        /*  if a neighbor is a valid node on the board, 
            it is added to the list as each direction is itterated through */
        for (int[] dir : directions) {
            int newRow = row + dir[0];
            int newColumn = col + dir[1];
            if (isNodeInBounds(newRow, newColumn) && board[newRow][newColumn] != 1) {
                neighbors.add(new Node(newRow, newColumn, board[newRow][newColumn]));
            }
        }

        return neighbors;
    }

    // this is how the Manhattan distance heuristic between two nodes is calculated
    private static int manhattanDistanceHeuristic(Node node, Node endNode) {
        return Math.abs(node.getRow() - endNode.getRow()) + Math.abs(node.getCol() - endNode.getCol());
    }

    // create a path from start to current node
    private static List<Node> reconstructPath(Node currentNode) {
        List<Node> path = new ArrayList<>();
        while (currentNode != null) {
            path.add(currentNode);
            currentNode = currentNode.getParent();
        }
        // the path displayed wouldve been from end to start so we need to reverse it for the display
        Collections.reverse(path);
        return path;
    }
}
