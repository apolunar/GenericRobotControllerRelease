package org.firstinspires.ftc.teamcode.math.navigation.navigators;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.math.navigation.CoordSys;
import org.firstinspires.ftc.teamcode.math.navigation.Navigator;
import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.obstacle.Obstacle;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.obstacle.RectangleObstacle;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Set;

import lombok.RequiredArgsConstructor;

/**
 * References:
 *  - <a href="https://en.wikipedia.org/wiki/Occupancy_grid_mapping">Obstacle Grid Mapping (Wikipedia)</a>
 *  - <a href="https://fab.cba.mit.edu/classes/865.21/topics/path_planning/robotic.html">Mobile Robot Path Planning (MIT)</a>
 *  - <a href="https://en.wikipedia.org/wiki/K-d_tree">K-d tree</a>
 */
public class AStarNavigator implements Navigator {
    private static final int[][] DIRECTIONS = new int[][]{
            new int[]{0, 1},
            new int[]{0, -1},
            new int[]{1, 0},
            new int[]{-1, 0},
            new int[]{1, 1},
            new int[]{1, -1},
            new int[]{-1, 1},
            new int[]{-1, -1}
    };

    @RequiredArgsConstructor
    static
    class Cell {
        private final Translation2d pos;
        private Translation2d parent;
        private boolean closed = false;
        private double gCostFromStartToCell         = 0;
        private double hHeuristicCostFromCellToDest = 0;
        private double fTotalCostCell               = Double.MAX_VALUE;
    }

    private final RobotSize robotSize;
    private final double gridSizeIn;
    private final int minGridUnitIn;
    private final TrajectoryConfig trajectoryConfig;
    private ArrayList<Obstacle> internalObstacleMap;
    private Cell[][] grid;

    private final int gridMax;
    private final double safetyFactor;

    public AStarNavigator(ObstacleMap obstacleMap, double gridSizeIn, RobotSize robotSize, TrajectoryConfig trajectoryConfig, double safetyFactor) {
        this.gridSizeIn   = gridSizeIn;
        this.safetyFactor = safetyFactor;

        Obstacle minGridObstacle =
                Arrays.stream(obstacleMap.getObstacles()).min(
                    Comparator.comparing(Obstacle::getMinSize)
                )
                .orElse(null);
        this.minGridUnitIn = minGridObstacle != null ? (int) minGridObstacle.getMinSize() : (int) gridSizeIn;

        this.robotSize = new RobotSize(robotSize.sqSize/minGridUnitIn, robotSize.height);

        internalObstacleMap = generateObstacles(obstacleMap); // TODO: Expand each obstacle by robot size & add/remove based on height
        Navigator.logObstacleMap(internalObstacleMap);

        gridMax = (int) gridSizeIn / minGridUnitIn;

        RobotLog.dd("NAVIGATOR", "AStar properties : (robotSize: %s gridSizeIn: %s minGridUnitIn: %s gridMax: %s)", robotSize.sqSize, gridSizeIn, minGridUnitIn, gridMax);

        this.trajectoryConfig = trajectoryConfig;
    }

    public AStarNavigator(ObstacleMap obstacleMap, double gridSizeIn, RobotSize robotSize, TrajectoryConfig trajectoryConfig) {
        this(obstacleMap, gridSizeIn, robotSize, trajectoryConfig, 0.5);
    }

    Translation2d fieldToAStarCoords(Translation2d fieldCoords) {
        return new Translation2d((int)((fieldCoords.getX() + gridSizeIn/2)/minGridUnitIn), (int)((-fieldCoords.getY() + gridSizeIn/2)/minGridUnitIn));
    }

    Translation2d aStarToFieldCoords(Translation2d aStarCoords) {
        return new Translation2d((aStarCoords.getX()*minGridUnitIn - gridSizeIn/2) - minGridUnitIn, -(aStarCoords.getY()*minGridUnitIn - gridSizeIn/2) - minGridUnitIn);
    }

    Cell[][] createNewGrid() {
        Cell[][] grid = new Cell[gridMax][gridMax];

        for (int y = 0; y < gridMax; y++) {
            for (int x = 0; x < gridMax; x++) {
                grid[y][x] = new Cell(
                        new Translation2d(x, y)
                );
            }
        }

        return grid;
    }

    public ArrayList<Obstacle> generateObstacles(ObstacleMap obstacleMap) {
        Obstacle[] obstacles = obstacleMap.getObstacles().clone();

        for (Obstacle obstacle : obstacles) {
            obstacle.scale(1. / minGridUnitIn);
            obstacle.expand(robotSize.sqSize*safetyFactor);
            obstacle.setCenter(
                    fieldToAStarCoords(obstacle.getCenter())
            );
        }

        return new ArrayList<>(Arrays.asList(obstacles));
    }


    @Override
    public void addObstacle(Obstacle obstacle) { // TODO: Should we care that this modifies the obstacle object itself? The reason we convert was for convenience of creating ObstacleMap
        obstacle.scale(1. / minGridUnitIn);
        obstacle.expand(robotSize.sqSize*safetyFactor);
        obstacle.setCenter(
                fieldToAStarCoords(obstacle.getCenter())
        );
        internalObstacleMap.add(obstacle);
    }

    Cell getCell(Translation2d pos) {
        return grid[(int) pos.getY()][(int) pos.getX()];
    }

    boolean isOnGrid(Translation2d pos) {
        return pos.getX() >= robotSize.sqSize*safetyFactor && pos.getX() < gridMax - robotSize.sqSize*safetyFactor && pos.getY() >= robotSize.sqSize*safetyFactor && pos.getY() < gridMax - robotSize.sqSize*safetyFactor;
    }

    boolean pointInRect(Translation2d point, Translation2d rectCenter, double rectSizeX, double rectSizeY) {
        return (
            point.getX() < (rectCenter.getX() + Math.ceil(rectSizeX/2)) && point.getX() >= (rectCenter.getX() - Math.ceil(rectSizeX/2)) &&
            point.getY() < (rectCenter.getY() + Math.ceil(rectSizeY/2)) && point.getY() >= (rectCenter.getY() - Math.ceil(rectSizeY/2))
        );
    }

    boolean isBlocked(Translation2d pos) {
        return internalObstacleMap.stream().anyMatch(
                cell -> pointInRect(pos, cell.getCenter(), ((RectangleObstacle) cell).getSizeX(), ((RectangleObstacle) cell).getSizeY())
        );
    }

    double calculateHeuristicValue(Translation2d source, Translation2d target) {
        return source.getDistance(target);
    }

    Translation2d[] tracePath(Translation2d source, Translation2d target) {
        ArrayList<Translation2d> path = new ArrayList<>();

        Translation2d currentPos = target;

        Cell currentCell = getCell(currentPos);

        while (currentCell.parent != source) {
            path.add(currentPos);

            currentCell = getCell(currentPos); // This is technically redundant the first loop
            currentPos = currentCell.parent;
        }

        Collections.reverse(path);

        return path.toArray(new Translation2d[0]);
    }

    ArrayList<Translation2d> directionChanges(Translation2d[] path) {
        if (path.length <= 2) {
            return (ArrayList<Translation2d>) Arrays.asList(path);
        }

        ArrayList<Translation2d> directionChanges = new ArrayList<>();

        int prevDx = (int) (path[1].getX() - path[0].getX());
        int prevDy = (int) (path[1].getY() - path[0].getY());

        int dx, dy;

        for (int i = 2; i < path.length; i++) {
            dx = (int) (path[i].getX() - path[i - 1].getX());
            dy = (int) (path[i].getY() - path[i - 1].getY());

            if (dx != prevDx || dy != prevDy) {
                directionChanges.add(path[i - 1]);
                prevDx = dx;
                prevDy = dy;
            }
        }

        return directionChanges;
    }

    @Override
    public Trajectory search(Translation2d remoteSource, Translation2d remoteTarget) { // TODO: Add orClosest param to find closest unblocked position to path to
        RobotLog.dd("NAVIGATOR", "Remote source : %s", remoteSource);
        RobotLog.dd("NAVIGATOR", "Remote target : %s", remoteTarget);

        Translation2d source = fieldToAStarCoords(remoteSource);
        Translation2d target = fieldToAStarCoords(remoteTarget);

        RobotLog.dd("NAVIGATOR", "Source : %s", source);
        RobotLog.dd("NAVIGATOR", "Target : %s", target);

        if (minGridUnitIn == gridSizeIn) {
            RobotLog.dd("NAVIGATOR", "No obstacles, generate direct remote source to remote target");

            double angleSource = Navigator.angleBetween(remoteSource, remoteTarget);
            double angleTarget = Navigator.angleBetween(remoteTarget, remoteSource);

            Pose2d robotSource = CoordSys.fieldToRobot(new Pose2d(remoteSource.getX(), remoteSource.getY(), new Rotation2d(angleSource)));
            Pose2d robotTarget = CoordSys.fieldToRobot(new Pose2d(remoteTarget.getX(), remoteTarget.getY(), new Rotation2d(angleTarget)));

            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                    robotSource,
                    Collections.emptyList(),
                    robotTarget,
                    trajectoryConfig
            );
            TrajectorySequence.logTrajectory(trajectory);
            return trajectory;
        }

        grid = createNewGrid();

        if (!isOnGrid(source) || !isOnGrid(target) || isBlocked(source) || isBlocked(target)) {
            RobotLog.dd("NAVIGATOR", "Source or target is not on the grid or is blocked!");
            return null;
        }

        Cell startCell = getCell(source);
        startCell.fTotalCostCell = calculateHeuristicValue(source, target);
        startCell.parent = source;

        Set<Cell> openList = new HashSet<>();
        openList.add(startCell);

        Cell currentCell;

        double gNew, hNew, fNew;
        Cell newCell;
        Translation2d newPos;

        Translation2d[] path = new Translation2d[0];

        while (!openList.isEmpty()) {
            currentCell = openList.stream().min(
                    Comparator.comparing((cell) -> cell.fTotalCostCell)
            ).orElse(null);

            openList.remove(currentCell);

            currentCell.closed = true;

            for (int[] dir : DIRECTIONS) {
                newPos = new Translation2d(currentCell.pos.getX() + dir[0], currentCell.pos.getY() + dir[1]);

                if (isOnGrid(newPos)) {
                    newCell = getCell(newPos);
                } else {
                    continue;
                }

                if (newPos.getX() == target.getX() && newPos.getY() == target.getY()) {
                    RobotLog.dd("NAVIGATOR", "Solution found!");

                    newCell.parent = currentCell.pos;

                    path = tracePath(source, target);

                    openList.clear();
                    break;
                }

//                if (isBlocked(newPos))
//                     RobotLog.dd("NAVIGATOR", "%s, %s is blocked", newPos.getX(), newPos.getY());

                if (!isBlocked(newPos) && !newCell.closed) {
                    gNew = currentCell.gCostFromStartToCell + 1;
                    hNew = calculateHeuristicValue(newPos, target);
                    fNew = gNew + hNew;

                    if (newCell.closed || fNew <= newCell.fTotalCostCell) {
                        openList.add(newCell);

                        newCell.gCostFromStartToCell = gNew;
                        newCell.hHeuristicCostFromCellToDest = gNew;
                        newCell.fTotalCostCell = fNew;
                        newCell.parent = currentCell.pos;
                    }
                }
            }
        }

        if (path.length == 0) {
            RobotLog.dd("NAVIGATOR", "No path!");
            return null;
        }

        RobotLog.dd("NAVIGATOR", "Path : %s", Arrays.toString(path));

        ArrayList<Translation2d> aStarWaypoints = directionChanges(path);
        ArrayList<Translation2d> waypoints = new ArrayList<>();

        RobotLog.dd("NAVIGATOR", "Direction changes : %s", aStarWaypoints.toString());

        for (Translation2d pos : aStarWaypoints) {
            waypoints.add(
                CoordSys.fieldToRobot(new Pose2d(aStarToFieldCoords(pos), new Rotation2d())).getTranslation()
            );
        }

        RobotLog.dd("NAVIGATOR", "Waypoints : %s", waypoints.toString());

        double angleSource = Navigator.angleBetween(source, path[1]);
        double angleTarget = Navigator.angleBetween(path[path.length - 2], target);

        Pose2d robotSource = CoordSys.fieldToRobot(new Pose2d(remoteSource.getX(), remoteSource.getY(), new Rotation2d(angleSource)));
        Pose2d robotTarget = CoordSys.fieldToRobot(new Pose2d(remoteTarget.getX(), remoteTarget.getY(), new Rotation2d(angleTarget)));

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                robotSource,
                waypoints,
                robotTarget,
                trajectoryConfig
        );

        TrajectorySequence.logTrajectory(trajectory);

        return trajectory;
    }

    @Override
    public void removeObstacle(Obstacle obstacle) {
        internalObstacleMap.remove(obstacle);
    }

    @Override
    public ArrayList<Obstacle> getObstacles() {
        return internalObstacleMap;
    }
}
