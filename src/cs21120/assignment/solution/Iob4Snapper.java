/*
 * Iob4Snapper implements the dikstra algorithm to find the path on an image and
 * draw the line according to the weights in a map. The drawing line follows
 * the next shortest pixel of the each pixel. The weights of the images are
 * according to the pictures's contrast. 
 * It should work like that: click inside the image and drag the mouse. the 
 * line should change direction following the mouse and estimating the 
 * shortest path to the end point of the pointer. Realesing the mouse it should 
 * stop drawing. Clicking to a new point it starts a new thread with the same 
 * process from the beggining. When the pointer goes out of the boundaries and 
 * returns inside again the line continue drawing, otherwise the app returns 
 * null for the path and this causes it to hang. 
 * I suppose that the program should be avoiding draw to the blank pixel. This 
 * is not implemented. For example you can draw lines on the white space if you
 * click and draw the mouse on the background. Draging over the subject of the
 * image has a small effect on the line's route. I think could be solved ignoring the 
 * pixels out of the subject and setting a point value such that it could be checked
 * inside the getPath.
 * There is another issue when you use auto-small.png. The start point can be 
 * anywhere and it does not count whether starts out of the pictures' boundaries.
 */
package cs21120.assignment.solution;

import cs21120.assignment2.FloatImage;
import cs21120.assignment2.ISnapper;
import java.awt.Point;
import java.util.LinkedList;
import java.util.concurrent.PriorityBlockingQueue;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Iob4Snapper implements the ISnapper and Runnable Interfaces.
 * The PriorityBlockingQueue<PixelEdges> edges: is used to create the graph
 * of the algorithm
 * 
 * boolean visited[][]: tracks the nodes that have been vitited.
 * 
 * Point map[][]: create the map for the line
 * 
 * Thread t: include the main function of Dikstra
 * 
 * int sizex, sizey: image's width and height respectively
 * 
 * FloatImage[] fimg: store the floatImage array to use it outside of the setSeed
 * 
 * The class includes and implements two overrided function from ISnapper, one
 * the run function from Runnable and one inner support class which represents
 * the edges of pixels.
 * @author iob4
 */
public class Iob4Snapper implements ISnapper, Runnable {

    private PriorityBlockingQueue<PixelEdges> edges;
    private boolean visited[][];
    private Point map[][];
    
    private Thread t;
    private int sizex = 0;
    private int sizey = 0;

    private FloatImage[] fimg;
    private float weight;
    private Point source;

    /**
     * setSeed initializes the needed values for the dikstra algorithm. 
     * Create a new Point with x=i, y=i1 and define the position of this as 
     * visited and call the getNeighbors to find all the neighbors of that point.
     * The initial weight for each neighbors is set to zero.
     * When all the variable are initialized it calls the thread.
     * 
     * @param i the x value of the selected Point
     * @param i1 the y value of selected Point
     * @param fis gives an array with the East, SouthEast, South and SouthWest weight of the (i,i1)
     */
    @Override
    public void setSeed(int i, int i1, FloatImage[] fis) {

        this.edges = new PriorityBlockingQueue<PixelEdges>();
        this.sizex = fis[0].getWidth();
        this.sizey = fis[0].getHeight();

        this.visited = new boolean[this.sizex][this.sizey];

        this.source = new Point(i, i1);
        this.map = new Point[sizex][sizey];

        this.fimg = fis;
        this.getNeighbors(this.source, 0, fimg, this.edges);

        this.visited[i][i1] = true;
        t = new Thread(this);
        t.start();
        try {
            t.join();
        } catch (InterruptedException ex) {
            Logger.getLogger(Iob4Snapper.class.getName()).log(Level.SEVERE, null, ex);
        }

    }

    /**
     * 
     * @param i the x value of the end Point
     * @param i1 the y value of end Point
     * @return LinkedList which represents the shortest path
     */
    @Override
    public LinkedList<Point> getPath(int i, int i1) {
        
        LinkedList<Point> route = new LinkedList<Point>();

        Point p = new Point(i, i1);

        while (p != null) {
            if (p.x > 0 && p.x < this.sizex - 1) {
                if (p.y > 0 && p.y < this.sizey - 1) {
                    route.add(p);
                    p = map[p.x][p.y];
                } else {
                    p=null;

                }
            } else {
                p=null;

            }
        }

        return route;
    }

    /**
     * The main loop of dikstra algorithm. Remove the first element of the 
     * list and became the current element of the algorithm.
     * if the end point of the edge has not been visited is added to the map 
     * and finds their neighbors calculated the new weights for each one and adds
     * them to the priority queue.
     */
    @Override
    public void run() {

        while (!this.edges.isEmpty()) {

            PixelEdges minedge = this.edges.remove();
            Point end = new Point(minedge.getEndPoint());

            this.weight = minedge.getWeight();
            if (!visited[end.x][end.y]) {
                visited[end.x][end.y] = true;
                this.map[end.x][end.y] = minedge.getStartPoint();

                this.getNeighbors(end, this.weight, fimg, this.edges);

            }
        }
    }

    /**
     * for each of the eight neighbors there is a condition to check whether 
     * the point is at the boundaries of the image and if the condition is true
     * create a new PixelEdges with start point the given point in parameters and 
     * the end point to the respective direction. 
     * 
     * @param p the point that need to find the neighbors
     * @param w the previous weight to add to the next nodes(pixels)
     * @param fis the array with the known weights
     * @param pq the priority queue where the neighbors are added to used in the next loop looking building the map
     */
    private void getNeighbors(Point p, float w, FloatImage[] fis, PriorityBlockingQueue<PixelEdges> pq) {
        //nw
        if (p.x > 0 && p.y > 0) {
            pq.add(new PixelEdges(p, new Point(p.x - 1, p.y - 1), w + fis[3].get_nocheck(p.x, p.y)));
        }
        if (p.y > 0) {
            pq.add(new PixelEdges(p, new Point(p.x, p.y - 1), w + fis[2].get_nocheck(p.x, p.y)));
        } //North
        if (p.x < sizex - 1 && p.y > 0) {
            pq.add(new PixelEdges(p, new Point(p.x + 1, p.y - 1), w + fis[1].get_nocheck(p.x, p.y)));
        }//North-East
        if (p.x > 0) {
            pq.add(new PixelEdges(p, new Point(p.x - 1, p.y), w + fis[0].get_nocheck(p.x - 1, p.y)));
        } //West
        if (p.x < sizex - 1) {
            pq.add(new PixelEdges(p, new Point(p.x + 1, p.y), w + fis[0].get_nocheck(p.x, p.y)));
        }//East
        if (p.x > 0 && p.y < sizey - 1) {
            pq.add(new PixelEdges(p, new Point(p.x - 1, p.y + 1), w + fis[1].get_nocheck(p.x - 1, p.y + 1)));
        }//South West
        if (p.y < sizey - 1) {
            pq.add(new PixelEdges(p, new Point(p.x, p.y + 1), w + fis[2].get_nocheck(p.x, p.y + 1)));
        }//South
        if (p.x < sizex - 1 && p.y < sizey - 1) {
            pq.add(new PixelEdges(p, new Point(p.x + 1, p.y + 1), w + fis[3].get_nocheck(p.x + 1, p.y + 1)));
        }//South East

    }

    /**
     * PixelEdges with a start, end point and a weight between these two. The class 
     * is probably more extensive than what it actually needs. it include two 
     * constructors and all the getters and setter for the variables.
     */
    class PixelEdges implements Comparable {

        private Point startPoint;
        private Point endPoint;
        private float weight;

        public PixelEdges(Point startPoint, Point endPoint, float weight) {
            this.startPoint = startPoint;
            this.endPoint = endPoint;
            this.weight = weight;
        }

        public PixelEdges(Point startPoint, Point endPoint) {
            this.startPoint = startPoint;
            this.endPoint = endPoint;
            this.weight = 0;
        }

        public Point getStartPoint() {
            return startPoint;
        }

        public void setStartPoint(Point startPoint) {
            this.startPoint = startPoint;
        }

        public Point getEndPoint() {
            return endPoint;
        }

        public void setEndPoint(Point endPoint) {
            this.endPoint = endPoint;
        }

        public float getWeight() {
            return weight;
        }

        public void setWeight(float weight) {
            this.weight = weight;
        }

        /**
         * 
         * @param o another PixelEdges
         * @return 0 if equal, 1 if the weight of this object is bigger otherwise -1
         */
        @Override
        public int compareTo(Object o) {
            PixelEdges e = (PixelEdges) o;
            if (e.weight > weight) {
                return -1;
            } else if (e.weight < weight) {
                return 1;
            }
            return 0;
        }

    }
}
