using System;
using System.Collections.Generic;
using System.Drawing;
using System.Security.Cryptography;
using System.Windows.Forms;
using Priority_Queue;

namespace MagniSnap
{
    #region
    /// 4d17639adfad0a300acd78759e07a4f2
    #endregion
    public partial class MainForm : Form
    {
        RGBPixel[,] ImageMatrix;
        bool isLassoEnabled = false;
//-------------------------------------------
    //Extra Variables: 

        // Graph Weights
        double[,] weightRight;
        double[,] weightDown;
        //***** 8-connectivity ***************
        double[,] weightDiagDownRight;
        double[,] weightDiagDownLeft;
        double[,] weightDiagUpRight;
        double[,] weightDiagUpLeft;
        //************************************

        // Dijkstra arrays
        double[,] dist;
        bool[,] visited;
        int[,] prevX;
        int[,] prevY;

        // Anchor point
        int anchorX = -1;
        int anchorY = -1;

        // Path that will be drawn
        List<Point> currentPath = new List<Point>();
//-------------------------------------------
        public MainForm()
        {
            InitializeComponent();
            mainPictureBox.Paint += mainPictureBox_Paint;
            indicator_pnl.Hide();
        }

        private void menuButton_Click(object sender, EventArgs e)
        {
            #region Do Change Remove Template Code
            /// 4d17639adfad0a300acd78759e07a4f2
            #endregion

            indicator_pnl.Top = ((Control)sender).Top;
            indicator_pnl.Height = ((Control)sender).Height;
            indicator_pnl.Left = ((Control)sender).Left;
            ((Control)sender).BackColor = Color.FromArgb(37, 46, 59);
            indicator_pnl.Show();
        }

        private void menuButton_Leave(object sender, EventArgs e)
        {
            ((Control)sender).BackColor = Color.FromArgb(26, 32, 40);
        }

        private void exitToolStripMenuItem_Click(object sender, EventArgs e)
        {         
            Application.Exit();
        }

        private void openToolStripMenuItem_Click(object sender, EventArgs e)
        {
            #region Do Change Remove Template Code
            /// 4d17639adfad0a300acd78759e07a4f2
            #endregion

//~~~~~ Exception (Parameter invalid) fix -> Can reopen large image after small image ~~~~
            if (mainPictureBox.Image != null)
            {
                mainPictureBox.Image.Dispose(); // manually deletes the old image from memory.
                mainPictureBox.Image = null;
            }

            // Clear previous algorithm state
            ImageMatrix = null;
            currentPath.Clear();
            anchorX = -1;
            anchorY = -1;

            dist = null;
            visited = null;
            prevX = null;
            prevY = null;

            weightRight = null;
            weightDown = null;
            weightDiagDownRight = null;
            weightDiagDownLeft = null;
            weightDiagUpRight = null;
            weightDiagUpLeft = null;

            //// Optional but safe for large images
            //GC.Collect();
            //GC.WaitForPendingFinalizers();
            //GC.Collect();

            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {

                //Open the browsed image and display it
                string OpenedFilePath = openFileDialog1.FileName;
                ImageMatrix = ImageToolkit.OpenImage(OpenedFilePath);
                ImageToolkit.ViewImage(ImageMatrix, mainPictureBox);
               
//------------------------------------------------------------------------
                BuildGraphWeights(); //call
//------------------------------------------------------------------------
                int width = ImageToolkit.GetWidth(ImageMatrix);
                txtWidth.Text = width.ToString();
                int height = ImageToolkit.GetHeight(ImageMatrix);
                txtHeight.Text = height.ToString();
            }
        }

        private void clearToolStripMenuItem_Click(object sender, EventArgs e)
        {
            mainPictureBox.Refresh();
        }

        private void btnLivewire_Click(object sender, EventArgs e)
        {
            menuButton_Click(sender, e);

            mainPictureBox.Cursor = Cursors.Cross;

            isLassoEnabled = true;
        }

        private void btnLivewire_Leave(object sender, EventArgs e)
        {
            menuButton_Leave(sender, e);

            mainPictureBox.Cursor = Cursors.Default;
            isLassoEnabled = false;
        }

        private void mainPictureBox_MouseClick(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {

          
                if (ImageMatrix != null && isLassoEnabled)
                {
                    anchorX = e.X;
                    anchorY = e.Y;

                    // Run shortest path tree from anchor
                    RunDijkstra(anchorX, anchorY);

                    // To force a first path draw (small dot)
                    BacktrackPath(anchorX, anchorY);

                    mainPictureBox.Refresh();
                }

            }
        }

        private void mainPictureBox_MouseMove(object sender, MouseEventArgs e)
        {
            txtMousePosX.Text = e.X.ToString();
            txtMousePosY.Text = e.Y.ToString();

        //Replacing startup code: 
            //if (ImageMatrix != null && isLassoEnabled)
            //{
            //    // Refresh to redraw points
            //    mainPictureBox.Refresh();
            //}
        //With: 
            if (ImageMatrix != null && isLassoEnabled && anchorX != -1)
            {
                BacktrackPath(e.X, e.Y);
            }

        }
//-----------------------------------------------------------------------------
         //T1: constructing graph 
        private void BuildGraphWeights()
        {
            int h = ImageToolkit.GetHeight(ImageMatrix);
            int w = ImageToolkit.GetWidth(ImageMatrix);

            // Allocate Dijkstra arrays ONCE per image
            if (dist == null || dist.GetLength(0) != h || dist.GetLength(1) != w)
            {
                dist = new double[h, w];
                visited = new bool[h, w];
                prevX = new int[h, w];
                prevY = new int[h, w];
            }


            weightRight = new double[h, w];
            weightDown = new double[h, w];

            //*********** 8 CONNECTIVITY *************
            weightDiagDownRight = new double[h, w];
            weightDiagDownLeft = new double[h, w];
            weightDiagUpRight = new double[h, w];
            weightDiagUpLeft = new double[h, w];
            //***************************************

            for (int y = 0; y < h; y++)
            {
                for (int x = 0; x < w; x++)
                {
                    Vector2D energy = ImageToolkit.CalculatePixelEnergies(x, y, ImageMatrix);

                    // Scale gradients (your choice to keep) -> remove direction (so it doesnt affect) ; Strong edges → very low cost 
                    double Gx = Math.Abs(energy.X) * 25.0;
                    double Gy = Math.Abs(energy.Y) * 25.0;

                    //********* FIXED DIAGONAL LOGIC (8 CONNECTIVITY) ************
                    // Use REAL diagonal gradient, not (Gx+Gy)/2 -> mathematically incorrect 
                    double Gdiag = Math.Sqrt(Gx * Gx + Gy * Gy); //Calculate hypo

                    
                    double baseRight = 1.0 / (Gx * Gx + 1e-6); //weights + epsilon (handle math error) 
                    double baseDown = 1.0 / (Gy * Gy + 1e-6);

                    // Apply diagonal distance penalty (critical fix)
                    double diagPenalty = 1.41421356; // sqrt(2) -> diagonal penality 

                    double baseDiag = diagPenalty * (1.0 / (Gdiag * Gdiag + 1e-6));

                    // Store diagonal weights
                    weightDiagDownRight[y, x] = baseDiag;
                    weightDiagDownLeft[y, x] = baseDiag;
                    weightDiagUpRight[y, x] = baseDiag;
                    weightDiagUpLeft[y, x] = baseDiag;
                    //*************************************************************

                    // store Horizontal + vertical weights 
                    weightRight[y, x] = baseRight;
                    weightDown[y, x] = baseDown;
                }
            }
        }




 //-------------------------------------------------------------------------
        // T2: Calculating shortest path:
        private void InitializeDijkstra(int anchorX, int anchorY)
        {
            int h = ImageToolkit.GetHeight(ImageMatrix);
            int w = ImageToolkit.GetWidth(ImageMatrix);

          //these four lines allocate huge arrays every click
          //This is what causes OutOfMemoryException
            //dist = new double[h, w];
            //visited = new bool[h, w];
            //prevX = new int[h, w];
            //prevY = new int[h, w];

            // Initialize all distances to infinity
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                {
                    dist[y, x] = double.MaxValue;
                    visited[y, x] = false;
                    prevX[y, x] = -1;
                    prevY[y, x] = -1;
                }

            dist[anchorY, anchorX] = 0; // start point
        }

        private void RunDijkstra(int startX, int startY)   //DIJSKTRA WORKS DP (Horrayyyy!!!)
        {
            InitializeDijkstra(startX, startY);

            int h = ImageToolkit.GetHeight(ImageMatrix);
            int w = ImageToolkit.GetWidth(ImageMatrix);

            //  FastPriorityQueue pq = new FastPriorityQueue();
            
            SimplePriorityQueue<Point, double> pq = new SimplePriorityQueue<Point,double>();
            Point point = new Point(startX, startY);

            pq.Enqueue(point, 0);

            int counter = 0;

            while (pq.Count!=0)
            {
                // Avoid UI freezing
                counter++;
                if (counter % 50000 == 0)
                    Application.DoEvents();  //Without this, the window would appear “Not Responding” 

                Point node = pq.Dequeue();

                int x = node.X;
                int y = node.Y;

                // If already finalized → skip
                if (visited[y, x]) continue;
                visited[y, x] = true;

                double baseDist = dist[y, x];

                // RIGHT
                if (x + 1 < w && !visited[y, x + 1])
                {
                    double wght = weightRight[y, x];
                    double newDist = baseDist + wght;

                    if (newDist < dist[y, x + 1])
                    {
                        dist[y, x + 1] = newDist;
                        prevX[y, x + 1] = x;
                        prevY[y, x + 1] = y;

                        Point pointR = new Point(x+1, y);
                       
                        pq.Enqueue(pointR,newDist);
                    }
                }

                // LEFT
                if (x - 1 >= 0 && !visited[y, x - 1])
                {
                    double wght = weightRight[y, x - 1];
                    double newDist = baseDist + wght;

                    if (newDist < dist[y, x - 1])
                    {
                        dist[y, x - 1] = newDist;
                        prevX[y, x - 1] = x;
                        prevY[y, x - 1] = y;

                        Point pointL = new Point(x - 1, y);
                        pq.Enqueue(pointL, newDist);
                    }
                }

                // DOWN
                if (y + 1 < h && !visited[y + 1, x])
                {
                    double wght = weightDown[y, x];
                    double newDist = baseDist + wght;

                    if (newDist < dist[y + 1, x])
                    {
                        dist[y + 1, x] = newDist;
                        prevX[y + 1, x] = x;
                        prevY[y + 1, x] = y;
                        Point pointD = new Point(x, y+1);
                        pq.Enqueue(pointD, newDist);
                    }
                }

                // UP
                if (y - 1 >= 0 && !visited[y - 1, x])
                {
                    double wght = weightDown[y - 1, x];
                    double newDist = baseDist + wght;

                    if (newDist < dist[y - 1, x])
                    {
                        dist[y - 1, x] = newDist;
                        prevX[y - 1, x] = x;
                        prevY[y - 1, x] = y;
                        Point pointU = new Point(x, y-1);
                        pq.Enqueue(pointU, newDist);
                    }
                }
            //*********** 8 CONNECTIVITY ******************************
                // DOWN-RIGHT (x+1, y+1)
                if (x + 1 < w && y + 1 < h && !visited[y + 1, x + 1])
                {
                    double newDist = baseDist + weightDiagDownRight[y, x];
                    if (newDist < dist[y + 1, x + 1])
                    {
                        dist[y + 1, x + 1] = newDist;
                        prevX[y + 1, x + 1] = x;
                        prevY[y + 1, x + 1] = y;
                        Point pointDR = new Point(x + 1, y+1);
                        pq.Enqueue(pointDR, newDist);
                    }
                }

                // DOWN-LEFT (x-1, y+1)
                if (x - 1 >= 0 && y + 1 < h && !visited[y + 1, x - 1])
                {
                    double newDist = baseDist + weightDiagDownLeft[y, x];
                    if (newDist < dist[y + 1, x - 1])
                    {
                        dist[y + 1, x - 1] = newDist;
                        prevX[y + 1, x - 1] = x;
                        prevY[y + 1, x - 1] = y;

                        Point pointDL = new Point(x - 1, y+1);
                        pq.Enqueue(pointDL, newDist);
                    }
                }

                // UP-RIGHT (x+1, y-1)
                if (x + 1 < w && y - 1 >= 0 && !visited[y - 1, x + 1])
                {
                    double newDist = baseDist + weightDiagUpRight[y, x];
                    if (newDist < dist[y - 1, x + 1])
                    {
                        dist[y - 1, x + 1] = newDist;
                        prevX[y - 1, x + 1] = x;
                        prevY[y - 1, x + 1] = y;
                        Point pointUR = new Point(x + 1, y-1);
                        pq.Enqueue(pointUR, newDist);
                    }
                }

                // UP-LEFT (x-1, y-1)
                if (x - 1 >= 0 && y - 1 >= 0 && !visited[y - 1, x - 1])
                {
                    double newDist = baseDist + weightDiagUpLeft[y, x];
                    if (newDist < dist[y - 1, x - 1])
                    {
                        dist[y - 1, x - 1] = newDist;
                        prevX[y - 1, x - 1] = x;
                        prevY[y - 1, x - 1] = y;
                        Point pointUL = new Point(x - 1, y-1);
                        pq.Enqueue(pointUL, newDist);
                    }
                }


            }
        }

//---------------------------------------------------------------------------------
        //T3: Back track shortest path: 

        private void BacktrackPath(int mouseX, int mouseY)
        {
            currentPath.Clear();

            int x = mouseX;
            int y = mouseY;

            // Ensure mouse position is inside image
            if (x < 0 || y < 0 || x >= ImageToolkit.GetWidth(ImageMatrix) || y >= ImageToolkit.GetHeight(ImageMatrix))
                return;

            // Follow prev pointers until anchor or invalid
            while (x != -1 && y != -1)
            {
                currentPath.Add(new Point(x, y));

                if (x == anchorX && y == anchorY)
                    break;

                int px = prevX[y, x];
                int py = prevY[y, x];

                x = px;
                y = py;
            }
            
            // After storing the path → refresh to draw it
            mainPictureBox.Refresh();
        }
//----------------------------------------------------------------------------------
        // T4: Draw Path
        private void mainPictureBox_Paint(object sender, PaintEventArgs e)
        {
            if (currentPath.Count > 1)
            {
                using (Pen pen = new Pen(Color.Yellow, 2))
                {
                    for (int i = 1; i < currentPath.Count; i++)
                    {
                        Point p1 = currentPath[i - 1];
                        Point p2 = currentPath[i];

                        e.Graphics.DrawLine(pen, p1, p2);
                    }
                }
            }
        }


    }

//------------------------------------------------------------------------
    //// Minimal priority queue for (distance, x, y)
    //class PixelNode : IComparable<PixelNode>
    //{
    //    public double dist;
    //    public int x, y;

    //    public int CompareTo(PixelNode other)
    //    {
    //        return dist.CompareTo(other.dist);
    //    }
    //}

    //class FastPriorityQueue
    //{
    //    private List<PixelNode> heap = new List<PixelNode>();

    //    public void Enqueue(double d, int x, int y)
    //    {
    //        heap.Add(new PixelNode { dist = d, x = x, y = y });
    //        HeapifyUp(heap.Count - 1);
    //    }

    //    public PixelNode Dequeue()
    //    {
    //        PixelNode root = heap[0];
    //        heap[0] = heap[heap.Count - 1];
    //        heap.RemoveAt(heap.Count - 1);
    //        HeapifyDown(0);
    //        return root;
    //    }

    //    public bool IsEmpty() => heap.Count == 0;

    //    private void HeapifyUp(int i)
    //    {
    //        while (i > 0)
    //        {
    //            int parent = (i - 1) / 2;
    //            if (heap[i].dist >= heap[parent].dist) break;

    //            (heap[i], heap[parent]) = (heap[parent], heap[i]);
    //            i = parent;
    //        }
    //    }

    //    private void HeapifyDown(int i)
    //    {
    //        int left, right, smallest;

    //        while (true)
    //        {
    //            left = 2 * i + 1;
    //            right = 2 * i + 2;
    //            smallest = i;

    //            if (left < heap.Count && heap[left].dist < heap[smallest].dist)
    //                smallest = left;

    //            if (right < heap.Count && heap[right].dist < heap[smallest].dist)
    //                smallest = right;

    //            if (smallest == i) break;

    //            (heap[i], heap[smallest]) = (heap[smallest], heap[i]);
    //            i = smallest;
    //        }
    //    }
    //}
//----------------------------------------------------------------------------------
// HEHE <3 :)
}
