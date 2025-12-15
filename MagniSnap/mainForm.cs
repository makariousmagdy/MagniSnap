using System;
using System.Collections.Generic;
using System.Drawing;
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
        double[,] weightDiagDR; //DR: Down Right
        double[,] weightDiagDL; //DL: Down Left
        double[,] weightDiagUR; //UR: Up Right
        double[,] weightDiagUL; //UL: Up Left
       
        //************************************

        // Dijkstra arrays
        double[,] shortestPath; //stores the shortest path cost from the anchor to each pixel.
        bool[,] visitedPixel; //mark pixels that have been finalized by dijsktra.
        int[,] parentX; //Previous X pixel
        int[,] parentY; //Pevious Y pixel

        // Anchor point
        int anchorX = -1;
        int anchorY = -1;

        

        // Path that will be drawn
        List<Point> currentPath = new List<Point>();

        // all Paths drawn before the new anchor point (MULTIPLE ANCHOR)
        List<List<Point>> wholePath = new List<List<Point>>();
        //-------------------------------------------
        public MainForm()
        {
            InitializeComponent();
            mainPictureBox.Paint += DrawPath;
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

            //~~~~~ Exception (Parameter invalid - container issue) fix -> Can reopen large image after small image ~~~~
            if (mainPictureBox.Image != null)
            {
                mainPictureBox.Image.Dispose(); // manually deletes the old image from memory.
                mainPictureBox.Image = null;
            }

            // Clear previous algorithm state
            ImageMatrix = null;
            currentPath.Clear(); 
            wholePath.Clear(); //MULTIPLE ANCHOR
            anchorX = -1; 
            anchorY = -1;

            shortestPath = null;
            visitedPixel = null;
            parentX = null;
            parentY = null;

            weightRight = null;
            weightDown = null;
            weightDiagDR = null;
            weightDiagDL = null;
            weightDiagUR = null;
            weightDiagUL = null;

            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            OpenFileDialog openFileDialog1 = new OpenFileDialog();
            if (openFileDialog1.ShowDialog() == DialogResult.OK)
            {

                //Open the browsed image and display it
                string OpenedFilePath = openFileDialog1.FileName;
                ImageMatrix = ImageToolkit.OpenImage(OpenedFilePath);
                ImageToolkit.ViewImage(ImageMatrix, mainPictureBox);

                //------------------------------------------------------------------------
                ConstructGraph(); //call
                //------------------------------------------------------------------------
                int width = ImageToolkit.GetWidth(ImageMatrix);
                txtWidth.Text = width.ToString();
                int height = ImageToolkit.GetHeight(ImageMatrix);
                txtHeight.Text = height.ToString();
            }
        }

 //############################################################
        private void clearToolStripMenuItem_Click(object sender, EventArgs e)
        {
            //Clear whole livewire drawings ->   menu option
            currentPath.Clear();
            wholePath.Clear(); //Clear Multiple Anchors
            anchorX = -1;
            anchorY = -1;
            mainPictureBox.Refresh();

        }

        //NEW FUNCTION 
        private void cleanToolStripMenuItemClickToolStripMenuItem_Click(object sender, EventArgs e)
        {
            //Clean extra livewire drawing ->   menu option
            currentPath.Clear();
            anchorX = -1;
            anchorY = -1;
            mainPictureBox.Refresh();
        }
//############################################################

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

                // Save previous path (MULTIPLE ANCHOR)
                wholePath.Add(new List<Point>(currentPath));

                if (ImageMatrix != null && isLassoEnabled)
                {
                    anchorX = e.X;
                    anchorY = e.Y;

                    // Run dijkstra from anchor
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
        //T1: constructing undirected weighted graph 
        private void ConstructGraph()
        {
            int h = ImageToolkit.GetHeight(ImageMatrix);
            int w = ImageToolkit.GetWidth(ImageMatrix);

            // Allocate Dijkstra arrays ONCE per image ->To avoid filling the memory
            if (shortestPath == null || shortestPath.GetLength(0) != h || shortestPath.GetLength(1) != w)
            {
                shortestPath = new double[h, w];
                visitedPixel = new bool[h, w];
                parentX = new int[h, w];
                parentY = new int[h, w];
            }


            weightRight = new double[h, w];
            weightDown = new double[h, w];

            //*********** 8 CONNECTIVITY *************
            weightDiagDR = new double[h, w];
            weightDiagDL = new double[h, w];
            weightDiagUR = new double[h, w];
            weightDiagUL = new double[h, w];
            //***************************************

            for (int y = 0; y < h; y++)
            {
                for (int x = 0; x < w; x++)
                {
                    Vector2D energy = ImageToolkit.CalculatePixelEnergies(x, y, ImageMatrix);

                    // Absolute to remove direction (so it doesnt affect) ; Strong edges → very low cost 
                    double edgeStrengthX = Math.Abs(energy.X) * 25.0;
                    double edgeStrengthY = Math.Abs(energy.Y) * 25.0;

                    //********* (8 CONNECTIVITY) ************
                    // Using REAL diagonal gradient, not : (edgeStrengthX+edgeStrengthY)/2 -> mathematically incorrect 
                    double edgeStrengthDiag = Math.Sqrt(edgeStrengthX * edgeStrengthX + edgeStrengthY * edgeStrengthY); //Calculate hypo


                    double costRight = 1.0 / (edgeStrengthX * edgeStrengthX + 1e-6); // "+ epsilon" (handle division by zero) 
                    double costDown = 1.0 / (edgeStrengthY * edgeStrengthY + 1e-6);

                    // Apply diagonal distance penalty (increase diag cost, so diajkstra avoids)
                    double diagPenalty = 1.41421356; // sqrt(2) -> diagonal penality 

                    double costDiag = diagPenalty * (1.0 / (edgeStrengthDiag * edgeStrengthDiag + 1e-6));

                    // Store diagonal weights
                    weightDiagDR[y, x] = costDiag;
                    weightDiagDL[y, x] = costDiag;
                    weightDiagUR[y, x] = costDiag;
                    weightDiagUL[y, x] = costDiag;
                    //*************************************************************

                    // store Horizontal + vertical weights 
                    weightRight[y, x] = costRight;
                    weightDown[y, x] = costDown;
                }
            }
        }




        //-------------------------------------------------------------------------
        // T2: Calculating shortest path:
        private void InitializeDijkstra(int anchorX, int anchorY)
        {
            int h = ImageToolkit.GetHeight(ImageMatrix);
            int w = ImageToolkit.GetWidth(ImageMatrix);


            // Initialize all distances to infinity -> Because before running Dijkstra no pixel is reachable.
            for (int y = 0; y < h; y++)
                for (int x = 0; x < w; x++)
                {
                    shortestPath[y, x] = double.MaxValue;
                    visitedPixel[y, x] = false;
                    parentX[y, x] = -1;
                    parentY[y, x] = -1;
                }

            shortestPath[anchorY, anchorX] = 0; // start point
        }

        private void RunDijkstra(int firstX, int firstY)   //DIJSKTRA WORKS DP (Horrayyyy!!!)
        {
            InitializeDijkstra(firstX, firstY);

            int height = ImageToolkit.GetHeight(ImageMatrix);
            int width = ImageToolkit.GetWidth(ImageMatrix);

            //  FastPriorityQueue priorityQ = new FastPriorityQueue();

            SimplePriorityQueue<Point, double> priorityQ = new SimplePriorityQueue<Point, double>();
            Point point = new Point(firstX, firstY);

            priorityQ.Enqueue(point, 0);

            int counter = 0;

            while (priorityQ.Count != 0)
            {
                // Avoid UI freezing
                counter++;
                if (counter % 50000 == 0)
                    Application.DoEvents();  //Without this, the window would appear “Not Responding” 

                Point currentPixel = priorityQ.Dequeue();

                int x = currentPixel.X;
                int y = currentPixel.Y;

                // If already finalized → skip
                if (visitedPixel[y, x]) 
                    continue;
                visitedPixel[y, x] = true;

                double costDist = shortestPath[y, x];

                // RIGHT (R)
                if (x + 1 < width && !visitedPixel[y, x + 1])
                {
                    double weight = weightRight[y, x];
                    double newDist = costDist + weight;

                    if (newDist < shortestPath[y, x + 1])
                    {
                        shortestPath[y, x + 1] = newDist;
                        parentX[y, x + 1] = x;
                        parentY[y, x + 1] = y;

                        Point pointR = new Point(x + 1, y);

                        priorityQ.Enqueue(pointR, newDist);
                    }
                }

                // LEFT (L)
                if (x - 1 >= 0 && !visitedPixel[y, x - 1])
                {
                    double weight = weightRight[y, x - 1];
                    double newDist = costDist + weight;

                    if (newDist < shortestPath[y, x - 1])
                    {
                        shortestPath[y, x - 1] = newDist;
                        parentX[y, x - 1] = x;
                        parentY[y, x - 1] = y;

                        Point pointL = new Point(x - 1, y);
                        priorityQ.Enqueue(pointL, newDist);
                    }
                }

                // DOWN (D)
                if (y + 1 < height && !visitedPixel[y + 1, x])
                {
                    double weight = weightDown[y, x];
                    double newDist = costDist + weight;

                    if (newDist < shortestPath[y + 1, x])
                    {
                        shortestPath[y + 1, x] = newDist;
                        parentX[y + 1, x] = x;
                        parentY[y + 1, x] = y;
                        Point pointD = new Point(x, y + 1);
                        priorityQ.Enqueue(pointD, newDist);
                    }
                }

                // UP (U)
                if (y - 1 >= 0 && !visitedPixel[y - 1, x])
                {
                    double weight = weightDown[y - 1, x];
                    double newDist = costDist + weight;

                    if (newDist < shortestPath[y - 1, x])
                    {
                        shortestPath[y - 1, x] = newDist;
                        parentX[y - 1, x] = x;
                        parentY[y - 1, x] = y;
                        Point pointU = new Point(x, y - 1);
                        priorityQ.Enqueue(pointU, newDist);
                    }
                }
                //*********** 8 CONNECTIVITY ******************************
                // DOWN-RIGHT DR (x+1, y+1)
                if (x + 1 < width && y + 1 < height && !visitedPixel[y + 1, x + 1])
                {
                    double newDist = costDist + weightDiagDR[y, x];
                    if (newDist < shortestPath[y + 1, x + 1])
                    {
                        shortestPath[y + 1, x + 1] = newDist;
                        parentX[y + 1, x + 1] = x;
                        parentY[y + 1, x + 1] = y;
                        Point pointDR = new Point(x + 1, y + 1);
                        priorityQ.Enqueue(pointDR, newDist);
                    }
                }

                // DOWN-LEFT DL (x-1, y+1)
                if (x - 1 >= 0 && y + 1 < height && !visitedPixel[y + 1, x - 1])
                {
                    double newDist = costDist + weightDiagDL[y, x];
                    if (newDist < shortestPath[y + 1, x - 1])
                    {
                        shortestPath[y + 1, x - 1] = newDist;
                        parentX[y + 1, x - 1] = x;
                        parentY[y + 1, x - 1] = y;

                        Point pointDL = new Point(x - 1, y + 1);
                        priorityQ.Enqueue(pointDL, newDist);
                    }
                }

                // UP-RIGHT (x+1, y-1)
                if (x + 1 < width && y - 1 >= 0 && !visitedPixel[y - 1, x + 1])
                {
                    double newDist = costDist + weightDiagUR[y, x];
                    if (newDist < shortestPath[y - 1, x + 1])
                    {
                        shortestPath[y - 1, x + 1] = newDist;
                        parentX[y - 1, x + 1] = x;
                        parentY[y - 1, x + 1] = y;
                        Point pointUR = new Point(x + 1, y - 1);
                        priorityQ.Enqueue(pointUR, newDist);
                    }
                }

                // UP-LEFT (x-1, y-1)
                if (x - 1 >= 0 && y - 1 >= 0 && !visitedPixel[y - 1, x - 1])
                {
                    double newDist = costDist + weightDiagUL[y, x];
                    if (newDist < shortestPath[y - 1, x - 1])
                    {
                        shortestPath[y - 1, x - 1] = newDist;
                        parentX[y - 1, x - 1] = x;
                        parentY[y - 1, x - 1] = y;
                        Point pointUL = new Point(x - 1, y - 1);
                        priorityQ.Enqueue(pointUL, newDist);
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

                int px = parentX[y, x];
                int py = parentY[y, x];

                x = px;
                y = py;
            }

            // After storing the path → refresh to draw it
            mainPictureBox.Refresh();
        }
        //----------------------------------------------------------------------------------
        // T4: Draw Path

        private void DrawPath(object sender, PaintEventArgs e)
        {
            using (Pen pen = new Pen(Color.Yellow, 2))
            {
                // Draw all stored Path when using Multi Anchor 
                foreach (var line in wholePath)
                {
                    for (int i = 1; i < line.Count; i++)
                    {
                        e.Graphics.DrawLine(pen, line[i - 1], line[i]);
                    }
                }
                if (currentPath.Count > 1) 
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
}
// HEHE <3 :)

