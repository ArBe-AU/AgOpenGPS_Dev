using OpenTK;
using OpenTK.Graphics.OpenGL;
using System;
using System.Drawing;
using System.Windows.Forms;

namespace AgOpenGPS
{
    public partial class FormSelf : Form
    {
        //access to the main GPS form and all its variables
        private readonly FormGPS mf = null;

        readonly private double gain = 1;
        private Point fixPt;

        private bool isA = true, isB = false, isC = false;
        public vec2 lastABLineP1 = new vec2(0.0, 0.0);

        private void btnExit_Click(object sender, EventArgs e)
        {
            Close();
        }

        private void btnMaze_Click(object sender, EventArgs e)
        {
            mf.self.SolveMaze();
        }

        private void FormSelf_Load(object sender, EventArgs e)
        {
            double mazeY = (mf.maxFieldY - mf.minFieldY) / 2;
            double mazeX = (mf.maxFieldX - mf.minFieldX) / 2;

            mf.self.MakeMaze((int)(mazeY), (int)mazeX);

            lblPasses.Text = "Start";
            int cnt = mf.self.cellDecList.Count;
            tbox1.Text = "";
            for (int i = 0; i < cnt; i++)
            {
                tbox1.Text += i + "\t" + mf.self.cellDecList[i].lines + "\t" + mf.self.cellDecList[i].cells + "\r\n";
            }
        }

        public vec2 lastABLineP2 = new vec2(0.0, 1.0);

        private void oglSelf_MouseDown(object sender, MouseEventArgs e)
        {
            Point pt = oglSelf.PointToClient(Cursor.Position);

            //Convert to Origin in the center of window, 800 pixels
            fixPt.X = pt.X - 400;
            fixPt.Y = (800 - pt.Y - 400);
            vec3 plotPt = new vec3
            {
                //convert screen coordinates to field coordinates
                easting = ((double)fixPt.X) * (double)mf.maxFieldDistance / 723.0 * gain,
                northing = ((double)fixPt.Y) * (double)mf.maxFieldDistance / 723.0 * gain,
                heading = 0
            };

            plotPt.easting += mf.fieldCenterX;
            plotPt.northing += mf.fieldCenterY;

            lblX.Text = plotPt.easting.ToString();
            lblY.Text = plotPt.northing.ToString();

            mf.self.pint1.easting = plotPt.easting;
            mf.self.pint1.northing = plotPt.northing;

            if (isA)
            {
                mf.self.linePtList?.Clear();
                mf.self.lineList?.Clear();

                mf.ABLine.ResetABLine();

                lastABLineP1.easting = 9999; lastABLineP1.northing = 9999;
                lastABLineP2.easting = 9999; lastABLineP2.northing = 9999;

                mf.ABLine.refPoint1.easting = plotPt.easting;
                mf.ABLine.refPoint1.northing = plotPt.northing;

                mf.self.pint2.easting = plotPt.easting;
                mf.self.pint2.northing = plotPt.northing;

                isA = false;
                isB = true;
            }
            else if (isB)
            {
                mf.ABLine.refPoint2.easting = plotPt.easting;
                mf.ABLine.refPoint2.northing = plotPt.northing;
                isA = true;
                isB = false;
                //isC = true;

                //calculate the AB Heading
                mf.ABLine.abHeading = Math.Atan2(mf.ABLine.refPoint2.easting - mf.ABLine.refPoint1.easting, mf.ABLine.refPoint2.northing - mf.ABLine.refPoint1.northing);
                if (mf.ABLine.abHeading < 0) mf.ABLine.abHeading += glm.twoPI;

                //sin x cos y for endpoints, opposite for additional lines
                mf.ABLine.refABLineP1.easting = mf.ABLine.refPoint1.easting - (Math.Sin(mf.ABLine.abHeading) * 1500.0);
                mf.ABLine.refABLineP1.northing = mf.ABLine.refPoint1.northing - (Math.Cos(mf.ABLine.abHeading) * 1500.0);

                mf.ABLine.refABLineP2.easting = mf.ABLine.refPoint1.easting + (Math.Sin(mf.ABLine.abHeading) * 1500.0);
                mf.ABLine.refABLineP2.northing = mf.ABLine.refPoint1.northing + (Math.Cos(mf.ABLine.abHeading) * 1500.0);

                mf.ABLine.isABLineSet = true;
                mf.FileSaveABLine();
                mf.self.SolveMaze();
            }

            //borrowed snap code for last line of field
            else if (isC)
            {
                //move the ABLine over based on the overlap amount set in vehicle
                double widthMinusOverlap = mf.vehicle.toolWidth - mf.vehicle.toolOverlap;

                //x2-x1
                double dx = mf.ABLine.refABLineP2.easting - mf.ABLine.refABLineP1.easting;
                //z2-z1
                double dy = mf.ABLine.refABLineP2.northing - mf.ABLine.refABLineP1.northing;

                //how far are we away from the reference line at 90 degrees
                double distanceFromRefLine = ((dy * plotPt.easting) - (dx * plotPt.northing)
                    + (mf.ABLine.refABLineP2.easting * mf.ABLine.refABLineP1.northing)
                    - (mf.ABLine.refABLineP2.northing * mf.ABLine.refABLineP1.easting))
                                            / Math.Sqrt((dy * dy) + (dx * dx));

                //sign of distance determines which side of line we are on
                int refLineSide;
                if (distanceFromRefLine > 0) refLineSide = 1;
                else refLineSide = -1;

                //absolute the distance
                distanceFromRefLine = Math.Abs(distanceFromRefLine);

                //Which ABLine is the vehicle on, negative is left and positive is right side
                double howManyPathsAway = Math.Round(distanceFromRefLine / widthMinusOverlap, 0, MidpointRounding.AwayFromZero);

                //generate that pass number as signed integer
                mf.self.lastPassNumber = Convert.ToInt32(refLineSide * howManyPathsAway);

                //calculate the new point that is number of implement widths over
                double toolOffset = mf.vehicle.toolOffset;
                vec2 point1 = new vec2((Math.Cos(-mf.ABLine.abHeading) * ((widthMinusOverlap * howManyPathsAway * refLineSide)
                                        - toolOffset)) + mf.ABLine.refPoint1.easting,
                                    (Math.Sin(-mf.ABLine.abHeading) * ((widthMinusOverlap * howManyPathsAway * refLineSide)
                                        - toolOffset)) + mf.ABLine.refPoint1.northing);

                //create the new line extent points for current ABLine based on original heading of AB line
                lastABLineP1.easting = point1.easting - (Math.Sin(mf.ABLine.abHeading) * 1500.0);
                lastABLineP1.northing = point1.northing - (Math.Cos(mf.ABLine.abHeading) * 1500.0);

                lastABLineP2.easting = point1.easting + (Math.Sin(mf.ABLine.abHeading) * 1500.0);
                lastABLineP2.northing = point1.northing + (Math.Cos(mf.ABLine.abHeading) * 1500.0);

                //isC = false;
                //isA = true;
                lblPasses.Text = mf.self.lastPassNumber.ToString();

                //build the lines, determine bounds
                //mf.self.BuildLines();

                int cnt = mf.self.cellDecList.Count;
                tbox1.Text = "";
                for (int i = 0; i < cnt; i++)
                {
                    tbox1.Text += i + "\t" + mf.self.cellDecList[i].lines + "\t" + mf.self.cellDecList[i].cells + "\r\n";
                }
            }
        }

        private void oglSelf_Paint(object sender, PaintEventArgs e)
        {
            oglSelf.MakeCurrent();

            GL.Clear(ClearBufferMask.DepthBufferBit | ClearBufferMask.ColorBufferBit);
            GL.LoadIdentity();                  // Reset The View

            //if (!isCalcMinMax)
            //{
            //    CalculateMinMax();
            //    mf.self.CalculateMinMax();
            //    //isCalcMinMax = true;
            //}

            //back the camera up
            GL.Translate(0, 0, -mf.maxFieldDistance * gain);

            //translate to that spot in the world
            GL.Translate(-mf.fieldCenterX, -mf.fieldCenterY, 0);

            GL.Color3(0, 0.6, 0.4);

            //draw patches j= # of sections
            for (int j = 0; j < mf.vehicle.numSuperSection; j++)
            {
                //every time the section turns off and on is a new patch
                int patchCount = mf.section[j].patchList.Count;

                if (patchCount > 0)
                {
                    //for every new chunk of patch
                    foreach (var triList in mf.section[j].patchList)
                    {
                        {
                            //draw the triangle in each triangle strip
                            GL.Begin(PrimitiveType.TriangleStrip);
                            int count2 = triList.Count;
                            const int mipmap = 8;

                            //if large enough patch and camera zoomed out, fake mipmap the patches, skip triangles
                            if (count2 >= mipmap)
                            {
                                int step = mipmap;
                                for (int i = 0; i < count2; i += step)
                                {
                                    GL.Vertex3(triList[i].easting, triList[i].northing, 0); i++;
                                    GL.Vertex3(triList[i].easting, triList[i].northing, 0); i++;

                                    //too small to mipmap it
                                    if (count2 - i <= (mipmap + 2)) step = 0;
                                }
                            }
                            else { for (int i = 0; i < count2; i++) GL.Vertex3(triList[i].easting, triList[i].northing, 0); }
                            GL.End();
                        }
                    }
                }
            } //end of section patches

            //draw the ABLine
            if (mf.ABLine.isABLineSet || mf.ABLine.isABLineBeingSet)
            {
                //Draw AB Ref Line Points
                GL.PointSize(8.0f);
                GL.Begin(PrimitiveType.Points);

                GL.Color3(0.95f, 0.0f, 0.0f);
                GL.Vertex3(mf.ABLine.refPoint1.easting, mf.ABLine.refPoint1.northing, 0.0);
                GL.Color3(0.0f, 0.90f, 0.95f);
                GL.Vertex3(mf.ABLine.refPoint2.easting, mf.ABLine.refPoint2.northing, 0.0);
                GL.End();

                //Draw reference AB line ref
                GL.LineWidth(2);
                GL.Enable(EnableCap.LineStipple);
                GL.LineStipple(1, 0x00F0);
                GL.Begin(PrimitiveType.Lines);
                GL.Color3(0.9f, 0.15f, 0.17f);
                GL.Vertex3(mf.ABLine.refABLineP1.easting, mf.ABLine.refABLineP1.northing, 0);
                GL.Color3(0.2f, 0.95f, 0.17f);
                GL.Vertex3(mf.ABLine.refABLineP2.easting, mf.ABLine.refABLineP2.northing, 0);
                GL.End();
                GL.Disable(EnableCap.LineStipple);

                //raw current AB Line
                GL.PointSize(2.0f);
                GL.Begin(PrimitiveType.Points);
                //GL.Color3(0.9f, 0.0f, 0.0f);
                //GL.Vertex3(mf.ABLine.currentABLineP1.easting, mf.ABLine.currentABLineP1.northing, 0.0);
                //GL.Vertex3(mf.ABLine.currentABLineP2.easting, mf.ABLine.currentABLineP2.northing, 0.0);
                //GL.Color3(0.9f, 0.5f, 0.0f);
                //GL.Vertex3(lastABLineP1.easting, lastABLineP1.northing, 0);
                //GL.Vertex3(lastABLineP2.easting, lastABLineP2.northing, 0);

                foreach (var ptList in mf.self.lineList)
                {
                    int count2 = ptList.Count;

                    for (int i = 0; i < count2; i++) GL.Vertex3(ptList[i].easting, ptList[i].northing, 0);
                }

                GL.End();
            }

            //draw all the boundaries
            mf.bnd.DrawBoundaryLines();
            mf.turn.DrawTurnLines();

            //Draw the maze             //row is Y, col is X   int[Y,X] [i,j]

            if (mf.self.mazeList != null)
            {
                int ptCount = mf.self.mazeList.Count;
                if (ptCount > 1)
                {
                    GL.PointSize(2.0f);
                    GL.Begin(PrimitiveType.Points);
                    GL.Color3(0.95f, 0.920f, 0.90f);
                    for (int h = 0; h < ptCount; h++)
                    {
                        GL.Vertex3(
                        (mf.self.mazeList[h].easting * 2) + (int)mf.minFieldX,
                        (mf.self.mazeList[h].northing * 2) + (int)mf.minFieldY,
                        0);
                    }

                    GL.End();
                }
            }

            //for (int i = 0; i < mf.self.m_iRowDimensions; i++)
            //{
            //    for (int j = 0; j < mf.self.m_iColDimensions; j++)
            //    {
            //        if (mf.self.solved[i, j] == 100)
            //        {
            //            double east = j*5 + (int) mf.self.minFieldX;
            //            double north = i*5 + (int)mf.self.minFieldY;
            //            GL.Vertex3(east, north, 0.0);
            //        }
            //    }
            //}

            GL.PointSize(8.0f);
            GL.Begin(PrimitiveType.Points);
            GL.Color3(0.95f, 0.90f, 0.0f);
            GL.Vertex3(mf.self.pint1.easting, mf.self.pint1.northing, 0.0);
            GL.End();
            GL.PointSize(1.0f);

            GL.Flush();
            oglSelf.SwapBuffers();

            //if (!isCalcMinMax)
            //{
            //    CalculateMinMax();
            //    mf.self.CalculateMinMax();
            //    isCalcMinMax = true;
            //}
        }

        private void oglSelf_Resize(object sender, EventArgs e)
        {
            oglSelf.MakeCurrent();
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadIdentity();

            //58 degrees view
            Matrix4 mat = Matrix4.CreatePerspectiveFieldOfView(1.01f, 1.0f, 1.0f, 20000);
            GL.LoadMatrix(ref mat);

            GL.MatrixMode(MatrixMode.Modelview);
        }

        private void oglSelf_Load(object sender, EventArgs e)
        {
            oglSelf.MakeCurrent();
            GL.Enable(EnableCap.CullFace);
            GL.CullFace(CullFaceMode.Back);
            GL.ClearColor(0.23122f, 0.2318f, 0.2315f, 1.0f);
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            oglSelf.Refresh();
        }

        public FormSelf(Form callingForm)
        {
            //get copy of the calling main form
            mf = callingForm as FormGPS;

            InitializeComponent();
        }
    }
}