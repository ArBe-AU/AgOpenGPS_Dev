﻿using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;

namespace AgOpenGPS
{
    public class CCell
    {
        public vec3 start { get; set; }
        public vec3 end { get; set; }
        public double heading { get; set; }
        public int turn { get; set; }
        public bool fullLength { get; set; }

        //constructor
        public CCell(vec3 _start, vec3 _end, double _heading, int _turn = 0, bool _fullLength = false)
        {
            start = _start;
            end = _end;
            heading = _heading;
            turn = _turn;
            fullLength = _fullLength;
        }
    }

    public class CCellDec
    {
        public int lines { get; set; }
        public int cells { get; set; }

        //constructor
        public CCellDec(int _lines, int _cells)
        {
            lines = _lines;
            cells = _cells;
        }
    }

    public class CMorseLine
    {
        public vec3 start { get; set; }
        public vec3 end { get; set; }
        public vec3 start2 { get; set; }
        public vec3 end2 { get; set; }
        public int turnNum { get; set; }

        //constructor
        public CMorseLine(vec3 _start, vec3 _end, vec3 _start2, vec3 _end2, int _turnNum = 0)
        {
            start = _start;
            end = _end;
            start2 = _start2;
            end2 = _end2;
            turnNum = _turnNum;
        }
    }

    public class CSelf
    {
        private readonly FormGPS mf;
        public vec3 pint1 = new vec3();
        public vec3 pint2 = new vec3();

        //the clicked last pass if negative intial turn left else right.
        public int lastPassNumber = 0;

        //position and autosteer
        private vec3 pivotPos = new vec3();

        public vec3 homePos = new vec3();
        public double distanceFromRefLine, distanceFromCurrentLine, refLineSide = 1.0;
        private int A, B, C;
        public double abFixHeadingDelta, segmentHeading;
        public bool isABSameAsFixHeading = true, isOnRightSideCurrentLine = true;
        public int lastPointFound = -1, currentPositonIndex;
        public vec2 goalPointSD = new vec2(0, 0);
        public vec2 radiusPointSD = new vec2(0, 0);
        public double steerAngleSD, rEastSD, rNorthSD, ppRadiusSD;

        //the dubins path for intertransport
        public List<CRecPathPt> dubList = new List<CRecPathPt>();

        public int dubListCount;

        //list of vec3 points of Dubins shortest path between 2 points - To be converted to RecPt
        public List<vec3> dubinsList = new List<vec3>();

        public bool isBtnFollowOn, isEndOfTheRecLine, isRecordOn;
        public bool isPausedSelfDriving, isSelfDriving, isFollowingDubinsToPath, isFollowingRecPath, isFollowingDubinsHome;

        //list of single pointed AB Lines
        public List<vec3> linePtList = new List<vec3>();

        //list of the cells from morse distribution
        public List<CCellDec> cellDecList = new List<CCellDec>();

        //line inner turn boundary collisions
        public List<vec3> colList = new List<vec3>();

        //list of pointed AB Lines
        public List<List<vec3>> lineList = new List<List<vec3>>();

        readonly private List<CMorseLine> mLine = new List<CMorseLine>();

        //constructor
        public CSelf(FormGPS _f)
        {
            mf = _f;
        }

        public bool StartSelfDriving()
        {
            CDubins.turningRadius = mf.vehicle.minTurningRadius * 1.5;

            //calculate the AB Heading
            double startPointHeading = Math.Atan2(mf.ABLine.refPoint2.easting - mf.ABLine.refPoint1.easting,
                                            mf.ABLine.refPoint2.northing - mf.ABLine.refPoint1.northing);
            if (startPointHeading < 0) startPointHeading += glm.twoPI;

            //the goal is the first point of path, the start is the current position
            vec3 goal = new vec3(mf.ABLine.refPoint1.easting, mf.ABLine.refPoint1.northing, startPointHeading);

            //save a copy of where we started.
            homePos = mf.pivotAxlePos;

            //get the dubins for approach to recorded path
            GetDubinsPath(goal);
            dubListCount = dubList.Count;

            //has a valid dubins path been created?
            if (dubListCount == 0) return false;

            //turn off and reset
            if (!mf.btnEnableAutoYouTurn.Enabled) mf.btnEnableAutoYouTurn.Enabled = true;
            if (!mf.btnAutoSteer.Enabled) mf.btnAutoSteer.Enabled = true;

            if (mf.isAutoSteerBtnOn) mf.btnAutoSteer.PerformClick();
            if (mf.yt.isYouTurnBtnOn) mf.btnEnableAutoYouTurn.PerformClick();
            mf.yt.ResetYouTurn();

            //technically all good if we get here so set all the flags
            isFollowingDubinsHome = false;
            isFollowingRecPath = false;
            isFollowingDubinsToPath = true;
            isEndOfTheRecLine = false;
            currentPositonIndex = 0;
            isSelfDriving = true;

            mf.btnDrivePath.Image = Properties.Resources.AutoGo;
            isPausedSelfDriving = false;
            return true;
        }

        public void StopSelfDriving()
        {
            isFollowingDubinsHome = false;
            isFollowingRecPath = false;
            isFollowingDubinsToPath = false;
            dubList.Clear();
            dubinsList.Clear();
            mf.sim.stepDistance = 0;
            isSelfDriving = false;
            mf.btnGoSelf.Image = Properties.Resources.AutoGo;
            isPausedSelfDriving = false;
        }

        public bool trig;
        public double north;

        private CMazePath Maze;

        public int[] m_iMaze;
        public int m_iRowDimensions;
        public int m_iColDimensions;
        public List<vec3> mazeList = new List<vec3>();
        //public List<int> mazeList = new List<int>();

        public void MakeMaze(int _y, int _x)
        {
            m_iRowDimensions = _y;
            m_iColDimensions = _x;
            m_iMaze = new int[m_iRowDimensions * m_iColDimensions];

            //row is Y, col is X   int[Y,X] [i,j] [row,col]
            vec2 pot = new vec2();

            mf.yt.triggerDistanceOffset += 3;
            mf.turn.BuildTurnLines();

            for (int i = 0; i < mf.self.m_iRowDimensions; i++)
            {
                for (int j = 0; j < mf.self.m_iColDimensions; j++)
                {
                    pot.easting = (j * 2) + (int)mf.minFieldX;
                    pot.northing = (i * 2) + (int)mf.minFieldY;
                    if (!mf.turn.PointInsideWorkArea(pot))
                    {
                        m_iMaze[(i * m_iColDimensions) + j] = 1;
                    }
                    else
                    {
                        m_iMaze[(i * m_iColDimensions) + j] = 0;
                    }
                }
            }

            mf.yt.triggerDistanceOffset -= 3;
            mf.turn.BuildTurnLines();

            Maze = new CMazePath(m_iRowDimensions, m_iColDimensions, m_iMaze);
        }

        public void SolveMaze()
        {
            //vec3 pt1;
            //vec3 pt2;
            //mazeList = Maze.Search((int)((pint1.northing - mf.minFieldY) / 2), (int)((pint1.easting - mf.minFieldX) / 2),
            //    (int)((pint2.northing - mf.minFieldY) / 2), (int)((pint2.easting - mf.minFieldX) / 2));
        }

        public void BuildLines()
        {
            double widthMinusOverlap = mf.vehicle.toolWidth - mf.vehicle.toolOverlap;
            double head = mf.ABLine.abHeading;
            double syne = Math.Sin(head);
            double cosyne = Math.Cos(head);
            vec3 pt = new vec3(0, 0, 0);
            vec3 refPt = new vec3(0, 0, 0);

            int side = 1;
            if (lastPassNumber < 0) side = -1;

            linePtList?.Clear();
            lineList?.Clear();

            int step = 0;

            bool isInsideInner = false;
            bool isInWorkArea = false;

            int currentBnd = 0;
            int lastCurrentBnd = 0;

            for (int i = 0; i <= Math.Abs(lastPassNumber); i++)
            {
                linePtList = new List<vec3>();
                lineList.Add(linePtList);

                //draw segmented lines at full length
                refPt = new vec3(
                    refPt.easting = mf.ABLine.refABLineP1.easting + (Math.Sin(head + glm.PIBy2) * widthMinusOverlap * side * i),
                    refPt.northing = mf.ABLine.refABLineP1.northing + (Math.Cos(head + glm.PIBy2) * widthMinusOverlap * side * i),
                    refPt.heading = head
                );

                step = 0;
                currentBnd = 0;
                lastCurrentBnd = 0;

                for (int j = 0; j < 3000; j++)
                {
                    isInsideInner = false;
                    isInWorkArea = false;

                    pt.easting = refPt.easting + (syne * j);
                    pt.northing = refPt.northing + (cosyne * j);
                    pt.heading = step;

                    //if inside outer boundary, then potentially add
                    if (mf.turn.turnArr[0].IsPointInTurnWorkArea(pt))
                    {
                        isInWorkArea = true;
                        currentBnd = 0;

                        for (int b = 1; b < FormGPS.MAXBOUNDARIES; b++)
                        {
                            if (mf.bnd.bndArr[b].isSet)
                            {
                                if (mf.turn.turnArr[b].IsPointInTurnWorkArea(pt))
                                {
                                    //point is in an inner turn area
                                    currentBnd = b;
                                    isInsideInner = true;
                                }
                            }
                        }

                        if (currentBnd != lastCurrentBnd)
                        {
                            if (!isInsideInner) step++;
                            lastCurrentBnd = currentBnd;
                        }
                    }
                    else
                    {
                        isInWorkArea = false;
                    }

                    if (isInWorkArea && !isInsideInner)
                    {
                        pt.heading = step;
                        linePtList.Add(pt);
                    }
                }
            }

            //for each line determine cells
            cellDecList?.Clear();
            int numLines = lineList.Count;
            int ptsInPassLine = lineList[0].Count - 1;
            step = 1;
            lastCurrentBnd = (int)lineList[0][ptsInPassLine].heading;

            for (int passLine = 1; passLine < numLines; passLine++)
            {
                ptsInPassLine = lineList[passLine].Count - 1;
                currentBnd = (int)lineList[passLine][ptsInPassLine].heading;
                if (currentBnd == lastCurrentBnd)
                {
                    step++;
                }
                else
                {
                    CCellDec cdec = new CCellDec(step, lastCurrentBnd);
                    lastCurrentBnd = currentBnd;
                    cellDecList.Add(cdec);
                    step = 1;
                }
            }
            //capture the last pass
            CCellDec cdeclast = new CCellDec(step, lastCurrentBnd);
            lastCurrentBnd = currentBnd;
            cellDecList.Add(cdeclast);

            numLines = Math.Abs(lastPassNumber) + 1;
            //vec3[] arr = new vec3[numLines];

            colList?.Clear();
            mLine?.Clear();

            //line calculate obstacle cells
            vec3 start = new vec3();
            vec3 end = new vec3();
            //vec3 start2 = new vec3();
            //vec3 end2 = new vec3();

            //for each line
            for (int passLine = 0; passLine < numLines; passLine++)
            {
                ptsInPassLine = lineList[passLine].Count;
                int colBnd = 0;
                int colPt = 0;

                //scan for full length
                for (int j = 0; j < ptsInPassLine; j++)
                {
                    if (lineList[passLine][j].easting == 9999)
                    {
                        colBnd = (int)lineList[passLine][j].heading;
                        colPt = j;
                        break;
                    }
                }  //at this point if tBnd = 0 its full length, otherwise tBnd is inner turn line collides with

                if (colBnd == 0)
                {
                    start = lineList[passLine][10];
                    end = lineList[passLine][ptsInPassLine - 11];
                    CMorseLine mL = new CMorseLine(start, end, start, end, 0);
                    mLine.Add(mL);
                }
                else
                {
                    start = lineList[passLine][10];
                    end = lineList[passLine][colPt - 11];
                    CMorseLine mL = new CMorseLine(start, end, start, end, colBnd);
                    mLine.Add(mL);
                }
            }
        }

        public void UpdatePosition()
        {
            if (isFollowingDubinsToPath)
            {
                //set a speed of 7 kmh
                mf.sim.stepDistance = 11 / 17.86;

                pivotPos = mf.pivotAxlePos;

                FindGoalPointDubinsPath(dubListCount);
                PurePursuit();

                //check if close to recorded path
                double distSqr = glm.DistanceSquared(pivotPos.northing, pivotPos.easting, mf.ABLine.refPoint1.northing, mf.ABLine.refPoint1.easting);
                if (distSqr < 3)
                {
                    //isFollowingRecPath = true;
                    isFollowingDubinsToPath = false;
                    //isFollowingDubinsHome = true;
                    dubList.Clear();
                    dubinsList.Clear();
                    mf.yt.ResetYouTurn();

                    if (!mf.isAutoSteerBtnOn) mf.btnAutoSteer.PerformClick();
                    if (!mf.yt.isYouTurnBtnOn) mf.btnEnableAutoYouTurn.PerformClick();

                    //make sure we turn the right way first
                    if (lastPassNumber > 0)
                    {
                        if (mf.yt.isYouTurnRight) mf.btnSwapDirection.PerformClick();
                    }
                    else
                    {
                        if (!mf.yt.isYouTurnRight) mf.btnSwapDirection.PerformClick();
                    }
                }
            }

            //if (isFollowingRecPath)
            //{
            //    pivotPos = mf.pivotAxlePos;

            //    FindGoalPointRecPath(recListCount);
            //    PurePursuit();

            //    //if end of the line then stop
            //    if (!isEndOfTheRecLine)
            //    {
            //        mf.sim.stepDistance = recList[C].speed / 17.86;
            //        north = recList[C].northing;

            //        //section control - only if different click the button
            //        bool autoBtn = (mf.autoBtnState == FormGPS.btnStates.Auto);
            //        trig = autoBtn;
            //        if (autoBtn != recList[C].autoBtnState) mf.btnSectionOffAutoOn.PerformClick();
            //    }
            //    else
            //    {
            //        //create the dubins path based on start and goal to start of recorded path
            //        GetDubinsPath(homePos);
            //        dubListCount = dubList.Count;

            //        //its too small
            //        if (dubListCount < 5)
            //        {
            //            StopDrivingRecordedPath();
            //        }

            //        //set all the flags
            //        isFollowingDubinsHome = true;
            //        isFollowingRecPath = false;
            //        isFollowingDubinsToPath = false;
            //        isEndOfTheRecLine = false;
            //    }
            //}

            if (isFollowingDubinsHome)
            {
                mf.sim.stepDistance = 12 / 17.86;
                pivotPos = mf.pivotAxlePos;

                FindGoalPointDubinsPath(dubListCount);
                PurePursuit();

                //check if close to home position
                double distSqr = glm.DistanceSquared(pivotPos.easting, pivotPos.northing, homePos.easting, homePos.northing);
                if (distSqr < 3)
                {
                    StopSelfDriving();
                }
            }

            //if paused, set the sim to 0
            if (isPausedSelfDriving) mf.sim.stepDistance = 0 / 17.86;
        }

        private void GetDubinsPath(vec3 goal)
        {
            CDubins dubPath = new CDubins();

            //
            pivotPos = mf.pivotAxlePos;

            //bump it forward
            vec3 pt2 = new vec3
            {
                easting = pivotPos.easting + (Math.Sin(pivotPos.heading) * 4),
                northing = pivotPos.northing + (Math.Cos(pivotPos.heading) * 4),
                heading = pivotPos.heading
            };

            //get the dubins path vec3 point coordinates of turn
            dubinsList.Clear();
            dubinsList = dubPath.GenerateDubins(pt2, goal, mf.gf);

            dubinsList.Insert(0, mf.pivotAxlePos);

            //transfer point list to recPath class point style
            dubList.Clear();
            for (int i = 0; i < dubinsList.Count; i++)
            {
                CRecPathPt pt = new CRecPathPt(dubinsList[i].easting, dubinsList[i].northing, dubinsList[i].heading, 5.0, false);
                dubList.Add(pt);
            }

            //no longer needed
            dubinsList?.Clear();
        }

        private void PurePursuit()
        {
            //calc "D" the distance from pivot axle to lookahead point
            double goalPointDistanceSquared = glm.DistanceSquared(goalPointSD.northing, goalPointSD.easting, pivotPos.northing, pivotPos.easting);

            //calculate the the delta x in local coordinates and steering angle degrees based on wheelbase
            double localHeading = glm.twoPI - mf.fixHeading;
            ppRadiusSD = goalPointDistanceSquared / (2 * (((goalPointSD.easting - pivotPos.easting) * Math.Cos(localHeading)) + ((goalPointSD.northing - pivotPos.northing) * Math.Sin(localHeading))));

            steerAngleSD = glm.toDegrees(Math.Atan(2 * (((goalPointSD.easting - pivotPos.easting) * Math.Cos(localHeading))
                + ((goalPointSD.northing - pivotPos.northing) * Math.Sin(localHeading))) * mf.vehicle.wheelbase / goalPointDistanceSquared));

            if (steerAngleSD < -mf.vehicle.maxSteerAngle) steerAngleSD = -mf.vehicle.maxSteerAngle;
            if (steerAngleSD > mf.vehicle.maxSteerAngle) steerAngleSD = mf.vehicle.maxSteerAngle;

            if (ppRadiusSD < -500) ppRadiusSD = -500;
            if (ppRadiusSD > 500) ppRadiusSD = 500;

            radiusPointSD.easting = pivotPos.easting + (ppRadiusSD * Math.Cos(localHeading));
            radiusPointSD.northing = pivotPos.northing + (ppRadiusSD * Math.Sin(localHeading));

            //angular velocity in rads/sec  = 2PI * m/sec * radians/meters
            double angVel = glm.twoPI * 0.277777 * mf.pn.speed * (Math.Tan(glm.toRadians(steerAngleSD))) / mf.vehicle.wheelbase;

            //clamp the steering angle to not exceed safe angular velocity
            if (Math.Abs(angVel) > mf.vehicle.maxAngularVelocity)
            {
                steerAngleSD = glm.toDegrees(steerAngleSD > 0 ?
                        (Math.Atan((mf.vehicle.wheelbase * mf.vehicle.maxAngularVelocity) / (glm.twoPI * mf.pn.speed * 0.277777)))
                    : (Math.Atan((mf.vehicle.wheelbase * -mf.vehicle.maxAngularVelocity) / (glm.twoPI * mf.pn.speed * 0.277777))));
            }
            //Convert to centimeters
            distanceFromCurrentLine = Math.Round(distanceFromCurrentLine * 1000.0, MidpointRounding.AwayFromZero);

            //distance is negative if on left, positive if on right
            //if you're going the opposite direction left is right and right is left
            if (isABSameAsFixHeading)
            {
                if (!isOnRightSideCurrentLine) distanceFromCurrentLine *= -1.0;
            }

            //opposite way so right is left
            else if (isOnRightSideCurrentLine)
            {
                distanceFromCurrentLine *= -1.0;
            }

            mf.guidanceLineDistanceOff = (Int16)distanceFromCurrentLine;
            mf.guidanceLineSteerAngle = (Int16)(steerAngleSD * 100);
        }

        private vec2 FindGoalPointDubinsPath(int ptCount)
        {
            //distanceFromCurrentLine = 9999;
            //find the closest 2 points to current fix
            double minDistA = 9999999999;
            for (int t = 0; t < ptCount; t++)
            {
                double dist = ((pivotPos.easting - dubList[t].easting) * (pivotPos.easting - dubList[t].easting))
                                + ((pivotPos.northing - dubList[t].northing) * (pivotPos.northing - dubList[t].northing));
                if (dist < minDistA)
                {
                    minDistA = dist;
                    A = t;
                }
            }

            //save the closest point
            C = A;
            //next point is the next in list
            B = A + 1;
            if (B == ptCount) { A--; B--; }                //don't go past the end of the list - "end of the line" trigger

            //get the distance from currently active AB line
            //x2-x1
            double dx = dubList[B].easting - dubList[A].easting;
            //z2-z1
            double dz = dubList[B].northing - dubList[A].northing;

            if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dz) < Double.Epsilon) return goalPointSD;

            //abHeading = Math.Atan2(dz, dx);
            segmentHeading = dubList[A].heading;

            //how far from current AB Line is fix
            distanceFromCurrentLine = ((dz * pivotPos.easting) - (dx * pivotPos
                .northing) + (dubList[B].easting
                        * dubList[A].northing) - (dubList[B].northing * dubList[A].easting))
                            / Math.Sqrt((dz * dz) + (dx * dx));

            //are we on the right side or not
            isOnRightSideCurrentLine = distanceFromCurrentLine > 0;

            //absolute the distance
            distanceFromCurrentLine = Math.Abs(distanceFromCurrentLine);

            // ** Pure pursuit ** - calc point on ABLine closest to current position
            double U = (((pivotPos.easting - dubList[A].easting) * dx)
                        + ((pivotPos.northing - dubList[A].northing) * dz))
                        / ((dx * dx) + (dz * dz));

            rEastSD = dubList[A].easting + (U * dx);
            rNorthSD = dubList[A].northing + (U * dz);

            //Subtract the two headings, if > 1.57 its going the opposite heading as refAB
            abFixHeadingDelta = (Math.Abs(mf.fixHeading - segmentHeading));
            if (abFixHeadingDelta >= Math.PI) abFixHeadingDelta = Math.Abs(abFixHeadingDelta - glm.twoPI);

            //used for accumulating distance to find goal point
            double distSoFar;

            //update base on autosteer settings and distance from line
            double goalPointDistance = mf.vehicle.UpdateGoalPointDistance(distanceFromCurrentLine);
            mf.lookaheadActual = goalPointDistance;

            // used for calculating the length squared of next segment.
            double tempDist = 0.0;

            //counting up
            isABSameAsFixHeading = true;
            distSoFar = glm.Distance(dubList[B].easting, dubList[B].northing, rEastSD, rNorthSD);

            //Is this segment long enough to contain the full lookahead distance?
            if (distSoFar > goalPointDistance)
            {
                //treat current segment like an AB Line
                goalPointSD.easting = rEastSD + (Math.Sin(dubList[A].heading) * goalPointDistance);
                goalPointSD.northing = rNorthSD + (Math.Cos(dubList[A].heading) * goalPointDistance);
            }

            //multiple segments required
            else
            {
                //cycle thru segments and keep adding lengths. check if end and break if so.
                while (B < ptCount - 1)
                {
                    B++; A++;
                    tempDist = glm.Distance(dubList[B].easting, dubList[B].northing, dubList[A].easting, dubList[A].northing);

                    //will we go too far?
                    if ((tempDist + distSoFar) > goalPointDistance)
                    {
                        //A--; B--;
                        break; //tempDist contains the full length of next segment
                    }

                    distSoFar += tempDist;
                }

                // the remainder to yet travel
                double t = (goalPointDistance - distSoFar);
                t /= tempDist;

                goalPointSD.easting = (((1 - t) * dubList[A].easting) + (t * dubList[B].easting));
                goalPointSD.northing = (((1 - t) * dubList[A].northing) + (t * dubList[B].northing));
            }

            return goalPointSD;
        }

        public void DrawDubins()
        {
            GL.PointSize(2);
            if (dubList.Count > 1)
            {
                GL.LineWidth(2);
                GL.Color3(0.298f, 0.96f, 0.2960f);
                GL.Begin(PrimitiveType.Points);
                for (int h = 0; h < dubList.Count; h++)
                    GL.Vertex3(dubList[h].easting, dubList[h].northing, 0);
                GL.End();
            }

            //raw current AB Line
            GL.Begin(PrimitiveType.Lines);
            GL.Color3(0.9f, 0.5f, 0.0f);

            foreach (var ptList in mLine)
            {
                GL.Vertex3(ptList.start.easting, ptList.start.northing, 0);
                GL.Vertex3(ptList.end.easting, ptList.end.northing, 0);
            }

            GL.End();
        }
    }
}