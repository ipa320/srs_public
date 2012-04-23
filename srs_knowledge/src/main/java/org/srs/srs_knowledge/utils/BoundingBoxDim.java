
package org.srs.srs_knowledge.utils;

import java.io.*;
import java.util.StringTokenizer;
import java.util.ArrayList;
import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.geometry_msgs.msg.Pose2D;
import org.srs.srs_knowledge.knowledge_engine.*;

import tfjava.*;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Point3d;
import javax.vecmath.Matrix4d;

import math.geom2d.*;
import math.geom2d.line.LineSegment2D;
import math.geom2d.polygon.Polygon2DUtils;
import math.geom2d.polygon.Polygon2D;
import math.geom2d.polygon.SimplePolygon2D;

import math.geom2d.Point2D;

public class BoundingBoxDim {
    public float l = 0;
    public float w = 0;
    public float h = 0;
}
