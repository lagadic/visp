package org.visp.test.imgproc;

import org.visp.core.Core;
import org.visp.core.Point;
import org.visp.core.Size;
import org.visp.imgproc.Imgproc;
import org.visp.test.ViSPTestCase;

public class ImgprocTest extends ViSPTestCase {

    Point anchorPoint;
    private int imgprocSz;
    Size size;

    @Override
    protected void setUp() throws Exception {
        super.setUp();

        imgprocSz = 2;
        anchorPoint = new Point(2, 2);
        size = new Size(3, 3);
    }
}
