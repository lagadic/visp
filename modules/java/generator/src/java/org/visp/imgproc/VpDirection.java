package org.visp.imgproc;

import org.visp.core.VpImagePoint;
import org.visp.core.VpImageUChar;

public class VpDirection {
  public int mDirectionType;
  public int m_dirx[], m_diry[];

  public VpDirection() {
    m_dirx = new int[8];
    m_diry = new int[8];

    m_dirx[0] = 0;
    m_dirx[1] = 1;
    m_dirx[2] = 1;
    m_dirx[3] = 1;
    m_dirx[4] = 0;
    m_dirx[5] = -1;
    m_dirx[6] = -1;
    m_dirx[7] = -1;

    m_diry[0] = -1;
    m_diry[1] = -1;
    m_diry[2] = 0;
    m_diry[3] = 1;
    m_diry[4] = 1;
    m_diry[5] = 1;
    m_diry[6] = 0;
    m_diry[7] = -1;
  }

  public VpDirection clockwise() {
    VpDirection direction = new VpDirection();
    int directionSize = VpDirectionType.LAST_DIRECTION;
    direction.mDirectionType = (mDirectionType + 1) % directionSize;
    return direction;
  }

  public VpDirection counterClockwise(){
    VpDirection direction = new VpDirection();
    int directionSize = VpDirectionType.LAST_DIRECTION;
    direction.mDirectionType = (mDirectionType - 1) % directionSize;
    return direction;
  }

  public VpImagePoint active(VpImageUChar I, VpImagePoint point){
      int yy = (int)(point.get_i() + m_diry[mDirectionType]);
      int xx = (int)(point.get_j() + m_dirx[mDirectionType]);

      if (xx < 0 || xx >= I.cols() || yy < 0 || yy >= I.rows()) {
        return new VpImagePoint(-1.0, -1.0);
      }

      int pixel = I.getPixel(yy,xx);
      return pixel != 0 ? new VpImagePoint(yy, xx) : new VpImagePoint(-1, -1);
  }

  public class VpDirectionType {
    public static final int NORTH = 0;
    public static final int NORTH_EAST = 1;
    public static final int EAST = 2;
    public static final int SOUTH_EAST = 3;
    public static final int SOUTH = 4;
    public static final int SOUTH_WEST = 5;
    public static final int WEST = 6;
    public static final int NORTH_WEST = 7;
    public static final int LAST_DIRECTION = 8;
  }
}
