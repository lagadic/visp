import org.visp.core.VpCameraParameters;
import org.visp.core.VpColVector;
import org.visp.core.VpImageRGBa;
import org.visp.core.VpImageUChar;
import org.visp.core.VpMatrix;
import org.visp.core.VpRGBa;

public class Started {
  static {
    System.loadLibrary("visp_java350");
  }
  public static void main(String[] args) {
    // VpMatrix
    VpMatrix vp = new VpMatrix(2,3,1.5);
    System.out.println(vp.getCol(0));
    System.out.println(vp.transpose());
                
    // VpColVector
    VpColVector vpColVector = new VpColVector(10,1.5);
    System.out.println(vpColVector.infinityNorm());
                
    // VpImageUChar
    VpImageUChar imageUChar = new VpImageUChar(2, 4, (byte)220);
    System.out.println(imageUChar);
                
    // VpImageRGBa
    VpImageRGBa colorImage = new VpImageRGBa(3, 5, new VpRGBa((char)255,(char)0,(char)0,(char)255));
    System.out.println(colorImage);
                
    // VpCameraParameters
    VpCameraParameters vpCameraParameters = new VpCameraParameters(1.0, 1.0, 1.0, 1.0);
    System.out.println(vpCameraParameters.get_K());
  }
}

