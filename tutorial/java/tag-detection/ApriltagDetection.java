import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.color.ColorSpace;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.List;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.filechooser.FileNameExtensionFilter;

import org.visp.core.VpCameraParameters;
import org.visp.core.VpHomogeneousMatrix;
import org.visp.core.VpImagePoint;
import org.visp.core.VpImageUChar;
import org.visp.core.VpPoint;
import org.visp.detection.VpDetectorAprilTag;
import org.visp.io.VpImageIo;

public class ApriltagDetection extends JFrame {
    static {
        System.loadLibrary("visp_java330");
    }

    private static final long serialVersionUID = 1L;
    private JLabel canvasLabel;
    private VpImageUChar I;
    private VpDetectorAprilTag detector;
    private BufferedImage canvas;
    private double tagSize = 0.053;
    private VpCameraParameters cam = new VpCameraParameters(615.1674805, 615.1675415, 312.1889954, 243.4373779);
    private static String[] tagFamilyNames = {"TAG_36h11", "TAG_25h9", "TAG_16h5", "TAG_CIRCLE21h7",
            "TAG_CIRCLE49h12", "TAG_CUSTOM48h12", "TAG_STANDARD41h12", "TAG_STANDARD52h13"};
    private static int[] tagFamilies = {0, 3, 5, 6, 7, 8, 9, 10};
    private static String[] poseEstimationMethodNames = {"HOMOGRAPHY", "HOMOGRAPHY_VIRTUAL_VS", "DEMENTHON_VIRTUAL_VS",
            "LAGRANGE_VIRTUAL_VS", "BEST_RESIDUAL_VIRTUAL_VS", "HOMOGRAPHY_ORTHOGONAL_ITERATION"};

    public ApriltagDetection() {
        super("Apriltag detection");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        I = new VpImageUChar();
        detector = new VpDetectorAprilTag();

        JPanel newContentPane = new JPanel(new GridBagLayout());
        GridBagConstraints c = new GridBagConstraints();
        c.fill = GridBagConstraints.BOTH;
        c.gridx = 0;
        c.gridy = 0;
        canvas = new BufferedImage(640, 480, BufferedImage.TYPE_INT_ARGB);
        canvasLabel = new JLabel(new ImageIcon(canvas));
        newContentPane.add(canvasLabel, c);

        c = new GridBagConstraints();
        c.gridx = 0;
        c.gridy = 1;
        JPanel south = new JPanel();
        JButton openButton = new JButton("Open");
        JFileChooser fc = new JFileChooser();
        fc.setFileFilter(new FileNameExtensionFilter("Image Files (*.jpg, *.jpeg, *.png, *.pgm, *.ppm)",
                                                                   "jpg", "jpeg", "png", "pgm", "ppm"));
        openButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int returnVal = fc.showOpenDialog(ApriltagDetection.this);
                if (returnVal == JFileChooser.APPROVE_OPTION) {
                    File file = fc.getSelectedFile();
                    VpImageIo.read(I, file.getAbsolutePath());

                    try {
                        BufferedImage tmp = ImageIO.read(new File(file.getAbsolutePath()));
                        if (tmp.getColorModel().getColorSpace().getType() == ColorSpace.TYPE_RGB) {
                            canvas = tmp;
                        } else {
                            Graphics2D g2d = canvas.createGraphics();
                            g2d.drawImage(tmp, 0, 0, null);
                            g2d.dispose();
                        }
                    } catch (IOException e1) {
                        System.err.println("Cannot read: " + file.getAbsolutePath());
                    }
                    canvasLabel.setIcon(new ImageIcon(canvas));
                    repaint();
                    pack();
                }
            }
        });
        south.add(openButton);

        JButton detectButton = new JButton("Detect");
        detectButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                List<VpHomogeneousMatrix> cMo_vec = detector.detect(I, tagSize, cam);
                for (VpHomogeneousMatrix cMo : cMo_vec) {
                    displayFrame(canvas, cMo, cam, tagSize/2, 3);
                }

                List<List<VpImagePoint>> tags_corners = detector.getTagsCorners();
                for (List<VpImagePoint> corners : tags_corners) {
                    displayLine(canvas, corners.get(0).get_i(), corners.get(0).get_j(),
                            corners.get(1).get_i(), corners.get(1).get_j(), Color.RED, 3);
                    displayLine(canvas, corners.get(0).get_i(), corners.get(0).get_j(),
                            corners.get(3).get_i(), corners.get(3).get_j(), Color.GREEN, 3);
                    displayLine(canvas, corners.get(1).get_i(), corners.get(1).get_j(),
                            corners.get(2).get_i(), corners.get(2).get_j(), Color.YELLOW, 3);
                    displayLine(canvas, corners.get(2).get_i(), corners.get(2).get_j(),
                            corners.get(3).get_i(), corners.get(3).get_j(), Color.BLUE, 3);
                }

                int[] tagsId = detector.getTagsId();
                for (int i = 0; i < tagsId.length; i++) {
                    double[] centroid = computeCentroid(tags_corners.get(i));
                    displayText(canvas, new String("Id: " + String.valueOf(tagsId[i])),
                            centroid[0] + 10, centroid[1] + 20, Color.RED, 3);
                }

                repaint();
            }
        });
        south.add(detectButton);

        JComboBox<String> tagFamilyComboBox = new JComboBox<>(tagFamilyNames);
        tagFamilyComboBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                @SuppressWarnings("unchecked")
                JComboBox<String> cb = (JComboBox<String>)e.getSource();
                detector.setAprilTagFamily(tagFamilies[cb.getSelectedIndex()]);
            }
        });
        south.add(tagFamilyComboBox);

        JComboBox<String> poseEstimationComboBox = new JComboBox<>(poseEstimationMethodNames);
        poseEstimationComboBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                @SuppressWarnings("unchecked")
                JComboBox<String> cb = (JComboBox<String>)e.getSource();
                detector.setAprilTagPoseEstimationMethod(cb.getSelectedIndex());
            }
        });
        south.add(poseEstimationComboBox);

        newContentPane.add(south, c);
        setContentPane(newContentPane);

        setMinimumSize(new Dimension(640, 480));
        pack();
        setLocationRelativeTo(null);
        setVisible(true);
    }

    public double[] computeCentroid(List<VpImagePoint> corners) {
        double[] centroid = {0, 0};

        for (VpImagePoint pt : corners) {
            centroid[0] += pt.get_i();
            centroid[1] += pt.get_j();
        }
        if (!corners.isEmpty()) {
            centroid[0] /= corners.size();
            centroid[1] /= corners.size();
        }

        return centroid;
    }

    public void displayLine(BufferedImage I, double i1, double j1, double i2, double j2, Color color, int thickness) {
        displayLine(I, (int) i1, (int) j1, (int) i2, (int) j2, color, thickness);
    }

    public void displayLine(BufferedImage I, int i1, int j1, int i2, int j2, Color color, int thickness) {
        Graphics2D g = I.createGraphics();
        g.setStroke(new BasicStroke(thickness));
        g.setColor(color);
        g.drawLine(j1, i1, j2, i2);
    }

    public void displayArrow(BufferedImage I, double i1, double j1, double i2, double j2, Color color, int w, int h, int thickness) {
        displayArrow(I, (int) i1, (int) j1, (int) i2, (int) j2, color, w, h,thickness);
    }

    public void displayArrow(BufferedImage I, int i1, int j1, int i2, int j2, Color color, int w, int h, int thickness) {
        Graphics2D g = I.createGraphics();
        g.setStroke(new BasicStroke(thickness));
        g.setColor(color);

        double a = i2 - i1;
        double b = j2 - j1;
        double lg = Math.sqrt(a*a + b*b);

        if (Math.abs(a) <= Math.ulp(1.0) &&
            Math.abs(b) <= Math.ulp(1.0)) {
        } else {
          a /= lg;
          b /= lg;

          double i3 = i2 - w*a;
          double j3 = j2 - w*b;

          double i4 = i3 - b*h;
          double j4 = j3 + a*h;

          double dist = Math.sqrt((i2 - i4)*(i2 - i4) + (j2 - j4)*(j2 - j4));
          if (lg > 2 * dist) {
            displayLine(I, i2, j2, i4, j4, color, thickness);
          }

          i4 = i3 + b*h;
          j4 = j3 - a*h;

          dist = Math.sqrt((i2 - i4)*(i2 - i4) + (j2 - j4)*(j2 - j4));
          if (lg > 2 * dist) {
              displayLine(I, i2, j2, i4, j4, color, thickness);
          }

          displayLine(I, i1, j1, i2, j2, color, thickness);
        }
    }

    public void displayFrame(BufferedImage I, VpHomogeneousMatrix mat, VpCameraParameters cam, double size, int thickness) {
        VpPoint o = new VpPoint(0.0, 0.0, 0.0);
        VpPoint x = new VpPoint(size, 0.0, 0.0);
        VpPoint y = new VpPoint(0.0, size, 0.0);
        VpPoint z = new VpPoint(0.0, 0.0, size);

        o.changeFrame(mat);
        o.projection();

        x.changeFrame(mat);
        x.projection();

        y.changeFrame(mat);
        y.projection();

        z.changeFrame(mat);
        z.projection();

        displayArrow(I, o.get_y()*cam.get_py() + cam.get_v0(), o.get_x()*cam.get_px() + cam.get_u0(),
                 x.get_y()*cam.get_py() + cam.get_v0(), x.get_x()*cam.get_px() + cam.get_u0(),
                 Color.RED, 4 * thickness, 2 * thickness, thickness);

        displayArrow(I, o.get_y()*cam.get_py() + cam.get_v0(), o.get_x()*cam.get_px() + cam.get_u0(),
                 y.get_y()*cam.get_py() + cam.get_v0(), y.get_x()*cam.get_px() + cam.get_u0(),
                 Color.GREEN, 4 * thickness, 2 * thickness, thickness);

        displayArrow(I, o.get_y()*cam.get_py() + cam.get_v0(), o.get_x()*cam.get_px() + cam.get_u0(),
                 z.get_y()*cam.get_py() + cam.get_v0(), z.get_x()*cam.get_px() + cam.get_u0(),
                 Color.BLUE, 4 * thickness, 2 * thickness, thickness);
    }

    public void displayText(BufferedImage I, String text, double i, double j, Color color, int thickness) {
        displayText(I, text, (int) i, (int) j, color, thickness);
    }

    public void displayText(BufferedImage I, String text, int i, int j, Color color, int thickness) {
        Graphics2D g = I.createGraphics();
        g.setStroke(new BasicStroke(thickness));
        g.setColor(color);
        g.drawString(text, j, i);
    }

    public static void main(String[] args) {
        //Schedule a job for the event-dispatching thread:
        //creating and showing this application's GUI.
        javax.swing.SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                new ApriltagDetection();
            }
        });
    }
}
