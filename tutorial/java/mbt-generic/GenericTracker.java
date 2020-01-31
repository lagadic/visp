import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.color.ColorSpace;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.File;
import java.lang.reflect.InvocationTargetException;

import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

import org.visp.core.VpCameraParameters;
import org.visp.core.VpHomogeneousMatrix;
import org.visp.core.VpImageUChar;
import org.visp.core.VpPoint;
import org.visp.io.VpImageIo;
import org.visp.mbt.VpMbGenericTracker;

public class GenericTracker extends JFrame {
    static {
        System.loadLibrary("visp_java330");
    }

    private static final long serialVersionUID = 1L;
    private JLabel canvasLabel;
    private VpImageUChar I;
    private BufferedImage canvas;
    private JButton trackButton;
    private JComboBox<String> sequenceComboBox;
    private VpMbGenericTracker tracker;
    private VpCameraParameters cam;
    private static String[] sequences = {"Cube", "Cube + Cylinder", "Castle"};
    private static String[] trackerTypes = {"Edges", "KLT", "Edges + KLT"};
    private static String vispInputImagePath = new String();
    private int sequenceId = 0;
    private String input = new String();
    private int firstFrame = 0;
    private int trackerType = 1;

    public GenericTracker() {
        super("Generic Model-Based Tracking");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        I = new VpImageUChar();
        tracker = new VpMbGenericTracker();
        cam = new VpCameraParameters();

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
        JButton openButton = new JButton("Open ViSP Input Image");
        JFileChooser fc = new JFileChooser();
        fc.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
        openButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int returnVal = fc.showOpenDialog(GenericTracker.this);
                if (returnVal == JFileChooser.APPROVE_OPTION) {
                    File file = fc.getSelectedFile();
                    vispInputImagePath = file.getAbsolutePath();
                    System.out.println("vispInputImagePath: " + vispInputImagePath);

                    trackButton.setEnabled(true);
                    sequenceComboBox.setEnabled(true);
                }
            }
        });
        south.add(openButton);

        sequenceComboBox = new JComboBox<>(sequences);
        sequenceComboBox.setEnabled(false);
        sequenceComboBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                @SuppressWarnings("unchecked")
                JComboBox<String> cb = (JComboBox<String>)e.getSource();
                sequenceId = cb.getSelectedIndex();
            }
        });
        south.add(sequenceComboBox);

        trackButton = new JButton("Track");
        trackButton.setEnabled(false);
        trackButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                tracker = new VpMbGenericTracker(1, trackerType);

                VpHomogeneousMatrix cMo_init = new VpHomogeneousMatrix();
                if (sequenceId == 0 || sequenceId == 1) {
                    //Cube
                    firstFrame = 0;
                    String rootDir = new String(vispInputImagePath + "/mbt/");
                    String configFilename = new String(rootDir + "cube.xml");
                    tracker.loadConfigFile(configFilename);
                    tracker.getCameraParameters(cam);

                    String modelFilename = new String(rootDir + (sequenceId == 0 ? "cube.cao" : "cube_and_cylinder.cao"));
                    tracker.loadModel(modelFilename);
                    tracker.setDisplayFeatures(true);

                    input = new String(rootDir + "cube/image%04d.pgm");
                    cMo_init = new VpHomogeneousMatrix(0.02231950571, 0.1071368004, 0.5071128378,
                            2.100485509, 1.146812236, -0.4560126437);
                } else {
                    //Castle
                    firstFrame = 1;
                    String rootDir = new String(vispInputImagePath + "/mbt-depth/Castle-simu/");
                    String configFilename = new String(rootDir + "Config/chateau.xml");
                    tracker.loadConfigFile(configFilename);
                    tracker.getCameraParameters(cam);

                    String modelFilename = new String(rootDir + "Models/chateau.cao");
                    tracker.loadModel(modelFilename);
                    modelFilename = new String(rootDir + "Models/cube.cao");
                    VpHomogeneousMatrix T = new VpHomogeneousMatrix(-0.2, 0.12, -0.15, 0, 2.221441469, 2.221441469);
                    tracker.loadModel(modelFilename, false, T);
                    tracker.setDisplayFeatures(true);

                    input = new String(rootDir + "Images/Image_%04d.pgm");
                    cMo_init = new VpHomogeneousMatrix(0.05000004917, 0.1058986038, 0.6010702848, -2.705260346, 0, 0);
                }

                String imgFilename = String.format(input, firstFrame);
                VpImageIo.read(I, imgFilename);
                tracker.initFromPose(I, cMo_init);

                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        for (int idx = firstFrame;; idx++) {
                            String imgFilename = String.format(input, idx);
                            System.out.println("\nimgFilename: " + imgFilename);
                            if (!new File(imgFilename).isFile()) {
                                break;
                            }
                            VpImageIo.read(I, imgFilename);
                            BufferedImage tmp = toBufferedImage(I);

                            long start = System.currentTimeMillis();
                            tracker.track(I);
                            VpHomogeneousMatrix cMo = new VpHomogeneousMatrix();
                            tracker.getPose(cMo);
                            long elapsedTimeMillis = System.currentTimeMillis()-start;

                            System.out.println("cMo:\n" + cMo);
                            double[][] models = tracker.getModelForDisplay(I.cols(), I.rows(), cMo, cam, false);
                            double[][] features = tracker.getFeaturesForDisplay();

                            try {
                                SwingUtilities.invokeAndWait(new Runnable() {
                                    @Override
                                    public void run() {
                                        if (tmp.getColorModel().getColorSpace().getType() == ColorSpace.TYPE_RGB) {
                                            canvas = tmp;
                                        } else {
                                            Graphics2D g2d = canvas.createGraphics();
                                            g2d.drawImage(tmp, 0, 0, null);
                                            g2d.dispose();
                                        }

                                        canvasLabel.setIcon(new ImageIcon(canvas));

                                        displayFrame(canvas, cMo, cam, 0.05, 3);
                                        displayText(canvas, new String("Computation time: " + String.valueOf(elapsedTimeMillis) + " ms"),
                                                20, 20, Color.RED, 1, 1.5F);

                                        for (double[] model : models) {
                                            if (model[0] == 0) {
                                                displayLine(canvas, model[1], model[2], model[3], model[4], Color.RED, 3);
                                            }
                                        }

                                        for (double[] feature : features) {
                                            if (feature[0] == 0) {
                                                Color color = Color.YELLOW;
                                                if (feature[3] == 0) {
                                                    color = Color.GREEN;
                                                } else if (feature[3] == 1) {
                                                    color = Color.BLUE;
                                                } else if (feature[3] == 2) {
                                                    color = Color.PINK;
                                                } else if (feature[3] == 3) {
                                                    color = Color.RED;
                                                } else if (feature[3] == 4) {
                                                    color = Color.CYAN;
                                                }
                                                displayCross(canvas, feature[1], feature[2], 3, color, 1);
                                            } else if (feature[0] == 1) {
                                                displayCross(canvas, feature[1], feature[2], 10, Color.RED, 1);
                                                displayText(canvas, String.valueOf(feature[5]), feature[3], feature[4], Color.RED, 1, 1);
                                            }
                                        }

                                        repaint();
                                        pack();
                                    }
                                });
                            } catch (InvocationTargetException | InterruptedException e) {
                                e.printStackTrace();
                            }

                            try {
                                Thread.sleep(30);
                            } catch (InterruptedException e1) {
                                e1.printStackTrace();
                            }
                        }
                    }
                }).start();
            }
        });
        south.add(trackButton);

        JComboBox<String> trackerTypeComboBox = new JComboBox<>(trackerTypes);
        trackerTypeComboBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                @SuppressWarnings("unchecked")
                JComboBox<String> cb = (JComboBox<String>)e.getSource();
                trackerType = cb.getSelectedIndex()+1;
            }
        });
        south.add(trackerTypeComboBox);

        newContentPane.add(south, c);
        setContentPane(newContentPane);

        setMinimumSize(new Dimension(640, 480));
        pack();
        setLocationRelativeTo(null);
        setVisible(true);
    }

    public static BufferedImage toBufferedImage(VpImageUChar image) {
        int type = BufferedImage.TYPE_BYTE_GRAY;
        byte[] b = image.getPixels(); // get all the pixels
        BufferedImage I = new BufferedImage(image.cols(), image.rows(), type);
        final byte[] targetPixels = ((DataBufferByte) I.getRaster().getDataBuffer()).getData();
        System.arraycopy(b, 0, targetPixels, 0, b.length);
        return I;
    }

    public void displayCross(BufferedImage I, double i, double j, int crossSize, Color color, int thickness) {
        displayCross(I, (int) i, (int) j, crossSize, color, thickness);
    }

    public void displayCross(BufferedImage I, int i, int j, int crossSize, Color color, int thickness) {
        Graphics2D g = I.createGraphics();
        g.setStroke(new BasicStroke(thickness));
        g.setColor(color);

        int i1 = i - crossSize/2;
        int j1 = j;
        int i2 = i + crossSize/2;
        int j2 = j;
        g.drawLine(j1, i1, j2, i2);

        i1 = i;
        j1 = j - crossSize/2;
        i2 = i;
        j2 = j + crossSize/2;
        g.drawLine(j1, i1, j2, i2);
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
        displayArrow(I, (int) i1, (int) j1, (int) i2, (int) j2,color,w,h,thickness);
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

    public void displayText(BufferedImage I, String text, double i, double j, Color color, int thickness, float fontSizeScale) {
        displayText(I, text, (int) i, (int) j, color, thickness, fontSizeScale);
    }

    public void displayText(BufferedImage I, String text, int i, int j, Color color, int thickness, float fontSizeScale) {
        Graphics2D g = I.createGraphics();
        g.setStroke(new BasicStroke(thickness));
        g.setColor(color);
        Font currentFont = g.getFont();
        Font newFont = currentFont.deriveFont(currentFont.getSize() * fontSizeScale);
        g.setFont(newFont);
        g.drawString(text, j, i);
    }

    public static void main(String[] args) {
        //Schedule a job for the event-dispatching thread:
        //creating and showing this application's GUI.
        javax.swing.SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                new GenericTracker();
            }
        });
    }
}
