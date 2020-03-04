import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dialog;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Point;
import java.awt.color.ColorSpace;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.File;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;

import javax.swing.BoxLayout;
import javax.swing.DefaultCellEditor;
import javax.swing.ImageIcon;
import javax.swing.InputVerifier;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JComponent;
import javax.swing.JDialog;
import javax.swing.JFileChooser;
import javax.swing.JFormattedTextField;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;
import javax.swing.event.PopupMenuEvent;
import javax.swing.event.PopupMenuListener;
import javax.swing.filechooser.FileNameExtensionFilter;
import javax.swing.table.DefaultTableModel;

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
    private VpCameraParameters cam = new VpCameraParameters(615.1674805, 615.1675415, 312.1889954, 243.4373779);
    private static String[] tagFamilyNames = {"TAG_36h11", "TAG_25h9", "TAG_16h5", "TAG_CIRCLE21h7",
            "TAG_CIRCLE49h12", "TAG_CUSTOM48h12", "TAG_STANDARD41h12", "TAG_STANDARD52h13"};
    private static int[] tagFamilies = {0, 3, 5, 6, 7, 8, 9, 10};
    private static String[] poseEstimationMethodNames = {"HOMOGRAPHY", "HOMOGRAPHY_VIRTUAL_VS", "DEMENTHON_VIRTUAL_VS",
            "LAGRANGE_VIRTUAL_VS", "BEST_RESIDUAL_VIRTUAL_VS", "HOMOGRAPHY_ORTHOGONAL_ITERATION"};
    private Object[][] data = { { new Integer(-1), new Double(0.053) } };
    private JTextArea poseArea;

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
        JPanel south1 = new JPanel();
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

                    BufferedImage tmp = toBufferedImage(I);
                    if (tmp.getColorModel().getColorSpace().getType() == ColorSpace.TYPE_RGB) {
                        canvas = tmp;
                    } else {
                        Graphics2D g2d = canvas.createGraphics();
                        g2d.drawImage(tmp, 0, 0, null);
                        g2d.dispose();
                    }

                    canvasLabel.setIcon(new ImageIcon(canvas));
                    repaint();
                    pack();
                }
            }
        });
        south1.add(openButton);

        JButton detectButton = new JButton("Detect");
        detectButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                if (I.cols() * I.rows() > 0) {
                    StringBuilder info = new StringBuilder();
                    if (detector.detect(I)) {
                        int[] tagsId = detector.getTagsId();

                        List<VpHomogeneousMatrix> cMo_vec = new ArrayList<>();
                        for (int i = 0; i < tagsId.length; i++) {
                            VpHomogeneousMatrix cMo = new VpHomogeneousMatrix();
                            double tagSize = Double.parseDouble(data[0][1].toString());
                            for (Object[] d : data) {
                                if (Integer.parseInt(d[0].toString()) == tagsId[i]) {
                                    tagSize = Double.parseDouble(d[1].toString());
                                }
                            }
                            detector.getPose(i, tagSize, cam, cMo);
                            cMo_vec.add(cMo);
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

                        for (int i = 0; i < tagsId.length; i++) {
                            double[] centroid = computeCentroid(tags_corners.get(i));
                            displayText(canvas, new String("Id: " + String.valueOf(tagsId[i])),
                                    centroid[0] + 10, centroid[1] + 20, Color.RED, 3);

                            info.append("Tag id: ");
                            info.append(tagsId[i]);
                            VpHomogeneousMatrix cMo = cMo_vec.get(i);
                            info.append("\ncMo:\n" + cMo);
                            info.append("\n");
                            info.append("\n");
                        }

                        poseArea.setText(info.toString());
                        repaint();
                    }
                }
            }
        });
        south1.add(detectButton);

        JButton sizeButton = new JButton("Set tag size");
        JDialog sizeDialog = new JDialog(this, Dialog.ModalityType.DOCUMENT_MODAL);
        sizeDialog.setTitle("Add tag id <==> tag size");
        sizeDialog.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                e.getWindow().dispose();
            }

        });
        Container sizeContainer = sizeDialog.getContentPane();
        DefaultTableModel tableModel = new DefaultTableModel(data, new Object[]{"Tag Id", "Size"}) {
            private static final long serialVersionUID = 1L;
            @Override
            public boolean isCellEditable(int row, int column) {
                if (row == 0 && column == 0) {
                    return false;
                }
                return true;
            }
        };
        final InputVerifier iv = new InputVerifier() {
            @Override
            public boolean verify(JComponent input) {
                JTextField field = (JTextField) input;
                try {
                    Double.parseDouble(field.getText());
                } catch (NumberFormatException e) {
                    return false;
                }
                return true;
            }

            @Override
            public boolean shouldYieldFocus(JComponent input) {
                boolean valid = verify(input);
                if (!valid) {
                    JOptionPane.showMessageDialog(null, "Invalid input!\nNumber is expected.");
                }
                return valid;
            }

        };
        DefaultCellEditor editor = new DefaultCellEditor(new JTextField()) {
            private static final long serialVersionUID = 1L;
            {
                getComponent().setInputVerifier(iv);
            }

            @Override
            public boolean stopCellEditing() {
                if (!iv.shouldYieldFocus(getComponent())) return false;
                return super.stopCellEditing();
            }

            @Override
            public JTextField getComponent() {
                return (JTextField) super.getComponent();
            }

        };

        JTable table = new JTable(tableModel);
        table.setDefaultEditor(Object.class, editor);
        table.setToolTipText("Double click in a cell to add tag id or tag size. Right click to delete a row.");
        final JPopupMenu popupMenu = new JPopupMenu();
        popupMenu.addPopupMenuListener(new PopupMenuListener() {
            @Override
            public void popupMenuWillBecomeVisible(PopupMenuEvent e) {
                SwingUtilities.invokeLater(new Runnable() {
                    @Override
                    public void run() {
                        int rowAtPoint = table.rowAtPoint(SwingUtilities.convertPoint(popupMenu, new Point(0, 0), table));
                        if (rowAtPoint > -1) {
                            table.setRowSelectionInterval(rowAtPoint, rowAtPoint);
                        }
                    }
                });
            }

            @Override
            public void popupMenuWillBecomeInvisible(PopupMenuEvent e) {
            }

            @Override
            public void popupMenuCanceled(PopupMenuEvent e) {
            }
        });
        JMenuItem deleteItem = new JMenuItem("Delete the row?");
        deleteItem.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                int[] rows = table.getSelectedRows();
                for (int i = 0; i < rows.length; i++) {
                    if (rows[i] > 0) {
                        tableModel.removeRow(rows[i] - i);
                    } else {
                        JOptionPane.showMessageDialog(null, "Tag Id: -1 must not be deleted.");
                    }
                }
                tableModel.fireTableDataChanged();
            }
        });
        popupMenu.add(deleteItem);
        table.setComponentPopupMenu(popupMenu);
        sizeButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                sizeDialog.setLocationRelativeTo(sizeDialog.getParent());
                sizeDialog.setVisible(true);
            }
        });

        sizeDialog.setMinimumSize(new Dimension(300, 200));
        JPanel sizePanel = new JPanel();
        sizePanel.setLayout(new BoxLayout(sizePanel, BoxLayout.Y_AXIS));
        JScrollPane sizeScroll = new JScrollPane(table);
        sizePanel.add(sizeScroll);
        JButton addSize = new JButton("Add size");
        addSize.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                tableModel.addRow(new Object[] {"", ""});
            }
        });

        JButton validateSize = new JButton("Validate");
        validateSize.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                if (table.isEditing()) {
                    table.getCellEditor().stopCellEditing();
                }

                List<Object[]> currentData = new ArrayList<>();
                for (int count = 0; count < table.getModel().getRowCount(); count++) {
                    try {
                        int id = Integer.parseInt(table.getModel().getValueAt(count, 0).toString());
                        double sz = Double.parseDouble(table.getModel().getValueAt(count, 1).toString());
                        currentData.add(new Object[] {id, sz});
                    } catch (NumberFormatException ex) {
                    }
                }
                data = new Object[currentData.size()][2];
                for (int i = 0; i < currentData.size(); i++) {
                    data[i][0] = currentData.get(i)[0];
                    data[i][1] = currentData.get(i)[1];
                }

                sizeDialog.dispose();
            }
        });

        JPanel addSizePanel = new JPanel();
        addSizePanel.add(addSize);
        addSizePanel.add(validateSize);
        sizePanel.add(addSizePanel);
        sizeContainer.add(sizePanel);
        south1.add(sizeButton);

        JButton camButton = new JButton("Set camera parameters");
        JDialog camDialog = new JDialog(this, Dialog.ModalityType.DOCUMENT_MODAL);
        camDialog.setTitle("Set intrinsics");
        camDialog.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                e.getWindow().dispose();
            }

        });
        camButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                camDialog.setLocationRelativeTo(camDialog.getParent());
                camDialog.setVisible(true);
            }
        });
        Container camContainer = camDialog.getContentPane();
        camContainer.setLayout(new BorderLayout());
        //Intrinsics labels
        JLabel pxLabel = new JLabel("px:");
        JLabel pyLabel = new JLabel("py:");
        JLabel u0Label = new JLabel("u0:");
        JLabel v0Label = new JLabel("v0:");
        //Intrinsics input
        JFormattedTextField pxField = new JFormattedTextField(NumberFormat.getNumberInstance());
        pxField.setValue(new Double(cam.get_py()));
        pxField.setColumns(10);
        JFormattedTextField pyField = new JFormattedTextField(NumberFormat.getNumberInstance());
        pyField.setValue(new Double(cam.get_py()));
        pyField.setColumns(10);
        JFormattedTextField u0Field = new JFormattedTextField(NumberFormat.getNumberInstance());
        u0Field.setValue(new Double(cam.get_u0()));
        u0Field.setColumns(10);
        JFormattedTextField v0Field = new JFormattedTextField(NumberFormat.getNumberInstance());
        v0Field.setValue(new Double(cam.get_v0()));
        v0Field.setColumns(10);
        //Tell accessibility tools about label/textfield pairs.
        pxLabel.setLabelFor(pxField);
        pyLabel.setLabelFor(pyField);
        u0Label.setLabelFor(u0Field);
        v0Label.setLabelFor(v0Field);
        //Lay out the labels in a panel.
        JPanel camLabelPane = new JPanel(new GridLayout(0,1));
        camLabelPane.add(pxLabel);
        camLabelPane.add(pyLabel);
        camLabelPane.add(u0Label);
        camLabelPane.add(v0Label);
        //Layout the text fields in a panel.
        JPanel camFieldPane = new JPanel(new GridLayout(0,1));
        camFieldPane.add(pxField);
        camFieldPane.add(pyField);
        camFieldPane.add(u0Field);
        camFieldPane.add(v0Field);
        camContainer.add(camLabelPane, BorderLayout.CENTER);
        camContainer.add(camFieldPane, BorderLayout.LINE_END);
        JButton camOk = new JButton("Ok");
        camOk.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                cam = new VpCameraParameters(((Number)pxField.getValue()).doubleValue(),
                        ((Number)pyField.getValue()).doubleValue(),
                        ((Number)u0Field.getValue()).doubleValue(),
                        ((Number)v0Field.getValue()).doubleValue());
                camDialog.dispose();
            }
        });
        JButton camNok = new JButton("Cancel");
        camNok.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                camDialog.dispose();
            }
        });

        JPanel camButtonPane = new JPanel();
        camButtonPane.add(camOk);
        camButtonPane.add(camNok);
        camContainer.add(camButtonPane, BorderLayout.PAGE_END);
        camDialog.setMinimumSize(new Dimension(180, 120));
        south1.add(camButton);

        JButton poseButton = new JButton("Show estimated poses");
        JDialog poseDialog = new JDialog(this, Dialog.ModalityType.APPLICATION_MODAL);
        poseDialog.setTitle("Show estimated poses");
        poseDialog.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                e.getWindow().dispose();
            }

        });
        poseButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                poseDialog.setLocationRelativeTo(poseDialog.getParent());
                poseDialog.setVisible(true);
            }
        });
        Container poseContainer = poseDialog.getContentPane();
        poseArea = new JTextArea();
        poseArea.setEditable(false);
        poseContainer.add(new JScrollPane(poseArea));

        poseDialog.setMinimumSize(new Dimension(450, 400));
        south1.add(poseButton);

        JPanel south2 = new JPanel();
        JComboBox<String> tagFamilyComboBox = new JComboBox<>(tagFamilyNames);
        tagFamilyComboBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                @SuppressWarnings("unchecked")
                JComboBox<String> cb = (JComboBox<String>)e.getSource();
                detector.setAprilTagFamily(tagFamilies[cb.getSelectedIndex()]);
            }
        });
        south2.add(tagFamilyComboBox);

        JComboBox<String> poseEstimationComboBox = new JComboBox<>(poseEstimationMethodNames);
        poseEstimationComboBox.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                @SuppressWarnings("unchecked")
                JComboBox<String> cb = (JComboBox<String>)e.getSource();
                detector.setAprilTagPoseEstimationMethod(cb.getSelectedIndex());
            }
        });
        south2.add(poseEstimationComboBox);

        JPanel south = new JPanel();
        south.setLayout(new BoxLayout(south, BoxLayout.Y_AXIS));
        south.add(south1);
        south.add(south2);
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

    public BufferedImage toBufferedImage(VpImageUChar image) {
        int type = BufferedImage.TYPE_BYTE_GRAY;
        byte[] b = image.getPixels(); // get all the pixels
        BufferedImage I = new BufferedImage(image.cols(), image.rows(), type);
        final byte[] targetPixels = ((DataBufferByte) I.getRaster().getDataBuffer()).getData();
        System.arraycopy(b, 0, targetPixels, 0, b.length);
        return I;
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
