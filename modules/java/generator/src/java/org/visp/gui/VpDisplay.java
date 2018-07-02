package org.visp.gui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.visp.core.VpImageUChar;
import org.visp.core.VpImageRGBa;

public class VpDisplay {
	
	// Java developers will find it easy to handle frames directly
	public JFrame frame;
	
	public VpDisplay() {
		frame = new JFrame();
	}
	
	public static Image toBufferedImage(VpImageUChar image) {
		int type = BufferedImage.TYPE_BYTE_GRAY;
		byte[] b = image.getPixels(); // get all the pixels
		BufferedImage I = new BufferedImage(image.cols(), image.rows(), type);
		final byte[] targetPixels = ((DataBufferByte) I.getRaster().getDataBuffer()).getData();
		System.arraycopy(b, 0, targetPixels, 0, b.length);
		return I;
	}

	public static Image toBufferedImage(VpImageRGBa image) {
		int type = BufferedImage.TYPE_4BYTE_ABGR;
		byte[] b = image.getPixels(); // get all the pixels
		BufferedImage I = new BufferedImage(image.cols(), image.rows(), type);
		final byte[] targetPixels = ((DataBufferByte) I.getRaster().getDataBuffer()).getData();

		// Note that in c++, visp returns RGBA format
		// While java accepts ABGR, exactly the opposite for each pixel
		// That's why this strange allocation
		for (int j = 0; j < targetPixels.length/4; j++) {
			targetPixels[4*j] = b[4*j + 3];
			targetPixels[4*j + 1] = b[4*j + 2];
			targetPixels[4*j + 2] = b[4*j + 1];
			targetPixels[4*j+3] = b[4*j];
		}
		return I;
	}
	
	public void display(VpImageUChar imageUChar) {
		ImageIcon image = new ImageIcon(toBufferedImage(imageUChar));
		JLabel imageLabel = new JLabel(image);
		frame.add(imageLabel);
		frame.pack(); // pack screen size to fit content
		frame.setVisible(true);
		frame.setFocusable(true);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}

	public void display(VpImageRGBa imageRGBa) {
		ImageIcon image = new ImageIcon(toBufferedImage(imageRGBa));
		JLabel imageLabel = new JLabel(image);
		frame.add(imageLabel);
		frame.pack(); // pack screen size to fit content
		frame.setVisible(true);
		frame.setFocusable(true);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}
	
	public void setTitle(String title) {
		frame.setTitle(title);
	}
	
	public void displayLine(VpImageUChar imageUChar, int i1, int j1, int i2, int j2) {
		displayLine(imageUChar,i1,j1,i2,j2,Color.RED,1); // default thickness 1. default color RED
	}
	
	public void displayLine(VpImageUChar imageUChar, int i1, int j1, int i2, int j2, Color color) {
		displayLine(imageUChar,i1,j1,i2,j2,color,1); // default thickness 1.
	}

	public void displayLine(VpImageUChar imageUChar, int i1, int j1, int i2, int j2,Color color,int thickness) {
		BufferedImage in = (BufferedImage) toBufferedImage(imageUChar);

		Graphics2D g = in.createGraphics();
		g.setStroke(new BasicStroke(thickness));
		g.setColor(color);
		g.drawLine(i1, j1, i2, j2);

		ImageIcon image = new ImageIcon(in);
		JLabel imageLabel = new JLabel(image);
		frame = new JFrame();
		frame.add(imageLabel);
		frame.pack(); // pack screen size to fit content
		frame.setVisible(true);
		frame.setFocusable(true);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}
	
	public void displayLine(VpImageRGBa imageRGBa, int i1, int j1, int i2, int j2) {
		displayLine(imageRGBa,i1,j1,i2,j2,Color.RED,1); // default thickness 1. default color RED
	}
	
	public void displayLine(VpImageRGBa imageRGBa, int i1, int j1, int i2, int j2, Color color) {
		displayLine(imageRGBa,i1,j1,i2,j2,color,1); // default thickness 1.
	}

	public void displayLine(VpImageRGBa imageRGBa, int i1, int j1, int i2, int j2,Color color,int thickness) {
		BufferedImage in = (BufferedImage) toBufferedImage(imageRGBa);

		Graphics2D g = in.createGraphics();
		g.setStroke(new BasicStroke(thickness));
		g.setColor(color);
		g.drawLine(i1, j1, i2, j2);

		ImageIcon image = new ImageIcon(in);
		JLabel imageLabel = new JLabel(image);
		frame = new JFrame();
		frame.add(imageLabel);
		frame.pack(); // pack screen size to fit content
		frame.setVisible(true);
		frame.setFocusable(true);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}

	public void displayArrow(VpImageUChar imageUChar, int i1, int j1, int i2, int j2,Color color,int w, int h, int thickness) {
		BufferedImage in = (BufferedImage) toBufferedImage(imageUChar);

		Graphics2D g = in.createGraphics();
		g.setStroke(new BasicStroke(thickness));
		g.setColor(color);
		g.drawLine(i1, j1, i2, j2);
		double m = (j2-j1)*1.0/(i2-i1), x1 = i2 - h/Math.sqrt(1+m*m), y1 = j2 - h*m/Math.sqrt(1+m*m);
		
		g.drawLine((int) (x1 + m*w/(2*Math.sqrt(1+m*m))), (int) (y1 - w/(2*Math.sqrt(1+m*m))), i2, j2);
		g.drawLine((int) (x1 - m*w/(2*Math.sqrt(1+m*m))), (int) (y1 + w/(2*Math.sqrt(1+m*m))), i2, j2);

		ImageIcon image = new ImageIcon(in);
		JLabel imageLabel = new JLabel(image);
		frame = new JFrame();
		frame.add(imageLabel);
		frame.pack(); // pack screen size to fit content
		frame.setVisible(true);
		frame.setFocusable(true);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}
	
	public void displayArrow(VpImageRGBa imageRGBa, int i1, int j1, int i2, int j2,Color color,int w, int h, int thickness) {
		BufferedImage in = (BufferedImage) toBufferedImage(imageRGBa);

		Graphics2D g = in.createGraphics();
		g.setStroke(new BasicStroke(thickness));
		g.setColor(color);
		g.drawLine(i1, j1, i2, j2);
		double m = (j2-j1)*1.0/(i2-i1), x1 = i2 - h/Math.sqrt(1+m*m), y1 = j2 - h*m/Math.sqrt(1+m*m);
		
		g.drawLine((int) (x1 + m*w/(2*Math.sqrt(1+m*m))), (int) (y1 - w/(2*Math.sqrt(1+m*m))), i2, j2);
		g.drawLine((int) (x1 - m*w/(2*Math.sqrt(1+m*m))), (int) (y1 + w/(2*Math.sqrt(1+m*m))), i2, j2);

		ImageIcon image = new ImageIcon(in);
		JLabel imageLabel = new JLabel(image);
		frame = new JFrame();
		frame.add(imageLabel);
		frame.pack(); // pack screen size to fit content
		frame.setVisible(true);
		frame.setFocusable(true);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}
	
	/*
	 * Exit on a mouse click
	 */
	public void getClick() {
		frame.addMouseListener(new MouseListener() {
		    @Override
		    public void mouseClicked(MouseEvent e) {
		    	frame.dispatchEvent(new WindowEvent(frame, WindowEvent.WINDOW_CLOSING));
		    }

			@Override
			public void mouseEntered(MouseEvent arg0) {}
			@Override
			public void mouseExited(MouseEvent arg0) {}
			@Override
			public void mousePressed(MouseEvent arg0) {}
			@Override
			public void mouseReleased(MouseEvent arg0) {}
		});
	}
}
