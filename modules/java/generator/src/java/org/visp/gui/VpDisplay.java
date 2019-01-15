/*
package org.visp.gui;

import java.lang.String;
import java.awt.BasicStroke; // Produces an error: package java.awt does not exist
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
import org.visp.core.VpPoint;
import org.visp.core.VpHomogeneousMatrix;
import org.visp.core.VpCameraParameters;

public class VpDisplay {
	
	// Java developers will find it easy to handle frames directly
	public JFrame frame;
	public BufferedImage I;
	
	public VpDisplay() {
		frame = new JFrame();
		I = null;
	}
	
	public static BufferedImage toBufferedImage(VpImageUChar image) {
		int type = BufferedImage.TYPE_BYTE_GRAY;
		byte[] b = image.getPixels(); // get all the pixels
		BufferedImage I = new BufferedImage(image.cols(), image.rows(), type);
		final byte[] targetPixels = ((DataBufferByte) I.getRaster().getDataBuffer()).getData();
		System.arraycopy(b, 0, targetPixels, 0, b.length);
		return I;
	}

	public static BufferedImage toBufferedImage(VpImageRGBa image) {
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
		I = toBufferedImage(imageUChar);
	}

	public void display(VpImageRGBa imageRGBa) {
		I = toBufferedImage(imageRGBa);
	}
	
	public void setTitle(String title) {
		frame.setTitle(title);
	}
	
	public void displayLine(int i1, int j1, int i2, int j2) {
		displayLine(i1,j1,i2,j2,Color.RED,1); // default thickness 1. default color RED
	}
	
	public void displayLine(int i1, int j1, int i2, int j2, Color color) {
		displayLine(i1,j1,i2,j2,color,1); // default thickness 1.
	}

	public void displayLine(int i1, int j1, int i2, int j2,Color color,int thickness) {
		Graphics2D g = I.createGraphics();
		//g.setStroke(new BasicStroke(thickness));
		g.setColor(color);
		g.drawLine(i1, j1, i2, j2);
	}
	
	public void displayArrow(double i1, double j1, double i2, double j2,Color color,int w, int h, int thickness) {
		displayArrow((int) i1, (int) j1, (int) i2, (int) j2,color,w,h,thickness);
	}

	public void displayArrow(int i1, int j1, int i2, int j2,Color color,int w, int h, int thickness) {
		Graphics2D g = I.createGraphics();
		//g.setStroke(new BasicStroke(thickness));
		g.setColor(color);
		if (i1==i2){
			--i1;
			++i2;
		}
		if (j1==j2){
			--j1;
			++j2;
		}
		g.drawLine(i1, j1, i2, j2);
		double m = (j2-j1)*1.0/(i2-i1), x1 = i2 - h/Math.sqrt(1+m*m), y1 = j2 - h*m/Math.sqrt(1+m*m);

		int x11 = (int) (x1 + m*w/(2*Math.sqrt(1+m*m)));
		if ((x11-i1)*(x11-i2) > 0)
			x11 = 2*i2 - x11;
		
		int y11 = (int) (y1 - w/(2*Math.sqrt(1+m*m)));
		if ((y11-j1)*(y11-j2) > 0)
			y11 = 2*j2 - y11;
		g.drawLine(x11, y11, i2, j2);
		
		x11 = (int) (x1 - m*w/(2*Math.sqrt(1+m*m)));
		if ((x11-i1)*(x11-i2) > 0)
			x11 = 2*i2 - x11;
		y11 = (int) (y1 + w/(2*Math.sqrt(1+m*m)));
		if ((y11-j1)*(y11-j2) > 0)
			y11 = 2*j2 - y11;
		g.drawLine(x11, y11, i2, j2);
	}
	
	public void displayText(String text, int i, int j) {
		displayText(text, i, j, Color.GREEN, 2); // default color green, default thickness 2 
	}
	
	public void displayText(String text, int i, int j, Color color, int thickness) {
		Graphics2D g = I.createGraphics();
		//g.setStroke(new BasicStroke(thickness));
		g.setColor(color);
		g.drawString(text, i, j);
	}
	
	public void displayFrame(VpHomogeneousMatrix mat, VpCameraParameters cam, double size, int thickness) {
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
		
		displayArrow(o.get_x()*cam.get_px() + cam.get_u0(),o.get_y()*cam.get_py() + cam.get_v0(),
				 x.get_x()*cam.get_px() + cam.get_u0(),x.get_y()*cam.get_py() + cam.get_v0(), 
				 Color.RED, 4 * thickness, 2 * thickness, thickness);
		
		displayArrow(o.get_x()*cam.get_px() + cam.get_u0(),o.get_y()*cam.get_py() + cam.get_v0(),
				 y.get_x()*cam.get_px() + cam.get_u0(),y.get_y()*cam.get_py() + cam.get_v0(), 
				 Color.GREEN, 4 * thickness, 2 * thickness, thickness);
		
		displayArrow(o.get_x()*cam.get_px() + cam.get_u0(),o.get_y()*cam.get_py() + cam.get_v0(),
				 z.get_x()*cam.get_px() + cam.get_u0(),z.get_y()*cam.get_py() + cam.get_v0(), 
				 Color.BLUE, 4 * thickness, 2 * thickness, thickness);
	}
	
	// Flushes the output buffer associated to image
	// It's necessary to use this function to see the results of any drawing
	public void flush() {
		ImageIcon image = new ImageIcon(I);
		JLabel imageLabel = new JLabel(image);
		frame = new JFrame();
		frame.add(imageLabel);
		frame.pack(); // pack screen size to fit content
		frame.setVisible(true);
		frame.setFocusable(true);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}
	
	//
	// Exit on a mouse click
	//
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
*/
