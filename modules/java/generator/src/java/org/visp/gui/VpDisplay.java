package org.visp.gui;

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
