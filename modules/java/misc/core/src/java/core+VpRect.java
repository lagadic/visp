package org.visp.core;

public class VpRect {
	private double left,top,width,height;
	
	public VpRect() {
		top = left = 0;
		width = height = 1;
	}
	
	// C++: vpRect (double left, double top, double width, double height)
	public VpRect(double l, double t, double w, double h) {
		left = l;
		top = t;
		width = w;
		height = h;
	}
	
	// C++: vpRect& operator=(const vpRect&)
	public void init(VpRect other) {
		this.top = other.top;
		this.left = other.left;
		this.width = other.width;
		this.height = other.height;
	}	
	
	// C++: operator<<
	public String dump() {
		return left + ", " + top + ", " + width + ", " + height;
	}
}
