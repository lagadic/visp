package org.visp.core;

// C++: vpImagePoint

public class VpImagePoint {

	public double i,j;
	
	public VpImagePoint() {
		this(0,0);
	}
	
	public VpImagePoint(double i, double j) {
		this.i = i;
		this.j = j;
	}
	
	public VpImagePoint(VpImagePoint other) {
		this(other.i,other.i);
	}
	
	public void add(VpImagePoint other) {
		this.i += other.i;
		this.j += other.j;
	}
	
	public void subtract(VpImagePoint other) {
		this.i -= other.i;
		this.j -= other.j;
	}
	
	public void multiply(VpImagePoint other) {
		this.i *= other.i;
		this.j *= other.j;
	}
	
	public void divide(VpImagePoint other) {
		this.i /= other.i;
		this.j /= other.j;
	}
	
	public double distance(VpImagePoint other) {
		return Math.sqrt(Math.pow(this.i - other.i,2) + Math.pow(this.j - other.j,2)); 
	}
	
	@Override
	public String toString(){
		return "(" + i + ", " + j + ")";
	}
	
	@Override
	public boolean equals(Object o){

		if (o == this)
            return true;

		if (!(o instanceof VpImagePoint))
            return false;

		VpImagePoint other = (VpImagePoint) o;
		return this.i == other.i && this.j == other.j;  
	}
}
