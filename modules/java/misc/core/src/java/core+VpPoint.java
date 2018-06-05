package org.visp.core;

public class VpPoint extends VpForwardProjection{
	public void init() {
		p.resize(3);
		p.setAllTo(0);
		p.set(2, 1);
		
		oP.resize(4);
		oP.setAllTo(0);
		oP.set(3, 1);
		
		cP.resize(4);
		cP.setAllTo(0);
		cP.set(3, 1);
		
		setZ(1);
	}
	
	public VpPoint() {
		init();
	}
	
//	public void changeFrame(VpHomogenousMatrix cMo) {
//		double X = cMo[0][0] * oP[0] + cMo[0][1] * oP[1] + cMo[0][2] * oP[2] + cMo[0][3] * oP[3];
//		double Y = cMo[1][0] * oP[0] + cMo[1][1] * oP[1] + cMo[1][2] * oP[2] + cMo[1][3] * oP[3];
//		double Z = cMo[2][0] * oP[0] + cMo[2][1] * oP[1] + cMo[2][2] * oP[2] + cMo[2][3] * oP[3];
//		double W = cMo[3][0] * oP[0] + cMo[3][1] * oP[1] + cMo[3][2] * oP[2] + cMo[3][3] * oP[3];
//
//		double d = 1 / W;
//		cP.set(0, X * d);
//		cP.set(1, Y * d);
//		cP.set(2, Z * d);
//		cP.set(3, 1);
//	}
	
	public VpPoint(double oX,double oY,double oZ) {
		init();
		setWorldCoordinates(oX, oY, oZ);
	}
	
	public void setWorldCoordinates(double oX,double oY,double oZ) {
		oP.set(0, oX);
		oP.set(1, oY);
		oP.set(2, oZ);
		oP.set(3, 1);
	}
	
	public void projection() {
		double d = 1 / cP.data.get(2);
		p.set(0,cP.data.get(0) * d);
		p.set(1,cP.data.get(1) * d);
		p.set(2,1);
	}
	
	public VpPoint(VpPoint vpp) {
		p = vpp.p;
		cP = vpp.cP;
		oP = vpp.oP;
		cPAvailable = vpp.cPAvailable;
	}
	
	void setX(double d) {
		cP.set(0, d);
	}
	
	void setY(double d) {
		cP.set(1, d);
	}

	void setZ(double d) {
		cP.set(2, d);
	}
	
	void setW(double d) {
		cP.set(3, d);
	}
	
	void setOX(double d) {
		oP.set(0, d);
	}
	
	void setOY(double d) {
		oP.set(1, d);
	}

	void setOZ(double d) {
		oP.set(2, d);
	}
	
	void setOW(double d) {
		oP.set(3, d);
	}
	
	void setx(double d) {
		p.set(0, d);
	}
	
	void sety(double d) {
		p.set(1, d);
	}

	void setz(double d) {
		p.set(2, d);
	}
	
	public double getx() {
		return p.data.get(0);
	}
	
	public double gety() {
		return p.data.get(1);
	}

	public double getz() {
		return p.data.get(2);
	}
}
