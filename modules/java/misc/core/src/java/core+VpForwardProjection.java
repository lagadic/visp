package org.visp.core;

import java.util.Arrays;

public class VpForwardProjection extends VpTracker{
	public VpColVector oP;
	
	public VpForwardProjection() {
		super();
		oP = new VpColVector();
	}
	
	public void init() {
		super.init();
	}
	
	public void print() {
		System.out.println(Arrays.toString(p.data.toArray()));
		System.out.println(Arrays.toString(cP.data.toArray()));
		System.out.println(Arrays.toString(oP.data.toArray()));
	}
}
