package org.visp.core;

public class VpTracker {
	public VpColVector p,cP;
	public boolean cPAvailable;
	
	public VpTracker() {
		cPAvailable = false;
		p = new VpColVector();
		cP = new VpColVector();
	}
	
	public void init() {
		cPAvailable = false;
	}

	public VpTracker(VpTracker tracker){
		p = tracker.p;
		cP = tracker.cP;
		cPAvailable = tracker.cPAvailable;
	}
}
