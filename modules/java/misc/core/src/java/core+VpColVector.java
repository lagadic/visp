package org.visp.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

// C++: class vpColVector

public class VpColVector {

    public List<Double> data;

    public VpColVector(){
		data = new ArrayList<Double>();
	}
    
    // C++: vpColVector::vpColVector(int n)
    public VpColVector(int n) {
		try {
			data = new ArrayList<Double>(n);
		}catch (Exception e) {
			e.printStackTrace();
		}
    }
    
    // C++: vpColVector::vpColVector(int n, double d)
    public VpColVector(int n, double d) {
		try {
			data = new ArrayList<Double>(Collections.nCopies(n, d));
		}catch (Exception e) {
			e.printStackTrace();
		}
    }

	public int size(){
		return data.size();
	}
    
    // C++: vpColVector vpColVector::operator-(vpColVector const&) const
    public VpColVector subtract(VpColVector other) {
    	try {
    		// what if other.size() != data.size()
			VpColVector res = new VpColVector(other.size());
			for(int i=0;i<other.size();++i)
				res.data.add(data.get(i) - other.data.get(i));
			return res;
		} catch (Exception e) {
			 e.printStackTrace();
		}
    	return null;
    }

	// C++: vpColVector vpColVector::operator+(vpColVector const&) const
    public VpColVector add(VpColVector other) {
		try{
			// what if other.size() != data.size()
			VpColVector res = new VpColVector(other.size());
			for(int i=0;i<other.size();++i)
				res.data.add(data.get(i) + other.data.get(i));
			return res;
		} catch (Exception e) {
			 e.printStackTrace();
		}
   	return null;
    }

	// C++: void vpColVector::operator=(double)
    public void setAllTo(double d) {
		for(int i=0;i<data.size();++i)
			data.set(i,d);
    }

	// C++: void vpColVector::operator=(vpColVector const&)
    public void setTo(VpColVector other) {
		data.clear();
		data.addAll(other.data);
    }

	// C++: double vpColVector::sum()
	public double sum() {
		double sum = 0;
		for (double i: data)
		    sum += i;
		return sum;
	}

	// C++: double vpColVector::sumSquare()
	public double sumSquare() {
		double sum = 0;
		for (double i: data)
		    sum += i*i;
		return sum;
	}

	// C++: void vpColVector::operator*=(double)
	public void multiply(double d){
		for (int i=0;i<data.size();++i)
		    data.set(i,data.get(i)*d);
	}

	// C++: void resize(int n)
	public void resize(int n) {
		try {
			data = new ArrayList<Double>(Collections.nCopies(n, 0.0));
		}catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	public void set(int n,double d) {
		data.set(n,d);
	}
}
