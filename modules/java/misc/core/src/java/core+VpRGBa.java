package org.visp.core;

// C++: class vrRGBa	
public class VpRGBa {

	public char R,G,B,A;
	public static final char alphaDefault = 255;

    public VpRGBa(){
		this((char) 0,(char) 0,(char) 0,alphaDefault);
    }

	public VpRGBa(char r,char g, char b){
		this(r,g,b,alphaDefault);
	}

	public VpRGBa(char r,char g, char b, char a){
		this.R = r;
		this.G = g;
		this.B = b;
		this.A = a;
	}

	public VpRGBa(char v){
		this(v,v,v,v);
	}

	public VpRGBa(VpRGBa c){
		this(c.R,c.G,c.B,c.A);
	}

	public void add(VpRGBa other){
		this.R += other.R;
		this.G += other.G;
		this.B += other.B;
		this.A += other.A;
	}

	@Override
	public String toString(){
		return "(" + (int) this.R + "," + (int) this.G + "," + (int) this.B + "," + (int) this.A + ")";
	}

    @Override
	public boolean equals(Object o){

		if (o == this)
            return true;

		if (!(o instanceof VpRGBa))
            return false;

		VpRGBa other = (VpRGBa) o;
		return (this.R == other.R) && (this.G == other.G) && (this.B == other.B) && (this.A == other.A);
	}
}
