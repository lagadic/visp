package org.visp.core;

// C++: class vpColor	
public class VpColor extends VpRGBa {

    public VpColor(){
		super();
    }

	public VpColor(char r, char g, char b){
        super(r,g,b);
    }

	public VpColor(int r, int g, int b){
        super((char) r,(char) g,(char) b);
    }

	public VpColor(char value){
		super(value);
	}

	public VpColor(int value){
		super((char) value);
	}

    @Override
	public boolean equals(Object color){

		if (color == this)
			return true;

		if (!(color instanceof VpColor))
			return false;

		VpColor c = (VpColor) color;

		return (super.R == c.R) && (super.G == c.G) && (super.B == c.B);
	}

	@Override
    public String toString(){
        return "(" + (int) super.R + "," + (int) super.G + "," + (int) super.B + ")";
    }
}
