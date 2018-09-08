package org.visp.core;

// C++: class vpImage<vpRGBa>

public class VpImageRGBa {

    public final long nativeObj;

    public VpImageRGBa(long addr){
        if (addr == 0)
            throw new java.lang.UnsupportedOperationException("Native object address is NULL");
        nativeObj = addr;
    }
    
    // C++: vpImage::vpImage<vpRGBa>()
    public VpImageRGBa() {
    	nativeObj = n_VpImageRGBa();
    }
    
    // C++: vpImage<vpRGBa>(unsigned int r, unsigned int c)
    public VpImageRGBa(int r, int c) {
    	nativeObj = n_VpImageRGBa(r,c);
    }
    
    // C++: vpImage<vpRGBa>(unsigned int r, unsigned int c, vpRGBa val)
    public VpImageRGBa(int r, int c, VpRGBa val) {
    	nativeObj = n_VpImageRGBa(r,c,val.R,val.G,val.B,val.A);
    }
    
    // C++: vpImage<vpRGBa>(Type *const array, int height, int width, bool copyData=false)
    // The byte array would be read from a stream
    public VpImageRGBa(byte[] array, int height, int width, boolean copyData) {
    	nativeObj = n_VpImageRGBa( array, height, width, copyData);
    }
    
    // C++: vpImage<vpRGBa>::getCols()
    public int cols() {
        return n_cols(nativeObj);
    }

    // C++: vpImage<vpRGBa>::getRows()
    public int rows() {
        return n_rows(nativeObj);
    }

    // C++: vpImage<vpRGBa>::operator()
    public VpRGBa getPixel(int i, int j) {
		byte res[] = n_getPixel(nativeObj, i, j);
		VpRGBa val = new VpRGBa((char) (res[0] >= 0?res[0]:res[0]+256),
								(char) (res[1] >= 0?res[1]:res[1]+256),
								(char) (res[2] >= 0?res[2]:res[2]+256),
								(char) (res[3] >= 0?res[3]:res[3]+256));
		return val;
    }

	// C++: vpImage<vpRGBa>:: <vpRGBa> *bitmap
    public byte[] getPixels() {
        return n_getPixels(nativeObj);
    }

    public long getNativeObjAddr() {
        return nativeObj;
    }
    
	@Override
    public String toString(){
    	return n_dump(nativeObj);
    }
    
    // C++: vpImage::vpImage<vpRGBa>()
    private static native long n_VpImageRGBa();
    
    // C++: vpImage::vpImage<vpRGBa>(int rows, int cols)
    private static native long n_VpImageRGBa(int rows, int cols);

    // C++: vpImage::vpImage<vpRGBa>(int rows, int cols, vpRGBa val)
    private static native long n_VpImageRGBa(int rows, int cols, char R, char G, char B, char A);
    
    // C++: vpImage::vpImage<vpRGBa>(Type const* array, int rows, int cols, bool copyData)
    private static native long n_VpImageRGBa(byte[] array, int rows, int cols, boolean copyData);
    
    // C++: int vpImage::cols()
    private static native int n_cols(long nativeObj);

    // C++: int vpImage::rows()
    private static native int n_rows(long nativeObj);

    // C++: int vpImage<vpRGBa>::operator()
    private static native byte[] n_getPixel(long nativeObj, int i, int j);

    // C++: int vpImage<vpRGBa>:: Type *bitmap
    private static native byte[] n_getPixels(long nativeObj);

    // C++: String <<&(ostream ss)
    private static native String n_dump(long nativeObj);

}
