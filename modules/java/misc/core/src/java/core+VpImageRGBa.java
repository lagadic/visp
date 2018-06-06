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
    
    // C++: vpImage (Type *const array, int height, int width, bool copyData=false)
    // The byte array would be read from a stream
    public VpImageRGBa(byte[] array, int height, int width, boolean copyData) {
    	nativeObj = n_VpImageRGBa( array, height, width, copyData);
    }
    
    // C++: VpImageRGBa::getCols()
    public int cols() {
        return n_cols(nativeObj);
    }

    // C++: VpImageRGBa::getRows()
    public int rows() {
        return n_rows(nativeObj);
    }

    public long getNativeObjAddr() {
        return nativeObj;
    }
    
    // Prints current image
    public String dump(){
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

    // C++: String <<&(ostream ss)
    private static native String n_dump(long nativeObj);

}
