package org.visp.core;

// C++: class vpImage<unsigned char>

public class VpImageUChar {

    public final long nativeObj;

    public VpImageUChar(long addr){
        if (addr == 0)
            throw new java.lang.UnsupportedOperationException("Native object address is NULL");
        nativeObj = addr;
    }
    
    // C++: vpImage::vpImage<uchar>()
    public VpImageUChar() {
    	nativeObj = n_VpImageUChar();
    }
    
    // C++: vpImage::vpImage<uchar>(int rows, int cols)
    public VpImageUChar(int rows, int cols) {
    	nativeObj = n_VpImageUChar(rows,cols);
    }
    
    // C++: vpImage::vpImage<uchar>(int rows, int cols, uchar val)
    public VpImageUChar(int rows, int cols, byte b) {
    	nativeObj = n_VpImageUChar(rows,cols,b);
    }
    
    // C++: vpImage<uchar>(Type *const array, int height, int width, bool copyData=false)
    // The byte array would be read from a stream
    public VpImageUChar(byte[] array, int height, int width, boolean copyData) {
    	nativeObj = n_VpImageUChar(array, height, width, copyData);
    }
    
    // C++: vpImage::vpImage<uchar>::getCols()
    public int cols() {
        return n_cols(nativeObj);
    }

    // C++: vpImage::vpImage<uchar>::getRows()
    public int rows() {
        return n_rows(nativeObj);
    }

    // C++: vpImage::vpImage<uchar>::operator()
    public int getPixel(int i, int j) {
        return n_getPixel(nativeObj, i, j);
    }

	// C++: vpImage:: Type *bitmap
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
    
    // C++: vpImage::vpImage<uchar>()
    private static native long n_VpImageUChar();
    
    // C++: vpImage::vpImage(int rows, int cols)
    private static native long n_VpImageUChar(int rows, int cols);
    
    // C++: vpImage::vpImage(int rows, int cols, uchar val)
    private static native long n_VpImageUChar(int rows, int cols, byte b);
    
    // C++: vpImage::vpImage(Type const* array, int rows, int cols, bool copyData)
    private static native long n_VpImageUChar(byte[] array, int rows, int cols, boolean copyData);
    
    // C++: int vpImage::cols()
    private static native int n_cols(long nativeObj);

    // C++: int vpImage::rows()
    private static native int n_rows(long nativeObj);

    // C++: int vpImage::operator()
    private static native int n_getPixel(long nativeObj, int i, int j);

    // C++: int vpImage:: Type *bitmap
    private static native byte[] n_getPixels(long nativeObj);

    // C++: String <<&(ostream ss)
    private static native String n_dump(long nativeObj);

}
