package org.visp.core;

// C++: class vpArray2D<double>

public class VpArray2D {

    public final long nativeObj;

    public VpArray2D(long addr){
        if (addr == 0)
            throw new java.lang.UnsupportedOperationException("Native object address is NULL");
        nativeObj = addr;
    }
    
    // C++: vpArray2D::vpArray2D<double>()
    public VpArray2D() {
    	nativeObj = n_VpArray2D();
    }
    
    // C++: vpArray2D<double>(unsigned int r, unsigned int c, Type val)
    public VpArray2D(int r, int c, double val) {
    	nativeObj = n_VpArray2D(r,c,val);
    }
    
    // C++: vpArray2D::vpArray2D(int rows, int cols)
    public VpArray2D(int rows, int cols) {
    	nativeObj = n_VpArray2D(rows,cols);
    }
    
    // C++: VpArray2D::getCols()
    public int cols() {
        return n_cols(nativeObj);
    }

    // C++: VpArray2D::getRows()
    public int rows() {
        return n_rows(nativeObj);
    }

    public long getNativeObjAddr() {
        return nativeObj;
    }
    
    // C++: vpArray2D::vpArray2D,Type>()
    private static native long n_VpArray2D();
    
    // C++: vpArray2D::vpArray2D(int rows, int cols)
    private static native long n_VpArray2D( int rows, int cols);
    
    // C++: vpArray2D::vpArray2D(int rows, int cols, double value)
    private static native long n_VpArray2D(int rows, int cols, double value);
    
    // C++: int vpArray2D::cols()
    private static native int n_cols(long nativeObj);

    // C++: int vpArray2D::rows()
    private static native int n_rows(long nativeObj);

}
