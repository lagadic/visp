package org.visp.core;

// C++: class vpMatrix
public class VpMatrix {

    public final long nativeObj;

    // C++: vpMatrix::vpMatrix(int rows, int cols, double value)
    public VpMatrix(int rows, int cols, double value) {
    	nativeObj = n_vpMatrix(rows,cols,value);
    }
    
    // C++: vpMatrix::getCols()
    public int cols() {
        return n_cols(nativeObj);
    }

    // C++: vpMatrix::vpMatrix(int rows, int cols, double value)
    private static native long n_vpMatrix(int rows, int cols, double value);
    
    // C++: int vpMatrix::cols()
    private static native int n_cols(long nativeObj);
}
