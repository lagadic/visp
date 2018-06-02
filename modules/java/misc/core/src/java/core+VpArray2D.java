package org.visp.core;

// C++: class vpArray2D<Type>
// Note that Java can handle generics, JNI cant
// So better to have class as a member function parameter to support sort of generics like behavior
// Refer https://stackoverflow.com/questions/2228275/java-generics-and-jni

public class VpArray2D<T> {

	private final Class<T> type;
    public final long nativeObj;

    public VpArray2D(Class<T> type, long addr){
    	this.type = type;
        if (addr == 0)
            throw new java.lang.UnsupportedOperationException("Native object address is NULL");
        nativeObj = addr;
    }
    
    // C++: vpArray2D::vpArray2D<Type>()
    public VpArray2D(Class<T> type) {
    	this.type = type;
    	nativeObj = n_VpArray2D(type.toString());
    }
    
    // C++: vpArray2D<Type>(unsigned int r, unsigned int c, Type val)
    public VpArray2D(Class<T> type, int r, int c, T val) {
    	this.type = type;
    	nativeObj = n_VpArray2D(type.toString(),r,c,String.valueOf(val));
    }
    
    // C++: vpArray2D::vpArray2D(int rows, int cols)
    public VpArray2D(Class<T> type, int rows, int cols) {
    	this.type = type;
    	nativeObj = n_VpArray2D(type.toString(), rows,cols);
    }
    
    // C++: VpArray2D::getCols()
    public int cols() {
        return n_cols(type.toString(), nativeObj);
    }

    // C++: VpArray2D::getRows()
    public int rows() {
        return n_rows(type.toString(), nativeObj);
    }

    public long getNativeObjAddr() {
        return nativeObj;
    }
    
    // C++: vpArray2D::vpArray2D,Type>()
    private static native long n_VpArray2D(final String type);
    
    // C++: vpArray2D::vpArray2D(int rows, int cols)
    private static native long n_VpArray2D(final String type,  int rows, int cols);
    
    // C++: vpArray2D::vpArray2D(int rows, int cols, Type value)
    private static native long n_VpArray2D(final String type, int rows, int cols, String value);
    
    // C++: int vpArray2D::cols()
    private static native int n_cols(final String type, long nativeObj);

    // C++: int vpArray2D::rows()
    private static native int n_rows(final String type, long nativeObj);

}
