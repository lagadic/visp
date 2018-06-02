package org.visp.core;

// C++: class vpImage<Type>
// Note that Java can handle generics, JNI cant
// So better to have class as a member function parameter to support sort of generics like behavior
// Refer https://stackoverflow.com/questions/2228275/java-generics-and-jni

public class VpImage<T> {

	private final Class<T> type;
    public final long nativeObj;

    public VpImage(Class<T> type, long addr){
    	this.type = type;
        if (addr == 0)
            throw new java.lang.UnsupportedOperationException("Native object address is NULL");
        nativeObj = addr;
    }
    
    // C++: vpImage::vpImage<Type>()
    public VpImage(Class<T> type) {
    	this.type = type;
    	nativeObj = n_VpImage(type.toString());
    }
    
    // C++: vpImage<Type>(unsigned int r, unsigned int c, Type val)
    public VpImage(Class<T> type, int r, int c, T val) {
    	this.type = type;
    	nativeObj = n_VpImage(type.toString(),r,c,String.valueOf(val));
    }
    
    // C++: vpImage::vpImage(int rows, int cols)
    public VpImage(Class<T> type, int rows, int cols) {
    	this.type = type;
    	nativeObj = n_VpImage(type.toString(), rows,cols);
    }
    
    // C++: vpImage (Type *const array, int height, int width, bool copyData=false)
    // The byte array would be read from a stream
    public VpImage(Class<T> type, byte[] array, int height, int width, boolean copyData) {
    	this.type = type;
    	nativeObj = n_VpImage(type.toString(), array, height, width, copyData);
    }
    
    // C++: VpImage::getCols()
    public int cols() {
        return n_cols(type.toString(), nativeObj);
    }

    // C++: VpImage::getRows()
    public int rows() {
        return n_rows(type.toString(), nativeObj);
    }

    public long getNativeObjAddr() {
        return nativeObj;
    }
    
    // Prints current image
    public String dump(){
    	return n_dump(type.toString(), nativeObj);
    }
    
    // C++: vpImage::vpImage,Type>()
    private static native long n_VpImage(final String type);
    
    // C++: vpImage::vpImage(int rows, int cols)
    private static native long n_VpImage(final String type,  int rows, int cols);
    
    // C++: vpImage::vpImage(int rows, int cols, Type value)
    private static native long n_VpImage(final String type, int rows, int cols, String value);
    
    // C++: vpImage::vpImage(Type const* array, int rows, int cols, bool copyData)
    private static native long n_VpImage(final String type, byte[] array, int rows, int cols, boolean copyData);
    
    // C++: int vpImage::cols()
    private static native int n_cols(final String type, long nativeObj);

    // C++: int vpImage::rows()
    private static native int n_rows(final String type, long nativeObj);

    // C++: String <<&(ostream ss)
    private static native String n_dump(final String type, long nativeObj);

}
