package org.visp.imgproc;

public class VpContour {

    public final long nativeObj; 
    
    public VpContour(long addr){ 
        if (addr == 0) 
            throw new java.lang.UnsupportedOperationException("Native object address is NULL"); 
        nativeObj = addr; 
    }
    
    // C++:  vpContour()
    public VpContour() {
    	nativeObj = VpContour1();
    }
    
    // C++:  vpContour(vpContourType type)    
    public VpContour(int vpContourType) {
    	nativeObj = VpContour2(vpContourType);
    }
    
    // C++:  vpContour(vpContour contour)    
    public VpContour(VpContour contour) {
    	nativeObj = VpContour3(contour.nativeObj);
    }
    
    // C++:  void setParent(vpContour *parent)
    public void setParent(VpContour parent) {
    	n_setParent(nativeObj,parent.nativeObj);
    }
    
    @Override
    protected void finalize() throws Throwable {
        delete(nativeObj);
    }
    
    // C++: vpContour() 
    private static native long VpContour1(); 
    
    // C++: vpContour(vpContourType type) 
    private static native long VpContour2(int vpContourType);
    
    // C++: vpContour(vpContour contour) 
    private static native long VpContour3(long nativeObj);
    
    // C++:  void setParent(vpContour *parent)
    private static native void n_setParent(long nativeObj, long parent_nativeObj);
    
    // native support for java finalize()
    private static native void delete(long nativeObj);
    
    public class VpContourType{
    	public final static int CONTOUR_OUTER = 0;
    	public final static int CONTOUR_HOLE  = 1;
    }
    
    public class VpContourRetrievalType{
    	public final static int CONTOUR_RETR_TREE = 0;
    	public final static int CONTOUR_RETR_LIST = 1;
    	public final static int CONTOUR_RETR_EXTERNAL = 2;
    }
}

