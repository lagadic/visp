package org.visp.core;

import java.lang.Double;
import java.lang.Float;
import java.util.ArrayList;
import java.util.List;
import org.visp.core.VpColVector;
import org.visp.core.VpMatrix;

// C++: class VpRowVector
//javadoc: VpRowVector

public class VpRowVector {

    protected final long nativeObj;
    protected VpRowVector(long addr) { nativeObj = addr; }

    public long getNativeObjAddr() { return nativeObj; }

    //
    // C++:   vpRowVector(int n, double val)
    //

    //javadoc: VpRowVector::VpRowVector(n, val)
    public   VpRowVector(int n, double val)
    {
        
        nativeObj = VpRowVector_0(n, val);
        
        return;
    }


    //
    // C++:   vpRowVector(vector_double v)
    //

    //javadoc: VpRowVector::VpRowVector(v)
    public VpRowVector(double v[])
    {
		nativeObj = VpRowVector_1(v);
        
        return;
    }


    //
    // C++:   vpRowVector(vector_float v)
    //

    //javadoc: VpRowVector::VpRowVector(v)
    public VpRowVector(float v[])
    {
		nativeObj = VpRowVector_2(v);
        
        return;
    }


    //
    // C++:   vpRowVector(vpMatrix M, int i)
    //

    //javadoc: VpRowVector::VpRowVector(M, i)
    public   VpRowVector(VpMatrix M, int i)
    {
        
        nativeObj = VpRowVector_3(M.nativeObj, i);
        
        return;
    }


    //
    // C++:   vpRowVector(vpMatrix M)
    //

    //javadoc: VpRowVector::VpRowVector(M)
    public   VpRowVector(VpMatrix M)
    {
        
        nativeObj = VpRowVector_4(M.nativeObj);
        
        return;
    }


    //
    // C++:   vpRowVector(vpRowVector v, int c, int ncols)
    //

    //javadoc: VpRowVector::VpRowVector(v, c, ncols)
    public   VpRowVector(VpRowVector v, int c, int ncols)
    {
        
        nativeObj = VpRowVector_5(v.nativeObj, c, ncols);
        
        return;
    }


    //
    // C++:   vpRowVector(vpRowVector v)
    //

    //javadoc: VpRowVector::VpRowVector(v)
    public   VpRowVector(VpRowVector v)
    {
        
        nativeObj = VpRowVector_6(v.nativeObj);
        
        return;
    }


    //
    // C++:   vpRowVector()
    //

    //javadoc: VpRowVector::VpRowVector()
    public   VpRowVector()
    {
        
        nativeObj = VpRowVector_7();
        
        return;
    }


    //
    // C++:  explicit vpRowVector(int n)
    //

    // Return type 'explicit' is not supported, skipping the function


    //
    // C++:  double euclideanNorm()
    //

    //javadoc: VpRowVector::euclideanNorm()
    public  double euclideanNorm()
    {
        
        double retVal = euclideanNorm_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: static double mean(vpRowVector v)
    //

    //javadoc: VpRowVector::mean(v)
    public static double mean(VpRowVector v)
    {
        
        double retVal = mean_0(v.nativeObj);
        
        return retVal;
    }


    //
    // C++: static double median(vpRowVector v)
    //

    //javadoc: VpRowVector::median(v)
    public static double median(VpRowVector v)
    {
        
        double retVal = median_0(v.nativeObj);
        
        return retVal;
    }


    //
    // C++: static double stdev(vpRowVector v, bool useBesselCorrection = false)
    //

    //javadoc: VpRowVector::stdev(v, useBesselCorrection)
    public static double stdev(VpRowVector v, boolean useBesselCorrection)
    {
        
        double retVal = stdev_0(v.nativeObj, useBesselCorrection);
        
        return retVal;
    }

    //javadoc: VpRowVector::stdev(v)
    public static double stdev(VpRowVector v)
    {
        
        double retVal = stdev_1(v.nativeObj);
        
        return retVal;
    }


    //
    // C++:  double sum()
    //

    //javadoc: VpRowVector::sum()
    public  double sum()
    {
        
        double retVal = sum_0(nativeObj);
        
        return retVal;
    }


    //
    // C++:  double sumSquare()
    //

    //javadoc: VpRowVector::sumSquare()
    public  double sumSquare()
    {
        
        double retVal = sumSquare_0(nativeObj);
        
        return retVal;
    }


    //
    // C++:  int print(ostream s, int length, c_string intro = 0)
    //

    // Unknown type 'ostream' (I), skipping the function


    //
    // C++:  ostream cppPrint(ostream os, string matrixName = "A", bool octet = false)
    //

    // Return type 'ostream' is not supported, skipping the function


    //
    // C++:  ostream csvPrint(ostream os)
    //

    // Return type 'ostream' is not supported, skipping the function


    //
    // C++:  ostream maplePrint(ostream os)
    //

    // Return type 'ostream' is not supported, skipping the function


    //
    // C++:  ostream matlabPrint(ostream os)
    //

    // Return type 'ostream' is not supported, skipping the function


    //
    // C++:  void clear()
    //

    //javadoc: VpRowVector::clear()
    public  void clear()
    {
        
        clear_0(nativeObj);
        
        return;
    }


    //
    // C++:  void deg2rad()
    //

    //javadoc: VpRowVector::deg2rad()
    public  void deg2rad()
    {
        
        deg2rad_0(nativeObj);
        
        return;
    }


    //
    // C++:  void init(vpRowVector v, int c, int ncols)
    //

    //javadoc: VpRowVector::init(v, c, ncols)
    public  void init(VpRowVector v, int c, int ncols)
    {
        
        init_0(nativeObj, v.nativeObj, c, ncols);
        
        return;
    }


    //
    // C++:  void init()
    //

    //javadoc: VpRowVector::init()
    @Deprecated
    public  void init()
    {
        
        init_1(nativeObj);
        
        return;
    }


    //
    // C++:  void insert(int i, vpRowVector v)
    //

    //javadoc: VpRowVector::insert(i, v)
    public  void insert(int i, VpRowVector v)
    {
        
        insert_0(nativeObj, i, v.nativeObj);
        
        return;
    }


    //
    // C++:  void rad2deg()
    //

    //javadoc: VpRowVector::rad2deg()
    public  void rad2deg()
    {
        
        rad2deg_0(nativeObj);
        
        return;
    }


    //
    // C++:  void reshape(vpMatrix M, int nrows, int ncols)
    //

    //javadoc: VpRowVector::reshape(M, nrows, ncols)
    public  void reshape(VpMatrix M, int nrows, int ncols)
    {
        
        reshape_0(nativeObj, M.nativeObj, nrows, ncols);
        
        return;
    }


    //
    // C++:  void resize(int i, bool flagNullify = true)
    //

    //javadoc: VpRowVector::resize(i, flagNullify)
    public  void resize(int i, boolean flagNullify)
    {
        
        resize_0(nativeObj, i, flagNullify);
        
        return;
    }

    //javadoc: VpRowVector::resize(i)
    public  void resize(int i)
    {
        
        resize_1(nativeObj, i);
        
        return;
    }


    //
    // C++:  void resize(int nrows, int ncols, bool flagNullify)
    //

    //javadoc: VpRowVector::resize(nrows, ncols, flagNullify)
    public  void resize(int nrows, int ncols, boolean flagNullify)
    {
        
        resize_2(nativeObj, nrows, ncols, flagNullify);
        
        return;
    }


    //
    // C++:  void setIdentity(double val = 1.0)
    //

    //javadoc: VpRowVector::setIdentity(val)
    @Deprecated
    public  void setIdentity(double val)
    {
        
        setIdentity_0(nativeObj, val);
        
        return;
    }

    //javadoc: VpRowVector::setIdentity()
    @Deprecated
    public  void setIdentity()
    {
        
        setIdentity_1(nativeObj);
        
        return;
    }


    //
    // C++:  void stack(double d)
    //

    //javadoc: VpRowVector::stack(d)
    public  void stack(double d)
    {
        
        stack_0(nativeObj, d);
        
        return;
    }


    //
    // C++: static void stack(vpRowVector A, vpRowVector B, vpRowVector C)
    //

    //javadoc: VpRowVector::stack(A, B, C)
    public static void stack(VpRowVector A, VpRowVector B, VpRowVector C)
    {
        
        stack_1(A.nativeObj, B.nativeObj, C.nativeObj);
        
        return;
    }


    //
    // C++:  void stack(vpRowVector v)
    //

    //javadoc: VpRowVector::stack(v)
    public  void stack(VpRowVector v)
    {
        
        stack_2(nativeObj, v.nativeObj);
        
        return;
    }


    //
    // C++: static void stackMatrices(vpRowVector A, vpRowVector B, vpRowVector C)
    //

    //javadoc: VpRowVector::stackMatrices(A, B, C)
    @Deprecated
    public static void stackMatrices(VpRowVector A, VpRowVector B, VpRowVector C)
    {
        
        stackMatrices_0(A.nativeObj, B.nativeObj, C.nativeObj);
        
        return;
    }


    //
    // C++:  void stackMatrices(vpRowVector r)
    //

    //javadoc: VpRowVector::stackMatrices(r)
    @Deprecated
    public  void stackMatrices(VpRowVector r)
    {
        
        stackMatrices_1(nativeObj, r.nativeObj);
        
        return;
    }


    //
    // C++:  void transpose(vpColVector v)
    //

    //javadoc: VpRowVector::transpose(v)
    public  void transpose(VpColVector v)
    {
        
        transpose_0(nativeObj, v.nativeObj);
        
        return;
    }


    //
    // C++:  vpColVector t()
    //

    //javadoc: VpRowVector::t()
    public  VpColVector t()
    {
        
        return new VpColVector(t_0(nativeObj));
    }


    //
    // C++:  vpColVector transpose()
    //

    //javadoc: VpRowVector::transpose()
    public  VpColVector transpose()
    {
        
        return new VpColVector(transpose_1(nativeObj));
    }


    //
    // C++:  vpMatrix reshape(int nrows, int ncols)
    //

    //javadoc: VpRowVector::reshape(nrows, ncols)
    public  VpMatrix reshape(int nrows, int ncols)
    {
        
        VpMatrix retVal = new VpMatrix(reshape_1(nativeObj, nrows, ncols));
        
        return retVal;
    }


    //
    // C++:  vpRowVector extract(int c, int rowsize)
    //

    //javadoc: VpRowVector::extract(c, rowsize)
    public  VpRowVector extract(int c, int rowsize)
    {
        
        return new VpRowVector(extract_0(nativeObj, c, rowsize));
    }


    //
    // C++:  vpRowVector normalize(vpRowVector x)
    //

    //javadoc: VpRowVector::normalize(x)
    public  VpRowVector normalize(VpRowVector x)
    {
        
        return new VpRowVector(normalize_0(nativeObj, x.nativeObj));
    }


    //
    // C++:  vpRowVector normalize()
    //

    //javadoc: VpRowVector::normalize()
    public  VpRowVector normalize()
    {
        
        return new VpRowVector(normalize_1(nativeObj));
    }


    //
    // C++: static vpRowVector stack(vpRowVector A, vpRowVector B)
    //

    //javadoc: VpRowVector::stack(A, B)
    public static VpRowVector stack(VpRowVector A, VpRowVector B)
    {
        
        return new VpRowVector(stack_3(A.nativeObj, B.nativeObj));
    }


    //
    // C++: static vpRowVector stackMatrices(vpRowVector A, vpRowVector B)
    //

    //javadoc: VpRowVector::stackMatrices(A, B)
    @Deprecated
    public static VpRowVector stackMatrices(VpRowVector A, VpRowVector B)
    {
        
        return new VpRowVector(stackMatrices_2(A.nativeObj, B.nativeObj));
    }




    // C++:   vpRowVector(int n, double val)
    private static native long VpRowVector_0(int n, double val);

    // C++:   vpRowVector(vector_double v)
    private static native long VpRowVector_1(double v[]);

    // C++:   vpRowVector(vector_float v)
    private static native long VpRowVector_2(float v[]);

    // C++:   vpRowVector(vpMatrix M, int i)
    private static native long VpRowVector_3(long M_nativeObj, int i);

    // C++:   vpRowVector(vpMatrix M)
    private static native long VpRowVector_4(long M_nativeObj);

    // C++:   vpRowVector(vpRowVector v, int c, int ncols)
    private static native long VpRowVector_5(long v_nativeObj, int c, int ncols);

    // C++:   vpRowVector(vpRowVector v)
    private static native long VpRowVector_6(long v_nativeObj);

    // C++:   vpRowVector()
    private static native long VpRowVector_7();

    // C++:  double euclideanNorm()
    private static native double euclideanNorm_0(long nativeObj);

    // C++: static double mean(vpRowVector v)
    private static native double mean_0(long v_nativeObj);

    // C++: static double median(vpRowVector v)
    private static native double median_0(long v_nativeObj);

    // C++: static double stdev(vpRowVector v, bool useBesselCorrection = false)
    private static native double stdev_0(long v_nativeObj, boolean useBesselCorrection);
    private static native double stdev_1(long v_nativeObj);

    // C++:  double sum()
    private static native double sum_0(long nativeObj);

    // C++:  double sumSquare()
    private static native double sumSquare_0(long nativeObj);

    // C++:  void clear()
    private static native void clear_0(long nativeObj);

    // C++:  void deg2rad()
    private static native void deg2rad_0(long nativeObj);

    // C++:  void init(vpRowVector v, int c, int ncols)
    private static native void init_0(long nativeObj, long v_nativeObj, int c, int ncols);

    // C++:  void init()
    private static native void init_1(long nativeObj);

    // C++:  void insert(int i, vpRowVector v)
    private static native void insert_0(long nativeObj, int i, long v_nativeObj);

    // C++:  void rad2deg()
    private static native void rad2deg_0(long nativeObj);

    // C++:  void reshape(vpMatrix M, int nrows, int ncols)
    private static native void reshape_0(long nativeObj, long M_nativeObj, int nrows, int ncols);

    // C++:  void resize(int i, bool flagNullify = true)
    private static native void resize_0(long nativeObj, int i, boolean flagNullify);
    private static native void resize_1(long nativeObj, int i);

    // C++:  void resize(int nrows, int ncols, bool flagNullify)
    private static native void resize_2(long nativeObj, int nrows, int ncols, boolean flagNullify);

    // C++:  void setIdentity(double val = 1.0)
    private static native void setIdentity_0(long nativeObj, double val);
    private static native void setIdentity_1(long nativeObj);

    // C++:  void stack(double d)
    private static native void stack_0(long nativeObj, double d);

    // C++: static void stack(vpRowVector A, vpRowVector B, vpRowVector C)
    private static native void stack_1(long A_nativeObj, long B_nativeObj, long C_nativeObj);

    // C++:  void stack(vpRowVector v)
    private static native void stack_2(long nativeObj, long v_nativeObj);

    // C++: static void stackMatrices(vpRowVector A, vpRowVector B, vpRowVector C)
    private static native void stackMatrices_0(long A_nativeObj, long B_nativeObj, long C_nativeObj);

    // C++:  void stackMatrices(vpRowVector r)
    private static native void stackMatrices_1(long nativeObj, long r_nativeObj);

    // C++:  void transpose(vpColVector v)
    private static native void transpose_0(long nativeObj, long v_nativeObj);

    // C++:  vpColVector t()
    private static native long t_0(long nativeObj);

    // C++:  vpColVector transpose()
    private static native long transpose_1(long nativeObj);

    // C++:  vpMatrix reshape(int nrows, int ncols)
    private static native long reshape_1(long nativeObj, int nrows, int ncols);

    // C++:  vpRowVector extract(int c, int rowsize)
    private static native long extract_0(long nativeObj, int c, int rowsize);

    // C++:  vpRowVector normalize(vpRowVector x)
    private static native long normalize_0(long nativeObj, long x_nativeObj);

    // C++:  vpRowVector normalize()
    private static native long normalize_1(long nativeObj);

    // C++: static vpRowVector stack(vpRowVector A, vpRowVector B)
    private static native long stack_3(long A_nativeObj, long B_nativeObj);

    // C++: static vpRowVector stackMatrices(vpRowVector A, vpRowVector B)
    private static native long stackMatrices_2(long A_nativeObj, long B_nativeObj);

}
