package org.visp.core;

import java.lang.Double;
import java.lang.Float;
import java.util.ArrayList;
import java.util.List;
import org.visp.core.VpMatrix;
import org.visp.core.VpRowVector;

// C++: class VpColVector
//javadoc: VpColVector

public class VpColVector {

	public List<Double> data;

    protected final long nativeObj;
    protected VpColVector(long addr) { nativeObj = addr; }

    public long getNativeObjAddr() { return nativeObj; }

    //
    // C++:   vpColVector(int n, double val)
    //

    //javadoc: VpColVector::VpColVector(n, val)
    public   VpColVector(int n, double val)
    {
        
        nativeObj = VpColVector_0(n, val);
        
        return;
    }


    //
    // C++:   vpColVector(vector_double v)
    //

    //javadoc: VpColVector::VpColVector(v)
	public VpColVector(double v[])
    {
		nativeObj = VpColVector_1(v);
        
        return;
    }


	//
    // C++:   vpColVector(vector_float v)
    //

    //javadoc: VpColVector::VpColVector(v)
	public VpColVector(float v[])
    {
		nativeObj = VpColVector_2(v);
        
        return;
    }


    //
    // C++:   vpColVector(vpColVector v, int r, int nrows)
    //

    //javadoc: VpColVector::VpColVector(v, r, nrows)
    public   VpColVector(VpColVector v, int r, int nrows)
    {
        
        nativeObj = VpColVector_3(v.nativeObj, r, nrows);
        
        return;
    }


    //
    // C++:   vpColVector(vpColVector v)
    //

    //javadoc: VpColVector::VpColVector(v)
    public   VpColVector(VpColVector v)
    {
        
        nativeObj = VpColVector_4(v.nativeObj);
        
        return;
    }


    //
    // C++:   vpColVector(vpMatrix M, int j)
    //

    //javadoc: VpColVector::VpColVector(M, j)
    public   VpColVector(VpMatrix M, int j)
    {
        
        nativeObj = VpColVector_6(M.nativeObj, j);
        
        return;
    }


    //
    // C++:   vpColVector(vpMatrix M)
    //

    //javadoc: VpColVector::VpColVector(M)
    public   VpColVector(VpMatrix M)
    {
        
        nativeObj = VpColVector_7(M.nativeObj);
        
        return;
    }

	public void set(int n,double d) { 
		data.set(n,d); 
	} 

	// C++: void vpColVector::operator=(double) 
    public void setAllTo(double d) { 
    	for(int i=0;i<data.size();++i) 
    		data.set(i,d); 
    } 


    //
    // C++:   vpColVector(vpPoseVector p)
    //

    // Unknown type 'vpPoseVector' (I), skipping the function


    //
    // C++:   vpColVector(vpRotationVector v)
    //

    // Unknown type 'vpRotationVector' (I), skipping the function


    //
    // C++:   vpColVector(vpTranslationVector t)
    //

    // Unknown type 'vpTranslationVector' (I), skipping the function


    //
    // C++:   vpColVector()
    //

    //javadoc: VpColVector::VpColVector()
    public   VpColVector()
    {
        
        nativeObj = VpColVector_8();
        
        return;
    }


    //
    // C++:  explicit vpColVector(int n)
    //

    // Return type 'explicit' is not supported, skipping the function


    //
    // C++: static double dotProd(vpColVector a, vpColVector b)
    //

    //javadoc: VpColVector::dotProd(a, b)
    public static double dotProd(VpColVector a, VpColVector b)
    {
        
        double retVal = dotProd_0(a.nativeObj, b.nativeObj);
        
        return retVal;
    }


    //
    // C++:  double euclideanNorm()
    //

    //javadoc: VpColVector::euclideanNorm()
    public  double euclideanNorm()
    {
        
        double retVal = euclideanNorm_0(nativeObj);
        
        return retVal;
    }


    //
    // C++:  double infinityNorm()
    //

    //javadoc: VpColVector::infinityNorm()
    public  double infinityNorm()
    {
        
        double retVal = infinityNorm_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: static double mean(vpColVector v)
    //

    //javadoc: VpColVector::mean(v)
    public static double mean(VpColVector v)
    {
        
        double retVal = mean_0(v.nativeObj);
        
        return retVal;
    }


    //
    // C++: static double median(vpColVector v)
    //

    //javadoc: VpColVector::median(v)
    public static double median(VpColVector v)
    {
        
        double retVal = median_0(v.nativeObj);
        
        return retVal;
    }


    //
    // C++: static double stdev(vpColVector v, bool useBesselCorrection = false)
    //

    //javadoc: VpColVector::stdev(v, useBesselCorrection)
    public static double stdev(VpColVector v, boolean useBesselCorrection)
    {
        
        double retVal = stdev_0(v.nativeObj, useBesselCorrection);
        
        return retVal;
    }

    //javadoc: VpColVector::stdev(v)
    public static double stdev(VpColVector v)
    {
        
        double retVal = stdev_1(v.nativeObj);
        
        return retVal;
    }


    //
    // C++:  double sum()
    //

    //javadoc: VpColVector::sum()
    public  double sum()
    {
        
        double retVal = sum_0(nativeObj);
        
        return retVal;
    }


    //
    // C++:  double sumSquare()
    //

    //javadoc: VpColVector::sumSquare()
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

    //javadoc: VpColVector::clear()
    public  void clear()
    {
        
        clear_0(nativeObj);
        
        return;
    }


    //
    // C++:  void deg2rad()
    //

    //javadoc: VpColVector::deg2rad()
    public  void deg2rad()
    {
        
        deg2rad_0(nativeObj);
        
        return;
    }


    //
    // C++:  void init(vpColVector v, int r, int nrows)
    //

    //javadoc: VpColVector::init(v, r, nrows)
    public  void init(VpColVector v, int r, int nrows)
    {
        
        init_0(nativeObj, v.nativeObj, r, nrows);
        
        return;
    }


    //
    // C++:  void insert(int i, vpColVector v)
    //

    //javadoc: VpColVector::insert(i, v)
    public  void insert1(int i, VpColVector v)
    {
        
        insert_0(nativeObj, i, v.nativeObj);
        
        return;
    }


    //
    // C++:  void insert(vpColVector v, int i)
    //

    //javadoc: VpColVector::insert(v, i, c=0)
    public  void insert(VpColVector v, int i)
    {
        
        insert_2(nativeObj, v.nativeObj, i, 0);
        
        return;
    }


    //
    // C++:  void insert(vpColVector v, int r, int c = 0)
    //

    //javadoc: VpColVector::insert(v, r, c)
    public  void insert(VpColVector v, int r, int c)
    {
        
        insert_2(nativeObj, v.nativeObj, r, c);
        
        return;
    }


    //
    // C++:  void rad2deg()
    //

    //javadoc: VpColVector::rad2deg()
    public  void rad2deg()
    {
        
        rad2deg_0(nativeObj);
        
        return;
    }


    //
    // C++:  void reshape(vpMatrix M, int nrows, int ncols)
    //

    //javadoc: VpColVector::reshape(M, nrows, ncols)
    public  void reshape(VpMatrix M, int nrows, int ncols)
    {
        
        reshape_0(nativeObj, M.nativeObj, nrows, ncols);
        
        return;
    }


    //
    // C++:  void resize(int i, bool flagNullify = true)
    //

    //javadoc: VpColVector::resize(i, flagNullify)
    public  void resize(int i, boolean flagNullify)
    {
        
        resize_0(nativeObj, i, flagNullify);
        
        return;
    }

    //javadoc: VpColVector::resize(i)
    public  void resize(int i)
    {
        
        resize_1(nativeObj, i);
        
        return;
    }


    //
    // C++:  void resize(int nrows, int ncols, bool flagNullify)
    //

    //javadoc: VpColVector::resize(nrows, ncols, flagNullify)
    public  void resize(int nrows, int ncols, boolean flagNullify)
    {
        
        resize_2(nativeObj, nrows, ncols, flagNullify);
        
        return;
    }


    //
    // C++:  void setIdentity(double val = 1.0)
    //

    //javadoc: VpColVector::setIdentity(val)
    @Deprecated
    public  void setIdentity(double val)
    {
        
        setIdentity_0(nativeObj, val);
        
        return;
    }

    //javadoc: VpColVector::setIdentity()
    @Deprecated
    public  void setIdentity()
    {
        
        setIdentity_1(nativeObj);
        
        return;
    }


    //
    // C++:  void stack(double d)
    //

    //javadoc: VpColVector::stack(d)
    public  void stack(double d)
    {
        
        stack_0(nativeObj, d);
        
        return;
    }


    //
    // C++: static void stack(vpColVector A, vpColVector B, vpColVector C)
    //

    //javadoc: VpColVector::stack(A, B, C)
    public static void stack(VpColVector A, VpColVector B, VpColVector C)
    {
        
        stack_1(A.nativeObj, B.nativeObj, C.nativeObj);
        
        return;
    }


    //
    // C++:  void stack(vpColVector v)
    //

    //javadoc: VpColVector::stack(v)
    public  void stack(VpColVector v)
    {
        
        stack_2(nativeObj, v.nativeObj);
        
        return;
    }


    //
    // C++: static void stackMatrices(vpColVector A, vpColVector B, vpColVector C)
    //

    //javadoc: VpColVector::stackMatrices(A, B, C)
    @Deprecated
    public static void stackMatrices(VpColVector A, VpColVector B, VpColVector C)
    {
        
        stackMatrices_0(A.nativeObj, B.nativeObj, C.nativeObj);
        
        return;
    }


    //
    // C++:  void stackMatrices(vpColVector r)
    //

    //javadoc: VpColVector::stackMatrices(r)
    @Deprecated
    public  void stackMatrices(VpColVector r)
    {
        
        stackMatrices_1(nativeObj, r.nativeObj);
        
        return;
    }


    //
    // C++:  void transpose(vpRowVector v)
    //

    //javadoc: VpColVector::transpose(v)
    public  void transpose(VpRowVector v)
    {
        
        transpose_0(nativeObj, v.nativeObj);
        
        return;
    }


    //
    // C++: static vpColVector cross(vpColVector a, vpColVector b)
    //

    //javadoc: VpColVector::cross(a, b)
    public static VpColVector cross(VpColVector a, VpColVector b)
    {
        
        return new VpColVector( cross_0(a.nativeObj, b.nativeObj));
    }


    //
    // C++: static vpColVector crossProd(vpColVector a, vpColVector b)
    //

    //javadoc: VpColVector::crossProd(a, b)
    public static VpColVector crossProd(VpColVector a, VpColVector b)
    {
        
        return new VpColVector( crossProd_0(a.nativeObj, b.nativeObj));
    }


    //
    // C++:  vpColVector extract(int r, int colsize)
    //

    //javadoc: VpColVector::extract(r, colsize)
    public  VpColVector extract(int r, int colsize)
    {
        
        return new VpColVector( extract_0(nativeObj, r, colsize));
    }


    //
    // C++:  vpColVector hadamard(vpColVector v)
    //

    //javadoc: VpColVector::hadamard(v)
    public  VpColVector hadamard(VpColVector v)
    {
        
        return new VpColVector( hadamard_0(nativeObj, v.nativeObj));
    }


    //
    // C++: static vpColVector invSort(vpColVector v)
    //

    //javadoc: VpColVector::invSort(v)
    public static VpColVector invSort(VpColVector v)
    {
        
        return new VpColVector( invSort_0(v.nativeObj));
    }


    //
    // C++:  vpColVector normalize(vpColVector x)
    //

    //javadoc: VpColVector::normalize(x)
    public  VpColVector normalize(VpColVector x)
    {
        
        return new VpColVector( normalize_0(nativeObj, x.nativeObj));
    }


    //
    // C++:  vpColVector normalize()
    //

    //javadoc: VpColVector::normalize()
    public  VpColVector normalize()
    {
        
        return new VpColVector( normalize_1(nativeObj));
    }


    //
    // C++:  vpColVector rows(int first_row, int last_row)
    //

    //javadoc: VpColVector::rows(first_row, last_row)
    @Deprecated
    public  VpColVector rows(int first_row, int last_row)
    {
        
        return new VpColVector( rows_0(nativeObj, first_row, last_row));
    }


    //
    // C++: static vpColVector sort(vpColVector v)
    //

    //javadoc: VpColVector::sort(v)
    public static VpColVector sort(VpColVector v)
    {
        
        return new VpColVector( sort_0(v.nativeObj));
    }


    //
    // C++: static vpColVector stack(vpColVector A, vpColVector B)
    //

    //javadoc: VpColVector::stack(A, B)
    public static VpColVector stack(VpColVector A, VpColVector B)
    {
        
        return new VpColVector( stack_3(A.nativeObj, B.nativeObj));
    }


    //
    // C++: static vpColVector stackMatrices(vpColVector A, vpColVector B)
    //

    //javadoc: VpColVector::stackMatrices(A, B)
    @Deprecated
    public static VpColVector stackMatrices(VpColVector A, VpColVector B)
    {
        
        return new VpColVector( stackMatrices_2(A.nativeObj, B.nativeObj));
    }


    //
    // C++:  vpMatrix reshape(int nrows, int ncols)
    //

    //javadoc: VpColVector::reshape(nrows, ncols)
    public  VpMatrix reshape(int nrows, int ncols)
    {
        
        VpMatrix retVal = new VpMatrix(reshape_1(nativeObj, nrows, ncols));
        
        return retVal;
    }


    //
    // C++: static vpMatrix skew(vpColVector v)
    //

    //javadoc: VpColVector::skew(v)
    public static VpMatrix skew(VpColVector v)
    {
        
        VpMatrix retVal = new VpMatrix(skew_0(v.nativeObj));
        
        return retVal;
    }


    //
    // C++:  vpRowVector t()
    //

    //javadoc: VpColVector::t()
    public  VpRowVector t()
    {
        
        return new VpRowVector( t_0(nativeObj));
    }


    //
    // C++:  vpRowVector transpose()
    //

    //javadoc: VpColVector::transpose()
    public  VpRowVector transpose()
    {
        
        return new VpRowVector( transpose_1(nativeObj));
    }




    // C++:   vpColVector(int n, double val)
    private static native long VpColVector_0(int n, double val);

    // C++:   vpColVector(vector_double v)
    private static native long VpColVector_1(double v[]);

    // C++:   vpColVector(vector_float v)
    private static native long VpColVector_2(float v[]);

    // C++:   vpColVector(vpColVector v, int r, int nrows)
    private static native long VpColVector_3(long v_nativeObj, int r, int nrows);

    // C++:   vpColVector(vpColVector v)
    private static native long VpColVector_4(long v_nativeObj);

    // C++:   vpColVector(vpMatrix M, int j)
    private static native long VpColVector_6(long M_nativeObj, int j);

    // C++:   vpColVector(vpMatrix M)
    private static native long VpColVector_7(long M_nativeObj);

    // C++:   vpColVector()
    private static native long VpColVector_8();

    // C++: static double dotProd(vpColVector a, vpColVector b)
    private static native double dotProd_0(long a_nativeObj, long b_nativeObj);

    // C++:  double euclideanNorm()
    private static native double euclideanNorm_0(long nativeObj);

    // C++:  double infinityNorm()
    private static native double infinityNorm_0(long nativeObj);

    // C++: static double mean(vpColVector v)
    private static native double mean_0(long v_nativeObj);

    // C++: static double median(vpColVector v)
    private static native double median_0(long v_nativeObj);

    // C++: static double stdev(vpColVector v, bool useBesselCorrection = false)
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

    // C++:  void init(vpColVector v, int r, int nrows)
    private static native void init_0(long nativeObj, long v_nativeObj, int r, int nrows);

    // C++:  void insert(int i, vpColVector v)
    private static native void insert_0(long nativeObj, int i, long v_nativeObj);

    // C++:  void insert(vpColVector v, int r, int c = 0)
    private static native void insert_2(long nativeObj, long v_nativeObj, int r, int c);

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

    // C++: static void stack(vpColVector A, vpColVector B, vpColVector C)
    private static native void stack_1(long A_nativeObj, long B_nativeObj, long C_nativeObj);

    // C++:  void stack(vpColVector v)
    private static native void stack_2(long nativeObj, long v_nativeObj);

    // C++: static void stackMatrices(vpColVector A, vpColVector B, vpColVector C)
    private static native void stackMatrices_0(long A_nativeObj, long B_nativeObj, long C_nativeObj);

    // C++:  void stackMatrices(vpColVector r)
    private static native void stackMatrices_1(long nativeObj, long r_nativeObj);

    // C++:  void transpose(vpRowVector v)
    private static native void transpose_0(long nativeObj, long v_nativeObj);

    // C++: static vpColVector cross(vpColVector a, vpColVector b)
    private static native long cross_0(long a_nativeObj, long b_nativeObj);

    // C++: static vpColVector crossProd(vpColVector a, vpColVector b)
    private static native long crossProd_0(long a_nativeObj, long b_nativeObj);

    // C++:  vpColVector extract(int r, int colsize)
    private static native long extract_0(long nativeObj, int r, int colsize);

    // C++:  vpColVector hadamard(vpColVector v)
    private static native long hadamard_0(long nativeObj, long v_nativeObj);

    // C++: static vpColVector invSort(vpColVector v)
    private static native long invSort_0(long v_nativeObj);

    // C++:  vpColVector normalize(vpColVector x)
    private static native long normalize_0(long nativeObj, long x_nativeObj);

    // C++:  vpColVector normalize()
    private static native long normalize_1(long nativeObj);

    // C++:  vpColVector rows(int first_row, int last_row)
    private static native long rows_0(long nativeObj, int first_row, int last_row);

    // C++: static vpColVector sort(vpColVector v)
    private static native long sort_0(long v_nativeObj);

    // C++: static vpColVector stack(vpColVector A, vpColVector B)
    private static native long stack_3(long A_nativeObj, long B_nativeObj);

    // C++: static vpColVector stackMatrices(vpColVector A, vpColVector B)
    private static native long stackMatrices_2(long A_nativeObj, long B_nativeObj);

    // C++:  vpMatrix reshape(int nrows, int ncols)
    private static native long reshape_1(long nativeObj, int nrows, int ncols);

    // C++: static vpMatrix skew(vpColVector v)
    private static native long skew_0(long v_nativeObj);

    // C++:  vpRowVector t()
    private static native long t_0(long nativeObj);

    // C++:  vpRowVector transpose()
    private static native long transpose_1(long nativeObj);

}
