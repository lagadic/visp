package org.visp.utils;

import java.util.ArrayList;
import java.util.List;

import org.visp.core.VpColVector;
import org.visp.core.VpHomogeneousMatrix;

public class Converters {

    public static List<VpColVector> Array_to_vector_vpColVector(long array[]) {
		List<VpColVector> list = new ArrayList<VpColVector>();
		for(long address: array){
			list.add(new VpColVector(address));
		}
        return list;
    }

    public static List<VpHomogeneousMatrix> Array_to_vector_vpHomogeneousMatrix(long array[]) {
		List<VpHomogeneousMatrix> list = new ArrayList<VpHomogeneousMatrix>();
		for(long address: array){
			list.add(new VpHomogeneousMatrix(address));
		}
        return list;
    }

    public static long[] vector_vpHomogeneousMatrix_to_Array(List<VpHomogeneousMatrix> list) {
		long[] array = new long[list.size()];
		for(int i=0;i<list.size();++i)
			array[i] = list.get(i).nativeObj;
        return array;
    }
}
