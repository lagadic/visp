package org.visp.utils;

import java.util.ArrayList;
import java.util.List;

import org.visp.core.VpColVector;

public class Converters {

    public static List<VpColVector> Array_to_vector_vpColVector(long array[]) {
		List<VpColVector> list = new ArrayList<>();
		for(long address: array){
			list.add(new VpColVector(address));
		}
        return list;
    }
}
