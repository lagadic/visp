package org.visp.utils;

import java.util.ArrayList;
import java.util.List;

import org.visp.core.VpColVector;
import org.visp.core.VpHomogeneousMatrix;
import org.visp.core.VpImagePoint;
import org.visp.core.VpImageUChar;
import org.visp.core.VpCameraParameters;

public class Converters {

  public static List<VpColVector> Array_to_vector_vpColVector(long array[]) {
    List<VpColVector> list = new ArrayList<VpColVector>();
    for(long address: array) {
      list.add(new VpColVector(address));
    }
    return list;
  }

  public static List<VpHomogeneousMatrix> Array_to_vector_vpHomogeneousMatrix(long array[]) {
    List<VpHomogeneousMatrix> list = new ArrayList<VpHomogeneousMatrix>();
    for(long address: array) {
      list.add(new VpHomogeneousMatrix(address));
    }
    return list;
  }

  public static List<List<VpImagePoint>> Array_Array_to_vector_vector_vpImagePoint(long matrix[][]) {
    if (matrix == null) {
      return new ArrayList<List<VpImagePoint>>();
    }
    List<List<VpImagePoint>> list = new ArrayList<List<VpImagePoint>>(matrix.length);
    for (int i = 0; i < matrix.length; i++) {
      list.add(i, new ArrayList<VpImagePoint>(matrix[i].length));
      for (int j = 0; j < matrix[i].length; j++) {
        list.get(i).add(j, new VpImagePoint(matrix[i][j]));
      }
    }
    return list;
  }

  public static long[] vector_vpHomogeneousMatrix_to_Array(List<VpHomogeneousMatrix> list) {
    long[] array = new long[list.size()];
    for(int i = 0; i < list.size(); i++) {
      array[i] = list.get(i).nativeObj;
    }
    return array;
  }

  public static long[] array_vpImageUChar_to_array_native(VpImageUChar[] images) {
    long[] native_images = new long[images.length];
    for(int i = 0; i < images.length; i++) {
      native_images[i] = images[i].nativeObj;
    }
    return native_images;
  }

  public static long[][] matrix_vpColVector_to_matrix_native(VpColVector[][] matrix) {
    long[][] native_matrix = new long[matrix.length][];
    for(int i = 0; i < matrix.length; i++) {
      native_matrix[i] = new long[matrix[i].length];
      for (int j = 0; j < matrix[i].length; j++) {
        native_matrix[i][j] = matrix[i][j].nativeObj;
      }
    }
    return native_matrix;
  }

  public static long[] array_vpHomogeneousMatrix_to_array_native(VpHomogeneousMatrix[] poses) {
    long[] native_poses = new long[poses.length];
    for(int i = 0; i < poses.length; i++) {
      native_poses[i] = poses[i].nativeObj;
    }
    return native_poses;
  }

  public static long[] array_vpCameraParameters_to_array_native(VpCameraParameters[] cameras) {
    long[] native_cameras = new long[cameras.length];
    for(int i = 0; i < cameras.length; i++) {
      native_cameras[i] = cameras[i].nativeObj;
    }
    return native_cameras;
  }
}
