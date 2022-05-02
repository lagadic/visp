from __future__ import print_function

import numpy as np
from numpy import linspace
import matplotlib
from matplotlib import cm
import matplotlib.pyplot as plt
import argparse
import glob

print("matplotlib version:", matplotlib.__version__)

def inverse_homogeneoux_matrix(M):
    """
    Perform homogeneous matrix inverse.

    Parameters
    ----------
    M : numpy matrix
        The input homogeneous matrix.

    Returns:
    -------
    numpy matrix
        Inverse of M.
    """
    R = M[0:3, 0:3]
    T = M[0:3, 3]
    M_inv = np.identity(4)
    M_inv[0:3, 0:3] = R.T
    M_inv[0:3, 3] = -(R.T).dot(T)

    return M_inv

def draw_square(ax, square_size):
    """
    Draw a red square of size square_size at Z=0.

    Parameters
    ----------
    ax: matplotlib axis
        The Matplotlib axis.
    square_size: int
        The square size.
    """
    X = [+square_size, +square_size, -square_size, -square_size, +square_size]
    Y = [-square_size, +square_size, +square_size, -square_size, -square_size]
    Z = [0, 0, 0, 0, 0]

    Xs = []
    Ys = []
    Zs = []

    for i in range(len(X)):
        pt = np.matrix([X[i], Y[i], Z[i], 1])
        Xs.append(pt[0, 0])
        Ys.append(pt[0, 1])
        Zs.append(pt[0, 2])

    ax.plot3D(Xs, Ys, Zs, color='r')

def getNED(lon_, lat_, r, in_radian=False):
    """
    Get the homogeneous transformation matrix corresponding to the local tangent plane transformation at the specified
    longitude/latitude and radius coordinates, using the NED and ECEF conventions and a perfect sphere.
    See also:
        - https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
        - https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates

    Parameters
    ----------
    lon_: float
        The longitude coordinate.
    lat_: float
        The latitude coordinate.
    r: float
        The sphere radius.
    in_radian: boolean
        If true coordinates are in radian, otherwise in degree.

    Returns:
    -------
    numpy matrix
        The homogeneous matrix allowing converting a 3D point expressed in the NED frame to the ECEF frame.
    """
    if not in_radian:
        # lambda
        lon = np.radians(lon_)
        # phi
        lat = np.radians(lat_)
    else:
        lon = lon_
        lat = lat_

    Tdata = [ [-np.sin(lat)*np.cos(lon), -np.sin(lon), -np.cos(lat)*np.cos(lon), r*np.cos(lon)*np.cos(lat)], \
              [-np.sin(lat)*np.sin(lon),  np.cos(lon), -np.cos(lat)*np.sin(lon), r*np.sin(lon)*np.cos(lat)], \
              [ np.cos(lat),              0,           -np.sin(lat),             r*np.sin(lat)], \
              [ 0,                        0,            0,                       1] \
            ]
    T = np.matrix(Tdata)

    return T

def getENU(lon_, lat_, r, in_radian=False):
    """
    Get the homogeneous transformation matrix corresponding to the local tangent plane transformation at the specified
    longitude/latitude and radius coordinates, using the ENU and ECEF conventions and a perfect sphere.
    See also:
        - https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system
        - https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates

    Parameters
    ----------
    lon_: float
        The longitude coordinate.
    lat_: float
        The latitude coordinate.
    r: float
        The sphere radius.
    in_radian: boolean
        If true coordinates are in radian, otherwise in degree.

    Returns:
    -------
    numpy matrix
        The homogeneous matrix allowing converting a 3D point expressed in the ENU frame to the ECEF frame.
    """
    if not in_radian:
        # lambda
        lon = np.radians(lon_)
        # phi
        lat = np.radians(lat_)
    else:
        lon = lon_
        lat = lat_

    Tdata = [ [-np.sin(lon), -np.sin(lat)*np.cos(lon), np.cos(lat)*np.cos(lon), r*np.cos(lon)*np.cos(lat)], \
              [ np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)*np.sin(lon), r*np.sin(lon)*np.cos(lat)], \
              [ 0,            np.cos(lat),             np.sin(lat),             r*np.sin(lat)], \
              [ 0,            0,                       0,                       1] \
            ]
    T = np.matrix(Tdata)

    return T

def regular_on_sphere_points(num, full_sphere=False):
    """
    Generate equidistributed points on the surface of a sphere.
    From:
      - "How to generate equidistributed points on the surface of a sphere", Markus Deserno
      - https://www.cmu.edu/biolphys/deserno/pdf/sphere_equi.pdf
      - https://gist.github.com/dinob0t/9597525

    Parameters
    ----------
    num: int
        The desired number of points on the surface of a sphere.

    Returns:
    -------
    list
        The list of equidistributed points on the surface of a sphere in the lon-lat coordinates.

    Note:
    -------
        This method does not return exactly the specified number of points.
    """
    r = 1
    points = []
    # Break out if zero points
    if num == 0:
        return points

    a = 4.0 * np.pi*(r**2.0 / num)
    d = np.sqrt(a)
    m_theta = int(round(np.pi / d))
    d_theta = np.pi / m_theta
    d_phi = a / d_theta
    pi_2 = np.pi/2

    if full_sphere:
        m_upper_bound = m_theta
    else:
        m_upper_bound = int(m_theta/2)

    for m in range(m_upper_bound):
        theta = np.pi * (m + 0.5) / m_theta
        m_phi = int(round(2.0 * np.pi * np.sin(theta) / d_phi))

        for n in range(m_phi):
            phi = 2.0 * np.pi * n / m_phi
            lon = phi
            lat = pi_2-theta
            points.append([lon,lat])

    return points

def create_camera_model(camera_matrix, width, height, scale_focal, draw_frame_axis=True):
    """
    Create a camera model for Matplotlib 3D plotting.

    Parameters
    ----------
    camera_matrix: numpy matrix
        The camera intrinsic parameters matrix.
    width: int
        The width of the camera image plane for drawing.
    height: int
        The height of the camera image plane for drawing.
    scale_focal: float
        A scale factor to draw the camera model.
    draw_frame_axis: boolean
        If true the camera frame is also drawn.

    Returns:
    -------
    list
        The list of 3D line points for Matplotlib 3D plotting.
    """
    fx = camera_matrix[0,0]
    fy = camera_matrix[1,1]
    focal = 2 / (fx + fy)
    f_scale = scale_focal * focal

    # Draw camera image plane
    X_img_plane = np.ones((4,5))
    X_img_plane[0:3,0] = [-width, height, f_scale]
    X_img_plane[0:3,1] = [width, height, f_scale]
    X_img_plane[0:3,2] = [width, -height, f_scale]
    X_img_plane[0:3,3] = [-width, -height, f_scale]
    X_img_plane[0:3,4] = [-width, height, f_scale]

    # Draw triangle above the camera image plane
    X_triangle = np.ones((4,3))
    X_triangle[0:3,0] = [-width, -height, f_scale]
    X_triangle[0:3,1] = [0, -2*height, f_scale]
    X_triangle[0:3,2] = [width, -height, f_scale]

    # Draw camera
    X_center1 = np.ones((4,2))
    X_center1[0:3,0] = [0, 0, 0]
    X_center1[0:3,1] = [-width, height, f_scale]

    X_center2 = np.ones((4,2))
    X_center2[0:3,0] = [0, 0, 0]
    X_center2[0:3,1] = [width, height, f_scale]

    X_center3 = np.ones((4,2))
    X_center3[0:3,0] = [0, 0, 0]
    X_center3[0:3,1] = [width, -height, f_scale]

    X_center4 = np.ones((4,2))
    X_center4[0:3,0] = [0, 0, 0]
    X_center4[0:3,1] = [-width, -height, f_scale]

    # Draw camera frame axis
    X_frame1 = np.ones((4,2))
    X_frame1[0:3,0] = [0, 0, 0]
    X_frame1[0:3,1] = [f_scale/2, 0, 0]

    X_frame2 = np.ones((4,2))
    X_frame2[0:3,0] = [0, 0, 0]
    X_frame2[0:3,1] = [0, f_scale/2, 0]

    X_frame3 = np.ones((4,2))
    X_frame3[0:3,0] = [0, 0, 0]
    X_frame3[0:3,1] = [0, 0, f_scale/2]

    if draw_frame_axis:
        return [X_img_plane, X_triangle, X_center1, X_center2, X_center3, X_center4, X_frame1, X_frame2, X_frame3]
    else:
        return [X_img_plane, X_triangle, X_center1, X_center2, X_center3, X_center4]

def drawCameraModel(cam_model, w_T_cv, ax, color, draw_frame_axis):
    """
    Draw a camera at the specified pose.

    Parameters
    ----------
    cam_model: list
        The camera model for drawing.
    w_T_cv: numpy matrix
        The pose of the camera with respect to the global frame.
    ax: Matplotlib axis object
        The Matplotlib axis.
    color: Matplotlib color
        The color of the camera.
    draw_frame_axis: boolean
        If true the camera frame is also drawn.

    Returns:
    -------
    list
        The list of 3D line points for Matplotlib 3D plotting.
    """
    cam_frame_colors = ['r', 'g', 'b']
    for i in range(len(cam_model)):
        X = np.zeros(cam_model[i].shape)
        for j in range(cam_model[i].shape[1]):
            X[:,j] = w_T_cv @ cam_model[i][:,j]
            if draw_frame_axis and i >= len(cam_model)-3:
                ax.plot3D(X[0,:], X[1,:], X[2,:], color=cam_frame_colors[i-6])
            else:
                ax.plot3D(X[0,:], X[1,:], X[2,:], color=color)

def axisEqual3D(ax):
    """
    Try to do 3D plotting with equal X, Y and Z axes.

    Parameters
    ----------
    ax: Matplotlib axis object
        The Matplotlib axis.
    """
    extents = np.array([getattr(ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
    sz = extents[:,1] - extents[:,0]
    centers = np.mean(extents, axis=1)
    maxsize = max(abs(sz))
    r = maxsize/2
    for ctr, dim in zip(centers, 'xyz'):
        getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)

def readCamPoses(filenames):
    """
    Read camera poses stored in txt files.

    Parameters
    ----------
    filenames: List
        The list of camera pose filenames.

    Returns:
    -------
    list
        The list of camera pose matrices.
    """
    cam_poses = []
    for filename in filenames:
        cam_poses.append(np.loadtxt(filename))
    return cam_poses

def main():
    parser = argparse.ArgumentParser(description='Plot camera poses using the NED or ENU conventions.',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--min-lon', type=float, default=0,
                        help='Minimum longitude.')
    parser.add_argument('--min-lat', type=float, default=0,
                        help='Minimum latitude.')
    parser.add_argument('--max-lon', type=float, default=360,
                        help='Maximum longitude.')
    parser.add_argument('--max-lat', type=float, default=90,
                        help='Maximum latitude.')
    parser.add_argument('--nlon', type=int, default=20,
                        help='Number of longitude subdivisions.')
    parser.add_argument('--nlat', type=int, default=10,
                        help='Number of latitude subdivisions.')
    parser.add_argument('--radius', type=float, default=5,
                        help='Sphere radius.')
    parser.add_argument('--cam-width', type=float, default=0.64/2,
                        help='Width/2 of the displayed camera.')
    parser.add_argument('--cam-height', type=float, default=0.48/2,
                        help='Height/2 of the displayed camera.')
    parser.add_argument('--scale-focal', type=float, default=0.5,
                        help='Value to scale the focal length.')
    parser.add_argument('--cam-frame', action='store_true',
                        help='Display camera frames.')
    parser.add_argument('--enu', action='store_true',
                        help='Use ENU coordinate system instead of NED.')
    parser.add_argument('--full-sphere', action='store_true',
                        help='Use full sphere for the equidistributed mode.')
    parser.add_argument('--save', action='store_true',
                        help='Save plotting figure.')
    parser.add_argument('--dpi', type=int, default=300,
                        help='Image dpi when saving the png file.')
    parser.add_argument('--verbose', action='store_true',
                        help='Print all the transformations.')
    parser.add_argument('--folder', type=str, default="",
                        help='Folder that contains the transformations from local to ECEF frame.')
    args = parser.parse_args()

    verbose = args.verbose
    print("Verbose?", verbose)

    radius = args.radius
    print("Sphere radius:", radius)

    # Camera
    cam_width = args.cam_width
    cam_height = args.cam_height
    print("Camera width:", cam_width, "Camera height:", cam_height)
    draw_frame_axis = args.cam_frame
    print("Display camera frames?", draw_frame_axis)
    scale_focal = args.scale_focal
    print("Scale focal:", scale_focal)
    cam = create_camera_model(np.eye(3), cam_width, cam_height, scale_focal, draw_frame_axis)
    print("Camera matrix:\n", cam)

    # Transformation from CV frame to ENU frame
    enu_T_cv = np.eye(4)
    enu_T_cv[1,1] = -1
    enu_T_cv[2,2] = -1
    print("enu_T_cv:\n", enu_T_cv)

    # Transformation from CV frame to NED frame
    ned_T_cv = np.eye(4)
    ned_T_cv[0,0] = 0
    ned_T_cv[0,1] = -1
    ned_T_cv[1,0] = 1
    ned_T_cv[1,1] = 0
    print("ned_T_cv:\n", ned_T_cv)

    folder = args.folder
    if folder:
        cam_poses = readCamPoses(sorted(glob.glob(folder + "/*.txt")))
        print(f"Read camera poses from: {folder} / Num cam poses: {len(cam_poses)}")

        fig = plt.figure()
        ax = fig.gca(projection='3d')

        ax.set_xlabel('X', fontsize=24)
        ax.set_ylabel('Y', fontsize=24)
        ax.set_zlabel('Z', fontsize=24)
        ax.set_title('Camera poses', fontsize=26)

        draw_square(ax, radius)

        cm_subsection = linspace(0.0, 1.0, len(cam_poses))
        colors = [cm.rainbow(x) for x in cm_subsection]
        for idx, cam_pose in enumerate(cam_poses):
            drawCameraModel(cam, cam_pose, ax, colors[idx], draw_frame_axis)

        # Try to make equal axes on 3D plotting
        axisEqual3D(ax)

        plt.show()

    else:
        min_lon = args.min_lon
        max_lon = args.max_lon
        min_lat = args.min_lat
        max_lat = args.max_lat
        nlon = args.nlon
        nlat = args.nlat
        print("Minimum longitude:", min_lon, "Maximum longitude:", max_lon, "Number of longitude coordinates:", nlon)
        print("Minimum latitude:", min_lat, "Maximum latitude:", max_lat, "Number of latitude coordinates:", nlat)

        longitudes = np.linspace(min_lon, max_lon, nlon, endpoint=True)
        latitudes = np.linspace(min_lat, max_lat, nlat, endpoint=True)
        print("longitudes:\n", longitudes)
        print("latitudes:\n", latitudes)

        use_enu = args.enu
        print("Use ENU?", use_enu)

        cm_subsection = linspace(0.0, 1.0, longitudes.shape[0]*latitudes.shape[0])
        colors = [cm.rainbow(x) for x in cm_subsection]

        fig1 = plt.figure()
        ax1 = fig1.gca(projection='3d')

        ax1.set_xlabel('X', fontsize=24)
        ax1.set_ylabel('Y', fontsize=24)
        ax1.set_zlabel('Z', fontsize=24)
        ax1.set_title('Camera poses from longitude-latitude sampling', fontsize=26)

        draw_square(ax1, radius)

        if verbose:
            print("\n=========Spherical sampling using Lon/Lat coordinates=========")

        idx = 0
        for lon in longitudes:
            for lat in latitudes:
                if use_enu:
                    ecef_T_enu = getENU(lon, lat, radius)
                    ecef_T_cv = ecef_T_enu @ enu_T_cv
                    drawCameraModel(cam, ecef_T_cv, ax1, colors[idx], draw_frame_axis)
                    if verbose:
                        print("\nLon-Lat ecef_T_enu:\n", ecef_T_enu)
                else:
                    ecef_T_ned = getNED(lon, lat, radius)
                    ecef_T_cv = ecef_T_ned @ ned_T_cv
                    drawCameraModel(cam, ecef_T_cv, ax1, colors[idx], draw_frame_axis)
                    if verbose:
                        print("\nLon-Lat ecef_T_ned:\n", ecef_T_ned)
                idx += 1

        # Try to make equal axes on 3D plotting
        axisEqual3D(ax1)

        fig2 = plt.figure()
        ax2 = fig2.gca(projection='3d')

        draw_square(ax2, radius)

        npoints = longitudes.shape[0]*latitudes.shape[0]
        full_sphere = args.full_sphere
        regular_surf_points = regular_on_sphere_points(npoints, full_sphere)
        print("Desired number of equidistributed points on the sphere:", npoints)
        print("Actual number of equidistributed points on the sphere:", len(regular_surf_points))

        cm_subsection = linspace(0.0, 1.0, len(regular_surf_points))
        colors = [ cm.rainbow(x) for x in cm_subsection ]

        if verbose:
            print("\n=========Equidistributed sphere sampling=========")

        for idx, point in enumerate(regular_surf_points):
            lon = np.rad2deg(point[0])
            lat = np.rad2deg(point[1])

            if use_enu:
                ecef_T_enu = getENU(lon, lat, radius)
                ecef_T_cv = ecef_T_enu @ enu_T_cv
                drawCameraModel(cam, ecef_T_cv, ax2, colors[idx], draw_frame_axis)
                if verbose:
                    print("\nEquidistributed ecef_T_enu:\n", ecef_T_enu)
            else:
                ecef_T_ned = getNED(lon, lat, radius)
                ecef_T_cv = ecef_T_ned @ ned_T_cv
                drawCameraModel(cam, ecef_T_cv, ax2, colors[idx], draw_frame_axis)
                if verbose:
                    print("\nEquidistributed ecef_T_ned:\n", ecef_T_ned)

        ax2.set_xlabel('X', fontsize=24)
        ax2.set_ylabel('Y', fontsize=24)
        ax2.set_zlabel('Z', fontsize=24)
        ax2.set_title('Camera poses from equidistributed sphere sampling', fontsize=26)

        # Copy axis limits from the first figure
        ax2.set_xlim(ax1.get_xlim())
        ax2.set_ylim(ax1.get_ylim())
        ax2.set_zlim(ax1.get_zlim())

        plt.show()

        if args.save:
            if use_enu:
                fig1.savefig('ENU.png', dpi=args.dpi, bbox_inches='tight')
            else:
                fig1.savefig('NED.png', dpi=args.dpi, bbox_inches='tight')

if __name__ == '__main__':
    main()
