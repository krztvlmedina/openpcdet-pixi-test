"""
Open3d visualization tool box
Written by Jihan YANG
All rights preserved from 2021 - present.
"""
import open3d
import torch
import matplotlib
import numpy as np

class PCDFrame():
    def __init__(self):
        """
        Args:
            pcd:
            gt_boxes:
            ref_boxes:
        """
        self.pcd = open3d.geometry.PointCloud()
        self.gt_boxes: list[open3d.geometry.LineSet] = []
        self.ref_boxes: list[open3d.geometry.LineSet] = []
        self.first_frame = True

    def reset_bounding_boxes(self):
        self.gt_boxes.clear()
        self.ref_boxes.clear()


box_colormap = [
    [1, 1, 1],
    [0, 1, 0],
    [0, 1, 1],
    [1, 1, 0],
]


def get_coor_colors(obj_labels):
    """
    Args:
        obj_labels: 1 is ground, labels > 1 indicates different instance cluster

    Returns:
        rgb: [N, 3]. color for each point.
    """
    colors = matplotlib.colors.XKCD_COLORS.values()
    max_color_num = obj_labels.max()

    color_list = list(colors)[:max_color_num+1]
    colors_rgba = [matplotlib.colors.to_rgba_array(color) for color in color_list]
    label_rgba = np.array(colors_rgba)[obj_labels]
    label_rgba = label_rgba.squeeze()[:, :3]

    return label_rgba


def draw_scenes(points, gt_boxes=None, ref_boxes=None, ref_labels=None, ref_scores=None, point_colors=None, draw_origin=True):
    if isinstance(points, torch.Tensor):
        points = points.cpu().numpy()
    if isinstance(gt_boxes, torch.Tensor):
        gt_boxes = gt_boxes.cpu().numpy()
    if isinstance(ref_boxes, torch.Tensor):
        ref_boxes = ref_boxes.cpu().numpy()

    vis = open3d.visualization.Visualizer()
    vis.create_window()

    vis.get_render_option().point_size = 1.0
    vis.get_render_option().background_color = np.zeros(3)

    # draw origin
    if draw_origin:
        axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        vis.add_geometry(axis_pcd)

    pts = open3d.geometry.PointCloud()
    pts.points = open3d.utility.Vector3dVector(points[:, :3])

    vis.add_geometry(pts)
    if point_colors is None:
        pts.colors = open3d.utility.Vector3dVector(np.ones((points.shape[0], 3)))
    else:
        pts.colors = open3d.utility.Vector3dVector(point_colors)

    if gt_boxes is not None:
        vis = draw_box(vis, gt_boxes, (0, 0, 1))

    if ref_boxes is not None:
        vis = draw_box(vis, ref_boxes, (0, 1, 0), ref_labels, ref_scores)

    vis.run()
    vis.destroy_window()


def translate_boxes_to_open3d_instance(gt_boxes): 
    # -> tuple[open3d.utility.Vector2iVector, open3d.geometry.OrientedBoundingBox]:
    """
             4-------- 6
           /|         /|
          5 -------- 3 .
          | |        | |
          . 7 -------- 1
          |/         |/
          2 -------- 0
    """
    center = gt_boxes[0:3]
    lwh = gt_boxes[3:6]
    axis_angles = np.array([0, 0, gt_boxes[6] + 1e-10])
    rot = open3d.geometry.get_rotation_matrix_from_axis_angle(axis_angles)
    box3d = open3d.geometry.OrientedBoundingBox(center, rot, lwh)

    line_set = open3d.geometry.LineSet.create_from_oriented_bounding_box(box3d)

    # import ipdb; ipdb.set_trace(context=20)
    lines = np.asarray(line_set.lines)
    lines = np.concatenate([lines, np.array([[1, 4], [7, 6]])], axis=0)

    line_set.lines = open3d.utility.Vector2iVector(lines)

    return line_set, box3d


def draw_box(vis, gt_boxes, color=(0, 1, 0), ref_labels=None, score=None):
    for i in range(gt_boxes.shape[0]):
        line_set, box3d = translate_boxes_to_open3d_instance(gt_boxes[i])
        if ref_labels is None:
            line_set.paint_uniform_color(color)
        else:
            line_set.paint_uniform_color(box_colormap[ref_labels[i]])

        vis.add_geometry(line_set)

        # if score is not None:
        #     corners = box3d.get_box_points()
        #     vis.add_3d_label(corners[5], '%.2f' % score[i])
    return vis

def generate_box(gt_boxes, color=(0, 1, 0), ref_labels=None, score=None):
    lineArray: list[open3d.geometry.LineSet] = []
    for i in range(gt_boxes.shape[0]):
        line_set, box3d = translate_boxes_to_open3d_instance(gt_boxes[i])
        if ref_labels is None:
            line_set.paint_uniform_color(color)
        else:
            line_set.paint_uniform_color(box_colormap[ref_labels[i]])

        lineArray.append(line_set)

        # if score is not None:
        #     corners = box3d.get_box_points()
        #     vis.add_3d_label(corners[5], '%.2f' % score[i])
    return lineArray


def update_scene(points, 
                    pcdFrame:PCDFrame,
                    vis:open3d.visualization.Visualizer, 
                    gt_boxes=None, 
                    ref_boxes=None, 
                    ref_labels=None, 
                    ref_scores=None, 
                    point_colors=None):
    
    if isinstance(points, torch.Tensor):
        points = points.cpu().numpy()
    if isinstance(gt_boxes, torch.Tensor):
        gt_boxes = gt_boxes.cpu().numpy()
    if isinstance(ref_boxes, torch.Tensor):
        ref_boxes = ref_boxes.cpu().numpy()

    pcdFrame.pcd.points = open3d.utility.Vector3dVector(points[:, :3])

    if pcdFrame.first_frame:
        vis.add_geometry(pcdFrame.pcd)
        pcdFrame.first_frame = False
    else:
        vis.update_geometry(pcdFrame.pcd)

    if point_colors is None:
        pcdFrame.pcd.colors = open3d.utility.Vector3dVector(np.ones((points.shape[0], 3)))
    else:
        pcdFrame.pcd.colors = open3d.utility.Vector3dVector(point_colors)

    for lineSet in pcdFrame.gt_boxes:
        vis.remove_geometry(lineSet, reset_bounding_box=False)

    for lineSet in pcdFrame.ref_boxes:
        vis.remove_geometry(lineSet, reset_bounding_box=False)

    pcdFrame.reset_bounding_boxes()

    if gt_boxes is not None:
        print("Generating GT boxes...")
        gt_lineSets = generate_box(gt_boxes, (0, 0, 1))
        for lineSet in gt_lineSets:
            pcdFrame.gt_boxes.append(lineSet)
            vis.add_geometry(lineSet, reset_bounding_box=False)

    if ref_boxes is not None:
        rf_lineSets = generate_box(ref_boxes, (0, 1, 0), ref_labels, ref_scores)
        print("Generated % s Ref bounding boxes..." % len(rf_lineSets))
        for lineSet in rf_lineSets:
            pcdFrame.ref_boxes.append(lineSet)
            vis.add_geometry(lineSet, reset_bounding_box=False)

    # print("Updating renderer...")
    vis.update_renderer()
    vis.run()

    