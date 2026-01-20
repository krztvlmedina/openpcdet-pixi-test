cd /data/reduced-kitti/

for file in *; do
    ln -s "$file" /OpenPCDet/workspace/
done


mkdir -p /OpenPCDet/workspace/testing /OpenPCDet/workspace/training

ln -s /data/reduced-kitti/training/calib     /OpenPCDet/workspace/training/calib
ln -s /data/reduced-kitti/training/image_2   /OpenPCDet/workspace/training/image_2
ln -s /data/reduced-kitti/training/label_2   /OpenPCDet/workspace/training/label_2

mkdir -p /OpenPCDet/workspace/training/velodyne

ln -s /data/reduced-kitti/testing/calib      /OpenPCDet/workspace/testing/calib
ln -s /data/reduced-kitti/testing/image_2    /OpenPCDet/workspace/testing/image_2
mkdir -p /OpenPCDet/workspace/testing/velodyne

ln -s /data/reduced-kitti/ImageSets /OpenPCDet/workspace/ImageSets
ln -s /data/reduced-kitti/db_infos /OpenPCDet/workspace/db_infos
ln -s /data/reduced-kitti/gt_database /OpenPCDet/workspace/gt_database
