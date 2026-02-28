#!/usr/bin/env bash
set -e

OPENPCDET_ROOT=/OpenPCDet
DATA_ROOT=${OPENPCDET_ROOT}/data/reduced-kitti
OUTPUT_ROOT=${OPENPCDET_ROOT}/output
FINAL_ROOT=${OPENPCDET_ROOT}/output_runs

TIMESTAMP=$(date +"%d-%m-%y_%H-%M")

RUN_DIR=${FINAL_ROOT}/${TIMESTAMP}/kitti_models
mkdir -p "${RUN_DIR}"

declare -A MODELS
MODELS[parta2_anchor]="parta2_anchor.yaml PartA2_7940.pth"
MODELS[pointpillar]="pointpillar.yaml pointpillar_7728.pth"
MODELS[pointrcnn_iou]="pointrcnn_iou.yaml pointrcnn_iou_7875.pth"
MODELS[pv_rcnn]="pv_rcnn.yaml pv_rcnn_8369.pth"
MODELS[second]="second.yaml second_7862.pth"
MODELS[second_iou]="second_iou.yaml second_iou7909.pth"

DATASET=$(basename "${DATASET_PATH}")
echo "Evaluating dataset: ${DATASET}"

for MODEL in "${!MODELS[@]}"; do
    read CFG CKPT <<< "${MODELS[$MODEL]}"

    echo "  Model: ${MODEL}"

    python3 src/lidar/tools/setupandruntest.py \
        --dataset-root "${DATA_ROOT}" \
        --lidar-root "${DATA_ROOT}" \
        --cfg_file src/lidar/tools/cfgs/kitti_models/downsampled/${CFG} \
        --batch_size 1 \
        --ckpt data/pretrained-models/${CKPT}

    SRC_DIR=${OUTPUT_ROOT}/lidar/tools/cfgs/kitti_models/downsampled/${MODEL}        
    DST_DIR=${RUN_DIR}/${DATASET}/${MODEL}

    mkdir -p "$(dirname "${DST_DIR}")"
    mv "${SRC_DIR}" "${DST_DIR}"
done

echo "All evaluations stored in ${FINAL_ROOT}/${TIMESTAMP}"
