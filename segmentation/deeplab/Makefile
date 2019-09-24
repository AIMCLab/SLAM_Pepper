SERVER = jyk@166.111.17.31:/home/jyk/models/research/deeplab
SERVER2 = jiayuekai@166.111.130.132:/home/jiayuekai/models/research/deeplab
FILES = *.py *.sh Makefile  utils/ core/

bs ?= 4
iters ?= 10
image ?=
crop ?= 513
dataset ?= pascal_voc_seg
train_split ?= train
classes ?= 21
resize ?= 0

WORK_DIR=$(shell pwd)
DATASET_DIR="datasets"
EXP_FOLDER="exp/train_on_trainval_set"
INIT_FOLDER="${WORK_DIR}/${DATASET_DIR}/${dataset}/init_models"
TRAIN_LOGDIR="${WORK_DIR}/${DATASET_DIR}/${dataset}/${EXP_FOLDER}/train"
EVAL_LOGDIR="${WORK_DIR}/${DATASET_DIR}/${dataset}/${EXP_FOLDER}/eval"
VIS_LOGDIR="${WORK_DIR}/${DATASET_DIR}/${dataset}/${EXP_FOLDER}/vis"
EXPORT_DIR="${WORK_DIR}/${DATASET_DIR}/${dataset}/${EXP_FOLDER}/export"
DATASET = "${WORK_DIR}/${DATASET_DIR}/${dataset}/tfrecord"

NYU_CKPT="${WORK_DIR}/${DATASET_DIR}/nyuv2_8/exp/train_on_trainval_set/train/model.ckpt-2411"

export PYTHONPATH=$PYTHONPATH:$(WORK_DIR)/../:$(WORK_DIR)/../slim

upload:
	scp -r $(FILES) $(SERVER)
	scp -r datasets/*.py $(SERVER)/datasets

upload2:
	scp -r $(FILES) $(SERVER2)
	scp -r datasets/*.py $(SERVER2)/datasets
	# scp -r datasets/pepper/* $(SERVER2)/datasets/pepper

download:
	scp -r $(SERVER)/*.py $(SERVER)/*.sh $(SERVER)/Makefile $(SERVER)/datasets/*.py $(SERVER)/datasets/*.sh .

train:
	python3 "${WORK_DIR}"/train.py \
	--logtostderr \
	--train_split="$(train_split)" \
	--model_variant="xception_65" \
	--atrous_rates=6 \
	--atrous_rates=12 \
	--atrous_rates=18 \
	--output_stride=16 \
	--decoder_output_stride=4 \
	--train_crop_size="$(crop),$(crop)" \
	--train_batch_size=$(bs) \
	--min_resize_value=$(resize) \
	--max_resize_value=$(resize) \
	--resize_factor=16 \
	--training_number_of_steps="$(iters)" \
	--fine_tune_batch_norm=false \
	--tf_initial_checkpoint="${NYU_CKPT}" \
	--train_logdir="${TRAIN_LOGDIR}" \
	--dataset_dir="${DATASET}" \
	--dataset="${dataset}" \
	--save_summaries_images=true \
	--initialize_last_layer=false \
	--last_layers_contain_logits_only=true

eval:
	python3 "${WORK_DIR}"/eval.py \
	--logtostderr \
	--eval_split="val" \
	--model_variant="xception_65" \
	--atrous_rates=6 \
	--atrous_rates=12 \
	--atrous_rates=18 \
	--output_stride=16 \
	--decoder_output_stride=4 \
	--min_resize_value=$(resize) \
	--max_resize_value=$(resize) \
	--resize_factor=16 \
	--eval_crop_size="$(crop),$(crop)" \
	--checkpoint_dir="${TRAIN_LOGDIR}" \
	--eval_logdir="${EVAL_LOGDIR}" \
	--dataset_dir="${DATASET}" \
	--dataset="${dataset}" \
	--max_number_of_evaluations=1

vis:
	python3 "${WORK_DIR}"/vis.py \
	--logtostderr \
	--vis_split="val" \
	--model_variant="xception_65" \
	--atrous_rates=6 \
	--atrous_rates=12 \
	--atrous_rates=18 \
	--output_stride=16 \
	--decoder_output_stride=4 \
	--min_resize_value=$(resize) \
	--max_resize_value=$(resize) \
	--resize_factor=16 \
	--vis_crop_size="$(crop),$(crop)" \
	--checkpoint_dir="${TRAIN_LOGDIR}" \
	--vis_logdir="${VIS_LOGDIR}" \
	--dataset_dir="${DATASET}" \
	--dataset="${dataset}" \
	--colormap_type="${dataset}" \
	--max_number_of_iterations=1 \
	--also_save_raw_predictions

CKPT_PATH="${TRAIN_LOGDIR}/model.ckpt-$(iters)"
EXPORT_PATH="${EXPORT_DIR}/frozen_inference_graph.pb"

export:
	python3 "${WORK_DIR}"/export_model.py \
	--logtostderr \
	--checkpoint_path="${CKPT_PATH}" \
	--export_path="${EXPORT_PATH}" \
	--model_variant="xception_65" \
	--atrous_rates=6 \
	--atrous_rates=12 \
	--atrous_rates=18 \
	--output_stride=16 \
	--decoder_output_stride=4 \
	--num_classes=$(classes) \
	--crop_size=$(crop) \
	--crop_size=$(crop) \
	--inference_scales=1.0

infer_nyuv2:
	python3 infer_nyuv2.py $(EXPORT_PATH) "${WORK_DIR}/${DATASET_DIR}/${dataset}/" val.txt --resize=$(resize)

infer_pepper:
	python3 infer_pepper.py "${WORK_DIR}/${DATASET_DIR}/${dataset}/${EXP_FOLDER}/export/frozen_inference_graph.pb" "${WORK_DIR}/${DATASET_DIR}/${dataset}/" val.txt --resize=$(resize)
