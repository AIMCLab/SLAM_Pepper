import sys, os
from sklearn.metrics import confusion_matrix
import numpy as np
import argparse
from PIL import Image
import tensorflow as tf
from deeplab.utils import get_dataset_colormap
from deeplab import input_preprocess

parser = argparse.ArgumentParser()
parser.add_argument('model_path', metavar='MODEL_PATH',
                    help='the model directory')
parser.add_argument('data_path', metavar='DATA_PATH',
                    help='the data directory')
parser.add_argument('image_list', metavar='image_list',
                    help='the input image list file')
parser.add_argument('--output', '-o', default='output',
                    help='the output image directory')
parser.add_argument('--colormap', '-c', choices=['nyuv2', 'nyuv2_8', 'pascal_voc', 'pepper'],
                    default='nyuv2_8', help='the type of colormap')
parser.add_argument('--resize', '-s', type=int, default=640,
                    help='the input image will be scaled to this size')
parser.add_argument('--report', '-r', default='report.csv',
                    help='the report file')
args = parser.parse_args()

LABEL_NAMES = np.asarray([
    'background', 'aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus',
    'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike',
    'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tv'
])
FULL_LABEL_MAP = np.arange(len(LABEL_NAMES)).reshape(len(LABEL_NAMES), 1)
FULL_COLOR_MAP = get_dataset_colormap.label_to_color_image(FULL_LABEL_MAP)

class DeepLabModel(object):
    """Class to load deeplab model and run inference."""
    INPUT_TENSOR_NAME = 'ImageTensor:0'
    OUTPUT_TENSOR_NAME = 'SemanticPredictions:0'
    def __init__(self, model_path):
        """Creates and loads pretrained deeplab model."""
        self.graph = tf.Graph()
        with open(model_path, 'rb') as fd:
            graph_def = tf.GraphDef.FromString(fd.read())
        with self.graph.as_default():
            tf.import_graph_def(graph_def, name='')
        self.sess = tf.Session(graph=self.graph)

    def run(self, image):
        """Runs inference on a single image.
        Args:
            image: A PIL.Image object, raw input image.
        Returns:
            resized_image: RGB image resized from original input image.
            seg_map: Segmentation map of `resized_image`.
        """

        width, height = image.size
        scale = min(args.resize / width, args.resize / height)
        new_size = (int(scale * width + 0.5), int(scale * height + 0.5))
        image = image.resize(new_size, Image.ANTIALIAS)
        print(width, height, image.size, scale)

        batch_seg_map = self.sess.run(
            self.OUTPUT_TENSOR_NAME,
            feed_dict={
                self.INPUT_TENSOR_NAME: [np.asarray(image)]
            })
        seg_map = batch_seg_map[0]
        print(seg_map.shape)
        size = (int(seg_map.shape[1] / scale+ 0.5), int(seg_map.shape[0] / scale + 0.5))
        seg_map = np.asarray(Image.fromarray(seg_map).resize(size, Image.NEAREST))
        return seg_map[:height, :width]

def compute_accuracy(cm):
    sum_diag = np.diag(cm)
    sum_row = cm.sum(axis=0)
    sum_col = cm.sum(axis=1)
    union = sum_row + sum_col - sum_diag
    class_iou = np.where(union > 0, sum_diag / union.astype(np.float32), np.zeros_like(sum_diag))
    class_acc = np.where(sum_col > 0, sum_diag / sum_col.astype(np.float32), np.zeros_like(sum_diag))

    iou = class_iou.sum() / np.sum((class_iou != 0))
    acc = class_acc.sum() / np.sum((class_acc != 0))
    pixel_acc = sum_diag.sum() / cm.sum()

    return iou, acc, pixel_acc, class_iou, class_acc

if __name__ == '__main__':
    classes = 9
    model = DeepLabModel(args.model_path)

    if not os.path.exists(args.output):
        os.mkdir(args.output)

    iou_list = []
    sum_cm = np.zeros((classes - 1, classes - 1), np.int64)
    for i, line in enumerate(open(os.path.join(args.data_path, args.image_list), "r")):
        names = line.split(' ')
        id = int(names[0].strip()) - 1
        image_name = '%s/nyu_images/%d.jpg' % (args.data_path, id)
        image = Image.open(image_name)

        label_name = '%s/nyuv2_all_class8/new_nyu_class8_%04d.png' % (args.data_path, id)
        label = np.asarray(Image.open(label_name).convert('L'))
        print(id, image_name, label_name)

        pred = model.run(image)
        cm = confusion_matrix(label.flatten(), pred.flatten(), labels=range(1, classes))
        sum_cm += cm

        iou, acc, pixel_acc, class_iou, class_acc = compute_accuracy(cm)
        print(iou, acc, pixel_acc)
        iou_list.append((iou, acc, pixel_acc, id, np.sum((np.diag(cm) > 10))))

        pred_color = get_dataset_colormap.label_to_color_image(pred, args.colormap).astype(np.uint8)

        output_file = '%s/%d.png' % (args.output, id)
        Image.fromarray(pred_color.astype(dtype=np.uint8)).save(output_file)

    iou, acc, pixel_acc, class_iou, class_acc = compute_accuracy(sum_cm)
    print("mean IoU: %.6f" % iou)
    for i, x in enumerate(class_iou):
        print('    class %d IoU: %.6lf' % (i, x))

    print("class accuracy: %.6f" % acc)
    for i, x in enumerate(class_acc):
        print('    class %d accuracy: %.6lf' % (i, x))

    print("pixel accuracy: %.6f\n" % pixel_acc)
    print(sorted(iou_list, reverse=True)[:20])

    with open(args.report, 'w') as f:
        for x in iou_list:
            f.write('%d, %.6f, %.6f, %.6f, %d\n' % (x[3], x[0], x[1], x[2], x[4]))
