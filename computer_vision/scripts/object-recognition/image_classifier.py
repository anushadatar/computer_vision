"""
Use tensorflow imagenet model to quickly recognize images. Contains classes
associated with the model itself and object name lookup.
Directly adapted from tensorflow tutorial for imagenet classification, which
falls under an Apache 2.0 license.
"""
import argparse
import os.path
import re
import sys
import tarfile

import numpy as np
from six.moves import urllib
import tensorflow as tf

# Constants associated with the model, can be modified as needed.
# Imagenet data to pull, using the inception dataset here but this can be anything.
URL = 'http://download.tensorflow.org/models/image/imagenet/inception-2015-12-05.tgz'
# The number of top predictions to keep from the imagenet output.
NUM_TOP_PREDICTIONS = 5 
# The temporary directory storing the model.
MODEL_DIR = '/tmp/imagenet'

class NodeLookup(object):
  """
  Converts IDs associated with Imagenet output to strings.
  Adapated from from tensorflow imagenet tutorial, which is held under the Apache license.
  """
  def __init__(self, label_lookup_path=None, uid_lookup_path=None):
    """
    Initialize node lookup object with label lookup and uid lookup paths.
    """
    if not label_lookup_path:
      label_lookup_path = os.path.join(
          MODEL_DIR, 'imagenet_2012_challenge_label_map_proto.pbtxt')
    if not uid_lookup_path:
      uid_lookup_path = os.path.join(
          MODEL_DIR, 'imagenet_synset_to_human_label_map.txt')
    self.node_lookup = self.load(label_lookup_path, uid_lookup_path)

  def load(self, label_lookup_path, uid_lookup_path):
    """
    Loads a human readable English name for each softmax node.
    label_lookup_path: string UID to integer node ID.
    uid_lookup_path: string UID to human-readable string.
    Returns: dict from integer node ID to human-readable string.
    """
    # Check that the both set sof files exist.
    if not tf.gfile.Exists(uid_lookup_path):
      tf.logging.fatal('File does not exist %s', uid_lookup_path)
    if not tf.gfile.Exists(label_lookup_path):
      tf.logging.fatal('File does not exist %s', label_lookup_path)

    # Parse descriptive output
    as_ascii_lines = tf.gfile.GFile(uid_lookup_path).readlines()
    uid_to_descriptive_string = {}
    expression = re.compile(r'[n\d]*[ \S,]*')
    for ascii_line in as_ascii_lines:
      parsed_output = expression.findall(ascii_line)
      uid = parsed_output[0]
      descriptive_string = parsed_output[2]
      uid_to_descriptive_string[uid] = descriptive_string

    # Map strings to IDs.
    node_id_to_uid = {}
    proto_as_ascii = tf.gfile.GFile(label_lookup_path).readlines()
    for line in proto_as_ascii:
      if line.startswith('  target_class:'):
        target_class = int(line.split(': ')[1])
      if line.startswith('  target_class_string:'):
        target_class_string = line.split(': ')[1]
        node_id_to_uid[target_class] = target_class_string[1:-2]

    # Load ID to descriptive string mapping.
    node_id_to_name = {}
    for key, val in node_id_to_uid.items():
      if val not in uid_to_descriptive_string:
        tf.logging.fatal('Failed to locate: %s', val)
      name = uid_to_descriptive_string[val]
      node_id_to_name[key] = name
    return node_id_to_name

  def id_to_string(self, node_id):
    if node_id not in self.node_lookup:
      return ''
    return self.node_lookup[node_id]

def create_graph():
  """
  Creates a graph from the saved file graph_def.pb.
  """
  with tf.gfile.GFile(os.path.join(
      MODEL_DIR, 'classify_image_graph_def.pb'), 'rb') as graph_def_file:
    graph_def = tf.GraphDef()
    graph_def.ParseFromString(graph_def_file.read())
    _ = tf.import_graph_def(graph_def, name='')


def run_inference_on_image(image):
  """
  Runs classification on an image.
  image: File name corresponding to the image.
  """
  if not tf.gfile.Exists(image):
    tf.logging.fatal('File does not exist %s', image)
  image_data = tf.gfile.Gfile(image, 'rb').read()
  create_graph()

  # Generates and interprets softmax tensor.
  with tf.Session() as current_session:
    softmax_tensor = current_session.graph.get_tensor_by_name('softmax:0')
    predictions = current_session.run(softmax_tensor,
                           {'DecodeJpeg/contents:0': image_data})
    predictions = np.squeeze(predictions)

    # Creates nodelookup for mapping nodes to words.
    node_lookup = NodeLookup()

    # Gets specified number of top predictions as readable strings.
    top_k = predictions.argsort()[-NUM_TOP_PREDICTIONS:][::-1]
    for node_id in top_k:
      descriptive_string = node_lookup.id_to_string(node_id)
      item_score = predictions[node_id]
      print('%s (score = %.5f)' % (descriptive_string, item_score))

def dowload_model():
  """
  Download and extract the model from the file-constant URL.
  """
  dest_directory = MODEL_DIR
  if not os.path.exists(dest_directory):
    os.makedirs(dest_directory)
  filename = URL.split('/')[-1]
  filepath = os.path.join(dest_directory, filename)
  if not os.path.exists(filepath):
    def _progress(count, block_size, total_size):
      sys.stdout.write('\r>> Downloading %s %.1f%%' % (
          filename, float(count * block_size) / float(total_size) * 100.0))
      sys.stdout.flush()
    filepath, _ = urllib.request.urlretrieve(URL, filepath, _progress)
    print()
    statinfo = os.stat(filepath)
    print('Successfully downloaded', filename, statinfo.st_size, 'bytes.')
  tarfile.open(filepath, 'r:gz').extractall(dest_directory)