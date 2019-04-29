"""Server-end for the ASR ROS requests."""
import argparse
import functools
import _init_paths
from utils.utility import add_arguments, print_arguments


NUM_CONV_LAYERS = 2
NUM_RNN_LAYERS = 3
RNN_LAYER_SIZE = 1024

USE_GRU = True
SHARE_RNN_WEIGHTS = False

SPECGRAM_TYPE = 'linear'


parser = argparse.ArgumentParser(description=__doc__)
add_arg = functools.partial(add_arguments, argparser=parser)

add_arg('beam_size',        int,    500,    "Beam search width.")
add_arg('alpha',            float,  2.5,   "Coef of LM for beam search.")
add_arg('beta',             float,  0.3,   "Coef of WC for beam search.")
add_arg('cutoff_prob',      float,  1.0,    "Cutoff probability for pruning.")
add_arg('cutoff_top_n',     int,    40,     "Cutoff number for pruning.")

add_arg('use_gpu',          bool,   False,   "Use GPU or not.")

add_arg('warmup_manifest',  str,
        'data/librispeech/manifest.test-clean',
        "Filepath of manifest to warm up.")
add_arg('mean_std_path',    str,
        'data/librispeech/mean_std.npz',
        "Filepath of normalizer's mean & std.")
add_arg('vocab_path',       str,
        'data/librispeech/eng_vocab.txt',
        "Filepath of vocabulary.")
add_arg('model_path',       str,
        './checkpoints/libri/params.latest.tar.gz',
        "If None, the training starts from scratch, "
        "otherwise, it resumes from the pre-trained model.")
add_arg('lang_model_path',  str,
        'lm/data/common_crawl_00.prune01111.trie.klm',
        "Filepath for language model.")
add_arg('decoding_method',  str,
        'ctc_beam_search',
        "Decoding method. Options: ctc_beam_search, ctc_greedy",
        choices = ['ctc_beam_search', 'ctc_greedy'])

args = parser.parse_args()


print_arguments(args)