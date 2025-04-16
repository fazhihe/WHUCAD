from cadlib.Catia_utils import *
from cadlib.CAD_Class import *
import win32com.client
import h5py
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--src', type=str, default=None, required=True)
args = parser.parse_args()

doc = None

input_path = args.src

f = h5py.File(input_path, 'r')
if 'out_vec' in f.keys():
    macro_vec = f['out_vec'][:]
else:
    macro_vec = f['vec'][:]
try:
    cad = Macro_Seq.from_vector(macro_vec, is_numerical=True, n=256)
    catia = win32com.client.Dispatch('catia.application')
    catia.visible = 1
    doc = catia.documents.add('Part')
    part = doc.part
    create_CAD_CATIA(cad, catia, doc, part)
    print(input_path, ' OK')
except:
    print(input_path, ' Error')

