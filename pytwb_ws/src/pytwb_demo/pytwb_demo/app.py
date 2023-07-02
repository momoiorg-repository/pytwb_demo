from pytwb.lib_main import run
from vector_map import init_visualize

def app_main(trees):
# insert application specific initialization routine here
    init_visualize()
    run(trees)