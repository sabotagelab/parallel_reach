. ~/cuda-venv/bin/activate;
export PYTHONPATH=/home/dev/cuda-venv/src:$PYTHONPATH
cuda-gdb --args python3 -m pycuda.debug run_f1zono.py
