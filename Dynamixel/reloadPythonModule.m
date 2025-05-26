% Matlab再起動なしでpythonプログラムの修正を反映
clear classes
mod = py.importlib.import_module('Dynamixel');
py.importlib.reload(mod);
