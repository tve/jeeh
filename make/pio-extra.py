Import('env')
import os

svdDir = os.path.expanduser("~/.platformio/platforms/ststm32/misc/svd")
svdPath = env.BoardConfig().get('debug.svd_path')

import codegen
codegen.processAll(os.path.join(svdDir, svdPath))
