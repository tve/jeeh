Import('env', 'projenv')
#print(env.Dump())
#print(projenv.Dump())

import os
import codegen

svdDir = os.path.expanduser("~/.platformio/platforms/ststm32/misc/svd")
svdPath = env.BoardConfig().get('debug.svd_path')
codegen.processAll(os.path.join(svdDir, svdPath))
