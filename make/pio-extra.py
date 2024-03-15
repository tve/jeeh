Import('env')
#print(env.Dump())
#Import('projenv')
#print(projenv.Dump())

genv = DefaultEnvironment()
#print(genv.Dump())
#print(genv["ENV"])
#print('    project_src',genv["PROJECT_SRC_DIR"])
#print('    pioenv',genv["PIOENV"])
#print('    opts',genv.GetProjectOptions(True))
#print('    config',genv.GetProjectConfig())

import os
import codegen

svdDir = os.path.expanduser("~/.platformio/platforms/ststm32/misc/svd")
svdPath = env.BoardConfig().get('debug.svd_path')
codegen.processAll(d=os.path.join(svdDir, svdPath),
                   e=genv["PIOENV"],
                   o=genv.GetProjectOptions(True),
                   p=genv["PROJECT_SRC_DIR"])
