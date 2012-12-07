# This patch is from Antons?
cd `rospack find gearbox`
patch -d src/hokuyo_aist/python --verbose < `rospack find gearbox`/aist_python_mt.patch