import statespace.LTVUnicycleController as ltvUni
import statespace.LTVDiffDriveController as ltvDiff

unicycle = ltvUni
unicycle.calcGains()

cte = ltvDiff
cte.calcGains(True)