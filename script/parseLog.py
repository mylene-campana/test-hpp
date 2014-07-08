#/usr/bin/env python
# Parser script to gather float vector from a (Log) file
import numpy as np

logFile = "/local/mcampana/devel/hpp/install/var/log/hpp/"
endWhile = "INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.hh:56: finalPath:"

def parseConfig (pid, prefix):
    l = len (prefix)
    lend = len(endWhile)
    isNotFinished = True
    with open (logFile + "journal." + str(pid) + ".log") as f:
        configs = []
        for line in f:
            if line [:l] == prefix and isNotFinished :
                suffix = line [l:]
                st = suffix.strip (',\n') # remove end characters
                sp = st.split (',') # separate numbers with coma
                try:
                    config = map (float, sp) # convert into float
                    configs.append (config)
                    
                except:
                    print ("st=%s"%st)
                    print ("sp=%s"%sp)
            if line[:lend] == endWhile :
                isNotFinished = False
    return np.array (zip (*configs)) # transpose and make array

"""
gradAtt :
_,    _,
_,  , _,  , ....
_,    _,
"""

# --------------------------------------------------------------------#

# Depend on how is written the vector in Log file.
# Easier if separated by comas than irregular spaces.
def parseGrad (pid, prefix):
    l = len (prefix)
    lend = len(endWhile)
    isNotFinished = True
    with open (logFile + "journal." + str(pid) + ".log") as f:
        grads = []
        for line in f:
            if line [:l] == prefix and isNotFinished :
                suffix = line [l:]
                st = suffix.strip ('\n') # remove end characters
                sp = st.split (',') # separate numbers with coma
                try:
                    grad = map (float, sp) # convert into float
                    grads.append (grad)
                    
                except:
                    print ("st=%s"%st)
                    print ("sp=%s"%sp)
            if line[:lend] == endWhile :
                isNotFinished = False
    return np.array (zip (*grads)) # transpose and make array



