import numpy as np

'''
 All copyright to Connor Johnson
 Copied from http://connor-johnson.com/2014/02/01/smoothing-with-exponentially-weighted-moving-averages/
'''


def holt_winters_second_order_ewma( x, span, beta ):
    N = len(x)
    alpha = 2.0 / ( 1 + span )
    s = np.zeros(( N, ))
    b = np.zeros(( N, ))
    s[0] = x[0]
    for i in range( 1, N ):
        s[i] = alpha * x[i] + ( 1 - alpha )*( s[i-1] + b[i-1] )
        b[i] = beta * ( s[i] - s[i-1] ) + ( 1 - beta ) * b[i-1]
    return s